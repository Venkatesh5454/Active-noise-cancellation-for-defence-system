// =============================================================================
// axi_mem_model.v  -  behavioural AXI4 write-only memory with protocol checks
// -----------------------------------------------------------------------------
// Stands in for DDR (or the block RAM) behind the DMA writer in simulation.
// It is an AXI4 slave for the write channels only (AW, W, B), with a 32-bit
// data bus.  Memory: MEM_BYTES bytes that answer addresses
// BASE_ADDR .. BASE_ADDR + MEM_BYTES - 1, all filled with FILL at time 0.
//
// How it works:
//   * AW requests and W beats are queued separately, so W beats may arrive
//     before, together with, or after their AW (AXI allows all three).
//   * As soon as an AW and W beats are both there, the beats are written
//     into the memory (byte lanes by wstrb).
//   * After the last beat of a burst, the B response is queued and given
//     0 .. b_max_delay clocks later.  bvalid is never raised before the last
//     W beat and the AW of that burst have been accepted.
//   * awready / wready are random: high in a clock with aw_ready_pct /
//     w_ready_pct percent chance (100 = always ready).
//
// Protocol checks (each one prints "<NAME>: PROTOCOL ERROR ..." and counts in
// proto_errors; the first MAX_PRINT are printed):
//   - AWVALID/WVALID dropped before READY, or AW/W signals changed while
//     waiting for READY
//   - X on valid/ready-side control signals, or on AW fields / strobed data
//   - burst type not INCR, awsize bigger than the 32-bit bus
//   - a burst that crosses a 4 KB boundary
//   - WLAST not exactly on beat awlen (early or missing), so the number of
//     beats must be awlen+1
//   - wstrb bits set outside the byte lanes of a narrow / unaligned beat
// Addresses outside the memory get DECERR (count: range_errors, not written).
//
// Run-time controls for a testbench (hierarchical calls, e.g. u_mem.rd8(a)):
//   set_ready(aw_pct, w_pct, b_max)   back-pressure (percent, percent, clocks)
//   inject_bresp(n, resp)             the next n bursts answer resp (2 SLVERR,
//                                     3 DECERR) and their data is NOT stored
//   rd8(addr) / rd32(addr)            backdoor read (rd32 is little-endian)
//   wr8(addr, v) / fill(v)            backdoor write / fill the whole memory
//   report                            print the statistics line
//   counters: n_aw, n_beats, n_bursts, n_b, proto_errors, range_errors,
//             err_bresp (bursts answered with a non-OKAY response)
// This is a simulation model (initial blocks, integers, tasks), not RTL.
// =============================================================================
`timescale 1ns / 1ps
module axi_mem_model #(
    parameter MEM_BYTES    = 65536,           // memory size in bytes
    parameter BASE_ADDR    = 32'h0000_0000,   // first byte address of the memory
    parameter SEED         = 1,               // random seed (repeatable runs)
    parameter AW_READY_PCT = 100,             // start value of aw_ready_pct
    parameter W_READY_PCT  = 100,             // start value of w_ready_pct
    parameter B_MAX_DELAY  = 0,               // start value of b_max_delay
    parameter [7:0] FILL   = 8'h00,           // memory contents at time 0
    parameter MAX_PRINT    = 20,              // errors printed (all are counted)
    parameter VERBOSE      = 0,               // 1: print every burst
    parameter NAME         = "axi_mem"
) (
    input  wire        clk,
    input  wire        rst_n,
    // write address channel
    input  wire [31:0] s_axi_awaddr,
    input  wire [7:0]  s_axi_awlen,
    input  wire [2:0]  s_axi_awsize,
    input  wire [1:0]  s_axi_awburst,
    input  wire [3:0]  s_axi_awcache,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    // write data channel
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wlast,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    // write response channel
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready
);
    localparam QD = 16;                // queue depth (AW, W and B)

    reg [7:0] mem [0:MEM_BYTES-1];

    // ---------------- run-time knobs ----------------
    integer   aw_ready_pct, w_ready_pct, b_max_delay;
    integer   err_left;                // bursts still to answer with err_resp
    reg [1:0] err_resp;
    integer   verbose;

    // ---------------- statistics ----------------
    integer n_aw, n_beats, n_bursts, n_b;
    integer proto_errors, range_errors, err_bresp;

    integer seed, cyc, k;

    // ---------------- queues ----------------
    reg [31:0] aq_addr  [0:QD-1];
    reg [7:0]  aq_len   [0:QD-1];
    reg [2:0]  aq_size  [0:QD-1];
    reg [1:0]  aq_burst [0:QD-1];
    integer    aq_rp, aq_wp, aq_n;

    reg [31:0] wq_data  [0:QD-1];
    reg [3:0]  wq_strb  [0:QD-1];
    reg        wq_last  [0:QD-1];
    integer    wq_rp, wq_wp, wq_n;

    reg [1:0]  bq_resp  [0:QD-1];
    integer    bq_due   [0:QD-1];
    integer    bq_rp, bq_wp, bq_n;

    // ---------------- burst being written ----------------
    reg        cb_on;
    reg [31:0] cb_start;               // awaddr
    reg [31:0] cb_aligned;             // awaddr aligned to the beat size
    reg [7:0]  cb_len;                 // awlen
    integer    cb_bytes;               // bytes per beat
    integer    cb_beat;                // beat number
    reg [1:0]  cb_resp;                // response for this burst

    // ---------------- last-clock copies for the "hold until ready" rules ----
    reg        p_awwait, p_wwait;
    reg [50:0] p_aw;                   // {addr, len, size, burst, cache, prot}
    reg [36:0] p_w;                    // {data, strb, last}

    wire [50:0] cur_aw = {s_axi_awaddr, s_axi_awlen, s_axi_awsize, s_axi_awburst,
                          s_axi_awcache, s_axi_awprot};
    wire [36:0] cur_w  = {s_axi_wdata, s_axi_wstrb, s_axi_wlast};

    initial begin
        for (k = 0; k < MEM_BYTES; k = k + 1) mem[k] = FILL;
        aw_ready_pct = AW_READY_PCT;
        w_ready_pct  = W_READY_PCT;
        b_max_delay  = B_MAX_DELAY;
        err_left     = 0;
        err_resp     = 2'b10;
        verbose      = VERBOSE;
        seed         = SEED;
        cyc          = 0;
        n_aw = 0; n_beats = 0; n_bursts = 0; n_b = 0;
        proto_errors = 0; range_errors = 0; err_bresp = 0;
    end

    // ---------------- testbench helpers ----------------
    task set_ready(input integer aw_pct, input integer w_pct, input integer b_max);
        begin
            aw_ready_pct = aw_pct;
            w_ready_pct  = w_pct;
            b_max_delay  = b_max;
        end
    endtask

    task inject_bresp(input integer n, input [1:0] resp);
        begin
            err_left = n;
            err_resp = resp;
        end
    endtask

    function [7:0] rd8(input [31:0] addr);
        begin
            if (addr >= BASE_ADDR && (addr - BASE_ADDR) < MEM_BYTES)
                rd8 = mem[addr - BASE_ADDR];
            else
                rd8 = 8'hxx;
        end
    endfunction

    function [31:0] rd32(input [31:0] addr);
        begin
            rd32 = {rd8(addr + 32'd3), rd8(addr + 32'd2), rd8(addr + 32'd1), rd8(addr)};
        end
    endfunction

    task wr8(input [31:0] addr, input [7:0] v);
        begin
            if (addr >= BASE_ADDR && (addr - BASE_ADDR) < MEM_BYTES)
                mem[addr - BASE_ADDR] = v;
        end
    endtask

    task fill(input [7:0] v);
        integer i;
        begin
            for (i = 0; i < MEM_BYTES; i = i + 1) mem[i] = v;
        end
    endtask

    task report;
        begin
            $display("%0s: %0d bursts, %0d beats, %0d B responses (%0d not OKAY), %0d protocol errors, %0d range errors",
                     NAME, n_bursts, n_beats, n_b, err_bresp, proto_errors, range_errors);
        end
    endtask

    task proto_error(input [8*64-1:0] msg);
        begin
            proto_errors = proto_errors + 1;
            if (proto_errors <= MAX_PRINT)
                $display("%0s: PROTOCOL ERROR: %0s (t=%0t)", NAME, msg, $time);
        end
    endtask

    // take the AW at the front of the queue and check it
    task start_burst;
        reg [2:0]  size;
        reg [1:0]  burst;
        reg [31:0] last_byte;
        begin
            cb_start = aq_addr[aq_rp];
            cb_len   = aq_len[aq_rp];
            size     = aq_size[aq_rp];
            burst    = aq_burst[aq_rp];
            aq_rp    = (aq_rp + 1) % QD;
            aq_n     = aq_n - 1;
            cb_on    = 1'b1;
            cb_beat  = 0;
            cb_resp  = 2'b00;
            n_bursts = n_bursts + 1;

            if (size > 3'd2) begin
                proto_error("awsize is wider than the 32-bit data bus");
                size = 3'd2;
            end
            if (burst != 2'b01) proto_error("burst type is not INCR");
            cb_bytes   = 1 << size;
            cb_aligned = cb_start & ~(cb_bytes - 1);
            last_byte  = cb_aligned + (cb_len + 1) * cb_bytes - 1;
            if (cb_start[31:12] != last_byte[31:12])
                proto_error("burst crosses a 4 KB boundary");

            if (cb_start < BASE_ADDR || last_byte < cb_start ||
                (last_byte - BASE_ADDR) >= MEM_BYTES) begin
                range_errors = range_errors + 1;
                cb_resp = 2'b11;                         // DECERR
                if (range_errors <= MAX_PRINT)
                    $display("%0s: address %h (len %0d) is outside the memory (t=%0t)",
                             NAME, cb_start, cb_len, $time);
            end else if (err_left > 0) begin
                cb_resp  = err_resp;                     // injected error
                err_left = err_left - 1;
            end
            if (verbose)
                $display("%0s: burst addr %h len %0d size %0d burst %0d -> resp %0d (t=%0t)",
                         NAME, cb_start, cb_len, size, burst, cb_resp, $time);
        end
    endtask

    // take the W beat at the front of the queue and write it
    task do_beat;
        reg [31:0] data;
        reg [3:0]  strb;
        reg        last, is_last;
        reg [31:0] baddr;
        reg [3:0]  lanes;
        integer    lo, hi, i;
        begin
            data  = wq_data[wq_rp];
            strb  = wq_strb[wq_rp];
            last  = wq_last[wq_rp];
            wq_rp = (wq_rp + 1) % QD;
            wq_n  = wq_n - 1;
            n_beats = n_beats + 1;

            // address of this beat and the byte lanes it may use
            if (cb_beat == 0) baddr = cb_start;
            else              baddr = cb_aligned + cb_beat * cb_bytes;
            lo = baddr[1:0];
            hi = ((baddr & ~(cb_bytes - 1)) & 3) + cb_bytes - 1;
            lanes = 4'b0000;
            for (i = 0; i < 4; i = i + 1)
                if (i >= lo && i <= hi) lanes[i] = 1'b1;
            if ((strb & ~lanes) != 4'b0000)
                proto_error("wstrb set outside the active byte lanes");

            if (cb_resp == 2'b00)
                for (i = 0; i < 4; i = i + 1)
                    if (strb[i] && lanes[i])
                        mem[(baddr & ~32'd3) + i - BASE_ADDR] = data[8*i +: 8];

            is_last = (cb_beat == cb_len);
            if (last && !is_last)  proto_error("wlast came before beat awlen (too few beats)");
            if (!last && is_last)  proto_error("wlast missing on beat awlen");
            cb_beat = cb_beat + 1;

            // the burst ends at wlast or after awlen+1 beats
            if (last || is_last) begin
                cb_on = 1'b0;
                bq_resp[bq_wp] = cb_resp;
                bq_due[bq_wp]  = cyc + ({$random(seed)} % (b_max_delay + 1));
                bq_wp = (bq_wp + 1) % QD;
                bq_n  = bq_n + 1;
                if (cb_resp != 2'b00) err_bresp = err_bresp + 1;
            end
        end
    endtask

    // ---------------- the slave, one clock at a time ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            aq_rp = 0; aq_wp = 0; aq_n = 0;
            wq_rp = 0; wq_wp = 0; wq_n = 0;
            bq_rp = 0; bq_wp = 0; bq_n = 0;
            cb_on = 1'b0;
            p_awwait = 1'b0;
            p_wwait  = 1'b0;
        end else begin
            cyc = cyc + 1;

            // ---- master rules: VALID stays high and payload stays put ----
            if (^{s_axi_awvalid, s_axi_wvalid, s_axi_bready} === 1'bx)
                proto_error("X on awvalid / wvalid / bready");
            if (p_awwait) begin
                if (s_axi_awvalid !== 1'b1)  proto_error("awvalid dropped before awready");
                else if (cur_aw !== p_aw)    proto_error("AW signals changed while waiting for awready");
            end
            if (p_wwait) begin
                if (s_axi_wvalid !== 1'b1)   proto_error("wvalid dropped before wready");
                else if (cur_w !== p_w)      proto_error("W signals changed while waiting for wready");
            end

            // ---- B handshake ----
            if (s_axi_bvalid && s_axi_bready) begin
                bq_rp = (bq_rp + 1) % QD;
                bq_n  = bq_n - 1;
                n_b   = n_b + 1;
            end

            // ---- AW handshake ----
            if (s_axi_awvalid === 1'b1 && s_axi_awready) begin
                if (^cur_aw === 1'bx) proto_error("X on the AW channel");
                aq_addr[aq_wp]  = s_axi_awaddr;
                aq_len[aq_wp]   = s_axi_awlen;
                aq_size[aq_wp]  = s_axi_awsize;
                aq_burst[aq_wp] = s_axi_awburst;
                aq_wp = (aq_wp + 1) % QD;
                aq_n  = aq_n + 1;
                n_aw  = n_aw + 1;
            end

            // ---- W handshake (allowed before its AW) ----
            if (s_axi_wvalid === 1'b1 && s_axi_wready) begin
                if (^{s_axi_wstrb, s_axi_wlast} === 1'bx ||
                    (s_axi_wstrb[0] && ^s_axi_wdata[7:0]   === 1'bx) ||
                    (s_axi_wstrb[1] && ^s_axi_wdata[15:8]  === 1'bx) ||
                    (s_axi_wstrb[2] && ^s_axi_wdata[23:16] === 1'bx) ||
                    (s_axi_wstrb[3] && ^s_axi_wdata[31:24] === 1'bx))
                    proto_error("X on the W channel");
                wq_data[wq_wp] = s_axi_wdata;
                wq_strb[wq_wp] = s_axi_wstrb;
                wq_last[wq_wp] = s_axi_wlast;
                wq_wp = (wq_wp + 1) % QD;
                wq_n  = wq_n + 1;
            end

            // ---- write every beat that has its AW ----
            while (wq_n > 0 && (cb_on || aq_n > 0)) begin
                if (!cb_on) start_burst;
                do_beat;
            end

            // ---- remember what is waiting for READY ----
            p_awwait = (s_axi_awvalid === 1'b1) && !s_axi_awready;
            p_wwait  = (s_axi_wvalid === 1'b1) && !s_axi_wready;
            p_aw     = cur_aw;
            p_w      = cur_w;

            // ---- READY for the next clock (random back-pressure) ----
            s_axi_awready <= (aq_n + bq_n + cb_on < QD - 1) &&
                             (({$random(seed)} % 100) < aw_ready_pct);
            s_axi_wready  <= (wq_n < QD - 1) &&
                             (({$random(seed)} % 100) < w_ready_pct);

            // ---- B response when its delay is over (held until bready) ----
            if (bq_n > 0 && bq_due[bq_rp] <= cyc) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= bq_resp[bq_rp];
            end else begin
                s_axi_bvalid <= 1'b0;
                s_axi_bresp  <= 2'b00;
            end
        end
    end
endmodule

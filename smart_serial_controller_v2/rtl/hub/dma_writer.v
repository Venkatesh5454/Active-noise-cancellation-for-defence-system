// =============================================================================
// dma_writer.v  -  NoC node 2: stores RECORD packets in a ring buffer in memory
// -----------------------------------------------------------------------------
// The sensor hub sends every measurement as a 16-byte RECORD packet (SPEC 3.3)
// to this node.  The DMA writer copies each record into a ring buffer in DDR
// (or the block RAM of the FPGA-only build) with one AXI4 burst, so the ARM
// can read a whole batch of records later instead of being interrupted for
// every single one.
//
// Ring buffer: SIZE = 2^SIZE_LOG2 slots of 16 bytes, starting at DMA_BASE.
//     slot i lives at BASE + 16*i
//     WR_IDX = next slot the hardware writes, RD_IDX = next slot software reads
//     the ring is full when (wr - rd) mod SIZE == SIZE-1 (one slot stays free,
//     so "full" and "empty" never look the same)
//
// One record, step by step (state machine):
//   IDLE   take the packet header from noc_pkt_rx.  A RECORD (type 3) with EN=1
//          is kept; every other packet (or any packet while EN=0) is consumed
//          and counted in IGNORED.
//   RECV   take the payload bytes, one per clock.  The first 16 bytes go into
//          the 128-bit record buffer (bytes that never arrive stay 0), extra
//          bytes are thrown away.
//   CHECK  ring full?  yes -> drop the record, OVERFLOW++, ev_overflow pulse.
//          no  -> start the AXI burst to BASE + wr*16.
//   AXI    AW and W are offered together (AXI allows any order).  4 beats of
//          32 bits, wstrb = F, wlast on beat 3.  The buffer shifts right by
//          one word after every beat, so wdata is always buffer[31:0].
//   RESP   wait for B.  A non-OKAY bresp counts in AXI_ERR (+ ev_axi_err), but
//          the slot is still used: wr = wr+1, COUNT++, PENDING++.
// Only one burst is ever outstanding; nothing is written while waiting for B.
//
// Batches (the interrupt): PENDING counts records written since the last
// ev_batch.  ev_batch pulses when PENDING reaches BATCH, or when TIMEOUT ms
// have passed since the oldest pending record was written; then PENDING = 0.
//
// Design choices (where the SPEC leaves room):
//   * BASE[3:0] are ignored (they read back as 0): records are 16-byte aligned,
//     so a burst never crosses a 4 KB boundary.
//   * EN is looked at when the header arrives.  With EN=0 RECORDs are
//     consumed and counted in IGNORED, nothing is written.
//   * A RECORD with length 0 writes 16 zero bytes ("pad with 0").
//   * SIZE_LOG2 writes are clamped to 1..12.  WR_IDX/RD_IDX are used (and read
//     back) modulo the ring size.  Change SIZE_LOG2 together with a RESET.
//   * RESET (DMA_CTRL bit 8, write 1) clears wr, pending and all counters as
//     the SPEC says, and also RD_IDX, so the ring is empty afterwards.  A
//     burst already on the bus cannot be cut short: it finishes, but it is not
//     counted (wr, COUNT, PENDING, AXI_ERR stay as RESET left them).
//     Anything that happens in the same clock as RESET is not counted either.
//   * An AXI error still uses the slot (the SPEC moves wr after any B), so a
//     bad memory region gives counted errors instead of a stuck DMA.
//   * BATCH = 0 disables the count trigger (only the time-out is left).
//   * The time-out counts ms_ticks: ev_batch comes on the TIMEOUT-th ms_tick
//     after the oldest pending record was written, i.e. between TIMEOUT-1 and
//     TIMEOUT ms later (ms_tick is not lined up with the record).
//   * The node never sends packets: out_valid = 0, out_flit = 0.
// =============================================================================
`timescale 1ns / 1ps
module dma_writer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ms_tick,
    input  wire [2:0]  my_id,
    // register port (region 0x700, byte offset in reg_addr)
    input  wire        reg_we,
    input  wire        reg_re,
    input  wire [7:0]  reg_addr,
    input  wire [31:0] reg_wdata,
    output reg  [31:0] reg_rdata,
    // NoC link (in_* from the network, out_* to the network)
    input  wire        in_valid,
    input  wire [33:0] in_flit,
    output wire        in_credit,
    output wire        out_valid,
    output wire [33:0] out_flit,
    input  wire        out_credit,
    // AXI4 write master
    output wire [31:0] m_axi_awaddr,
    output wire [7:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output wire [3:0]  m_axi_awcache,
    output wire [2:0]  m_axi_awprot,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [31:0] m_axi_wdata,
    output wire [3:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    // one-clock events for the interrupt register
    output reg         ev_batch,
    output reg         ev_overflow,
    output reg         ev_axi_err
);
    localparam [2:0] T_RECORD = 3'd3;

    localparam [2:0] S_IDLE  = 3'd0,
                     S_RECV  = 3'd1,
                     S_CHECK = 3'd2,
                     S_AXI   = 3'd3,
                     S_RESP  = 3'd4;

    // ---------------- software registers ----------------
    reg        en;            // DMA_CTRL[0]
    reg [27:0] base;          // DMA_BASE[31:4]
    reg [3:0]  size_log2;     // DMA_SIZE_LOG2, 1..12
    reg [11:0] rd_idx;        // DMA_RD_IDX (written by software)
    reg [7:0]  batch;         // DMA_BATCH
    reg [15:0] timeout;       // DMA_TIMEOUT, ms
    // ---------------- hardware state ----------------
    reg [11:0] wr_idx;        // DMA_WR_IDX
    reg [31:0] cnt_rec;       // COUNT
    reg [31:0] cnt_ovf;       // OVERFLOW
    reg [31:0] cnt_err;       // AXI_ERR
    reg [31:0] cnt_ign;       // IGNORED
    reg [31:0] pending;       // PENDING
    reg [15:0] age;           // ms_ticks since the oldest pending record

    reg [2:0]   state;
    reg [127:0] rec;          // record bytes, byte i at rec[8i+7:8i]
    reg [5:0]   bidx;         // payload byte number inside the packet
    reg         is_rec;       // this packet is a RECORD that will be written
    reg         kill;         // RESET came while busy: finish, but do not count
    reg [27:0]  aw_addr;      // burst address [31:4]
    reg         aw_pend;      // AWVALID
    reg         w_pend;       // WVALID
    reg [1:0]   beat;         // W beat number 0..3

    // ---------------- ring arithmetic ----------------
    // mask = SIZE-1 (SIZE_LOG2 = 12 gives 12'hFFF)
    wire [11:0] mask  = ~(12'hFFF << size_log2);
    wire [11:0] wr_m  = wr_idx & mask;
    wire [11:0] rd_m  = rd_idx & mask;
    wire [11:0] used  = (wr_m - rd_m) & mask;     // records not yet read
    wire        full  = (used == mask);
    wire        empty = (used == 12'd0);
    wire        busy  = (state != S_IDLE);

    // ---------------- packet receiver (noc_pkt_rx, SPEC 3.7) ----------------
    wire       hdr_valid;
    wire [2:0] ptype;
    wire [5:0] len;
    wire       b_valid;
    wire [7:0] b_data;
    wire       b_last;
    wire       hdr_ready = (state == S_IDLE);
    wire       b_ready   = (state == S_RECV);

    noc_pkt_rx #(.DEPTH(4)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_flit(in_flit), .in_credit(in_credit),
        .hdr_valid(hdr_valid), .ptype(ptype), .dest(), .src(), .prio(),
        .len(len), .tag(), .arg(), .hdr_ready(hdr_ready),
        .b_valid(b_valid), .b_data(b_data), .b_last(b_last), .b_ready(b_ready));

    // this node never sends anything
    assign out_valid = 1'b0;
    assign out_flit  = 34'd0;

    // ---------------- decoded events ----------------
    wire do_reset = reg_we && (reg_addr[7:2] == 6'h00) && reg_wdata[8];
    wire kill_now = kill || do_reset;         // this record no longer counts

    wire take_hdr = hdr_valid && hdr_ready;
    wire keep_hdr = (ptype == T_RECORD) && en;  // a RECORD we will write

    wire ign_now  = take_hdr && !keep_hdr;                   // IGNORED++
    wire ovf_now  = (state == S_CHECK) && !kill_now && full; // drop: OVERFLOW++
    wire b_take   = (state == S_RESP) && m_axi_bvalid;       // B handshake
    wire commit   = b_take && !kill_now;                     // record written
    wire err_now  = commit && (m_axi_bresp != 2'b00);        // AXI_ERR++

    // AW / W finished by the end of this clock?
    wire aw_done  = !aw_pend || m_axi_awready;
    wire w_done   = !w_pend || (m_axi_wready && beat == 2'd3);

    // ---------------- record state machine ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            rec     <= 128'd0;
            bidx    <= 6'd0;
            is_rec  <= 1'b0;
            kill    <= 1'b0;
            aw_addr <= 28'd0;
            aw_pend <= 1'b0;
            w_pend  <= 1'b0;
            beat    <= 2'd0;
        end else begin
            if (state == S_IDLE) kill <= 1'b0;
            else if (do_reset)   kill <= 1'b1;

            case (state)
                // take a header; a record starts as 16 zero bytes
                S_IDLE: if (hdr_valid) begin
                    rec    <= 128'd0;
                    bidx   <= 6'd0;
                    is_rec <= keep_hdr;
                    if (len != 6'd0)   state <= S_RECV;
                    else if (keep_hdr) state <= S_CHECK;  // empty RECORD: all zeros
                end

                // take the payload; keep only the first 16 bytes of a record
                S_RECV: begin
                    if (b_valid) begin
                        if (is_rec && bidx < 6'd16) rec[8*bidx[3:0] +: 8] <= b_data;
                        bidx <= bidx + 6'd1;
                    end
                    // last byte taken (a header showing up here can only mean
                    // a broken packet with no payload: stop waiting for it)
                    if ((b_valid && b_last) || hdr_valid)
                        state <= is_rec ? S_CHECK : S_IDLE;
                end

                // ring full -> drop, otherwise start the burst
                S_CHECK: begin
                    if (kill_now || full) begin
                        state <= S_IDLE;
                    end else begin
                        aw_addr <= base + {16'd0, wr_m};    // BASE + wr*16
                        aw_pend <= 1'b1;
                        w_pend  <= 1'b1;
                        beat    <= 2'd0;
                        state   <= S_AXI;
                    end
                end

                // AW and W handshakes, in any order
                S_AXI: begin
                    if (aw_pend && m_axi_awready) aw_pend <= 1'b0;
                    if (w_pend && m_axi_wready) begin
                        rec  <= {32'd0, rec[127:32]};       // next word to [31:0]
                        beat <= beat + 2'd1;
                        if (beat == 2'd3) w_pend <= 1'b0;
                    end
                    if (aw_done && w_done) state <= S_RESP;
                end

                // wait for the write response
                S_RESP: if (m_axi_bvalid) state <= S_IDLE;

                default: state <= S_IDLE;
            endcase
        end
    end

    // ---------------- counters and write index ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_idx  <= 12'd0;
            cnt_rec <= 32'd0;
            cnt_ovf <= 32'd0;
            cnt_err <= 32'd0;
            cnt_ign <= 32'd0;
        end else if (do_reset) begin
            wr_idx  <= 12'd0;
            cnt_rec <= 32'd0;
            cnt_ovf <= 32'd0;
            cnt_err <= 32'd0;
            cnt_ign <= 32'd0;
        end else begin
            if (ign_now) cnt_ign <= cnt_ign + 32'd1;
            if (ovf_now) cnt_ovf <= cnt_ovf + 32'd1;
            if (err_now) cnt_err <= cnt_err + 32'd1;
            if (commit) begin
                cnt_rec <= cnt_rec + 32'd1;
                wr_idx  <= (wr_m + 12'd1) & mask;     // wr = (wr + 1) mod SIZE
            end
        end
    end

    // ---------------- batch / time-out (ev_batch) ----------------
    wire [31:0] pend_next  = pending + {31'd0, commit};
    wire        batch_hit  = (batch != 8'd0) && (pend_next >= {24'd0, batch});
    wire        tmo_hit    = ms_tick && (timeout != 16'd0) && (pending != 32'd0) &&
                             (age >= timeout - 16'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending <= 32'd0;
            age     <= 16'd0;
        end else if (do_reset) begin
            pending <= 32'd0;
            age     <= 16'd0;
        end else if (batch_hit || tmo_hit) begin
            pending <= 32'd0;
            age     <= 16'd0;
        end else begin
            pending <= pend_next;
            if (pending == 32'd0)                    age <= 16'd0;  // timer starts at the first record
            else if (ms_tick && age != 16'hFFFF)     age <= age + 16'd1;
        end
    end

    // ---------------- events (registered one-clock pulses) ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ev_batch    <= 1'b0;
            ev_overflow <= 1'b0;
            ev_axi_err  <= 1'b0;
        end else begin
            ev_batch    <= !do_reset && (batch_hit || tmo_hit);
            ev_overflow <= ovf_now;
            ev_axi_err  <= err_now;
        end
    end

    // ---------------- software register writes ----------------
    // SIZE_LOG2 is clamped to 1..12
    wire [3:0] size_w = (reg_wdata[3:0] == 4'd0) ? 4'd1 :
                        (reg_wdata[3:0] > 4'd12) ? 4'd12 : reg_wdata[3:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en        <= 1'b0;
            base      <= 28'd0;
            size_log2 <= 4'd6;
            rd_idx    <= 12'd0;
            batch     <= 8'd32;
            timeout   <= 16'd1000;
        end else if (reg_we) begin
            case (reg_addr[7:2])
                6'h00: begin
                    en <= reg_wdata[0];
                    if (reg_wdata[8]) rd_idx <= 12'd0;   // RESET empties the ring
                end
                6'h01: base      <= reg_wdata[31:4];
                6'h02: size_log2 <= size_w;
                6'h04: rd_idx    <= reg_wdata[11:0];
                6'h05: batch     <= reg_wdata[7:0];
                6'h06: timeout   <= reg_wdata[15:0];
                default: ;
            endcase
        end
    end

    // ---------------- register read (combinational) ----------------
    always @(*) begin
        case (reg_addr[7:2])
            6'h00:   reg_rdata = {31'd0, en};                 // DMA_CTRL
            6'h01:   reg_rdata = {base, 4'd0};                // DMA_BASE
            6'h02:   reg_rdata = {28'd0, size_log2};          // DMA_SIZE_LOG2
            6'h03:   reg_rdata = {20'd0, wr_m};               // DMA_WR_IDX
            6'h04:   reg_rdata = {20'd0, rd_m};               // DMA_RD_IDX
            6'h05:   reg_rdata = {24'd0, batch};              // DMA_BATCH
            6'h06:   reg_rdata = {16'd0, timeout};            // DMA_TIMEOUT
            6'h07:   reg_rdata = cnt_rec;                     // COUNT
            6'h08:   reg_rdata = cnt_ovf;                     // OVERFLOW
            6'h09:   reg_rdata = cnt_err;                     // AXI_ERR
            6'h0A:   reg_rdata = pending;                     // PENDING
            6'h0B:   reg_rdata = {29'd0, empty, full, busy};  // STATUS
            6'h0C:   reg_rdata = cnt_ign;                     // IGNORED
            default: reg_rdata = 32'd0;
        endcase
    end

    // ---------------- AXI4 write channel outputs ----------------
    assign m_axi_awaddr  = {aw_addr, 4'd0};
    assign m_axi_awlen   = 8'd3;          // 4 beats
    assign m_axi_awsize  = 3'd2;          // 4 bytes per beat
    assign m_axi_awburst = 2'b01;         // INCR
    assign m_axi_awcache = 4'b0011;       // normal, bufferable, non-cacheable
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awvalid = aw_pend;
    assign m_axi_wdata   = rec[31:0];
    assign m_axi_wstrb   = 4'hF;
    assign m_axi_wlast   = w_pend && (beat == 2'd3);
    assign m_axi_wvalid  = w_pend;
    assign m_axi_bready  = (state == S_RESP);
endmodule

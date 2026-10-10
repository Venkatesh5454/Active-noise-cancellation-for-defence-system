// =============================================================================
// tb_ni_i2c.v  -  unit test: ni_i2c + real v1 ssc_i2c + PmodTMP2 model
// -----------------------------------------------------------------------------
// Set-up (exactly like ssc2_core.v wires it):
//
//   TB requester --noc_pkt_tx--> ni_i2c --noc_pkt_rx--> TB reply collector
//                                   |
//          CPU strobes --OR/mux--> ssc_i2c ==SCL/SDA== tb_i2c_slave 0x4B (TMP2)
//                                  (open drain,        tb_i2c_monitor (bus log)
//                                   pull-ups)          tb_i2c_master  (2nd master)
//                                                      "rogue" SCL holder
//
// While ni_i2c.active = 1 the engine is forced to: enabled, no auto-write,
// 7-bit address = addr7.  The "CPU" settings are deliberately different
// (disabled, auto-write on, 10-bit address 0x2A5) to prove the forcing.
// I2C runs at 1 MHz (prescale 24) to keep the simulation short, and at
// 100 kHz for the arbitration test (the second master model runs at 100 kHz).
//
// Tests:
//   1. slides 16-18: XFER_REQ 0x4B [01][02][00] -> [00][0C 80], bus log
//      "S 96 A 00 A Sr 97 A 0C A 80 N P" (repeated START)
//   2. write only, 3. write then read back, 4. read only (no write phase)
//   5. long read: rlen 60 (bytes popped as they arrive), heavy stall;
//      5b a 300 us stall fills the RX FIFO (the engine waits), 5c the stall
//      covers the end of a read, so ev_done comes before the last bytes
//   6. address probe (wlen = rlen = 0): ACK -> [00], no device -> [01]
//   7. NACK on the write address and on the read address -> [01]; the unsent
//      write byte is flushed (abort) and the next read is correct
//   8. arbitration lost against a second master -> [02]
//   9. time-out: a device holds SCL low -> abort pulse after timeout_us,
//      [03]; the next request works.  timeout_us = 0 -> [03], bus untouched
//  10. bad requests -> [04] (wlen > 16, rlen > 60, wrong lengths)
//  11. DATA: 5-byte write, 20-byte write (one transaction), DATA with NACK
//  12. the CPU leaves 2 old bytes in the RX FIFO; the NI throws them away.
//      NI requests that arrive while the CPU's own command runs wait for it;
//      a stuck CPU command does not get the NI's byte, the NI times out
//  13. other packet types and en = 0: consumed and dropped, bus untouched
//  14. heavy stall + 3 back-to-back requests + reply back-pressure
//  15. counters
// Global monitors: no engine strobe while stall = 1, no command while busy,
// no TX FIFO overflow, every reply leaves after the engine is idle and free.
// =============================================================================
`timescale 1ns / 1ps
module tb_ni_i2c;
    localparam [2:0] T_DATA = 3'd0, T_XFER = 3'd1, T_RESP = 3'd2;
    localparam [2:0] MY_ID = 3'd5;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0;
    integer errors = 0;
    task check(input cond, input [8*72-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    integer seed  = 5;      // requester gaps, data
    integer sseed = 17;     // stall generator
    integer rseed = 19;     // reply collector

    // ------------------------------------------------------------------
    // requester (sends packets to the NI)
    // ------------------------------------------------------------------
    reg        s_start = 1'b0;
    wire       s_hdr_ready;
    reg  [2:0] s_type = 3'd0, s_src = 3'd0;
    reg        s_prio = 1'b0;
    reg  [5:0] s_len = 6'd0;
    reg  [7:0] s_tag = 8'd0, s_arg = 8'd0;
    reg        s_bvalid = 1'b0;
    reg  [7:0] s_bdata = 8'd0;
    wire       s_bready;
    wire       req_valid, req_credit;
    wire [33:0] req_flit;

    noc_pkt_tx #(.DEPTH(4)) u_req (
        .clk(clk), .rst_n(rst_n),
        .start(s_start), .hdr_ready(s_hdr_ready),
        .ptype(s_type), .dest(MY_ID), .src(s_src), .prio(s_prio),
        .len(s_len), .tag(s_tag), .arg(s_arg),
        .b_valid(s_bvalid), .b_data(s_bdata), .b_ready(s_bready),
        .out_valid(req_valid), .out_flit(req_flit), .out_credit(req_credit));

    // ------------------------------------------------------------------
    // reply collector
    // ------------------------------------------------------------------
    wire        rep_valid, rep_credit;
    wire [33:0] rep_flit;
    wire        r_hv, r_prio, r_bv, r_blast;
    wire [2:0]  r_type, r_dest, r_src;
    wire [5:0]  r_len;
    wire [7:0]  r_tag, r_arg, r_bd;
    reg         r_hready = 1'b0, r_bready = 1'b0;
    reg         rep_block = 1'b0;

    noc_pkt_rx #(.DEPTH(4)) u_rep (
        .clk(clk), .rst_n(rst_n),
        .in_valid(rep_valid), .in_flit(rep_flit), .in_credit(rep_credit),
        .hdr_valid(r_hv), .ptype(r_type), .dest(r_dest), .src(r_src),
        .prio(r_prio), .len(r_len), .tag(r_tag), .arg(r_arg),
        .hdr_ready(r_hready),
        .b_valid(r_bv), .b_data(r_bd), .b_last(r_blast), .b_ready(r_bready));

    integer   nrp = 0;                 // complete replies received
    reg [2:0] rp_type [0:63];
    reg [2:0] rp_dest [0:63];
    reg [2:0] rp_src  [0:63];
    reg       rp_prio [0:63];
    reg [5:0] rp_len  [0:63];
    reg [7:0] rp_tag  [0:63];
    reg [7:0] rp_arg  [0:63];
    reg [7:0] rp_b    [0:64*64-1];
    reg       rx_in_body = 1'b0;
    integer   rbi = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            if (!rx_in_body) begin
                if (r_hv && r_hready) begin
                    rp_type[nrp] = r_type;  rp_dest[nrp] = r_dest; rp_src[nrp] = r_src;
                    rp_prio[nrp] = r_prio;  rp_len[nrp]  = r_len;  rp_tag[nrp] = r_tag;
                    rp_arg[nrp]  = r_arg;
                    rbi = 0;
                    if (r_len == 6'd0) nrp = nrp + 1;
                    else               rx_in_body = 1'b1;
                end
            end else if (r_bv && r_bready) begin
                rp_b[nrp*64 + rbi] = r_bd;
                rbi = rbi + 1;
                if (r_blast) begin
                    rx_in_body = 1'b0;
                    nrp = nrp + 1;
                end
            end
            r_hready <= !rep_block && (($random(rseed) & 3) != 0);
            r_bready <= !rep_block && (($random(rseed) & 3) != 0);
        end
    end

    // ------------------------------------------------------------------
    // time base (us_tick every 100 clocks)
    // ------------------------------------------------------------------
    wire [31:0] time_us;
    wire        us_tick, ms_tick;
    ssc2_timebase #(.CLK_HZ(100_000_000), .US_PER_MS(1000)) u_tb (
        .clk(clk), .rst_n(rst_n), .time_us(time_us), .us_tick(us_tick), .ms_tick(ms_tick));

    // ------------------------------------------------------------------
    // the DUT
    // ------------------------------------------------------------------
    reg         stall_gen = 1'b0;     // random "APB transfers"
    reg         cpu_stall = 1'b0;     // the CPU's own I2C accesses (test 12)
    wire        stall = stall_gen | cpu_stall;
    reg         en    = 1'b1;
    reg  [15:0] timeout_us = 16'd1000;
    wire        ni_tx_push, ni_rx_pop, ni_cmd_valid, ni_cmd_read, ni_cmd_stop;
    wire        ni_abort, ni_active;
    wire [7:0]  ni_tx_data, ni_cmd_len;
    wire [6:0]  ni_addr7;
    wire [15:0] pkts_in, pkts_out;
    wire        i_tx_full, i_rx_empty, i_busy, i_holding, i_nack, i_arb, i_ev_done;
    wire [7:0]  i2c_rx_data;

    ni_i2c dut (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(en), .my_id(MY_ID),
        .in_valid(req_valid), .in_flit(req_flit), .in_credit(req_credit),
        .out_valid(rep_valid), .out_flit(rep_flit), .out_credit(rep_credit),
        .pkts_in(pkts_in), .pkts_out(pkts_out),
        .tx_push(ni_tx_push), .tx_data(ni_tx_data), .tx_full(i_tx_full),
        .rx_pop(ni_rx_pop), .rx_data(i2c_rx_data), .rx_empty(i_rx_empty),
        .cmd_valid(ni_cmd_valid), .cmd_len(ni_cmd_len), .cmd_read(ni_cmd_read),
        .cmd_stop(ni_cmd_stop), .abort(ni_abort),
        .busy(i_busy), .holding(i_holding), .nack_flag(i_nack), .arb_flag(i_arb),
        .ev_done(i_ev_done),
        .timeout_us(timeout_us), .us_tick(us_tick),
        .active(ni_active), .addr7(ni_addr7));

    // ------------------------------------------------------------------
    // engine sharing, as in ssc2_core.v
    // ------------------------------------------------------------------
    reg         cpu_push = 1'b0, cpu_pop = 1'b0, cpu_cmd = 1'b0, cpu_abort = 1'b0;
    reg  [7:0]  cpu_data = 8'd0, cpu_len = 8'd0;
    reg         cpu_read = 1'b0, cpu_stop = 1'b0;
    reg         cpu_en = 1'b0, cpu_auto = 1'b1, cpu_ten = 1'b1;
    reg  [9:0]  cpu_addr = 10'h2A5;
    reg  [15:0] presc = 16'd24;           // 1 MHz SCL

    wire        i_tx_push   = cpu_push | ni_tx_push;
    wire [7:0]  i_tx_data   = cpu_push ? cpu_data : ni_tx_data;
    wire        i_rx_pop    = cpu_pop  | ni_rx_pop;
    wire        i_cmd_valid = cpu_cmd  | ni_cmd_valid;
    wire [7:0]  i_cmd_len   = cpu_cmd ? cpu_len  : ni_cmd_len;
    wire        i_cmd_read  = cpu_cmd ? cpu_read : ni_cmd_read;
    wire        i_cmd_stop  = cpu_cmd ? cpu_stop : ni_cmd_stop;
    wire        i_abort     = cpu_abort | ni_abort;
    wire        i_en        = ni_active | cpu_en;
    wire        i_auto      = ni_active ? 1'b0 : cpu_auto;
    wire [9:0]  i_addr      = ni_active ? {3'd0, ni_addr7} : cpu_addr;
    wire        i_ten       = ni_active ? 1'b0 : cpu_ten;

    wire        scl, sda, i_scl_oe, i_sda_oe;
    wire        i_tx_empty, i_bus_busy, i_ev_tx_ovf;
    wire [4:0]  i_rx_count;
    reg         rogue_scl = 1'b0;         // a broken device holding SCL low

    pullup (scl);
    pullup (sda);
    assign scl = i_scl_oe  ? 1'b0 : 1'bz;
    assign sda = i_sda_oe  ? 1'b0 : 1'bz;
    assign scl = rogue_scl ? 1'b0 : 1'bz;

    ssc_i2c u_i2c (
        .clk(clk), .rst_n(rst_n),
        .en(i_en), .auto_wr(i_auto), .prescale(presc),
        .addr(i_addr), .ten_bit(i_ten),
        .cmd_valid(i_cmd_valid), .cmd_len(i_cmd_len), .cmd_read(i_cmd_read),
        .cmd_stop(i_cmd_stop), .cmd_stop_only(1'b0),
        .tx_flush(1'b0), .rx_flush(1'b0), .abort(i_abort),
        .tx_push(i_tx_push), .tx_data(i_tx_data),
        .rx_pop(i_rx_pop), .rx_data(i2c_rx_data),
        .tx_empty(i_tx_empty), .tx_full(i_tx_full),
        .rx_empty(i_rx_empty), .rx_full(),
        .tx_count(), .rx_count(i_rx_count),
        .busy(i_busy), .holding(i_holding), .bus_busy(i_bus_busy),
        .nack_flag(i_nack), .arb_flag(i_arb),
        .scl_state(), .sda_state(),
        .ev_done(i_ev_done), .ev_nack(), .ev_arb_lost(),
        .ev_tx_ovf(i_ev_tx_ovf),
        .scl_in(scl), .scl_oe(i_scl_oe),
        .sda_in(sda), .sda_oe(i_sda_oe));

    tb_i2c_slave   #(.ADDR(10'h04B), .TEN(0)) tmp2 (.scl(scl), .sda(sda));
    tb_i2c_monitor mon (.scl(scl), .sda(sda));
    tb_i2c_master  m2  (.scl(scl), .sda(sda));

    // ------------------------------------------------------------------
    // stall generator: 2-clock "APB transfers" (light or heavy), and in
    // heavy mode sometimes a long burst of back-to-back transfers
    // ------------------------------------------------------------------
    integer stall_mode = 0;           // 0 off, 1 light, 2 heavy
    integer st_left = 0;
    always @(posedge clk) begin
        if (st_left != 0) begin
            stall_gen <= 1'b1;
            st_left   <= st_left - 1;
        end else begin
            stall_gen <= 1'b0;
            if (stall_mode == 1 && ($random(sseed) & 7) == 0)  st_left <= 2;
            if (stall_mode == 2) begin
                if (($random(sseed) & 31) == 0)    st_left <= 30;
                else if (($random(sseed) & 1) == 0) st_left <= 2;
            end
        end
    end

    // ------------------------------------------------------------------
    // monitors
    // ------------------------------------------------------------------
    integer stall_viol = 0, stall_busy_cycles = 0, cmd_busy = 0;
    integer push_cnt = 0, cmd_cnt = 0, abort_cnt = 0, act_rises = 0;
    integer ovf_bad = 0, rep_early = 0, max_rx = 0, nr_at_done = 0;
    reg     act_q = 1'b0;
    realtime cmd_t = 0.0, abort_t = 0.0;
    always @(posedge clk) if (rst_n) begin
        if (stall && (ni_tx_push || ni_rx_pop || ni_cmd_valid || ni_abort))
            stall_viol = stall_viol + 1;
        if (stall && ni_active) stall_busy_cycles = stall_busy_cycles + 1;
        if (ni_cmd_valid && i_busy) cmd_busy = cmd_busy + 1;
        if (ni_tx_push) push_cnt = push_cnt + 1;
        if (ni_cmd_valid) begin
            cmd_cnt = cmd_cnt + 1;
            cmd_t   = $realtime;
        end
        if (ni_abort) begin
            abort_cnt = abort_cnt + 1;
            abort_t   = $realtime;
        end
        if (ni_active && !act_q) act_rises = act_rises + 1;
        act_q = ni_active;
        if (i_ev_tx_ovf) ovf_bad = ovf_bad + 1;
        if (ni_active && i_rx_count > max_rx) max_rx = i_rx_count;
        if (i_ev_done && dut.st == 4'd5 && dut.issued) nr_at_done = dut.nr;
        // a reply head flit may only leave once the engine is idle and free
        if (rep_valid && rep_flit[33:32] != 2'b01 && rep_flit[33:32] != 2'b10 &&
            (ni_active || i_busy || i_holding))
            rep_early = rep_early + 1;
    end

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------
    reg [7:0] pay [0:63];             // payload of the next request
    reg [7:0] exp [0:63];             // expected reply payload
    integer   exp_ev [0:127];         // expected bus log
    integer   n_exp = 0;
    integer   sent = 0;

    task send_pkt;
        input [2:0] ty;
        input [2:0] src;
        input       pr;
        input [5:0] len;
        input [7:0] tag;
        input [7:0] arg;
        integer k;
        reg     taken;
        begin
            @(negedge clk);
            while (!s_hdr_ready) @(negedge clk);
            s_type = ty; s_src = src; s_prio = pr; s_len = len; s_tag = tag; s_arg = arg;
            s_start = 1'b1;
            @(negedge clk);
            s_start = 1'b0;
            k = 0;
            while (k < len) begin
                s_bdata  = pay[k];
                s_bvalid = (($random(seed) & 3) != 0);
                taken    = s_bvalid && s_bready;
                @(negedge clk);
                if (taken) k = k + 1;
            end
            s_bvalid = 1'b0;
            sent = sent + 1;
        end
    endtask

    // XFER_REQ [wlen][rlen][w0 w1 w2]
    task xfer;
        input [2:0] src;
        input [7:0] tag;
        input [7:0] addr;
        input [7:0] wl;
        input [7:0] rl;
        input [7:0] w0;
        input [7:0] w1;
        input [7:0] w2;
        begin
            pay[0] = wl; pay[1] = rl; pay[2] = w0; pay[3] = w1; pay[4] = w2;
            send_pkt(T_XFER, src, 1'b0, wl[5:0] + 6'd2, tag, addr);
        end
    endtask

    // the CPU's 2-clock APB transfers (strobe in the access phase)
    task cpu_tx_push(input [7:0] d);
        begin
            @(negedge clk); cpu_stall = 1'b1;
            @(negedge clk); cpu_push  = 1'b1; cpu_data = d;
            @(negedge clk); cpu_push  = 1'b0; cpu_stall = 1'b0;
        end
    endtask
    task cpu_command(input [7:0] len, input rd, input stp);
        begin
            @(negedge clk); cpu_stall = 1'b1;
            @(negedge clk); cpu_cmd   = 1'b1; cpu_len = len; cpu_read = rd; cpu_stop = stp;
            @(negedge clk); cpu_cmd   = 1'b0; cpu_stall = 1'b0;
        end
    endtask

    task wait_replies(input integer n);
        integer t;
        begin
            t = 0;
            while (nrp < n && t < 400000) begin @(posedge clk); t = t + 1; end
            if (nrp < n) $display("  (time-out waiting for reply %0d)", n);
        end
    endtask

    // NI idle, its input buffer empty, every credit back, engine and bus idle
    task wait_idle;
        integer t, q;
        begin
            t = 0; q = 0;
            while (q < 20 && t < 400000) begin
                @(posedge clk); t = t + 1;
                if (dut.st == 4'd0 && dut.u_rx.count == 0 && u_req.cred == 4 &&
                    s_hdr_ready && !ni_active && !i_busy && !i_bus_busy) q = q + 1;
                else q = 0;
            end
            if (q < 20) $display("  (time-out waiting for the NI to go idle)");
        end
    endtask

    task check_reply;
        input integer     i;
        input [2:0]       dest;
        input [7:0]       tag;
        input [7:0]       arg;
        input [5:0]       len;
        input [8*72-1:0]  what;
        integer k;
        reg     ok;
        begin
            ok = (i < nrp) && (rp_type[i] == T_RESP) && (rp_dest[i] == dest) &&
                 (rp_src[i] == MY_ID) && (rp_tag[i] == tag) && (rp_arg[i] == arg) &&
                 (rp_prio[i] == 1'b0) && (rp_len[i] == len);
            for (k = 0; k < len; k = k + 1)
                if (rp_b[i*64 + k] !== exp[k]) ok = 1'b0;
            if (!ok && i < nrp) begin
                $write("  reply %0d: type %0d dest %0d src %0d tag %h arg %h prio %0d len %0d:",
                       i, rp_type[i], rp_dest[i], rp_src[i], rp_tag[i], rp_arg[i],
                       rp_prio[i], rp_len[i]);
                for (k = 0; k < rp_len[i]; k = k + 1) $write(" %h", rp_b[i*64 + k]);
                $write("\n");
            end
            check(ok, what);
        end
    endtask

    task show_reply(input integer i);
        integer k;
        begin
            $write("    reply: tag %h, dest %0d, %0d bytes:", rp_tag[i], rp_dest[i], rp_len[i]);
            for (k = 0; k < rp_len[i] && k < 12; k = k + 1) $write(" %h", rp_b[i*64 + k]);
            if (rp_len[i] > 12) $write(" ...");
            $write("\n");
        end
    endtask

    task expect_bus(input [8*72-1:0] what);
        integer e;
        reg     ok;
        begin
            ok = (mon.nev == n_exp);
            for (e = 0; e < n_exp; e = e + 1)
                if (mon.ev[e] != exp_ev[e]) ok = 1'b0;
            mon.show;
            check(ok, what);
        end
    endtask

    // the TMP2 register file seen by the master (pointer wraps at 16)
    function [7:0] tmp2_reg(input integer k);
        tmp2_reg = tmp2.regs[k % 16];
    endfunction

    // ------------------------------------------------------------------
    // test sequence
    // ------------------------------------------------------------------
    integer i, k, n0, a0, c0, p0, d0;
    realtime t0;
    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            pay[i] = 8'd0;
            exp[i] = 8'd0;
        end
        #33 rst_n = 1'b1;
        stall_mode = 1;
        repeat (5) @(posedge clk);

        // ---- 1. TMP2 temperature read (slides 16-18) ----
        $display("[1] XFER_REQ 0x4B [01][02][00] -> XFER_RESP [00][0C 80] (repeated START)");
        mon.clear; n0 = nrp;
        xfer(3'd1, 8'h21, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        show_reply(n0);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0, 3'd1, 8'h21, 8'h4B, 6'd3, "1: reply [00][0C 80] to node 1, tag 21, arg 4B");
        n_exp = 8;
        exp_ev[0] = 1000; exp_ev[1] = 'h096; exp_ev[2] = 'h000; exp_ev[3] = 1001;
        exp_ev[4] = 'h097; exp_ev[5] = 'h00C; exp_ev[6] = 'h180; exp_ev[7] = 1002;
        expect_bus("1: bus S 96 A 00 A Sr 97 A 0C A 80 N P");
        check(!ni_active && !i_holding && !i_bus_busy, "1: engine handed back, bus free");

        // ---- 2. write only ----
        $display("[2] write only: [03][00][02 AA BB] -> [00]");
        mon.clear; n0 = nrp;
        xfer(3'd3, 8'h22, 8'h4B, 8'd3, 8'd0, 8'h02, 8'hAA, 8'hBB);
        wait_replies(n0 + 1);
        exp[0] = 8'h00;
        check_reply(n0, 3'd3, 8'h22, 8'h4B, 6'd1, "2: reply [00]");
        n_exp = 6;
        exp_ev[0] = 1000; exp_ev[1] = 'h096; exp_ev[2] = 'h002; exp_ev[3] = 'h0AA;
        exp_ev[4] = 'h0BB; exp_ev[5] = 1002;
        expect_bus("2: bus S 96 A 02 A AA A BB A P");
        check(tmp2.regs[2] == 8'hAA && tmp2.regs[3] == 8'hBB, "2: TMP2 registers 2, 3 written");

        // ---- 3. write then read back ----
        $display("[3] write-then-read: [01][02][02] -> [00][AA BB]");
        mon.clear; n0 = nrp;
        xfer(3'd3, 8'h23, 8'h4B, 8'd1, 8'd2, 8'h02, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00; exp[1] = 8'hAA; exp[2] = 8'hBB;
        check_reply(n0, 3'd3, 8'h23, 8'h4B, 6'd3, "3: reply [00][AA BB]");

        // ---- 4. read only ----
        $display("[4] read only: [00][04] -> [00][BB 44 45 46] (pointer was 3)");
        mon.clear; n0 = nrp; c0 = cmd_cnt;
        xfer(3'd3, 8'h24, 8'h4B, 8'd0, 8'd4, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00; exp[1] = 8'hBB; exp[2] = 8'h44; exp[3] = 8'h45; exp[4] = 8'h46;
        check_reply(n0, 3'd3, 8'h24, 8'h4B, 6'd5, "4: reply [00][BB 44 45 46]");
        n_exp = 7;
        exp_ev[0] = 1000; exp_ev[1] = 'h097; exp_ev[2] = 'h0BB; exp_ev[3] = 'h044;
        exp_ev[4] = 'h045; exp_ev[5] = 'h146; exp_ev[6] = 1002;
        expect_bus("4: bus S 97 A BB A 44 A 45 A 46 N P (no write phase)");
        check(cmd_cnt - c0 == 1, "4: a single (read) command");

        // ---- 5. long read ----
        $display("[5] long read: [01][60][00] -> 61-byte reply, heavy stall");
        stall_mode = 2;
        mon.clear; n0 = nrp; max_rx = 0;
        xfer(3'd0, 8'h25, 8'h4B, 8'd1, 8'd60, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        stall_mode = 1;
        show_reply(n0);
        exp[0] = 8'h00;
        for (i = 0; i < 60; i = i + 1) exp[1 + i] = tmp2_reg(i);
        check_reply(n0, 3'd0, 8'h25, 8'h4B, 6'd61, "5: 61-byte reply = [00] + TMP2 registers 0..59 mod 16");
        check(mon.nev == 3 + 1 + 1 + 60 + 1, "5: one transaction: S, 00, Sr, 97, 60 bytes, P");
        $display("    most bytes waiting in the RX FIFO: %0d", max_rx);
        check(max_rx <= 2, "5: bytes popped as they arrive (RX FIFO never filled)");
        // 5b: the CPU keeps the bus (stall) for 300 us in the middle of a read
        $display("    5b: read 24 bytes, stall held for 300 us after byte 2: RX FIFO fills");
        mon.clear; n0 = nrp; max_rx = 0;
        fork
            xfer(3'd0, 8'h2B, 8'h4B, 8'd1, 8'd24, 8'h00, 8'h00, 8'h00);
            begin
                wait (dut.st == 4'd5 && dut.issued && dut.nr == 2);
                @(negedge clk); cpu_stall = 1'b1;
                #300000;
                @(negedge clk); cpu_stall = 1'b0;
            end
        join
        wait_replies(n0 + 1);
        exp[0] = 8'h00;
        for (i = 0; i < 24; i = i + 1) exp[1 + i] = tmp2_reg(i);
        check_reply(n0, 3'd0, 8'h2B, 8'h4B, 6'd25, "5b: 25-byte reply correct after the long stall");
        check(max_rx == 16 && mon.nev == 3 + 1 + 1 + 24 + 1,
              "5b: RX FIFO full (16), engine waited, still one transaction");
        // 5c: the stall covers the end of the read: ev_done comes first
        $display("    5c: read 4 bytes, stall from byte 1 until after ev_done");
        n0 = nrp; nr_at_done = 99;
        fork
            xfer(3'd0, 8'h2C, 8'h4B, 8'd1, 8'd4, 8'h04, 8'h00, 8'h00);
            begin
                wait (dut.st == 4'd5 && dut.issued && dut.nr == 1);
                @(negedge clk); cpu_stall = 1'b1;
                @(posedge i_ev_done);
                #10000;
                @(negedge clk); cpu_stall = 1'b0;
            end
        join
        wait_replies(n0 + 1);
        exp[0] = 8'h00; exp[1] = 8'h44; exp[2] = 8'h45; exp[3] = 8'h46; exp[4] = 8'h47;
        check_reply(n0, 3'd0, 8'h2C, 8'h4B, 6'd5, "5c: reply [00][44 45 46 47]");
        check(nr_at_done == 1, "5c: ev_done came with 3 bytes still in the RX FIFO");

        // ---- 6. address probe ----
        $display("[6] address probe (wlen = rlen = 0)");
        mon.clear; n0 = nrp;
        xfer(3'd1, 8'h26, 8'h4B, 8'd0, 8'd0, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00;
        check_reply(n0, 3'd1, 8'h26, 8'h4B, 6'd1, "6: device 0x4B answers -> [00]");
        n_exp = 3;
        exp_ev[0] = 1000; exp_ev[1] = 'h096; exp_ev[2] = 1002;
        expect_bus("6: bus S 96 A P");
        mon.clear;
        xfer(3'd1, 8'h27, 8'h4C, 8'd0, 8'd0, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 2);
        exp[0] = 8'h01;
        check_reply(n0 + 1, 3'd1, 8'h27, 8'h4C, 6'd1, "6: no device at 0x4C -> [01]");

        // ---- 7. NACK ----
        $display("[7] NACK: wrong address 0x4C");
        wait_idle;
        mon.clear; n0 = nrp; a0 = abort_cnt;
        xfer(3'd1, 8'h31, 8'h4C, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h01;
        check_reply(n0, 3'd1, 8'h31, 8'h4C, 6'd1, "7: write-then-read to 0x4C -> [01], no data");
        n_exp = 3;
        exp_ev[0] = 1000; exp_ev[1] = 'h198; exp_ev[2] = 1002;
        expect_bus("7: bus S 98 N P (no read after the NACK)");
        check(abort_cnt - a0 == 1 && i_tx_empty, "7: one abort pulse flushed the unsent byte");
        mon.clear;
        xfer(3'd1, 8'h32, 8'h4C, 8'd0, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 2);
        check_reply(n0 + 1, 3'd1, 8'h32, 8'h4C, 6'd1, "7: read-only to 0x4C -> [01]");
        n_exp = 3;
        exp_ev[0] = 1000; exp_ev[1] = 'h199; exp_ev[2] = 1002;
        expect_bus("7: bus S 99 N P");
        xfer(3'd1, 8'h33, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 3);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0 + 2, 3'd1, 8'h33, 8'h4B, 6'd3, "7: next TMP2 read is correct");

        // ---- 8. arbitration lost ----
        $display("[8] arbitration: a second master starts at the same moment (100 kHz)");
        wait_idle;
        presc = 16'd249;
        mon.clear; n0 = nrp; a0 = abort_cnt; k = m2.done_count;
        pay[0] = 8'd1; pay[1] = 8'd2; pay[2] = 8'h00;
        fork
            m2.join_start(8'h20);                         // other master: address 0x10
            send_pkt(T_XFER, 3'd1, 1'b0, 6'd3, 8'h41, 8'h4B);
        join
        wait_replies(n0 + 1);
        mon.show;
        exp[0] = 8'h02;
        check_reply(n0, 3'd1, 8'h41, 8'h4B, 6'd1, "8: arbitration lost -> [02]");
        check(m2.won === 1'b1 && m2.done_count == k + 1, "8: other master finished undisturbed");
        check(abort_cnt - a0 == 1 && i_tx_empty, "8: one abort pulse, unsent byte flushed");
        wait_idle;
        presc = 16'd24;
        xfer(3'd1, 8'h42, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 2);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0 + 1, 3'd1, 8'h42, 8'h4B, 6'd3, "8: next TMP2 read is correct");

        // ---- 9. time-out ----
        $display("[9] time-out: a device holds SCL low during the address byte, timeout 50 us");
        wait_idle;
        timeout_us = 16'd50;
        mon.clear; n0 = nrp; a0 = abort_cnt;
        pay[0] = 8'd1; pay[1] = 8'd2; pay[2] = 8'h00;
        fork
            send_pkt(T_XFER, 3'd1, 1'b0, 6'd3, 8'h51, 8'h4B);
            begin
                wait (mon.nev >= 1);                      // our START
                @(negedge scl); @(negedge scl); @(negedge scl);
                rogue_scl = 1'b1;
            end
        join
        wait_replies(n0 + 1);
        exp[0] = 8'h03;
        check_reply(n0, 3'd1, 8'h51, 8'h4B, 6'd1, "9: time-out -> [03]");
        $display("    command to abort: %0.1f us", (abort_t - cmd_t) / 1000.0);
        check(abort_cnt - a0 == 1, "9: exactly one abort pulse");
        check((abort_t - cmd_t) >= 48000.0 && (abort_t - cmd_t) <= 51000.0,
              "9: abort about timeout_us after the command");
        check(!i_scl_oe && !i_sda_oe && !ni_active, "9: engine let go of SCL/SDA, NI handed it back");
        #100000;
        rogue_scl = 1'b0;                                 // the device recovers
        timeout_us = 16'd1000;
        wait_idle;
        mon.in_xfer = 1'b0;                               // (no STOP was seen)
        mon.clear;
        xfer(3'd1, 8'h52, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 2);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0 + 1, 3'd1, 8'h52, 8'h4B, 6'd3, "9: next request works: [00][0C 80]");
        n_exp = 8;
        exp_ev[0] = 1000; exp_ev[1] = 'h096; exp_ev[2] = 'h000; exp_ev[3] = 1001;
        exp_ev[4] = 'h097; exp_ev[5] = 'h00C; exp_ev[6] = 'h180; exp_ev[7] = 1002;
        expect_bus("9: bus S 96 A 00 A Sr 97 A 0C A 80 N P");
        $display("    timeout_us = 0: times out at once");
        timeout_us = 16'd0;
        mon.clear; a0 = abort_cnt; c0 = cmd_cnt;
        xfer(3'd1, 8'h53, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 3);
        exp[0] = 8'h03;
        check_reply(n0 + 2, 3'd1, 8'h53, 8'h4B, 6'd1, "9: timeout_us = 0 -> [03]");
        check(abort_cnt - a0 == 1 && cmd_cnt == c0 && mon.nev == 0 && i_tx_empty,
              "9: no command, bus untouched, TX FIFO clean");
        timeout_us = 16'd1000;

        // ---- 10. bad requests ----
        $display("[10] bad requests -> [04], engine untouched");
        wait_idle;
        mon.clear; n0 = nrp; p0 = push_cnt; a0 = act_rises; c0 = cmd_cnt;
        pay[0] = 8'd17; pay[1] = 8'd0;
        for (i = 0; i < 17; i = i + 1) pay[2 + i] = i;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd19, 8'h61, 8'h4B);          // wlen 17
        pay[0] = 8'd1; pay[1] = 8'd61; pay[2] = 8'h00;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'h62, 8'h4B);           // rlen 61
        pay[0] = 8'd2; pay[1] = 8'd1; pay[2] = 8'h00;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'h63, 8'h4B);           // wlen 2, has 1
        pay[0] = 8'd0; pay[1] = 8'd2; pay[2] = 8'h00;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'h64, 8'h4B);           // extra byte
        pay[0] = 8'd1;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd1, 8'h65, 8'h4B);           // payload of 1
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd0, 8'h66, 8'h4B);           // no payload
        wait_replies(n0 + 6);
        exp[0] = 8'h04;
        check_reply(n0 + 0, 3'd3, 8'h61, 8'h4B, 6'd1, "10: wlen 17 -> [04]");
        check_reply(n0 + 1, 3'd3, 8'h62, 8'h4B, 6'd1, "10: rlen 61 -> [04]");
        check_reply(n0 + 2, 3'd3, 8'h63, 8'h4B, 6'd1, "10: wlen bigger than the payload -> [04]");
        check_reply(n0 + 3, 3'd3, 8'h64, 8'h4B, 6'd1, "10: payload longer than 2 + wlen -> [04]");
        check_reply(n0 + 4, 3'd3, 8'h65, 8'h4B, 6'd1, "10: payload of 1 -> [04]");
        check_reply(n0 + 5, 3'd3, 8'h66, 8'h4B, 6'd1, "10: empty payload -> [04]");
        check(mon.nev == 0 && push_cnt == p0 && act_rises == a0 && cmd_cnt == c0,
              "10: bus, FIFOs and engine untouched");

        // ---- 11. DATA ----
        $display("[11] DATA: 5-byte write, 20-byte write, DATA with NACK");
        mon.clear; n0 = nrp;
        pay[0] = 8'h04; pay[1] = 8'h11; pay[2] = 8'h22; pay[3] = 8'h33; pay[4] = 8'h44;
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd5, 8'h71, 8'h4B);
        wait_idle;
        check(tmp2.regs[4] == 8'h11 && tmp2.regs[5] == 8'h22 && tmp2.regs[6] == 8'h33 &&
              tmp2.regs[7] == 8'h44, "11: 5-byte DATA wrote TMP2 registers 4..7");
        n_exp = 8;
        exp_ev[0] = 1000; exp_ev[1] = 'h096; exp_ev[2] = 'h004; exp_ev[3] = 'h011;
        exp_ev[4] = 'h022; exp_ev[5] = 'h033; exp_ev[6] = 'h044; exp_ev[7] = 1002;
        expect_bus("11: bus S 96 A 04 A 11 A 22 A 33 A 44 A P");
        check(nrp == n0, "11: DATA gets no reply");
        // more than 16 bytes: still one transaction
        mon.clear; d0 = tmp2.nwlog; c0 = cmd_cnt;
        stall_mode = 2;
        pay[0] = 8'h08;
        for (i = 1; i < 20; i = i + 1) pay[i] = $random(seed);
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd20, 8'h72, 8'h4B);
        wait_idle;
        stall_mode = 1;
        k = 1;
        for (i = 0; i < 20; i = i + 1) if (tmp2.wlog[d0 + i] !== pay[i]) k = 0;
        check(tmp2.nwlog - d0 == 20 && k == 1, "11: 20-byte DATA: all 20 bytes written in order");
        k = 1;
        if (mon.nev != 23 || mon.ev[0] != 1000 || mon.ev[1] != 'h096 || mon.ev[22] != 1002) k = 0;
        for (i = 0; i < 20; i = i + 1) if (mon.ev[2 + i] != pay[i]) k = 0;
        check(k == 1 && cmd_cnt - c0 == 1, "11: 20-byte DATA = ONE transaction (1 START, 1 STOP)");
        // a DATA packet to a missing device
        mon.clear; a0 = abort_cnt;
        for (i = 0; i < 20; i = i + 1) pay[i] = 8'hE0 + i;
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd20, 8'h73, 8'h4C);
        wait_idle;
        n_exp = 3;
        exp_ev[0] = 1000; exp_ev[1] = 'h198; exp_ev[2] = 1002;
        expect_bus("11: DATA to 0x4C: bus S 98 N P");
        check(nrp == n0 && abort_cnt - a0 == 1 && i_tx_empty,
              "11: no reply, one abort, queued bytes flushed");
        // put the TMP2 registers back
        for (i = 0; i < 16; i = i + 1) tmp2.regs[i] = 8'h40 + i;
        tmp2.regs[0] = 8'h0C; tmp2.regs[1] = 8'h80; tmp2.regs[2] = 8'h00;
        tmp2.regs[3] = 8'h00; tmp2.regs[11] = 8'hCB;

        // ---- 12. the CPU left old bytes in the RX FIFO ----
        $display("[12] CPU reads 2 bytes (pointer 4) itself and leaves them in the RX FIFO");
        cpu_en = 1'b1; cpu_auto = 1'b0; cpu_ten = 1'b0; cpu_addr = 10'h04B;
        cpu_tx_push(8'h04);
        cpu_command(8'd1, 1'b0, 1'b0);                    // write pointer, keep the bus
        i = 0;
        while (!(i_holding && !i_busy) && i < 100000) begin @(posedge clk); i = i + 1; end
        cpu_command(8'd2, 1'b1, 1'b1);                    // read 2 + STOP
        i = 0;
        while ((i_busy || i_holding || i_rx_count != 2) && i < 100000) begin
            @(posedge clk); i = i + 1;
        end
        cpu_en = 1'b0; cpu_auto = 1'b1; cpu_ten = 1'b1; cpu_addr = 10'h2A5;
        check(i_rx_count == 2 && i2c_rx_data == 8'h44, "12: CPU path works, 2 old bytes (44 45) left");
        n0 = nrp;
        xfer(3'd1, 8'h81, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0, 3'd1, 8'h81, 8'h4B, 6'd3, "12: old bytes thrown away, reply [00][0C 80]");
        check(i_rx_empty, "12: RX FIFO empty afterwards");
        // the CPU is in the middle of its own write when NI requests arrive:
        // the NI must wait until the engine is no longer busy
        $display("    CPU writes 3 bytes; a probe and a TMP2 read arrive meanwhile");
        wait_idle;
        cpu_en = 1'b1; cpu_auto = 1'b0; cpu_ten = 1'b0; cpu_addr = 10'h04B;
        mon.clear; n0 = nrp;
        cpu_tx_push(8'h0C); cpu_tx_push(8'h4C); cpu_tx_push(8'h4D);   // same values
        cpu_command(8'd3, 1'b0, 1'b1);
        xfer(3'd1, 8'h82, 8'h4B, 8'd0, 8'd0, 8'h00, 8'h00, 8'h00);   // probe
        xfer(3'd1, 8'h83, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);   // TMP2 read
        wait_replies(n0 + 2);
        cpu_en = 1'b0; cpu_auto = 1'b1; cpu_ten = 1'b1; cpu_addr = 10'h2A5;
        exp[0] = 8'h00;
        check_reply(n0, 3'd1, 8'h82, 8'h4B, 6'd1, "12: probe after the CPU's write -> [00]");
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0 + 1, 3'd1, 8'h83, 8'h4B, 6'd3, "12: TMP2 read after it -> [00][0C 80]");
        n_exp = 17;
        exp_ev[0]  = 1000;  exp_ev[1]  = 'h096; exp_ev[2]  = 'h00C; exp_ev[3]  = 'h04C;
        exp_ev[4]  = 'h04D; exp_ev[5]  = 1002;  exp_ev[6]  = 1000;  exp_ev[7]  = 'h096;
        exp_ev[8]  = 1002;  exp_ev[9]  = 1000;  exp_ev[10] = 'h096; exp_ev[11] = 'h000;
        exp_ev[12] = 1001;  exp_ev[13] = 'h097; exp_ev[14] = 'h00C; exp_ev[15] = 'h180;
        exp_ev[16] = 1002;
        expect_bus("12: CPU write, then NI probe, then NI read - in that order");
        // a CPU command that waits for a byte the CPU never pushes keeps the
        // engine busy: the NI must not feed it its own byte; it times out
        $display("    CPU write of 2 bytes with only 1 pushed (engine stuck), then a TMP2 read");
        wait_idle;
        cpu_en = 1'b1; cpu_auto = 1'b0; cpu_ten = 1'b0; cpu_addr = 10'h04B;
        timeout_us = 16'd100;
        mon.clear; n0 = nrp; a0 = abort_cnt; d0 = tmp2.nwlog;
        cpu_tx_push(8'h0C);
        cpu_command(8'd2, 1'b0, 1'b1);
        xfer(3'd1, 8'h84, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h03;
        check_reply(n0, 3'd1, 8'h84, 8'h4B, 6'd1, "12: engine never free -> time-out [03]");
        check(tmp2.nwlog - d0 == 1 && abort_cnt - a0 == 1 && i_tx_empty,
              "12: the NI's byte was not given to the CPU's command; abort cleaned up");
        cpu_en = 1'b0; cpu_auto = 1'b1; cpu_ten = 1'b1; cpu_addr = 10'h2A5;
        timeout_us = 16'd1000;
        wait_idle;
        mon.in_xfer = 1'b0;                               // (no STOP was seen)
        xfer(3'd1, 8'h85, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 2);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0 + 1, 3'd1, 8'h85, 8'h4B, 6'd3, "12: next request works");

        // ---- 13. other packet types, en = 0 ----
        $display("[13] other packet types and en = 0: consumed, nothing on the bus");
        wait_idle;
        mon.clear; n0 = nrp; p0 = push_cnt; a0 = act_rises; c0 = cmd_cnt;
        for (i = 0; i < 16; i = i + 1) pay[i] = i;
        send_pkt(3'd3, 3'd1, 1'b0, 6'd16, 8'h91, 8'h4B);
        send_pkt(3'd6, 3'd1, 1'b0, 6'd0,  8'h92, 8'h4B);
        send_pkt(T_RESP, 3'd1, 1'b0, 6'd3, 8'h93, 8'h4B);
        wait_idle;
        check(nrp == n0 && mon.nev == 0 && push_cnt == p0 && act_rises == a0,
              "13: RECORD / reserved / XFER_RESP consumed, no reply");
        en = 1'b0;
        pay[0] = 8'd1; pay[1] = 8'd2; pay[2] = 8'h00;
        send_pkt(T_XFER, 3'd1, 1'b0, 6'd3, 8'h94, 8'h4B);
        for (i = 0; i < 30; i = i + 1) pay[i] = 8'h5A;
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd30, 8'h95, 8'h4B);
        send_pkt(3'd5, 3'd1, 1'b0, 6'd7, 8'h96, 8'h4B);
        pay[0] = 8'd1;
        send_pkt(T_XFER, 3'd1, 1'b0, 6'd1, 8'h97, 8'h4B);
        wait_idle;
        check(nrp == n0, "13: no reply while disabled");
        check(mon.nev == 0 && push_cnt == p0 && act_rises == a0 && cmd_cnt == c0,
              "13: engine and bus never touched while disabled");
        check(u_req.cred == 4 && dut.u_rx.count == 0, "13: every flit consumed, credits back");
        en = 1'b1;
        xfer(3'd1, 8'h98, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0, 3'd1, 8'h98, 8'h4B, 6'd3, "13: works again after en = 1");

        // ---- 14. heavy stall, back-to-back requests, reply back-pressure ----
        $display("[14] heavy stall + 3 back-to-back requests + blocked reply path");
        stall_mode = 2;
        stall_busy_cycles = 0;
        n0 = nrp;
        rep_block = 1'b1;
        xfer(3'd1, 8'hA1, 8'h4B, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);
        xfer(3'd3, 8'hA2, 8'h4B, 8'd1, 8'd1, 8'h0B, 8'h00, 8'h00);   // ID register
        xfer(3'd0, 8'hA3, 8'h4C, 8'd1, 8'd2, 8'h00, 8'h00, 8'h00);   // NACK
        repeat (30000) @(posedge clk);
        check(nrp == n0, "14: nothing delivered while the reply path is blocked");
        rep_block = 1'b0;
        wait_replies(n0 + 3);
        exp[0] = 8'h00; exp[1] = 8'h0C; exp[2] = 8'h80;
        check_reply(n0 + 0, 3'd1, 8'hA1, 8'h4B, 6'd3, "14: reply 1 -> node 1 [00][0C 80]");
        exp[0] = 8'h00; exp[1] = 8'hCB;
        check_reply(n0 + 1, 3'd3, 8'hA2, 8'h4B, 6'd2, "14: reply 2 -> node 3 [00][CB] (ID)");
        exp[0] = 8'h01;
        check_reply(n0 + 2, 3'd0, 8'hA3, 8'h4C, 6'd1, "14: reply 3 -> node 0 [01] (NACK)");
        check(stall_busy_cycles > 200, "14: stall really was high during transactions");
        stall_mode = 1;

        // ---- 15. counters and global monitors ----
        wait_idle;
        $display("[15] counters and monitors: pkts_in %0d (sent %0d), pkts_out %0d (replies %0d)",
                 pkts_in, sent, pkts_out, nrp);
        check(pkts_in == sent, "15: pkts_in counts every packet (also dropped ones)");
        check(pkts_out == nrp, "15: pkts_out counts every reply");
        check(stall_viol == 0, "no engine strobe, command or abort while stall = 1");
        check(cmd_busy == 0, "a command is only issued while the engine is not busy");
        check(ovf_bad == 0, "no TX FIFO overflow");
        check(rep_early == 0, "every reply left after the engine was idle and free");

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end

    // watchdog
    initial begin
        #40_000_000;
        $display("FAIL: watchdog - simulation took too long");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule

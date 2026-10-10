// =============================================================================
// tb_ni_uart.v  -  unit test: UART network interface (NoC node 3)
// -----------------------------------------------------------------------------
// Set-up:
//
//   tb_uart_term (PC) <--rxd/txd--> ssc_uart (v1, 8N1) <--FIFOs--> ni_uart
//                                                                  |   ^
//                    TB network:  u_cap (noc_pkt_rx) <-- out link  |   | in link <-- u_inj (noc_pkt_tx)
//
// The UART runs at 100 MHz / 64 = 1.5625 Mbaud (6.4 us per byte) and the
// NI time-out is 20 us, so the test is short.  `stall` is driven randomly
// (25 % of clocks plus short bursts) for the whole run, and a monitor counts
// every push/pop in a stall cycle or while en = 0 (both must stay 0).
//
// Tests:
//   1  slide 17: PC "01 9F 03" -> XFER_REQ to node 4, payload 01 03 9F
//   2  XFER_RESP [00][20 BA 19] -> PC gets exactly 20 BA 19
//      (00 20 BA 19 with resp_status = 1; a status-only NACK gives nothing)
//   3  FRAME corner cases: wlen = 0, wlen = rlen = 60, pauses < time-out
//   4  incomplete frame: dropped timeout_us after its last byte, resync;
//      bytes waiting in the RX FIFO during a long stall are not "silence"
//   5  wlen / rlen > 60: frame dropped (with following bytes), resync
//   6  ADDRESSED FRAME: dest and arg from the frame, bad dest, time-out
//   7  RAW: 16-byte packets at once, short packets after the time-out,
//      network blocked mid-stream (bytes wait, none lost), mode change
//   8  network -> PC: DATA, RECORD, ALARM, XFER_REQ payloads
//   9  reserved types 5..7 consumed and dropped
//  10  header-only packets push nothing
//  11  4 x 63-byte packets with TX FIFO back-pressure: no byte lost
//  12  en = 0: packets drained, UART FIFOs untouched; en = 1 resumes
//  13  both directions at once with random frames and packets
//  14  pkts_in / pkts_out counters, global assertions
// =============================================================================
`timescale 1ns / 1ps
module tb_ni_uart;
    localparam BAUD_INT = 4;        // 16 x 4 = 64 clocks per bit
    localparam TMO      = 20;       // timeout_us used by the test
    localparam MAXP     = 128;      // captured packets
    localparam [2:0] T_DATA = 3'd0, T_XREQ = 3'd1, T_XRESP = 3'd2,
                     T_REC  = 3'd3, T_ALARM = 3'd4;

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

    integer seed_s = 7;     // stall generator
    integer seed_c = 11;    // capture consumer
    integer seed_d = 5;     // test data

    // ---------------- time base ----------------
    wire [31:0] time_us;
    wire        us_tick, ms_tick;
    ssc2_timebase #(.CLK_HZ(100_000_000), .US_PER_MS(1000)) u_time (
        .clk(clk), .rst_n(rst_n), .time_us(time_us), .us_tick(us_tick), .ms_tick(ms_tick));

    // ---------------- configuration ----------------
    reg        en          = 1'b1;
    reg  [1:0] mode        = 2'd1;
    reg  [2:0] cfg_dest    = 3'd4;
    reg        cfg_prio    = 1'b0;
    reg  [7:0] cfg_arg     = 8'h01;
    reg        resp_status = 1'b0;
    reg [15:0] timeout_us  = TMO;
    reg        stall       = 1'b0;
    reg        stall_on    = 1'b0;    // random stall
    reg        stall_force = 1'b0;    // stall held at 1

    // ---------------- UART engine + PC ----------------
    wire       tx_push, rx_pop, tx_full, rx_empty;
    wire [7:0] tx_data, rx_data;
    wire [4:0] rx_count;
    wire       txd, rxd;
    wire       ev_rx_ovf, ev_tx_ovf, ev_frame_err;

    ssc_uart u_uart (
        .clk(clk), .rst_n(rst_n),
        .tx_en(1'b1), .rx_en(1'b1), .data_bits(2'd3), .parity_en(1'b0),
        .parity_odd(1'b0), .stop2(1'b0), .flow_en(1'b0), .loopback(1'b0),
        .baud_int(BAUD_INT[15:0]), .baud_frac(4'd0),
        .tx_flush(1'b0), .rx_flush(1'b0),
        .tx_push(tx_push), .tx_data(tx_data), .rx_pop(rx_pop), .rx_data(rx_data),
        .tx_empty(), .tx_full(tx_full), .rx_empty(rx_empty), .rx_full(),
        .tx_count(), .rx_count(rx_count), .tx_busy(), .rx_busy(), .cts_ok(),
        .ev_parity_err(), .ev_frame_err(ev_frame_err), .ev_break(),
        .ev_rx_ovf(ev_rx_ovf), .ev_tx_ovf(ev_tx_ovf),
        .rxd(rxd), .txd(txd), .cts_n(1'b0), .rts_n());

    tb_uart_term term (.rx(txd), .tx(rxd));

    // ---------------- DUT ----------------
    wire        o_valid, o_credit;      // NI -> TB network
    wire [33:0] o_flit;
    wire        i_valid, i_credit;      // TB network -> NI
    wire [33:0] i_flit;
    wire [15:0] pkts_in, pkts_out;

    ni_uart dut (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(en), .my_id(3'd3),
        .in_valid(i_valid), .in_flit(i_flit), .in_credit(i_credit),
        .out_valid(o_valid), .out_flit(o_flit), .out_credit(o_credit),
        .pkts_in(pkts_in), .pkts_out(pkts_out),
        .mode(mode), .dest(cfg_dest), .prio(cfg_prio), .arg(cfg_arg),
        .resp_status(resp_status), .timeout_us(timeout_us), .us_tick(us_tick),
        .tx_push(tx_push), .tx_data(tx_data), .tx_full(tx_full),
        .rx_pop(rx_pop), .rx_data(rx_data), .rx_empty(rx_empty));

    // ---------------- TB network: capture the NI's packets ----------------
    wire       c_hv, c_prio, c_bv, c_blast;
    wire [2:0] c_type, c_dest, c_src;
    wire [5:0] c_len;
    wire [7:0] c_tag, c_arg, c_bd;
    reg        c_hready = 1'b0;
    reg        c_bready = 1'b0;
    reg        cap_hold = 1'b0;        // 1: the network takes nothing (no credits)

    noc_pkt_rx #(.DEPTH(4)) u_cap (
        .clk(clk), .rst_n(rst_n),
        .in_valid(o_valid), .in_flit(o_flit), .in_credit(o_credit),
        .hdr_valid(c_hv), .ptype(c_type), .dest(c_dest), .src(c_src), .prio(c_prio),
        .len(c_len), .tag(c_tag), .arg(c_arg), .hdr_ready(c_hready),
        .b_valid(c_bv), .b_data(c_bd), .b_last(c_blast), .b_ready(c_bready));

    reg [2:0] cp_type [0:MAXP-1];
    reg [2:0] cp_dest [0:MAXP-1];
    reg [2:0] cp_src  [0:MAXP-1];
    reg       cp_prio [0:MAXP-1];
    reg [5:0] cp_len  [0:MAXP-1];
    reg [7:0] cp_tag  [0:MAXP-1];
    reg [7:0] cp_arg  [0:MAXP-1];
    reg [7:0] cp_pay  [0:MAXP*64-1];
    time      cp_htime [0:MAXP-1];      // time the head flit crossed the link
    integer   cp_cnt = 0;               // complete packets
    integer   cp_idx = 0;
    reg       cp_phase = 1'b0;

    always @(posedge clk) begin
        if (rst_n) begin
            if (!cp_phase) begin
                if (c_hv && c_hready) begin
                    cp_type[cp_cnt] <= c_type;  cp_dest[cp_cnt] <= c_dest;
                    cp_src[cp_cnt]  <= c_src;   cp_prio[cp_cnt] <= c_prio;
                    cp_len[cp_cnt]  <= c_len;   cp_tag[cp_cnt]  <= c_tag;
                    cp_arg[cp_cnt]  <= c_arg;
                    if (c_len == 6'd0) cp_cnt <= cp_cnt + 1;
                    else begin
                        cp_phase <= 1'b1;
                        cp_idx   <= 0;
                    end
                end
            end else if (c_bv && c_bready) begin
                cp_pay[cp_cnt*64 + cp_idx] <= c_bd;
                if (c_blast) begin
                    cp_phase <= 1'b0;
                    cp_cnt   <= cp_cnt + 1;
                end else begin
                    cp_idx <= cp_idx + 1;
                end
            end
            // the network is not always ready: exercises the NI's credits
            c_hready <= !cap_hold && (($random(seed_c) & 3) != 0);
            c_bready <= !cap_hold && (($random(seed_c) & 3) != 0);
        end
    end

    // ---------------- TB network: inject packets into the NI ----------------
    reg        j_start = 1'b0;
    wire       j_hready;
    reg  [2:0] j_type = 3'd0;
    reg  [5:0] j_len  = 6'd0;
    reg  [7:0] j_tag  = 8'd0;
    reg        j_bv   = 1'b0;
    reg  [7:0] j_bd   = 8'd0;
    wire       j_bready;
    reg  [7:0] jpay [0:63];
    integer    n_inj = 0;

    noc_pkt_tx #(.DEPTH(4)) u_inj (
        .clk(clk), .rst_n(rst_n),
        .start(j_start), .hdr_ready(j_hready),
        .ptype(j_type), .dest(3'd3), .src(3'd4), .prio(1'b0),
        .len(j_len), .tag(j_tag), .arg(8'h00),
        .b_valid(j_bv), .b_data(j_bd), .b_ready(j_bready),
        .out_valid(i_valid), .out_flit(i_flit), .out_credit(i_credit));

    // send one packet with payload jpay[0 .. len-1]
    task inject(input [2:0] t, input integer len, input [7:0] tag);
        integer k;
        begin
            @(negedge clk);
            while (!j_hready) @(negedge clk);
            j_type = t; j_len = len; j_tag = tag; j_start = 1'b1;
            @(negedge clk);
            j_start = 1'b0;
            for (k = 0; k < len; k = k + 1) begin
                j_bv = 1'b1; j_bd = jpay[k];
                while (!j_bready) @(negedge clk);
                @(negedge clk);                    // taken at the posedge before
            end
            j_bv = 1'b0;
            n_inj = n_inj + 1;
        end
    endtask

    // ---------------- stall generator ----------------
    integer burst = 0;
    always @(posedge clk) begin
        if (burst > 0) burst = burst - 1;
        else if (($random(seed_s) & 511) == 0) burst = 50;    // a long CPU access
        stall <= stall_force ||
                 (stall_on && ((burst > 0) || (($random(seed_s) & 3) == 0)));
    end

    // ---------------- monitors and assertions ----------------
    integer bad_stall = 0;      // push/pop while stall = 1
    integer bad_en    = 0;      // push/pop while en = 0
    integer bad_full  = 0;      // push into a full TX FIFO (byte lost)
    integer bad_empty = 0;      // pop from an empty RX FIFO
    integer uart_ovf  = 0;      // UART RX / TX overflow events
    integer uart_ferr = 0;      // framing errors at the UART RX
    integer cov_stall = 0;      // stall really held the NI back
    integer cov_bp    = 0;      // NI waited on tx_full
    integer n_push    = 0;
    integer n_pop     = 0;
    integer nh        = 0;      // head flits NI -> network
    integer nh_in     = 0;      // head flits network -> NI
    time    last_pop_t = 0;     // time of the latest rx_pop
    time    tmo_t      = 0;     // time of the latest time-out with an effect

    always @(posedge clk) begin
        if (rst_n) begin
            if (stall && (tx_push || rx_pop)) bad_stall = bad_stall + 1;
            if (!en && (tx_push || rx_pop))   bad_en    = bad_en + 1;
            if (tx_push && tx_full)           bad_full  = bad_full + 1;
            if (rx_pop && rx_empty)           bad_empty = bad_empty + 1;
            if (ev_rx_ovf || ev_tx_ovf)       uart_ovf  = uart_ovf + 1;
            if (ev_frame_err)                 uart_ferr = uart_ferr + 1;
            if (stall && !stall_force && en && ((dut.nb_valid && !dut.n_drop && !tx_full) ||
                                (dut.collecting && !dut.rehome && !rx_empty)))
                cov_stall = cov_stall + 1;
            if (dut.nb_valid && !dut.n_drop && tx_full) cov_bp = cov_bp + 1;
            if (tx_push) n_push = n_push + 1;
            if (rx_pop) begin
                n_pop = n_pop + 1;
                last_pop_t = $time;
            end
            if (dut.tmo && ((dut.state != dut.home) || (dut.cnt != 6'd0)))
                tmo_t = $time;
            if (o_valid && (o_flit[33:32] == 2'b00 || o_flit[33:32] == 2'b11)) begin
                cp_htime[nh] = $time;
                nh = nh + 1;
            end
            if (i_valid && (i_flit[33:32] == 2'b00 || i_flit[33:32] == 2'b11))
                nh_in = nh_in + 1;
        end
    end

    // ---------------- PC helpers ----------------
    reg [7:0] exp_pc [0:4095];      // bytes the PC must receive, in order
    integer   exp_pc_n = 0;
    integer   pc_from  = 0;         // first byte not yet checked

    integer   pc_sent  = 0;         // bytes the PC has sent

    task pc_send(input [7:0] b);
        begin
            term.send(b, 1'b0, 1'b0);
            pc_sent = pc_sent + 1;
        end
    endtask

    task wait_us(input integer us);
        begin
            #(us * 1000);
        end
    endtask

    task expect_pc(input [7:0] b);
        begin
            exp_pc[exp_pc_n] = b;
            exp_pc_n = exp_pc_n + 1;
        end
    endtask

    // queue jpay[0 .. len-1] as expected PC bytes, starting at index from
    task expect_jpay(input integer from, input integer len);
        integer k;
        begin
            for (k = from; k < len; k = k + 1) expect_pc(jpay[k]);
        end
    endtask

    // wait for all expected bytes (plus 20 us for unwanted extras) and compare
    task check_pc(input [8*72-1:0] what);
        integer k, t;
        reg     ok;
        begin
            t = 0;
            while (term.rx_cnt < exp_pc_n && t < 200 + 10 * (exp_pc_n - pc_from)) begin
                wait_us(1);
                t = t + 1;
            end
            wait_us(20);
            ok = (term.rx_cnt == exp_pc_n);
            if (!ok) $display("  PC received %0d bytes, expected %0d", term.rx_cnt, exp_pc_n);
            for (k = pc_from; k < exp_pc_n; k = k + 1) begin
                if (term.rx_mem[k] !== exp_pc[k]) begin
                    if (ok) $display("  PC byte %0d: got %h want %h", k, term.rx_mem[k], exp_pc[k]);
                    ok = 1'b0;
                end
            end
            pc_from = term.rx_cnt;
            check(ok, what);
        end
    endtask

    // ---------------- captured-packet helpers ----------------
    reg [7:0] ep [0:63];            // expected payload

    task wait_cap(input integer n, input integer max_us);
        integer t;
        begin
            t = 0;
            while (cp_cnt < n && t < max_us * 100) begin
                @(posedge clk);
                t = t + 1;
            end
        end
    endtask

    // packet k must have these header fields (src 3, tag = k) and payload ep[]
    task check_pkt(input integer k, input [2:0] t, input [2:0] d, input p,
                   input integer len, input [7:0] a, input [8*72-1:0] what);
        integer i;
        reg     ok;
        begin
            ok = (cp_cnt > k);
            if (!ok) $display("  packet %0d never arrived", k);
            else begin
                ok = (cp_type[k] == t) && (cp_dest[k] == d) && (cp_src[k] == 3'd3) &&
                     (cp_prio[k] == p) && (cp_len[k] == len) && (cp_tag[k] == k[7:0]) &&
                     (cp_arg[k] == a);
                if (!ok)
                    $display("  packet %0d: type %0d dest %0d src %0d prio %0d len %0d tag %0d arg %h",
                             k, cp_type[k], cp_dest[k], cp_src[k], cp_prio[k], cp_len[k],
                             cp_tag[k], cp_arg[k]);
                for (i = 0; i < len; i = i + 1) begin
                    if (cp_pay[k*64 + i] !== ep[i]) begin
                        if (ok) $display("  packet %0d byte %0d: got %h want %h",
                                         k, i, cp_pay[k*64 + i], ep[i]);
                        ok = 1'b0;
                    end
                end
            end
            check(ok, what);
        end
    endtask

    // send the slide-17 frame "01 9F 03" and set ep[] to its payload
    task frame_9f;
        begin
            pc_send(8'h01); pc_send(8'h9F); pc_send(8'h03);
            ep[0] = 8'h01; ep[1] = 8'h03; ep[2] = 8'h9F;
        end
    endtask

    // ---------------- watchdog ----------------
    initial begin
        #(40_000_000);
        $display("FAIL: watchdog - the test did not finish in 40 ms");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end

    // ---------------- stress-test data (test 13) ----------------
    reg [5:0] fr_wl [0:7];
    reg [5:0] fr_rl [0:7];
    reg [7:0] fr_w  [0:8*64-1];
    reg [5:0] dp_len [0:7];
    reg [7:0] dp_pay [0:8*64-1];

    // ---------------- test sequence ----------------
    integer i, j, n0, nin0, push0, pop0, cov0;
    time    dt;
    initial begin
        #1 term.bit_ns = 640.0;               // 64 clocks per bit
        #32 rst_n = 1'b1;
        @(negedge clk);
        check(pkts_in == 0 && pkts_out == 0 && !o_valid && !tx_push && !rx_pop,
              "reset: counters 0, no strobes");
        repeat (10) @(posedge clk);
        stall_on = 1'b1;

        // ---- 1: slide 17 flow A, PC -> network ----
        frame_9f;
        wait_cap(1, 100);
        check(cp_cnt == 1, "1: frame 01 9F 03 gives one packet");
        check_pkt(0, T_XREQ, 3'd4, 1'b0, 3, 8'h01, "1: XFER_REQ to node 4, arg 01, payload 01 03 9F");
        check(cp_htime[0] - last_pop_t < 1000, "1: request sent at once after rlen");

        // ---- 2: reply 20 BA 19 back to the PC ----
        jpay[0] = 8'h00; jpay[1] = 8'h20; jpay[2] = 8'hBA; jpay[3] = 8'h19;
        inject(T_XRESP, 4, cp_tag[0]);
        expect_jpay(1, 4);
        check_pc("2: PC receives exactly 20 BA 19");
        resp_status = 1'b1;
        inject(T_XRESP, 4, 8'h00);
        expect_jpay(0, 4);
        check_pc("2: resp_status = 1: PC receives 00 20 BA 19");
        resp_status = 1'b0;
        push0 = n_push;
        jpay[0] = 8'h01;                        // NACK: status only
        inject(T_XRESP, 1, 8'h00);
        check_pc("2: status-only XFER_RESP sends nothing to the PC");
        check(n_push == push0, "2: status-only XFER_RESP: no TX push");

        // ---- 3: FRAME corner cases ----
        pc_send(8'h00); pc_send(8'h02);         // wlen 0, rlen 2
        wait_cap(2, 100);
        ep[0] = 8'h00; ep[1] = 8'h02;
        check_pkt(1, T_XREQ, 3'd4, 1'b0, 2, 8'h01, "3: wlen = 0 frame -> payload 00 02, tag 1");

        pc_send(8'd60);                         // wlen = rlen = 60 (largest)
        ep[0] = 8'd60; ep[1] = 8'd60;
        for (i = 0; i < 60; i = i + 1) begin
            ep[2 + i] = $random(seed_d);
            pc_send(ep[2 + i]);
        end
        pc_send(8'd60);
        wait_cap(3, 100);
        check_pkt(2, T_XREQ, 3'd4, 1'b0, 62, 8'h01, "3: wlen = rlen = 60 -> 62-byte XFER_REQ");

        pc_send(8'h02); wait_us(10);            // pauses shorter than the time-out
        pc_send(8'hAA); wait_us(10);
        pc_send(8'hBB); wait_us(10);
        pc_send(8'h05);
        wait_cap(4, 100);
        ep[0] = 8'h02; ep[1] = 8'h05; ep[2] = 8'hAA; ep[3] = 8'hBB;
        check_pkt(3, T_XREQ, 3'd4, 1'b0, 4, 8'h01, "3: pauses < timeout keep the frame");

        // ---- 4: incomplete frame is dropped after the time-out ----
        n0 = cp_cnt;
        pc_send(8'h02); pc_send(8'hAA);         // 2 of 4 bytes
        wait_us(2 * TMO);
        dt = tmo_t - last_pop_t;
        check(dt >= TMO * 1000 && dt <= (TMO + 1) * 1000 + 20,
              "4: incomplete frame dropped timeout_us after its last byte");
        check(cp_cnt == n0 && dut.state == dut.home, "4: nothing sent, parser back to start");
        frame_9f;
        wait_cap(n0 + 1, 100);
        check_pkt(n0, T_XREQ, 3'd4, 1'b0, 3, 8'h01, "4: next frame parsed correctly (resync)");

        // a byte waiting in the RX FIFO has arrived: a long stall is not silence
        pc_send(8'h02); pc_send(8'hAA);
        stall_force = 1'b1;
        pc_send(8'hBB); pc_send(8'h03);
        wait_us(2 * TMO);
        check(rx_count == 5'd2, "4: long stall: 2 bytes wait in the RX FIFO");
        stall_force = 1'b0;
        wait_cap(n0 + 2, 100);
        ep[0] = 8'h02; ep[1] = 8'h03; ep[2] = 8'hAA; ep[3] = 8'hBB;
        check_pkt(n0 + 1, T_XREQ, 3'd4, 1'b0, 4, 8'h01, "4: frame kept across a 40 us stall");

        // ---- 5: bad lengths ----
        n0 = cp_cnt;
        pc_send(8'h3D); pc_send(8'h11); pc_send(8'h22);              // wlen 61
        wait_us(2 * TMO);
        pc_send(8'h01); pc_send(8'h9F); pc_send(8'h3D);              // rlen 61
        wait_us(2 * TMO);
        pc_send(8'h3D); pc_send(8'h01); pc_send(8'h9F); pc_send(8'h03);  // + bytes after it
        wait_us(2 * TMO);
        pc_send(8'h3D);                                              // complete, wlen 61
        for (i = 0; i < 61; i = i + 1) pc_send(8'h00);
        pc_send(8'h03);
        wait_us(2 * TMO);
        check(cp_cnt == n0, "5: wlen/rlen > 60: frame and following bytes dropped");
        frame_9f;
        wait_cap(n0 + 1, 100);
        check_pkt(n0, T_XREQ, 3'd4, 1'b0, 3, 8'h01, "5: good frame after the silence");

        // ---- 6: ADDRESSED FRAME ----
        mode = 2'd2; cfg_prio = 1'b1;           // cfg dest/arg must be ignored
        n0 = cp_cnt;
        pc_send(8'h05); pc_send(8'h50); pc_send(8'h02);
        pc_send(8'h11); pc_send(8'h22); pc_send(8'h04);
        wait_cap(n0 + 1, 100);
        ep[0] = 8'h02; ep[1] = 8'h04; ep[2] = 8'h11; ep[3] = 8'h22;
        check_pkt(n0, T_XREQ, 3'd5, 1'b1, 4, 8'h50, "6: [05][50] 02 11 22 04 -> node 5, arg 50");
        pc_send(8'h01); pc_send(8'h00); pc_send(8'h00); pc_send(8'h03);
        wait_cap(n0 + 2, 100);
        ep[0] = 8'h00; ep[1] = 8'h03;
        check_pkt(n0 + 1, T_XREQ, 3'd1, 1'b1, 2, 8'h00, "6: [01][00] 00 03 -> node 1, payload 00 03");
        pc_send(8'h09); pc_send(8'h50); pc_send(8'h00); pc_send(8'h01);   // dest 9: bad
        wait_us(2 * TMO);
        pc_send(8'h05); pc_send(8'h50);                                   // incomplete
        wait_us(2 * TMO);
        check(cp_cnt == n0 + 2, "6: bad dest byte and incomplete frame dropped");
        pc_send(8'h04); pc_send(8'h02);
        frame_9f;
        wait_cap(n0 + 3, 100);
        check_pkt(n0 + 2, T_XREQ, 3'd4, 1'b1, 3, 8'h02, "6: [04][02] 01 9F 03 after resync");

        // ---- 7: RAW ----
        mode = 2'd0; cfg_dest = 3'd1; cfg_arg = 8'h5A; cfg_prio = 1'b1;
        n0 = cp_cnt;
        for (i = 0; i < 16; i = i + 1) begin
            ep[i] = $random(seed_d);
            pc_send(ep[i]);
        end
        wait_cap(n0 + 1, 50);
        check_pkt(n0, T_DATA, 3'd1, 1'b1, 16, 8'h5A, "7: 16 bytes -> one DATA packet");
        check(cp_htime[n0] - last_pop_t < 1000, "7: 16-byte packet sent at once");

        for (i = 0; i < 5; i = i + 1) begin
            ep[i] = $random(seed_d);
            pc_send(ep[i]);
        end
        wait_cap(n0 + 2, 100);
        check_pkt(n0 + 1, T_DATA, 3'd1, 1'b1, 5, 8'h5A, "7: 5 bytes + silence -> 5-byte packet");
        dt = cp_htime[n0 + 1] - last_pop_t;
        check(dt >= TMO * 1000 && dt <= (TMO + 1) * 1000 + 100,
              "7: short packet sent timeout_us after the last byte");

        // 40 bytes back to back -> 16 + 16 + 8.  The network blocks during
        // bytes 10..37: the first packet's last flit gets stuck (no credit),
        // the NI still fills its buffer with packet 2 and waits in S_HDR,
        // then bytes 32.. wait in the UART RX FIFO.  Nothing may be lost.
        for (i = 0; i < 40; i = i + 1) begin
            fr_w[i] = $random(seed_d);
            if (i == 10) cap_hold = 1'b1;
            if (i == 38) begin
                check(rx_count >= 5'd5 && dut.state == dut.S_HDR,
                      "7: network blocked: NI waits, bytes queue in the RX FIFO");
                cap_hold = 1'b0;
            end
            pc_send(fr_w[i]);
        end
        wait_cap(n0 + 5, 100);
        for (i = 0; i < 16; i = i + 1) ep[i] = fr_w[i];
        check_pkt(n0 + 2, T_DATA, 3'd1, 1'b1, 16, 8'h5A, "7: 40 bytes: packet 1 = bytes 0..15");
        for (i = 0; i < 16; i = i + 1) ep[i] = fr_w[16 + i];
        check_pkt(n0 + 3, T_DATA, 3'd1, 1'b1, 16, 8'h5A, "7: 40 bytes: packet 2 = bytes 16..31");
        for (i = 0; i < 8; i = i + 1) ep[i] = fr_w[32 + i];
        check_pkt(n0 + 4, T_DATA, 3'd1, 1'b1, 8, 8'h5A, "7: 40 bytes: packet 3 = bytes 32..39");

        for (i = 0; i < 3; i = i + 1) begin     // gaps < time-out: one packet
            ep[i] = $random(seed_d);
            pc_send(ep[i]);
            wait_us(10);
        end
        wait_cap(n0 + 6, 100);
        check_pkt(n0 + 5, T_DATA, 3'd1, 1'b1, 3, 8'h5A, "7: 3 bytes with 10 us gaps -> one packet");
        check(cp_cnt == n0 + 6, "7: no extra RAW packets");

        mode = 2'd1;                            // half a frame in FRAME mode ...
        pc_send(8'h02); pc_send(8'hAA);
        wait_us(1);
        mode = 2'd0;                            // ... is dropped by a mode change
        for (i = 0; i < 3; i = i + 1) begin
            ep[i] = $random(seed_d);
            pc_send(ep[i]);
        end
        wait_cap(n0 + 7, 100);
        check_pkt(n0 + 6, T_DATA, 3'd1, 1'b1, 3, 8'h5A, "7: mode change drops a half frame");

        // ---- 8: network -> PC, every forwarded type ----
        for (i = 0; i < 16; i = i + 1) jpay[i] = $random(seed_d);
        inject(T_DATA, 5, 8'h00);   expect_jpay(0, 5);
        for (i = 0; i < 16; i = i + 1) jpay[i] = $random(seed_d);
        inject(T_REC, 16, 8'h00);   expect_jpay(0, 16);
        for (i = 0; i < 16; i = i + 1) jpay[i] = $random(seed_d);
        inject(T_ALARM, 3, 8'h00);  expect_jpay(0, 3);
        for (i = 0; i < 16; i = i + 1) jpay[i] = $random(seed_d);
        inject(T_XREQ, 3, 8'h00);   expect_jpay(0, 3);
        check_pc("8: DATA, RECORD, ALARM, XFER_REQ payloads reach the PC");

        // ---- 9: reserved types are consumed and dropped ----
        push0 = n_push;
        for (i = 0; i < 64; i = i + 1) jpay[i] = $random(seed_d);
        inject(3'd5, 10, 8'h00);
        inject(3'd6, 0, 8'h00);
        inject(3'd7, 63, 8'h00);
        repeat (100) @(posedge clk);
        check(n_push == push0, "9: types 5, 6, 7: nothing pushed");
        jpay[0] = 8'hC3; jpay[1] = 8'h3C;
        inject(T_DATA, 2, 8'h00);   expect_jpay(0, 2);
        check_pc("9: the DATA after reserved packets still arrives");
        check(u_inj.cred == 5'd4 && dut.u_rx.count == 5'd0, "9: all credits returned, NI buffer empty");

        // ---- 10: header-only packets push nothing ----
        push0 = n_push;
        inject(T_DATA, 0, 8'h00);
        inject(T_XRESP, 0, 8'h00);
        resp_status = 1'b1;
        inject(T_XRESP, 0, 8'h00);
        inject(T_REC, 0, 8'h00);
        resp_status = 1'b0;
        repeat (100) @(posedge clk);
        check(n_push == push0, "10: header-only packets push nothing");
        check_pc("10: PC receives nothing");

        // ---- 11: long payloads with TX FIFO back-pressure ----
        cov0 = cov_bp;
        for (j = 0; j < 4; j = j + 1) begin
            for (i = 0; i < 63; i = i + 1) jpay[i] = $random(seed_d);
            inject(T_DATA, 63, 8'h00);
            expect_jpay(0, 63);
        end
        check_pc("11: 4 x 63 bytes reach the PC complete and in order");
        check(cov_bp - cov0 > 1000, "11: the NI really waited on tx_full");

        // ---- 12: en = 0 drains the network and leaves the UART alone ----
        en = 1'b0;
        nin0 = pkts_in; push0 = n_push; pop0 = n_pop; n0 = cp_cnt;
        for (i = 0; i < 64; i = i + 1) jpay[i] = $random(seed_d);
        inject(T_DATA, 20, 8'h00);
        inject(T_XRESP, 4, 8'h00);
        inject(3'd6, 8, 8'h00);
        inject(T_DATA, 63, 8'h00);
        for (i = 0; i < 5; i = i + 1) begin
            ep[i] = $random(seed_d);
            pc_send(ep[i]);
        end
        wait_us(2 * TMO);
        check(pkts_in == nin0 + 4, "12: en = 0: all 4 packets consumed");
        check(u_inj.cred == 5'd4 && dut.u_rx.count == 5'd0, "12: en = 0: credits returned, buffer empty");
        check(n_push == push0 && n_pop == pop0, "12: en = 0: no push and no pop");
        check(rx_count == 5'd5 && cp_cnt == n0, "12: en = 0: PC bytes wait in the RX FIFO");
        check_pc("12: en = 0: nothing reaches the PC");
        en = 1'b1;                              // RAW mode: the 5 bytes go out
        wait_cap(n0 + 1, 100);
        check_pkt(n0, T_DATA, 3'd1, 1'b1, 5, 8'h5A, "12: en = 1: waiting bytes sent as RAW packet");

        mode = 2'd1; cfg_dest = 3'd4; cfg_arg = 8'h01; cfg_prio = 1'b0;
        n0 = cp_cnt;
        pc_send(8'h02); pc_send(8'hAA);         // half a frame ...
        wait_us(1);
        en = 1'b0;                              // ... is thrown away by en = 0
        wait_us(1);
        en = 1'b1;
        frame_9f;                               // well within the time-out
        wait_cap(n0 + 1, 100);
        check_pkt(n0, T_XREQ, 3'd4, 1'b0, 3, 8'h01, "12: en = 0 drops a half frame");

        // ---- 13: both directions at once ----
        for (i = 0; i < 8; i = i + 1) begin
            fr_wl[i]  = {$random(seed_d)} % 13;
            fr_rl[i]  = {$random(seed_d)} % 61;
            dp_len[i] = 1 + {$random(seed_d)} % 40;
            for (j = 0; j < 64; j = j + 1) begin
                fr_w[i*64 + j]   = $random(seed_d);
                dp_pay[i*64 + j] = $random(seed_d);
            end
        end
        n0 = cp_cnt;
        fork
            begin : pc_side
                integer f, b;
                for (f = 0; f < 8; f = f + 1) begin
                    pc_send({2'b00, fr_wl[f]});
                    for (b = 0; b < fr_wl[f]; b = b + 1) pc_send(fr_w[f*64 + b]);
                    pc_send({2'b00, fr_rl[f]});
                    wait_us({$random(seed_d)} % 6);
                end
            end
            begin : net_side
                integer p, b;
                for (p = 0; p < 8; p = p + 1) begin
                    for (b = 0; b < 64; b = b + 1) jpay[b] = dp_pay[p*64 + b];
                    inject(T_DATA, dp_len[p], p[7:0]);
                    expect_jpay(0, dp_len[p]);
                    wait_us({$random(seed_d)} % 30);
                end
            end
        join
        wait_cap(n0 + 8, 200);
        for (i = 0; i < 8; i = i + 1) begin
            ep[0] = {2'b00, fr_wl[i]};
            ep[1] = {2'b00, fr_rl[i]};
            for (j = 0; j < 60; j = j + 1) ep[2 + j] = fr_w[i*64 + j];
            check_pkt(n0 + i, T_XREQ, 3'd4, 1'b0, fr_wl[i] + 2, 8'h01,
                      "13: random frame while replies flow back");
        end
        check_pc("13: random DATA packets reach the PC while frames are sent");

        // ---- 14: counters and global assertions ----
        repeat (20) @(posedge clk);
        check(pkts_in == n_inj && pkts_in == nh_in, "14: pkts_in = packets received");
        check(pkts_out == cp_cnt && pkts_out == nh, "14: pkts_out = packets sent");
        check(bad_stall == 0, "14: no push or pop while stall = 1");
        check(bad_en == 0, "14: no push or pop while en = 0");
        check(bad_full == 0 && bad_empty == 0, "14: never pushed when full / popped when empty");
        check(uart_ovf == 0 && uart_ferr == 0, "14: no UART overflow or framing error");
        check(cov_stall > 100, "14: random stall really held the NI back");
        check(n_pop == pc_sent && rx_count == 5'd0, "14: every byte from the PC was taken once");
        $display("pkts_in %0d  pkts_out %0d  pushes %0d  pops %0d  stall-blocked %0d  tx_full-waits %0d",
                 pkts_in, pkts_out, n_push, n_pop, cov_stall, cov_bp);

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end
endmodule

// =============================================================================
// tb_noc_pkt.v  -  unit test: noc_pkt_tx -> noc_pkt_rx loopback + time base
// -----------------------------------------------------------------------------
// The sender is wired straight to the receiver (valid/flit one way, credit
// back), which is exactly one NoC link.  The test:
//   A. 400 random packets (length 0..63, random fields and random gaps on
//      both sides).  Every header field and every byte must arrive in order.
//      A monitor on the link checks the flit kinds (HEAD/BODY/TAIL/SINGLE),
//      the flit count per packet and that unused TAIL bytes are 0.
//   B. Credit flow control: the receiver stops taking data, a 63-byte packet
//      (17 flits) is sent, and only DEPTH flits may cross the link.
//   C. A stray BODY flit in front of a header is thrown away.
//   D. ssc2_timebase: us_tick every 100 clocks, ms_tick every US_PER_MS us.
// =============================================================================
`timescale 1ns / 1ps
module tb_noc_pkt;
    localparam NPKT  = 400;
    localparam DEPTH = 4;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0;
    integer errors = 0;
    task check(input cond, input [8*64-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    // ---------------- packet list ----------------
    reg [2:0] p_type [0:NPKT-1];
    reg [2:0] p_dest [0:NPKT-1];
    reg [2:0] p_src  [0:NPKT-1];
    reg       p_prio [0:NPKT-1];
    reg [5:0] p_len  [0:NPKT-1];
    reg [7:0] p_tag  [0:NPKT-1];
    reg [7:0] p_arg  [0:NPKT-1];
    reg [7:0] p_pay  [0:NPKT*64-1];

    // ---------------- DUTs ----------------
    reg        start = 1'b0;
    wire       hdr_ready;
    reg  [2:0] t_type = 0, t_dest = 0, t_src = 0;
    reg        t_prio = 0;
    reg  [5:0] t_len = 0;
    reg  [7:0] t_tag = 0, t_arg = 0;
    reg        tb_valid = 1'b0;
    reg  [7:0] tb_data = 8'd0;
    wire       tb_ready;
    wire       link_valid;
    wire [33:0] link_flit;
    wire       link_credit;

    noc_pkt_tx #(.DEPTH(DEPTH)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(start), .hdr_ready(hdr_ready),
        .ptype(t_type), .dest(t_dest), .src(t_src), .prio(t_prio),
        .len(t_len), .tag(t_tag), .arg(t_arg),
        .b_valid(tb_valid), .b_data(tb_data), .b_ready(tb_ready),
        .out_valid(link_valid), .out_flit(link_flit), .out_credit(link_credit));

    wire       r_hv;
    wire [2:0] r_type, r_dest, r_src;
    wire       r_prio;
    wire [5:0] r_len;
    wire [7:0] r_tag, r_arg;
    reg        r_hready = 1'b0;
    wire       r_bv;
    wire [7:0] r_bd;
    wire       r_blast;
    reg        r_bready = 1'b0;

    noc_pkt_rx #(.DEPTH(DEPTH)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(link_valid), .in_flit(link_flit), .in_credit(link_credit),
        .hdr_valid(r_hv), .ptype(r_type), .dest(r_dest), .src(r_src),
        .prio(r_prio), .len(r_len), .tag(r_tag), .arg(r_arg),
        .hdr_ready(r_hready),
        .b_valid(r_bv), .b_data(r_bd), .b_last(r_blast), .b_ready(r_bready));

    // ---------------- producer ----------------
    integer prod_limit = 0;     // packets allowed to start
    integer tx_pkt = 0;
    integer tx_idx = 0;
    reg     tx_phase = 1'b0;    // 0 header, 1 payload

    always @(posedge clk) begin
        if (rst_n) begin
            if (!tx_phase) begin
                if (start && hdr_ready) begin
                    start <= 1'b0;
                    if (p_len[tx_pkt] == 0) begin
                        tx_pkt <= tx_pkt + 1;
                    end else begin
                        tx_phase <= 1'b1;
                        tx_idx   <= 0;
                        tb_valid <= $random & 1;
                        tb_data  <= p_pay[tx_pkt*64];
                    end
                end else if (!start && tx_pkt < prod_limit && ($random & 1)) begin
                    start  <= 1'b1;
                    t_type <= p_type[tx_pkt]; t_dest <= p_dest[tx_pkt];
                    t_src  <= p_src[tx_pkt];  t_prio <= p_prio[tx_pkt];
                    t_len  <= p_len[tx_pkt];  t_tag  <= p_tag[tx_pkt];
                    t_arg  <= p_arg[tx_pkt];
                end
            end else begin
                if (tb_valid && tb_ready) begin
                    if (tx_idx + 1 == p_len[tx_pkt]) begin
                        tb_valid <= 1'b0;
                        tx_phase <= 1'b0;
                        tx_pkt   <= tx_pkt + 1;
                    end else begin
                        tx_idx   <= tx_idx + 1;
                        tb_valid <= ($random % 4) != 0;
                        tb_data  <= p_pay[tx_pkt*64 + tx_idx + 1];
                    end
                end else begin
                    tb_valid <= ($random % 4) != 0;
                    tb_data  <= p_pay[tx_pkt*64 + tx_idx];
                end
            end
        end
    end

    // ---------------- consumer ----------------
    reg     cons_en = 1'b1;
    integer rx_pkt = 0;
    integer rx_idx = 0;
    reg     rx_phase = 1'b0;
    reg     pkt_ok = 1'b1;
    integer pkts_ok = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            if (!rx_phase) begin
                if (r_hv && r_hready) begin
                    pkt_ok = (r_type == p_type[rx_pkt]) && (r_dest == p_dest[rx_pkt]) &&
                             (r_src  == p_src[rx_pkt])  && (r_prio == p_prio[rx_pkt]) &&
                             (r_len  == p_len[rx_pkt])  && (r_tag  == p_tag[rx_pkt])  &&
                             (r_arg  == p_arg[rx_pkt]);
                    if (!pkt_ok)
                        $display("  header mismatch in packet %0d", rx_pkt);
                    if (p_len[rx_pkt] == 0) begin
                        if (pkt_ok) pkts_ok = pkts_ok + 1;
                        rx_pkt <= rx_pkt + 1;
                    end else begin
                        rx_phase <= 1'b1;
                        rx_idx   <= 0;
                    end
                end
            end else begin
                if (r_bv && r_bready) begin
                    if (r_bd !== p_pay[rx_pkt*64 + rx_idx] ||
                        r_blast !== (rx_idx + 1 == p_len[rx_pkt])) begin
                        pkt_ok = 1'b0;
                        $display("  byte %0d of packet %0d: got %h last %b", rx_idx,
                                 rx_pkt, r_bd, r_blast);
                    end
                    if (rx_idx + 1 == p_len[rx_pkt]) begin
                        if (pkt_ok) pkts_ok = pkts_ok + 1;
                        rx_phase <= 1'b0;
                        rx_pkt   <= rx_pkt + 1;
                    end else begin
                        rx_idx <= rx_idx + 1;
                    end
                end
            end
            r_hready <= cons_en && (($random % 4) != 0);
            r_bready <= cons_en && (($random % 4) != 0);
        end
    end

    // ---------------- link monitor ----------------
    integer mon_left = 0;        // payload flits still expected
    integer mon_lastbytes = 0;   // valid bytes in the TAIL flit
    integer mon_bad = 0;
    integer link_flits = 0;
    integer max_fill = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            if (u_rx.count > max_fill) max_fill = u_rx.count;
            if (link_valid) begin
                link_flits = link_flits + 1;
                case (link_flit[33:32])
                    2'b00: begin
                        if (mon_left != 0 || link_flit[21:16] == 0) mon_bad = mon_bad + 1;
                        mon_left      = (link_flit[21:16] + 3) / 4;
                        mon_lastbytes = link_flit[21:16] - 4 * (mon_left - 1);
                    end
                    2'b11: if (mon_left != 0 || link_flit[21:16] != 0) mon_bad = mon_bad + 1;
                    2'b01: begin
                        if (mon_left < 2) mon_bad = mon_bad + 1;
                        mon_left = mon_left - 1;
                    end
                    2'b10: begin
                        if (mon_left != 1) mon_bad = mon_bad + 1;
                        if (mon_lastbytes < 4 &&
                            (link_flit[31:0] >> (8 * mon_lastbytes)) != 0)
                            mon_bad = mon_bad + 1;
                        mon_left = 0;
                    end
                endcase
            end
        end
    end

    // ---------------- second receiver for the stray-flit test ----------------
    reg         s_valid = 1'b0;
    reg  [33:0] s_flit = 34'd0;
    wire        s_credit;
    wire        s_hv;
    wire [5:0]  s_len;
    wire [7:0]  s_tag;
    wire        s_bv;
    wire [7:0]  s_bd;
    wire        s_blast;
    reg         s_hready = 1'b0;
    reg         s_bready = 1'b0;
    integer     s_credits = 0;
    always @(posedge clk) if (s_credit) s_credits = s_credits + 1;

    noc_pkt_rx #(.DEPTH(DEPTH)) u_rx2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s_valid), .in_flit(s_flit), .in_credit(s_credit),
        .hdr_valid(s_hv), .ptype(), .dest(), .src(), .prio(),
        .len(s_len), .tag(s_tag), .arg(), .hdr_ready(s_hready),
        .b_valid(s_bv), .b_data(s_bd), .b_last(s_blast), .b_ready(s_bready));

    task send_flit(input [33:0] f);
        begin
            @(negedge clk); s_valid = 1'b1; s_flit = f;
            @(negedge clk); s_valid = 1'b0;
        end
    endtask

    // ---------------- time base ----------------
    wire [31:0] time_us;
    wire        us_tick, ms_tick;
    ssc2_timebase #(.CLK_HZ(100_000_000), .US_PER_MS(10)) u_tb (
        .clk(clk), .rst_n(rst_n), .time_us(time_us), .us_tick(us_tick), .ms_tick(ms_tick));

    integer n_us = 0, n_ms = 0, last_us = -1, us_gap_bad = 0, ms_gap_bad = 0, last_ms = -1;
    integer cyc = 0;
    always @(posedge clk) if (rst_n) begin
        cyc = cyc + 1;
        if (us_tick) begin
            if (last_us >= 0 && cyc - last_us != 100) us_gap_bad = us_gap_bad + 1;
            last_us = cyc; n_us = n_us + 1;
        end
        if (ms_tick) begin
            if (last_ms >= 0 && cyc - last_ms != 1000) ms_gap_bad = ms_gap_bad + 1;
            last_ms = cyc; n_ms = n_ms + 1;
        end
    end

    // ---------------- test sequence ----------------
    integer i, j, flits_before;
    integer seed;
    initial begin
        seed = 1;
        j = $random(seed);
        for (i = 0; i < NPKT; i = i + 1) begin
            p_type[i] = $random; p_dest[i] = $random; p_src[i] = $random;
            p_prio[i] = $random; p_tag[i] = $random;  p_arg[i] = $random;
            p_len[i]  = $random;
            for (j = 0; j < 64; j = j + 1) p_pay[i*64 + j] = $random;
        end
        // corner lengths at the start, and a 63-byte packet at the end
        p_len[0] = 0; p_len[1] = 1; p_len[2] = 3; p_len[3] = 4; p_len[4] = 5;
        p_len[5] = 8; p_len[6] = 63; p_len[7] = 0; p_len[8] = 0;
        p_len[NPKT-1] = 63;

        #33 rst_n = 1'b1;

        // ---- A: random packets ----
        prod_limit = NPKT - 1;
        i = 0;
        while (rx_pkt < NPKT - 1 && i < 400000) begin @(posedge clk); i = i + 1; end
        check(rx_pkt == NPKT - 1, "A: all random packets received");
        check(pkts_ok == NPKT - 1, "A: every header and byte matched");
        check(mon_bad == 0, "A: flit kinds / counts / TAIL padding correct");
        check(max_fill <= DEPTH, "A: receiver buffer never over-filled");
        $display("A: %0d packets, %0d flits, %0d good", rx_pkt, link_flits, pkts_ok);

        // ---- B: credit back-pressure ----
        repeat (5) @(posedge clk);
        cons_en = 1'b0;
        repeat (3) @(posedge clk);
        flits_before = link_flits;
        prod_limit = NPKT;
        repeat (400) @(posedge clk);
        check(link_flits - flits_before == DEPTH, "B: only DEPTH flits sent without credits");
        check(u_rx.count == DEPTH, "B: receiver buffer full, not over-full");
        check(u_tx.cred == 0, "B: sender has no credits left");
        cons_en = 1'b1;
        i = 0;
        while (rx_pkt < NPKT && i < 20000) begin @(posedge clk); i = i + 1; end
        check(rx_pkt == NPKT && pkts_ok == NPKT, "B: 63-byte packet complete after release");
        check(link_flits - flits_before == 17, "B: 63-byte packet = 17 flits");
        repeat (10) @(posedge clk);
        check(u_tx.cred == DEPTH, "B: all credits returned");
        check(max_fill <= DEPTH, "B: buffer never over-filled");

        // ---- C: stray flit is dropped ----
        send_flit({2'b01, 32'hDEAD_BEEF});                         // stray BODY
        send_flit({2'b00, 3'd0, 3'd1, 3'd2, 1'b0, 6'd2, 8'h5A, 8'h00});  // HEAD len 2
        send_flit({2'b10, 32'h0000_BBAA});                         // TAIL
        @(negedge clk);
        check(s_hv && s_len == 2 && s_tag == 8'h5A, "C: header seen after the stray flit");
        s_hready = 1'b1; @(negedge clk); s_hready = 1'b0;
        check(s_bv && s_bd == 8'hAA && !s_blast, "C: payload byte 0");
        s_bready = 1'b1; @(negedge clk);
        check(s_bv && s_bd == 8'hBB && s_blast, "C: payload byte 1 is last");
        @(negedge clk); s_bready = 1'b0;
        check(!s_bv && !s_hv, "C: receiver empty");
        @(negedge clk);
        check(s_credits == 3, "C: one credit per flit (3)");

        // ---- D: time base ----
        check(us_gap_bad == 0 && n_us > 100, "D: us_tick every 100 clocks");
        check(ms_gap_bad == 0 && n_ms > 10, "D: ms_tick every US_PER_MS microseconds");
        check(time_us == n_us + us_tick, "D: time_us counts us_ticks");

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end
endmodule

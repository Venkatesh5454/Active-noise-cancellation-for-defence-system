// =============================================================================
// tb_ni_se.v  -  unit test: serial-engine network interface (ni_se, node 0)
// -----------------------------------------------------------------------------
// ni_se sits between the network and the two state machines' FIFOs.  Here
// the FIFOs are behavioural (4 words each, like se_engine), so every word
// that ni_se pushes or pops can be checked exactly.  The network side uses
// noc_pkt_tx (packets in) and noc_pkt_rx (packets out) on direct links.
//
//   A. network -> SM: 300 random packets (all types, lengths 0..63, either
//      SM, OUT shifting right or left).  Every byte must become exactly one
//      TX word of the right SM, in [7:0] or [31:24].
//   B. SM -> network: random RX words on both SMs, IN shifting right or
//      left, 1 or 4 bytes per word.  The byte stream of each SM must come
//      out unchanged, in DATA packets to that SM's dest/prio, arg = SM,
//      source = my_id, at most 16 bytes each.
//   C. a lone word is sent at once (FIFO empty -> flush); 20 words in a
//      burst make 16 + 4 byte packets.
//   D. en = 0: packets are drained, the FIFOs are never touched.
//   E. bytes for a disabled SM whose TX FIFO is full are dropped, so a packet
//      for a stopped SM cannot block node 0 (and everything behind it).
// Throughout: stall is random, and no push/pop may happen while stall = 1,
// no push into a full FIFO and no pop from an empty one.
// =============================================================================
`timescale 1ns / 1ps
module tb_ni_se;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0, errors = 0;
    task check(input cond, input [8*72-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    // ---------------- configuration ----------------
    reg        en = 1'b1;
    reg        stall = 1'b0;
    reg        stall_rand = 1'b0;
    reg  [2:0] dest0 = 3'd3, dest1 = 3'd5;
    reg        prio0 = 1'b0, prio1 = 1'b1;
    reg        word4_0 = 1'b0, word4_1 = 1'b0;
    reg  [1:0] osl = 2'b00, isl = 2'b00;      // out / in shift left per SM
    reg  [1:0] sm_en = 2'b11;                 // SMs enabled

    // ---------------- behavioural SM FIFOs ----------------
    wire [1:0]  tx_push, rx_pop;
    wire [31:0] tx_data0, tx_data1;
    reg  [31:0] txq0 [0:3], txq1 [0:3], rxq0 [0:3], rxq1 [0:3];
    integer     txn0 = 0, txn1 = 0, rxn0 = 0, rxn1 = 0;
    wire [1:0]  tx_full  = {txn1 == 4, txn0 == 4};
    wire [1:0]  rx_empty = {rxn1 == 0, rxn0 == 0};
    wire [31:0] rx_data0 = rxq0[0];
    wire [31:0] rx_data1 = rxq1[0];

    // ---------------- DUT ----------------
    wire        n2d_valid, n2d_credit, d2n_valid, d2n_credit;
    wire [33:0] n2d_flit, d2n_flit;
    wire [15:0] pkts_in, pkts_out;

    ni_se dut (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(en), .my_id(3'd0),
        .dest0(dest0), .dest1(dest1), .prio0(prio0), .prio1(prio1),
        .word4_0(word4_0), .word4_1(word4_1),
        .out_shift_left(osl), .in_shift_left(isl), .sm_en(sm_en),
        .tx_push(tx_push), .tx_data0(tx_data0), .tx_data1(tx_data1), .tx_full(tx_full),
        .rx_pop(rx_pop), .rx_data0(rx_data0), .rx_data1(rx_data1), .rx_empty(rx_empty),
        .in_valid(n2d_valid), .in_flit(n2d_flit), .in_credit(n2d_credit),
        .out_valid(d2n_valid), .out_flit(d2n_flit), .out_credit(d2n_credit),
        .pkts_in(pkts_in), .pkts_out(pkts_out));

    // ---------------- packet injector (network -> DUT) ----------------
    reg        p_start = 1'b0;
    wire       p_hdr_ready;
    reg  [2:0] p_type = 0;
    reg  [5:0] p_len = 0;
    reg  [7:0] p_arg = 0;
    reg        p_bvalid = 1'b0;
    reg  [7:0] p_bdata = 0;
    wire       p_bready;
    noc_pkt_tx #(.DEPTH(4)) inj (
        .clk(clk), .rst_n(rst_n), .start(p_start), .hdr_ready(p_hdr_ready),
        .ptype(p_type), .dest(3'd0), .src(3'd4), .prio(1'b0), .len(p_len),
        .tag(8'h00), .arg(p_arg),
        .b_valid(p_bvalid), .b_data(p_bdata), .b_ready(p_bready),
        .out_valid(n2d_valid), .out_flit(n2d_flit), .out_credit(n2d_credit));

    // send one packet; payload byte i = base + 7*i
    task send_pkt(input [2:0] t, input [5:0] n, input [7:0] arg, input [7:0] base);
        integer i;
        begin
            @(negedge clk);
            while (!p_hdr_ready) @(negedge clk);
            p_type = t; p_len = n; p_arg = arg; p_start = 1'b1;
            @(negedge clk); p_start = 1'b0;
            for (i = 0; i < n; i = i + 1) begin
                p_bvalid = 1'b1; p_bdata = base + 7 * i;
                @(posedge clk); while (!p_bready) @(posedge clk);
                @(negedge clk);
            end
            p_bvalid = 1'b0;
        end
    endtask

    // ---------------- packet catcher (DUT -> network) ----------------
    wire       c_hv, c_bv, c_blast;
    wire [2:0] c_type, c_dest, c_src;
    wire       c_prio;
    wire [5:0] c_len;
    wire [7:0] c_tag, c_arg, c_byte;
    reg        c_hready = 1'b0, c_bready = 1'b0;
    noc_pkt_rx #(.DEPTH(4)) cat (
        .clk(clk), .rst_n(rst_n),
        .in_valid(d2n_valid), .in_flit(d2n_flit), .in_credit(d2n_credit),
        .hdr_valid(c_hv), .ptype(c_type), .dest(c_dest), .src(c_src), .prio(c_prio),
        .len(c_len), .tag(c_tag), .arg(c_arg), .hdr_ready(c_hready),
        .b_valid(c_bv), .b_data(c_byte), .b_last(c_blast), .b_ready(c_bready));

    // expected byte streams SM -> network, and what came out
    reg [7:0] exp_out0 [0:8191], exp_out1 [0:8191];
    integer   eo0 = 0, eo1 = 0;        // bytes expected so far
    integer   go0 = 0, go1 = 0;        // bytes received so far
    integer   out_bad = 0, hdr_bad = 0, npk = 0, maxlen = 0;
    integer   lens [0:4095];
    reg       c_sm;
    reg       c_in_pkt = 1'b0;

    always @(posedge clk) begin
        c_hready <= ($random % 4) != 0;
        c_bready <= ($random % 4) != 0;
        if (c_hv && c_hready) begin
            c_sm = c_arg[0];
            if (c_type !== 3'd0 || c_src !== 3'd0 || c_arg[7:1] !== 7'd0 ||
                c_dest !== (c_arg[0] ? dest1 : dest0) || c_prio !== (c_arg[0] ? prio1 : prio0) ||
                c_len == 0 || c_len > 16)
                hdr_bad = hdr_bad + 1;
            if (npk < 4096) lens[npk] = c_len;
            if (c_len > maxlen) maxlen = c_len;
            npk = npk + 1;
        end
        if (c_bv && c_bready) begin
            if (c_sm == 1'b0) begin
                if (c_byte !== exp_out0[go0]) out_bad = out_bad + 1;
                go0 = go0 + 1;
            end else begin
                if (c_byte !== exp_out1[go1]) out_bad = out_bad + 1;
                go1 = go1 + 1;
            end
        end
    end

    // ---------------- FIFO behaviour + protocol checks ----------------
    reg [31:0] exp_in0 [0:8191], exp_in1 [0:8191];   // expected TX words per SM
    integer    ei0 = 0, ei1 = 0, gi0 = 0, gi1 = 0, in_bad = 0, proto_bad = 0;
    reg        drain = 1'b1;                          // SM consumes TX words
    integer    k;

    always @(posedge clk) begin
        if (rst_n) begin
            if (stall && (tx_push != 0 || rx_pop != 0)) proto_bad = proto_bad + 1;
            if (!en && (tx_push != 0 || rx_pop != 0))   proto_bad = proto_bad + 1;
            if ((tx_push[0] && txn0 == 4) || (tx_push[1] && txn1 == 4)) proto_bad = proto_bad + 1;
            if ((rx_pop[0] && rxn0 == 0) || (rx_pop[1] && rxn1 == 0))   proto_bad = proto_bad + 1;
            // TX FIFO 0: push, and the "SM" takes words at random
            if (tx_push[0] && txn0 < 4) begin
                if (tx_data0 !== exp_in0[gi0]) in_bad = in_bad + 1;
                gi0 = gi0 + 1;
                txq0[txn0] = tx_data0; txn0 = txn0 + 1;
            end
            if (tx_push[1] && txn1 < 4) begin
                if (tx_data1 !== exp_in1[gi1]) in_bad = in_bad + 1;
                gi1 = gi1 + 1;
                txq1[txn1] = tx_data1; txn1 = txn1 + 1;
            end
            if (drain && txn0 > 0 && ($random % 3) == 0) begin
                for (k = 0; k < 3; k = k + 1) txq0[k] = txq0[k + 1];
                txn0 = txn0 - 1;
            end
            if (drain && txn1 > 0 && ($random % 3) == 0) begin
                for (k = 0; k < 3; k = k + 1) txq1[k] = txq1[k + 1];
                txn1 = txn1 - 1;
            end
            // RX FIFOs: the DUT pops
            if (rx_pop[0] && rxn0 > 0) begin
                for (k = 0; k < 3; k = k + 1) rxq0[k] = rxq0[k + 1];
                rxn0 = rxn0 - 1;
            end
            if (rx_pop[1] && rxn1 > 0) begin
                for (k = 0; k < 3; k = k + 1) rxq1[k] = rxq1[k + 1];
                rxn1 = rxn1 - 1;
            end
            if (stall_rand) stall <= ($random % 5) == 0;
        end
    end

    // expected TX words for a packet (call before sending it)
    task expect_in(input [5:0] n, input [7:0] arg, input [7:0] base);
        integer i;
        reg [7:0] b;
        begin
            for (i = 0; i < n; i = i + 1) begin
                b = base + 7 * i;
                if (arg[0] == 1'b0) begin
                    exp_in0[ei0] = osl[0] ? {b, 24'd0} : {24'd0, b}; ei0 = ei0 + 1;
                end else begin
                    exp_in1[ei1] = osl[1] ? {b, 24'd0} : {24'd0, b}; ei1 = ei1 + 1;
                end
            end
        end
    endtask

    // the SM "receives" a word: put it in its RX FIFO (waits for room)
    task rx_word(input sm, input [31:0] w);
        begin
            @(negedge clk);
            if (sm == 1'b0) begin
                while (rxn0 == 4) @(negedge clk);
                rxq0[rxn0] = w; rxn0 = rxn0 + 1;
                if (word4_0) begin
                    exp_out0[eo0] = w[7:0];   exp_out0[eo0+1] = w[15:8];
                    exp_out0[eo0+2] = w[23:16]; exp_out0[eo0+3] = w[31:24]; eo0 = eo0 + 4;
                end else begin
                    exp_out0[eo0] = isl[0] ? w[7:0] : w[31:24]; eo0 = eo0 + 1;
                end
            end else begin
                while (rxn1 == 4) @(negedge clk);
                rxq1[rxn1] = w; rxn1 = rxn1 + 1;
                if (word4_1) begin
                    exp_out1[eo1] = w[7:0];   exp_out1[eo1+1] = w[15:8];
                    exp_out1[eo1+2] = w[23:16]; exp_out1[eo1+3] = w[31:24]; eo1 = eo1 + 4;
                end else begin
                    exp_out1[eo1] = isl[1] ? w[7:0] : w[31:24]; eo1 = eo1 + 1;
                end
            end
        end
    endtask

    task wait_idle(input integer max);
        integer t;
        begin
            t = 0;
            while ((gi0 < ei0 || gi1 < ei1 || go0 < eo0 || go1 < eo1 || rxn0 || rxn1) && t < max) begin
                @(posedge clk); t = t + 1;
            end
            repeat (40) @(posedge clk);
        end
    endtask

    integer i, n, a, npk0, in0, out0, seed;
    reg [7:0] b8;
    initial begin
        seed = 42; k = $random(seed);
        for (i = 0; i < 4; i = i + 1) begin rxq0[i] = 0; rxq1[i] = 0; end
        #33 rst_n = 1'b1;
        stall_rand = 1'b1;

        // ---------------- A: network -> SM ----------------
        for (i = 0; i < 300; i = i + 1) begin
            if (i % 75 == 0) begin                    // change OUT shift only when idle
                wait_idle(50000);
                osl = i / 75;                         // 00, 01, 10, 11
            end
            n = $random & 63; if (i < 4) n = i;       // 0, 1, 2, 3 first
            a = $random & 255;
            b8 = $random;
            expect_in(n, a, b8);
            send_pkt($random, n, a, b8);
        end
        wait_idle(200000);
        check(gi0 == ei0 && gi1 == ei1, "A: every payload byte became one TX word");
        check(in_bad == 0, "A: TX words in the right SM, byte in [7:0] or [31:24]");
        check(pkts_in == 300, "A: pkts_in counts 300 packets");
        $display("A: %0d words to SM0, %0d to SM1", gi0, gi1);

        // ---------------- B: SM -> network, random ----------------
        npk0 = npk;
        for (i = 0; i < 600; i = i + 1) begin
            if (i % 150 == 0) begin
                wait_idle(50000);
                isl = (i / 150);                      // 00, 01, 10, 11
                word4_0 = (i / 150) == 2; word4_1 = (i / 150) == 3;
                dest0 = $random; dest1 = $random; prio0 = $random; prio1 = $random;
            end
            rx_word($random & 1, $random);
            if (($random % 4) == 0) repeat ($random & 31) @(posedge clk);
        end
        wait_idle(200000);
        check(go0 == eo0 && go1 == eo1, "B: every RX byte arrived in a packet");
        check(out_bad == 0, "B: byte streams unchanged and in order per SM");
        check(hdr_bad == 0, "B: DATA, dest/prio of the SM, arg = SM, src = my_id, len 1..16");
        check(maxlen <= 16 && maxlen >= 4, "B: packets carry up to 16 bytes");
        check(pkts_out == npk, "B: pkts_out counts the packets");
        $display("B: %0d + %0d bytes in %0d packets (longest %0d)", go0, go1, npk - npk0, maxlen);

        // ---------------- C: flush rules ----------------
        stall_rand = 1'b0; stall = 1'b0;
        isl = 2'b00; word4_0 = 1'b0; word4_1 = 1'b0;
        npk0 = npk;
        rx_word(1'b0, 32'hA1000000);
        wait_idle(1000);
        $display("C1: npk %0d -> %0d, len %0d, go0 %0d eo0 %0d", npk0, npk, lens[npk0], go0, eo0);
        check(npk == npk0 + 1 && lens[npk0] == 1, "C: a lone byte is sent at once (1-byte packet)");
        // 20 words while the catcher is slow -> 16 + 4
        npk0 = npk;
        drain = 1'b1;
        force c_hready = 1'b0;
        for (i = 0; i < 20; i = i + 1) rx_word(1'b1, {8'd100 + i[7:0], 24'd0});
        release c_hready;
        $display("C2: all 20 words taken at t=%0t", $time);
        wait_idle(5000);
        $display("C2: npk %0d -> %0d, lens %0d %0d", npk0, npk, lens[npk0], lens[npk0+1]);
        check(npk == npk0 + 2 && lens[npk0] == 16 && lens[npk0 + 1] == 4,
              "C: 20 queued bytes -> packets of 16 and 4");
        check(out_bad == 0, "C: bytes correct");

        // ---------------- D: en = 0 ----------------
        en = 1'b0;
        in0 = gi0 + gi1; out0 = npk;
        for (i = 0; i < 10; i = i + 1) send_pkt(3'd0, 6'd20, i, 8'h10);
        @(negedge clk); rxq0[0] = 32'h55000000; rxn0 = 1;      // a word waits in RX0
        repeat (2000) @(posedge clk);
        check(gi0 + gi1 == in0, "D: en = 0 pushes nothing");
        check(rxn0 == 1 && npk == out0, "D: en = 0 pops nothing, sends nothing");
        check(inj.cred == 4, "D: all packets were drained (credits back)");
        en = 1'b1;
        exp_out0[eo0] = 8'h55; eo0 = eo0 + 1;                   // now it may go
        wait_idle(2000);
        check(go0 == eo0 && out_bad == 0, "D: after en = 1 the waiting byte is sent");

        // ---------------- E: a stopped SM cannot block the network ----------------
        wait_idle(2000);
        drain = 1'b0;                     // SM0 stops emptying its TX FIFO ...
        sm_en = 2'b10;                    // ... because it is disabled
        repeat (5) @(posedge clk);
        n = 4 - txn0;                     // free words left in SM0's TX FIFO
        expect_in(n, 8'h00, 8'h30);       // only these bytes may be pushed
        send_pkt(3'd0, 6'd10, 8'h00, 8'h30);   // 10 bytes for SM0: the rest are dropped
        expect_in(3, 8'h01, 8'h50);
        send_pkt(3'd0, 6'd3, 8'h01, 8'h50);    // the next packet, for SM1, still gets through
        wait_idle(5000);
        check(gi0 == ei0 && txn0 == 4, "E: disabled SM0 got 4 words, the other 6 bytes were dropped");
        check(gi1 == ei1 && in_bad == 0, "E: the following packet for SM1 was delivered");
        check(inj.cred == 4, "E: nothing is stuck in the network");
        drain = 1'b1; sm_en = 2'b11;
        wait_idle(2000);

        check(proto_bad == 0, "no push/pop during stall or en=0, none into full/from empty");
        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("FAIL: watchdog");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule

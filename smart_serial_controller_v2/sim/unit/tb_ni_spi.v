// =============================================================================
// tb_ni_spi.v  -  unit test: ni_spi + real v1 ssc_spi + SPI flash model
// -----------------------------------------------------------------------------
// Set-up (exactly like ssc2_core.v wires it):
//
//   TB requester --noc_pkt_tx--> ni_spi --noc_pkt_rx--> TB reply collector
//                                   |
//            CPU strobes (idle) --OR/mux--> ssc_spi --> tb_spi_flash  (CS0)
//                                              \-----> tb_spi_slave  (CS1)
//
// While ni_spi.active = 1 the engine is forced to: enabled, master, 8-bit
// words, manual CS with the NI's cs_level / cs_sel.  The "CPU" settings are
// deliberately different (disabled, slave mode, 16-bit words) to prove that
// the forcing works.  A stall generator asserts stall (= APB psel) at random.
//
// Tests:
//   1. slide 17: XFER_REQ cs0 [01][03][9F] -> XFER_RESP [00][20 BA 19],
//      same tag, back to the requester, one CS-low window, 32 SCLK edges
//   2. DATA [06] (write enable): one CS frame, no reply
//   3. DATA page program, 44 bytes in ONE CS frame (more than 16: streaming)
//   4. XFER_REQ [01][01][05] status polling until the flash is ready
//   5. long read: wlen 4 + rlen 40 -> 41-byte reply, at most 16 in flight
//   6. longest read: rlen 60 -> 61-byte reply
//   7. two DATA packets back to back to CS1 (generic slave): bytes arrive,
//      two CS frames, CS0 untouched
//   8. the CPU uses the engine through the shared FIFOs and leaves 3 old
//      bytes in the RX FIFO; the next NI transfer throws them away first
//   9. bad requests -> status 4, the engine is never touched
//  10. unknown types / stray XFER_RESP: consumed, no reply
//  11. en = 0: everything consumed and dropped, engine untouched; en = 1 again
//  12. heavy stall + 3 back-to-back requests + reply back-pressure
//  13. packet counters
// Global monitors: no engine strobe while stall = 1, no SCLK edge with every
// CS high, no FIFO overflow, in-flight <= 16, the reply only leaves after CS
// is released, CS-high gap between frames >= 80 ns.
// =============================================================================
`timescale 1ns / 1ps
module tb_ni_spi;
    localparam [2:0] T_DATA = 3'd0, T_XFER = 3'd1, T_RESP = 3'd2;
    localparam [2:0] MY_ID = 3'd4;

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

    integer seed  = 7;      // requester gaps
    integer sseed = 11;     // stall generator
    integer rseed = 13;     // reply collector

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
    // the DUT
    // ------------------------------------------------------------------
    reg         stall_gen = 1'b0;     // random "APB transfers"
    reg         cpu_stall = 1'b0;     // the CPU's own SPI accesses (test 8)
    wire        stall = stall_gen | cpu_stall;
    reg         en    = 1'b1;
    wire        ni_tx_push, ni_rx_pop, ni_active, ni_cs_level;
    wire [7:0]  ni_tx_data;
    wire [1:0]  ni_cs_sel;
    wire [15:0] pkts_in, pkts_out;
    wire        s_tx_full, s_rx_empty, s_busy;
    wire [31:0] spi_rx_data;

    ni_spi dut (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(en), .my_id(MY_ID),
        .in_valid(req_valid), .in_flit(req_flit), .in_credit(req_credit),
        .out_valid(rep_valid), .out_flit(rep_flit), .out_credit(rep_credit),
        .pkts_in(pkts_in), .pkts_out(pkts_out),
        .tx_push(ni_tx_push), .tx_data(ni_tx_data), .tx_full(s_tx_full),
        .rx_pop(ni_rx_pop), .rx_data(spi_rx_data[7:0]), .rx_empty(s_rx_empty),
        .busy(s_busy), .active(ni_active), .cs_level(ni_cs_level), .cs_sel(ni_cs_sel));

    // ------------------------------------------------------------------
    // engine sharing, as in ssc2_core.v (the CPU only acts in test 8)
    // ------------------------------------------------------------------
    reg         cpu_push = 1'b0, cpu_pop = 1'b0;
    reg  [31:0] cpu_data = 32'd0;
    reg         cpu_en = 1'b0, cpu_slave = 1'b1, cpu_manual = 1'b0, cpu_level = 1'b0;
    reg  [4:0]  cpu_len_m1 = 5'd15;
    reg  [1:0]  cpu_cs_sel = 2'd3;

    wire        s_tx_push   = cpu_push | ni_tx_push;
    wire [31:0] s_tx_data   = cpu_push ? cpu_data : {24'd0, ni_tx_data};
    wire        s_rx_pop    = cpu_pop | ni_rx_pop;
    wire        s_en        = ni_active | cpu_en;
    wire        s_slave     = ni_active ? 1'b0 : cpu_slave;
    wire [4:0]  s_len_m1    = ni_active ? 5'd7 : cpu_len_m1;
    wire        s_cs_manual = ni_active | cpu_manual;
    wire        s_cs_level  = ni_active ? ni_cs_level : cpu_level;
    wire [1:0]  s_cs_sel    = ni_active ? ni_cs_sel   : cpu_cs_sel;

    wire        sclk, mosi, miso;
    wire [3:0]  cs_n;
    wire [4:0]  s_tx_count, s_rx_count;
    wire        s_ev_rx_ovf, s_ev_tx_ovf;

    ssc_spi u_spi (
        .clk(clk), .rst_n(rst_n),
        .en(s_en), .cpol(1'b0), .cpha(1'b0), .lsb_first(1'b0),
        .len_m1(s_len_m1), .cs_sel(s_cs_sel), .cs_manual(s_cs_manual),
        .cs_level(s_cs_level), .slave_mode(s_slave), .loopback(1'b0),
        .clk_div(16'd4),                                   // 10 MHz SCLK
        .tx_flush(1'b0), .rx_flush(1'b0), .abort(1'b0),
        .tx_push(s_tx_push), .tx_data(s_tx_data),
        .rx_pop(s_rx_pop), .rx_data(spi_rx_data),
        .tx_empty(), .tx_full(s_tx_full),
        .rx_empty(s_rx_empty), .rx_full(),
        .tx_count(s_tx_count), .rx_count(s_rx_count), .busy(s_busy),
        .ev_done(), .ev_rx_ovf(s_ev_rx_ovf), .ev_tx_ovf(s_ev_tx_ovf),
        .sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n),
        .s_sclk(1'b0), .s_mosi(1'b0), .s_cs_n(1'b1),
        .s_miso(), .s_miso_oe());

    pullup (miso);
    tb_spi_flash flash (.cs_n(cs_n[0]), .sclk(sclk), .mosi(mosi), .miso(miso));
    tb_spi_slave gslv  (.cs_n(cs_n[1]), .sclk(sclk), .mosi(mosi), .miso(miso));

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
    integer stall_viol = 0, stall_busy_cycles = 0;
    integer push_cnt = 0, act_rises = 0, ovf_bad = 0, max_infl = 0;
    integer rep_early = 0;
    reg     act_q = 1'b0;
    always @(posedge clk) if (rst_n) begin
        if (stall && (ni_tx_push || ni_rx_pop)) stall_viol = stall_viol + 1;
        if (stall && ni_active) stall_busy_cycles = stall_busy_cycles + 1;
        if (ni_tx_push) push_cnt = push_cnt + 1;
        if (ni_active && !act_q) act_rises = act_rises + 1;
        act_q = ni_active;
        if (s_ev_rx_ovf || s_ev_tx_ovf) ovf_bad = ovf_bad + 1;
        if (s_tx_count + s_rx_count > 16) ovf_bad = ovf_bad + 1;
        if (dut.ns - dut.nr > max_infl) max_infl = dut.ns - dut.nr;
        // a reply head flit may only leave after CS is released
        if (rep_valid && rep_flit[33:32] != 2'b01 && rep_flit[33:32] != 2'b10 &&
            (cs_n != 4'hF || ni_active))
            rep_early = rep_early + 1;
    end

    integer cs0_falls = 0, cs1_falls = 0, sclk_edges = 0, sclk_no_cs = 0;
    realtime cs_rise_t = 0.0;
    realtime min_gap = 1.0e9;
    // (gap and hold are only measured on frames made by the NI)
    always @(negedge cs_n[0]) begin
        cs0_falls = cs0_falls + 1;
        if (ni_active && $realtime - cs_rise_t < min_gap) min_gap = $realtime - cs_rise_t;
    end
    always @(negedge cs_n[1]) begin
        cs1_falls = cs1_falls + 1;
        if (ni_active && $realtime - cs_rise_t < min_gap) min_gap = $realtime - cs_rise_t;
    end
    // CS hold: from the last SCLK edge to CS going high again.  The engine
    // needs half an SCLK period (50 ns here) after the last edge; the NI waits
    // for !busy, so the hold is at least a whole period.
    realtime sclk_t = 0.0;
    realtime min_hold = 1.0e9;
    always @(sclk) sclk_t = $realtime;
    always @(posedge cs_n[0]) begin
        cs_rise_t = $realtime;
        if (ni_active && $realtime - sclk_t < min_hold) min_hold = $realtime - sclk_t;
    end
    always @(posedge cs_n[1]) begin
        cs_rise_t = $realtime;
        if (ni_active && $realtime - sclk_t < min_hold) min_hold = $realtime - sclk_t;
    end
    always @(posedge sclk) begin
        sclk_edges = sclk_edges + 1;
        if (cs_n === 4'hF) sclk_no_cs = sclk_no_cs + 1;
    end

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------
    reg [7:0] pay [0:63];             // payload of the next request
    reg [7:0] exp [0:63];             // expected reply payload
    reg [7:0] pdata [0:63];           // page-program data
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

    // the CPU writes SPI_TX: a 2-clock APB transfer, strobe in the access
    // phase, stall (= psel) high in both clocks
    task cpu_tx_push(input [31:0] d);
        begin
            @(negedge clk); cpu_stall = 1'b1;
            @(negedge clk); cpu_push  = 1'b1; cpu_data = d;
            @(negedge clk); cpu_push  = 1'b0; cpu_stall = 1'b0;
        end
    endtask

    task wait_replies(input integer n);
        integer t;
        begin
            t = 0;
            while (nrp < n && t < 300000) begin @(posedge clk); t = t + 1; end
            if (nrp < n) $display("  (time-out waiting for reply %0d)", n);
        end
    endtask

    // NI idle, its input buffer empty and every credit back at the requester
    task wait_idle;
        integer t, q;
        begin
            t = 0; q = 0;
            while (q < 20 && t < 300000) begin
                @(posedge clk); t = t + 1;
                if (dut.st == 4'd0 && dut.u_rx.count == 0 && u_req.cred == 4 &&
                    s_hdr_ready && !ni_active) q = q + 1;
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
        input             pr;
        input [5:0]       len;
        input [8*72-1:0]  what;
        integer k;
        reg     ok;
        begin
            ok = (i < nrp) && (rp_type[i] == T_RESP) && (rp_dest[i] == dest) &&
                 (rp_src[i] == MY_ID) && (rp_tag[i] == tag) && (rp_arg[i] == arg) &&
                 (rp_prio[i] == pr) && (rp_len[i] == len);
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

    // ------------------------------------------------------------------
    // test sequence
    // ------------------------------------------------------------------
    integer i, k, n0, c0, c1, e0, p0, a0, polls, sent0;
    reg [7:0] stat;
    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            pay[i] = 8'd0;
            exp[i] = 8'd0;
            pdata[i] = $random(seed);
        end
        #33 rst_n = 1'b1;
        stall_mode = 1;
        repeat (5) @(posedge clk);

        // ---- 1. flash ID (slide 17) ----
        $display("[1] XFER_REQ cs0 [01][03][9F] -> XFER_RESP [00][20 BA 19]");
        c0 = cs0_falls; e0 = sclk_edges;
        pay[0] = 8'h01; pay[1] = 8'h03; pay[2] = 8'h9F;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'h5A, 8'h00);
        wait_replies(1);
        exp[0] = 8'h00; exp[1] = 8'h20; exp[2] = 8'hBA; exp[3] = 8'h19;
        show_reply(0);
        check_reply(0, 3'd3, 8'h5A, 8'h00, 1'b0, 6'd4, "1: reply [00][20 BA 19] to node 3, tag 5A");
        check(cs0_falls - c0 == 1, "1: exactly one CS0-low window");
        check(sclk_edges - e0 == 32, "1: 4 bytes = 32 SCLK edges");
        check(cs_n == 4'hF && !ni_active, "1: CS released and engine handed back");

        // ---- 2. write enable as a DATA packet ----
        $display("[2] DATA [06] (write enable): one CS frame, no reply");
        c0 = cs0_falls; e0 = sclk_edges; n0 = nrp;
        pay[0] = 8'h06;
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd1, 8'h01, 8'h00);
        wait_idle;
        check(flash.wel === 1'b1, "2: flash write-enable latch set");
        check(cs0_falls - c0 == 1 && sclk_edges - e0 == 8, "2: one CS frame of 8 clocks");
        check(nrp == n0, "2: DATA gets no reply");

        // ---- 3. page program, 44 bytes in one frame ----
        $display("[3] DATA page program: 02 00 00 10 + 40 bytes = 44 bytes, one CS frame");
        c0 = cs0_falls; e0 = sclk_edges; max_infl = 0;
        pay[0] = 8'h02; pay[1] = 8'h00; pay[2] = 8'h00; pay[3] = 8'h10;
        for (i = 0; i < 40; i = i + 1) pay[4 + i] = pdata[i];
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd44, 8'h02, 8'h00);
        wait_idle;
        check(cs0_falls - c0 == 1, "3: 44 bytes in exactly one CS-low window");
        check(sclk_edges - e0 == 44 * 8, "3: 352 SCLK edges");
        check(flash.pp_n == 40 && flash.wip === 1'b1, "3: flash got 40 data bytes, programming");
        check(max_infl == 16, "3: 16 bytes in flight reached (and not more)");
        check(s_rx_empty, "3: received bytes were thrown away");

        // ---- 4. poll the status register ----
        $display("[4] XFER_REQ [01][01][05] status polling until the flash is ready");
        polls = 0;
        stat  = 8'h01;
        while (stat[0] && polls < 100) begin
            n0 = nrp;
            pay[0] = 8'h01; pay[1] = 8'h01; pay[2] = 8'h05;
            send_pkt(T_XFER, 3'd1, 1'b1, 6'd3, 8'h40 + polls, 8'h00);
            wait_replies(n0 + 1);
            stat  = rp_b[n0*64 + 1];
            if (polls == 0) begin
                exp[0] = 8'h00; exp[1] = 8'h03;          // WIP and WEL
                check_reply(n0, 3'd1, 8'h40, 8'h00, 1'b1, 6'd2,
                            "4: first poll [00][03] (busy), prio copied");
            end
            polls = polls + 1;
            repeat (500) @(posedge clk);
        end
        $display("    flash ready after %0d polls, status %h", polls, stat);
        check(stat == 8'h00 && polls > 1, "4: flash became ready (WIP = WEL = 0)");

        // ---- 5. long read ----
        $display("[5] long read: wlen 4 + rlen 40 (44 bytes, more than 16 in flight)");
        c0 = cs0_falls; e0 = sclk_edges; max_infl = 0; n0 = nrp;
        pay[0] = 8'd4; pay[1] = 8'd40; pay[2] = 8'h03; pay[3] = 8'h00; pay[4] = 8'h00;
        pay[5] = 8'h10;
        send_pkt(T_XFER, 3'd0, 1'b0, 6'd6, 8'hA5, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00;
        for (i = 0; i < 40; i = i + 1) exp[1 + i] = pdata[i];
        show_reply(n0);
        check_reply(n0, 3'd0, 8'hA5, 8'h00, 1'b0, 6'd41, "5: 41-byte reply = [00] + the 40 programmed bytes");
        check(cs0_falls - c0 == 1, "5: one CS-low window");
        check(sclk_edges - e0 == 44 * 8, "5: 352 SCLK edges");
        check(max_infl == 16, "5: at most 16 bytes in flight (limit reached)");

        // ---- 6. longest read ----
        $display("[6] longest read: rlen 60 -> 61-byte reply");
        n0 = nrp;
        pay[1] = 8'd60;
        send_pkt(T_XFER, 3'd2, 1'b0, 6'd6, 8'hA6, 8'h00);
        wait_replies(n0 + 1);
        for (i = 40; i < 60; i = i + 1) exp[1 + i] = 8'hFF;   // erased bytes
        check_reply(n0, 3'd2, 8'hA6, 8'h00, 1'b0, 6'd61, "6: 61-byte reply, data then erased FF");

        // ---- 7. DATA to chip select 1, two packets back to back ----
        $display("[7] DATA to CS1 (generic slave): 20 bytes and 8 bytes back to back");
        stall_mode = 0;                       // no stall: the shortest CS-high gap
        c0 = cs0_falls; c1 = cs1_falls; k = gslv.ngot;
        for (i = 0; i < 28; i = i + 1) exp[i] = $random(seed);
        for (i = 0; i < 28; i = i + 1) pay[i] = exp[i];
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd20, 8'h07, 8'h01);
        for (i = 0; i < 8; i = i + 1) pay[i] = exp[20 + i];
        send_pkt(T_DATA, 3'd1, 1'b0, 6'd8, 8'h08, 8'h01);
        wait_idle;
        stall_mode = 1;
        p0 = 1;
        for (i = 0; i < 28; i = i + 1)
            if (gslv.got[k + i] !== {24'd0, exp[i]}) p0 = 0;
        check(gslv.ngot - k == 28 && p0 == 1, "7: slave on CS1 received the 28 bytes in order");
        check(cs1_falls - c1 == 2 && cs0_falls == c0, "7: two CS1 windows, CS0 untouched");
        check(s_rx_empty, "7: the slave's answers were thrown away");

        // ---- 8. the CPU used the engine and left old bytes in the RX FIFO ----
        $display("[8] CPU sends 3 bytes to CS1 itself and leaves the answers in the RX FIFO");
        cpu_en = 1'b1; cpu_slave = 1'b0; cpu_len_m1 = 5'd7; cpu_cs_sel = 2'd1;
        k = gslv.ngot;
        cpu_tx_push(32'h11); cpu_tx_push(32'h22); cpu_tx_push(32'h33);
        i = 0;
        while ((s_busy || s_rx_count != 3) && i < 5000) begin @(posedge clk); i = i + 1; end
        cpu_en = 1'b0; cpu_slave = 1'b1; cpu_len_m1 = 5'd15; cpu_cs_sel = 2'd3;
        check(gslv.ngot - k == 3 && gslv.got[k] == 32'h11 && gslv.got[k+2] == 32'h33 &&
              s_rx_count == 3, "8: CPU path through the shared FIFOs works, 3 old bytes");
        n0 = nrp;
        pay[0] = 8'h01; pay[1] = 8'h03; pay[2] = 8'h9F;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'h88, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00; exp[1] = 8'h20; exp[2] = 8'hBA; exp[3] = 8'h19;
        check_reply(n0, 3'd3, 8'h88, 8'h00, 1'b0, 6'd4, "8: old bytes thrown away, reply still [00][20 BA 19]");
        check(s_rx_empty, "8: RX FIFO empty afterwards");

        // ---- 9. bad requests ----
        $display("[9] bad requests -> status 4, engine untouched");
        c0 = cs0_falls; c1 = cs1_falls; p0 = push_cnt; a0 = act_rises; n0 = nrp;
        exp[0] = 8'h04;
        pay[0] = 8'h05;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd1, 8'hB1, 8'h00);              // payload of 1
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd0, 8'hB2, 8'h00);              // no payload
        pay[0] = 8'd2; pay[1] = 8'd1; pay[2] = 8'h9F;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'hB3, 8'h00);              // wlen says 2, has 1
        pay[0] = 8'd1; pay[1] = 8'd61;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'hB4, 8'h00);              // rlen 61
        pay[0] = 8'd1; pay[1] = 8'd2; pay[2] = 8'h9F; pay[3] = 8'h00;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd4, 8'hB5, 8'h02);              // extra byte
        wait_replies(n0 + 5);
        check_reply(n0 + 0, 3'd3, 8'hB1, 8'h00, 1'b0, 6'd1, "9: payload of 1 byte -> [04]");
        check_reply(n0 + 1, 3'd3, 8'hB2, 8'h00, 1'b0, 6'd1, "9: empty payload -> [04]");
        check_reply(n0 + 2, 3'd3, 8'hB3, 8'h00, 1'b0, 6'd1, "9: wlen bigger than the payload -> [04]");
        check_reply(n0 + 3, 3'd3, 8'hB4, 8'h00, 1'b0, 6'd1, "9: rlen 61 -> [04]");
        check_reply(n0 + 4, 3'd3, 8'hB5, 8'h02, 1'b0, 6'd1, "9: payload longer than 2 + wlen -> [04]");
        check(cs0_falls == c0 && cs1_falls == c1 && push_cnt == p0 && act_rises == a0,
              "9: no CS, no push, active never raised");

        // ---- 10. other packet types ----
        $display("[10] RECORD / ALARM / reserved / stray XFER_RESP: consumed, no reply");
        n0 = nrp; p0 = push_cnt; c0 = cs0_falls;
        for (i = 0; i < 16; i = i + 1) pay[i] = i;
        send_pkt(3'd3, 3'd1, 1'b0, 6'd16, 8'hC3, 8'h00);
        send_pkt(3'd4, 3'd1, 1'b1, 6'd5,  8'hC4, 8'h00);
        send_pkt(3'd5, 3'd1, 1'b0, 6'd0,  8'hC5, 8'h00);
        send_pkt(3'd7, 3'd1, 1'b0, 6'd9,  8'hC7, 8'h00);
        send_pkt(T_RESP, 3'd1, 1'b0, 6'd3, 8'hC2, 8'h00);
        wait_idle;
        check(nrp == n0 && push_cnt == p0 && cs0_falls == c0, "10: all consumed, no reply, engine untouched");

        // ---- 11. en = 0 ----
        $display("[11] en = 0: packets are consumed and dropped");
        en = 1'b0;
        n0 = nrp; p0 = push_cnt; c0 = cs0_falls; a0 = act_rises;
        pay[0] = 8'h01; pay[1] = 8'h03; pay[2] = 8'h9F;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'hD1, 8'h00);
        for (i = 0; i < 30; i = i + 1) pay[i] = 8'h55;
        send_pkt(T_DATA, 3'd3, 1'b0, 6'd30, 8'hD2, 8'h00);
        send_pkt(3'd5, 3'd3, 1'b0, 6'd10, 8'hD3, 8'h00);
        pay[0] = 8'h01;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd1, 8'hD4, 8'h00);
        wait_idle;
        check(nrp == n0, "11: no reply while disabled");
        check(push_cnt == p0 && cs0_falls == c0 && act_rises == a0, "11: engine never touched");
        check(u_req.cred == 4 && dut.u_rx.count == 0, "11: every flit consumed, credits back");
        en = 1'b1;
        pay[0] = 8'h01; pay[1] = 8'h03; pay[2] = 8'h9F;
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'hD5, 8'h00);
        wait_replies(n0 + 1);
        exp[0] = 8'h00; exp[1] = 8'h20; exp[2] = 8'hBA; exp[3] = 8'h19;
        check_reply(n0, 3'd3, 8'hD5, 8'h00, 1'b0, 6'd4, "11: works again after en = 1");

        // ---- 12. heavy stall, back-to-back requests, reply back-pressure ----
        $display("[12] heavy stall + 3 back-to-back requests + blocked reply path");
        stall_mode = 2;
        stall_busy_cycles = 0;
        n0 = nrp; c0 = cs0_falls;
        rep_block = 1'b1;
        pay[0] = 8'h01; pay[1] = 8'h03; pay[2] = 8'h9F;
        send_pkt(T_XFER, 3'd1, 1'b0, 6'd3, 8'hE1, 8'h00);
        send_pkt(T_XFER, 3'd3, 1'b0, 6'd3, 8'hE2, 8'h00);
        send_pkt(T_XFER, 3'd0, 1'b1, 6'd3, 8'hE3, 8'h00);
        repeat (3000) @(posedge clk);
        check(nrp == n0, "12: nothing delivered while the reply path is blocked");
        rep_block = 1'b0;
        wait_replies(n0 + 3);
        check_reply(n0 + 0, 3'd1, 8'hE1, 8'h00, 1'b0, 6'd4, "12: reply 1 -> node 1, tag E1");
        check_reply(n0 + 1, 3'd3, 8'hE2, 8'h00, 1'b0, 6'd4, "12: reply 2 -> node 3, tag E2");
        check_reply(n0 + 2, 3'd0, 8'hE3, 8'h00, 1'b1, 6'd4, "12: reply 3 -> node 0, tag E3, prio 1");
        check(cs0_falls - c0 == 3, "12: three separate CS frames");
        check(stall_busy_cycles > 50, "12: stall really was high during transfers");
        stall_mode = 1;

        // ---- 13. counters and global monitors ----
        wait_idle;
        $display("[13] counters and monitors: pkts_in %0d (sent %0d), pkts_out %0d (replies %0d)",
                 pkts_in, sent, pkts_out, nrp);
        check(pkts_in == sent, "13: pkts_in counts every packet (also dropped ones)");
        check(pkts_out == nrp, "13: pkts_out counts every reply");
        check(stall_viol == 0, "no engine strobe while stall = 1");
        check(sclk_no_cs == 0, "no SCLK edge while every CS is high");
        check(ovf_bad == 0, "no FIFO overflow, TX + RX FIFO never above 16");
        check(rep_early == 0, "every reply left after CS was released");
        $display("    shortest CS-high gap between frames: %0.0f ns", min_gap);
        check(min_gap >= 80.0, "CS-high gap between frames >= 80 ns");
        $display("    shortest CS hold after the last SCLK edge: %0.0f ns", min_hold);
        check(min_hold >= 100.0, "CS released only after the engine is idle (hold >= 100 ns)");

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end

    // watchdog
    initial begin
        #20_000_000;
        $display("FAIL: watchdog - simulation took too long");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule

// =============================================================================
// tb_hub.v  -  unit test: sensor hub (node 1) on a one-link test "network"
// -----------------------------------------------------------------------------
// The hub's out link goes into a TB noc_pkt_rx (with random back-pressure);
// a TB noc_pkt_tx drives the hub's in link.  The TB plays every other node:
//   * every packet from the hub is logged (header, payload, the ms count and
//     time_us when its head flit left the hub)
//   * a responder model for nodes 3/4/5 answers XFER_REQ with XFER_RESP
//     (configurable delay and status, per-node data, optional junk packets
//     and wrong-tag responses before the real one)
//   * RECORD packets (node 2 by default) are checked field by field
// ssc2_timebase runs with US_PER_MS = 5, so one "ms" is 500 clocks.
//
//   A. reset values, register read/write, unused addresses
//   B. slide-18 task: every 100 ms I2C 0x4B write 00 read 2 -> RECORD with
//      time stamp, source 3, length 2, seq 1,2,3, data 0C 80
//   C. phase/period timing of a DATA task (no wait), W3 rewrite, phase 0
//   D. period 0 never runs by itself; RUN_NOW; HUB_STATUS while waiting;
//      RUN_NOW of a disabled task is ignored
//   E. several due tasks run one at a time in index order; HUB_CTRL.EN = 0
//      holds them; SEND_LAST payloads; EN = 0 lets a running task finish
//   F. flash logging: TRIG_RECORDS + ADDR_INC (with carry) + APPEND_REC, a
//      phase counted in records, and "records since the task last ran"
//   G. wrong tags and non-RESP packets are read and ignored
//   H. time-out -> ev_error + TMO, the late reply is ignored, the hub goes on
//   I. status NACK -> ev_error + ERR, no record
//   J. a 9-byte response keeps the first 8; HUB_REC_DEST change
//   L. ALARM type goes out with prio 1; the TASK_CFG prio bit
//   K. all counters, event pulses, tags, credits
// =============================================================================
`timescale 1ns / 1ps
module tb_hub;
    localparam USMS = 5;              // microseconds per "ms" in this test
    localparam CPM  = 100 * USMS;     // clocks per ms (500)
    localparam NP   = 160;            // packet log size
    localparam NQ   = 128;            // TB send queue size

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0;
    integer errors = 0;
    task check(input cond, input [8*80-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    integer seed;

    // ---------------- time base ----------------
    wire [31:0] time_us;
    wire        us_tick, ms_tick;
    ssc2_timebase #(.CLK_HZ(100_000_000), .US_PER_MS(USMS)) u_time (
        .clk(clk), .rst_n(rst_n), .time_us(time_us), .us_tick(us_tick), .ms_tick(ms_tick));

    // ---------------- DUT ----------------
    reg         reg_we = 1'b0, reg_re = 1'b0;
    reg  [7:0]  reg_addr = 8'd0;
    reg  [31:0] reg_wdata = 32'd0;
    wire [31:0] reg_rdata;
    wire        h_valid, h_credit;     // hub -> TB
    wire [33:0] h_flit;
    wire        t_valid, t_credit;     // TB -> hub
    wire [33:0] t_flit;
    wire        ev_error, ev_record;

    hub u_hub (
        .clk(clk), .rst_n(rst_n), .time_us(time_us), .ms_tick(ms_tick), .my_id(3'd1),
        .reg_we(reg_we), .reg_re(reg_re), .reg_addr(reg_addr),
        .reg_wdata(reg_wdata), .reg_rdata(reg_rdata),
        .in_valid(t_valid), .in_flit(t_flit), .in_credit(t_credit),
        .out_valid(h_valid), .out_flit(h_flit), .out_credit(h_credit),
        .ev_error(ev_error), .ev_record(ev_record));

    // ---------------- TB receiver (packets from the hub) ----------------
    wire        r_hv;
    wire [2:0]  r_type, r_dest, r_src;
    wire        r_prio;
    wire [5:0]  r_len;
    wire [7:0]  r_tag, r_arg;
    reg         r_hready = 1'b0;
    wire        r_bv;
    wire [7:0]  r_bd;
    wire        r_blast;
    reg         r_bready = 1'b0;

    noc_pkt_rx #(.DEPTH(4)) u_trx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(h_valid), .in_flit(h_flit), .in_credit(h_credit),
        .hdr_valid(r_hv), .ptype(r_type), .dest(r_dest), .src(r_src),
        .prio(r_prio), .len(r_len), .tag(r_tag), .arg(r_arg),
        .hdr_ready(r_hready),
        .b_valid(r_bv), .b_data(r_bd), .b_last(r_blast), .b_ready(r_bready));

    // ---------------- TB sender (packets to the hub) ----------------
    reg         s_start = 1'b0;
    wire        s_hready;
    reg  [2:0]  s_type = 3'd0, s_src = 3'd0;
    reg  [5:0]  s_len = 6'd0;
    reg  [7:0]  s_tag = 8'd0;
    reg         s_bvalid = 1'b0;
    reg  [7:0]  s_bdata = 8'd0;
    wire        s_bready;

    noc_pkt_tx #(.DEPTH(4)) u_ttx (
        .clk(clk), .rst_n(rst_n),
        .start(s_start), .hdr_ready(s_hready),
        .ptype(s_type), .dest(3'd1), .src(s_src), .prio(1'b0),
        .len(s_len), .tag(s_tag), .arg(8'd0),
        .b_valid(s_bvalid), .b_data(s_bdata), .b_ready(s_bready),
        .out_valid(t_valid), .out_flit(t_flit), .out_credit(t_credit));

    // ---------------- cycle / ms counters, link monitor, event counters ----------------
    integer cyc = 0;
    integer ms_count = 0;
    integer mon_n = 0;
    integer mon_ms  [0:NP-1];     // ms count when packet k's head flit left the hub
    integer mon_cyc [0:NP-1];
    reg [31:0] mon_tus [0:NP-1];  // time_us at that moment
    integer n_everr = 0, n_evrec = 0, bad_pulse = 0;
    integer everr_cyc = 0;
    reg     everr_d = 1'b0, evrec_d = 1'b0;

    always @(posedge clk) begin
        cyc = cyc + 1;
        if (ms_tick) ms_count = ms_count + 1;
        if (rst_n && h_valid && (h_flit[33:32] == 2'b00 || h_flit[33:32] == 2'b11)) begin
            mon_ms[mon_n]  = ms_count;
            mon_cyc[mon_n] = cyc;
            mon_tus[mon_n] = time_us;
            mon_n = mon_n + 1;
        end
        if (ev_error)  begin n_everr = n_everr + 1; everr_cyc = cyc; end
        if (ev_record) n_evrec = n_evrec + 1;
        if ((ev_error && everr_d) || (ev_record && evrec_d)) bad_pulse = bad_pulse + 1;
        everr_d = ev_error;
        evrec_d = ev_record;
    end

    // ---------------- TB send queue ----------------
    reg [2:0] q_type [0:NQ-1];
    reg [2:0] q_src  [0:NQ-1];
    reg [7:0] q_tag  [0:NQ-1];
    reg [5:0] q_len  [0:NQ-1];
    integer   q_time [0:NQ-1];    // earliest send cycle
    integer   q_sent [0:NQ-1];    // cycle the header was handed over
    reg [7:0] q_pay  [0:NQ*64-1];
    integer   q_wr = 0, q_rd = 0;

    task q_push(input [2:0] ty, input [2:0] s, input [7:0] tg, input [5:0] ln,
                input integer when);
        begin
            q_type[q_wr] = ty; q_src[q_wr] = s; q_tag[q_wr] = tg;
            q_len[q_wr]  = ln; q_time[q_wr] = when;
            q_wr = q_wr + 1;
        end
    endtask

    reg     s_phase = 1'b0;
    integer s_i = 0;
    always @(posedge clk) begin
        if (rst_n) begin
            if (!s_phase) begin
                if (s_start && s_hready) begin
                    s_start <= 1'b0;
                    q_sent[q_rd] = cyc;
                    if (q_len[q_rd] == 0) begin
                        q_rd = q_rd + 1;
                    end else begin
                        s_phase  <= 1'b1;
                        s_i       = 0;
                        s_bvalid <= 1'b1;
                        s_bdata  <= q_pay[q_rd*64];
                    end
                end else if (!s_start && q_rd != q_wr && cyc >= q_time[q_rd]) begin
                    s_start <= 1'b1;
                    s_type  <= q_type[q_rd];
                    s_src   <= q_src[q_rd];
                    s_tag   <= q_tag[q_rd];
                    s_len   <= q_len[q_rd];
                end
            end else if (s_bvalid && s_bready) begin
                if (s_i + 1 == q_len[q_rd]) begin
                    s_bvalid <= 1'b0;
                    s_phase  <= 1'b0;
                    q_rd = q_rd + 1;
                end else begin
                    s_i = s_i + 1;
                    s_bdata <= q_pay[q_rd*64 + s_i];
                end
            end
        end
    end

    // ---------------- packet log (everything the hub sends) ----------------
    reg [2:0] p_type [0:NP-1];
    reg [2:0] p_dest [0:NP-1];
    reg [2:0] p_src  [0:NP-1];
    reg       p_prio [0:NP-1];
    reg [5:0] p_len  [0:NP-1];
    reg [7:0] p_tag  [0:NP-1];
    reg [7:0] p_arg  [0:NP-1];
    reg [7:0] p_pay  [0:NP*64-1];
    integer   n_pk = 0;            // complete packets
    integer   nb = 0;
    reg       in_pk = 1'b0;
    integer   n_run = 0;           // requests seen (all non-RECORD packets)
    integer   bad_src = 0, bad_tag = 0, bad_len = 0;

    // ---------------- responder model (nodes 3, 4, 5) ----------------
    integer   rsp_delay  = 300;    // clocks from request to response
    reg [7:0] rsp_status = 8'd0;
    reg       rsp_junk   = 1'b0;   // send junk and wrong-tag packets first
    reg [7:0] rsp_mem [0:8*16-1];  // data returned by node n: rsp_mem[16n + i]
    integer   rsp_q_of [0:NP-1];   // queue entry of the real response to packet k
    integer   n_xreq = 0;

    task respond(input integer k);
        integer node, rl, i, b;
        begin
            node = p_dest[k];
            rl   = p_pay[k*64 + 1];
            n_xreq = n_xreq + 1;
            if (rsp_junk) begin
                // DATA packet with the right tag
                for (i = 0; i < 3; i = i + 1) q_pay[q_wr*64 + i] = 8'h11 * (i + 1);
                q_push(3'd0, node, p_tag[k], 6'd3, cyc + 20);
                // XFER_RESP with a wrong tag (other roll), status 0, data EE
                q_pay[q_wr*64] = 8'h00;
                for (i = 1; i <= rl; i = i + 1) q_pay[q_wr*64 + i] = 8'hEE;
                q_push(3'd2, node, p_tag[k] ^ 8'h08, 1 + rl, cyc + 30);
                // XFER_RESP with the tag of another task index, status 0, data DD
                q_pay[q_wr*64] = 8'h00;
                for (i = 1; i <= rl; i = i + 1) q_pay[q_wr*64 + i] = 8'hDD;
                q_push(3'd2, node, p_tag[k] ^ 8'h01, 1 + rl, cyc + 40);
                // RECORD-type packet with the right tag
                for (i = 0; i < 16; i = i + 1) q_pay[q_wr*64 + i] = 8'h77;
                q_push(3'd3, node, p_tag[k], 6'd16, cyc + 50);
                // XFER_REQ-type packet with the right tag
                q_pay[q_wr*64] = 8'h00; q_pay[q_wr*64 + 1] = 8'h00;
                q_push(3'd1, node, p_tag[k], 6'd2, cyc + 60);
                // reserved types 6 (header only) and 7 (5 bytes)
                q_push(3'd6, node, p_tag[k], 6'd0, cyc + 70);
                for (i = 0; i < 5; i = i + 1) q_pay[q_wr*64 + i] = 8'h66;
                q_push(3'd7, node, p_tag[k], 6'd5, cyc + 80);
            end
            // the real response
            q_pay[q_wr*64] = rsp_status;
            b = (rsp_status == 8'd0) ? rl : 0;
            for (i = 0; i < b; i = i + 1) q_pay[q_wr*64 + 1 + i] = rsp_mem[node*16 + i];
            rsp_q_of[k] = q_wr;
            q_push(3'd2, node, p_tag[k], 1 + b, cyc + rsp_delay);
        end
    endtask

    task pk_done;
        begin
            if (p_src[n_pk] != 3'd1) bad_src = bad_src + 1;
            if (p_type[n_pk] != 3'd3) begin
                // every run is one request: tag = {roll, task}, roll counts runs
                if (p_tag[n_pk][7:3] != n_run[4:0]) bad_tag = bad_tag + 1;
                n_run = n_run + 1;
            end
            if (p_type[n_pk] == 3'd1 && p_len[n_pk] != p_pay[n_pk*64] + 2)
                bad_len = bad_len + 1;
            if (p_type[n_pk] == 3'd1 && p_dest[n_pk] >= 3 && p_dest[n_pk] <= 5)
                respond(n_pk);
            n_pk = n_pk + 1;
        end
    endtask

    integer pi;
    always @(posedge clk) begin
        if (rst_n) begin
            if (!in_pk) begin
                if (r_hv && r_hready) begin
                    p_type[n_pk] = r_type; p_dest[n_pk] = r_dest; p_src[n_pk] = r_src;
                    p_prio[n_pk] = r_prio; p_len[n_pk]  = r_len;  p_tag[n_pk] = r_tag;
                    p_arg[n_pk]  = r_arg;
                    for (pi = 0; pi < 64; pi = pi + 1) p_pay[n_pk*64 + pi] = 8'h00;
                    if (r_len == 0) begin
                        pk_done;
                    end else begin
                        in_pk = 1'b1;
                        nb    = 0;
                    end
                end
            end else if (r_bv && r_bready) begin
                p_pay[n_pk*64 + nb] = r_bd;
                nb = nb + 1;
                if (r_blast) begin
                    in_pk = 1'b0;
                    pk_done;
                end
            end
            r_hready <= ($random(seed) % 3) != 0;      // random back-pressure
            r_bready <= ($random(seed) % 3) != 0;
        end
    end

    // ---------------- helpers ----------------
    reg [31:0] rd_val;

    task wr(input [7:0] a, input [31:0] d);
        begin
            @(negedge clk);
            reg_addr = a; reg_wdata = d; reg_we = 1'b1;
            @(negedge clk);
            reg_we = 1'b0;
        end
    endtask

    task rd(input [7:0] a);
        begin
            @(negedge clk);
            reg_addr = a; reg_re = 1'b1;
            #1 rd_val = reg_rdata;
            @(negedge clk);
            reg_re = 1'b0;
        end
    endtask

    // wait until n packets are logged (or max_ms ms pass)
    task wait_pk(input integer n, input integer max_ms);
        integer c;
        begin
            c = 0;
            while (n_pk < n && c < max_ms * CPM) begin @(posedge clk); c = c + 1; end
        end
    endtask

    task wait_ms(input integer n);
        integer m;
        begin
            m = ms_count;
            while (ms_count < m + n) @(posedge clk);
        end
    endtask

    // wait until the hub is idle, nothing is due and the TB queue is empty
    task wait_idle(input integer max_ms);
        integer c;
        reg     done;
        begin
            c = 0; done = 1'b0;
            while (!done && c < max_ms * CPM) begin
                rd(8'h04);
                if (!rd_val[0]) begin
                    rd(8'h00);
                    if (rd_val[15:8] == 8'd0 && q_rd == q_wr && !in_pk &&
                        mon_n == n_pk && u_hub.u_tx.hdr_ready) done = 1'b1;
                end
                c = c + 2;
            end
            repeat (10) @(posedge clk);
        end
    endtask

    // 8 payload bytes of packet k from offset o, little-endian
    function [63:0] pay8(input integer k, input integer o);
        integer i;
        begin
            pay8 = 64'd0;
            for (i = 0; i < 8; i = i + 1) pay8[8*i +: 8] = p_pay[k*64 + o + i];
        end
    endfunction

    // payload bytes of packet k from offset o are a copy of packet j's payload
    function same_bytes(input integer k, input integer o, input integer j, input integer n);
        integer i;
        begin
            same_bytes = 1'b1;
            for (i = 0; i < n; i = i + 1)
                if (p_pay[k*64 + o + i] !== p_pay[j*64 + i]) same_bytes = 1'b0;
        end
    endfunction

    // check RECORD packet k made from request packet rq
    task check_rec(input integer k, input integer rq, input [2:0] dst, input [2:0] t,
                   input [7:0] code, input [7:0] n, input [15:0] sq, input [63:0] data);
        reg [31:0] ts;
        begin
            ts = {p_pay[k*64+3], p_pay[k*64+2], p_pay[k*64+1], p_pay[k*64]};
            check(p_type[k] == 3'd3 && p_dest[k] == dst && p_src[k] == 3'd1 &&
                  p_len[k] == 6'd16 && p_prio[k] == 1'b0,
                  "record: header (type 3, dest, src 1, len 16)");
            check(p_tag[k] == p_tag[rq] && p_arg[k] == {5'd0, t},
                  "record: tag of the run, arg = task");
            check(ts == mon_tus[rq] || ts + 32'd1 == mon_tus[rq],
                  "record: time stamp = time_us at task start");
            check(p_pay[k*64+4] == code, "record: source code");
            check(p_pay[k*64+5] == n, "record: length");
            check({p_pay[k*64+7], p_pay[k*64+6]} == sq, "record: sequence number");
            check(pay8(k, 8) == data, "record: data padded with 0");
            if (pay8(k, 8) != data || {p_pay[k*64+7], p_pay[k*64+6]} != sq)
                $display("  record %0d: seq %h data %h (expected %h %h)", k,
                         {p_pay[k*64+7], p_pay[k*64+6]}, pay8(k, 8), sq, data);
        end
    endtask

    // ---------------- watchdog ----------------
    initial begin
        #(1000 * CPM * 10);           // 1000 ms of simulated time
        $display("FAIL: watchdog - the test did not finish");
        errors = errors + 1;
        $display("%0d OF %0d CHECKS FAILED", errors, checks + 1);
        $finish;
    end

    // ---------------- test sequence ----------------
    integer i, k, k0, m0, m1, n0, ts0, ts1;
    integer exp_req, exp_ok, exp_err, exp_tmo, exp_rec;
    reg     ok;

    initial begin
        seed = 7;
        for (i = 0; i < 8*16; i = i + 1) rsp_mem[i] = 8'h00;
        // node 3 (UART): 5A 11 22 33 ...   node 4 (SPI flash ID): 20 BA 19
        // node 5 (TMP2 temperature): 0C 80
        rsp_mem[3*16] = 8'h5A;
        for (i = 1; i < 16; i = i + 1) rsp_mem[3*16 + i] = 8'h11 * i;
        rsp_mem[4*16] = 8'h20; rsp_mem[4*16+1] = 8'hBA; rsp_mem[4*16+2] = 8'h19;
        rsp_mem[5*16] = 8'h0C; rsp_mem[5*16+1] = 8'h80;
        exp_req = 0; exp_ok = 0; exp_err = 0; exp_tmo = 0; exp_rec = 0;

        #33 rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ================= A: reset values and registers =================
        rd(8'h00); check(rd_val == 32'd0, "A: HUB_CTRL resets to 0");
        rd(8'h04); check(rd_val == 32'd0, "A: HUB_STATUS idle");
        rd(8'h08); check(rd_val == 32'd2, "A: HUB_REC_DEST resets to 2");
        rd(8'h0C); check(rd_val == 32'd50, "A: HUB_TIMEOUT resets to 50");
        rd(8'h10); check(rd_val == 32'd1, "A: HUB_SEQ resets to 1");
        ok = 1'b1;
        for (i = 8'h14; i <= 8'h30; i = i + 4) begin rd(i); if (rd_val != 0) ok = 1'b0; end
        check(ok, "A: HUB_LAST/LO/HI and counters reset to 0");
        ok = 1'b1;
        for (i = 8'h40; i < 8'hC0; i = i + 4) begin rd(i); if (rd_val != 0) ok = 1'b0; end
        check(ok, "A: task table resets to 0");
        wr(8'hB0, 32'hFFFF_FFFE);           // task 7, EN stays 0
        wr(8'hB4, 32'h1234_5678);
        wr(8'hB8, 32'h9ABC_DEF0);
        wr(8'hBC, 32'h0005_0003);
        rd(8'hB0); check(rd_val == 32'h1FFF_FFFE, "A: W0 keeps bits [28:0]");
        rd(8'hB4); check(rd_val == 32'h1234_5678, "A: W1 read back");
        rd(8'hB8); check(rd_val == 32'h9ABC_DEF0, "A: W2 read back");
        rd(8'hBC); check(rd_val == 32'h0005_0003, "A: W3 read back");
        rd(8'h40); check(rd_val == 32'd0, "A: other tasks untouched");
        wr(8'hB0, 0); wr(8'hB4, 0); wr(8'hB8, 0); wr(8'hBC, 0);
        wr(8'h0C, 32'hFFFF_0040); rd(8'h0C); check(rd_val == 32'h40, "A: HUB_TIMEOUT 16 bits");
        wr(8'h0C, 32'd50);
        wr(8'h10, 32'h55); wr(8'h20, 32'h55);  // read-only: no effect
        rd(8'h10); check(rd_val == 32'd1, "A: HUB_SEQ is read-only");
        ok = 1'b1;
        rd(8'h34); if (rd_val != 0) ok = 1'b0;
        rd(8'h38); if (rd_val != 0) ok = 1'b0;
        rd(8'h3C); if (rd_val != 0) ok = 1'b0;
        rd(8'hC0); if (rd_val != 0) ok = 1'b0;
        rd(8'hFC); if (rd_val != 0) ok = 1'b0;
        check(ok, "A: unused addresses read 0");
        repeat (20) @(posedge clk);
        check(n_pk == 0 && mon_n == 0, "A: nothing sent while idle");

        // ================= B: slide-18 task =================
        // T0: EN, dest 5 (I2C), XFER_REQ, arg 0x4B, wlen 1, rlen 2, RECORD
        wr(8'h44, 32'h0000_0000);           // write byte 0 = 0x00 (temperature register)
        wr(8'h4C, 32'h0000_0064);           // period 100 ms, phase 0
        wr(8'h00, 32'h0000_0001);           // HUB_CTRL.EN
        wr(8'h40, 32'h0121_4B1B);
        m0 = ms_count;
        wait_pk(6, 330);
        check(n_pk == 6, "B: 3 requests + 3 records in 300 ms");
        for (i = 0; i < 3; i = i + 1) begin
            k = 2 * i;
            check(p_type[k] == 3'd1 && p_dest[k] == 3'd5 && p_src[k] == 3'd1 &&
                  p_arg[k] == 8'h4B && p_len[k] == 6'd3 && p_prio[k] == 1'b0,
                  "B: XFER_REQ header (dest 5, arg 4B, len 3)");
            check(p_pay[k*64] == 8'd1 && p_pay[k*64+1] == 8'd2 && p_pay[k*64+2] == 8'd0,
                  "B: payload [01][02][00]");
            check(p_tag[k] == {i[4:0], 3'd0}, "B: tag = {roll, task 0}");
            check(mon_ms[k] == m0 + 100 * (i + 1), "B: runs every 100 ms (phase 0 = one period)");
            check_rec(k + 1, k, 3'd2, 3'd0, 8'd3, 8'd2, i + 1, 64'h0000_0000_0000_800C);
        end
        ts0 = {p_pay[1*64+3], p_pay[1*64+2], p_pay[1*64+1], p_pay[1*64]};
        ts1 = {p_pay[3*64+3], p_pay[3*64+2], p_pay[3*64+1], p_pay[3*64]};
        check(ts1 - ts0 == 100 * USMS, "B: time stamps 100 ms apart");
        check(n_evrec == 3 && n_everr == 0, "B: 3 ev_record pulses, no ev_error");
        wr(8'h40, 32'h0121_4B1A);           // T0 off
        exp_req = exp_req + 3; exp_ok = exp_ok + 3; exp_rec = exp_rec + 3;
        wait_idle(5);
        rd(8'h10); check(rd_val == 32'd4, "B: HUB_SEQ = 4");
        rd(8'h14); check(rd_val == 32'h0000_0002, "B: HUB_LAST = task 0, status 0, length 2");
        rd(8'h18); check(rd_val == 32'h0000_800C, "B: HUB_LAST_LO = 0C 80");
        rd(8'h1C); check(rd_val == 32'h0, "B: HUB_LAST_HI = 0");
        rd(8'h20); check(rd_val == 3, "B: REQ = 3");
        rd(8'h24); check(rd_val == 3, "B: RESP_OK = 3");
        rd(8'h30); check(rd_val == 3, "B: REC = 3");

        // ================= C: phase / period, DATA task =================
        // T1: EN, dest 0 (SE), DATA, arg 01, wlen 2 (AA BB), phase 3, period 7
        n0 = n_pk;
        wr(8'h54, 32'h0000_BBAA);
        wr(8'h5C, 32'h0003_0007);
        wr(8'h50, 32'h0002_0101);
        m0 = ms_count;
        wait_pk(n0 + 1, 10);
        repeat (20) @(posedge clk);
        rd(8'h04); check(rd_val[0] == 1'b0 && rd_val[8] == 1'b0, "C: DATA task does not wait");
        wait_pk(n0 + 3, 30);
        check(n_pk == n0 + 3, "C: 3 DATA packets");
        check(mon_ms[n0] == m0 + 3, "C: first run after phase (3 ms)");
        check(mon_ms[n0+1] == m0 + 10 && mon_ms[n0+2] == m0 + 17, "C: then every period (7 ms)");
        ok = 1'b1;
        for (k = n0; k < n0 + 3; k = k + 1)
            if (p_type[k] != 3'd0 || p_dest[k] != 3'd0 || p_len[k] != 6'd2 || p_arg[k] != 8'h01 ||
                p_tag[k][2:0] != 3'd1 || p_pay[k*64] != 8'hAA || p_pay[k*64+1] != 8'hBB)
                ok = 1'b0;
        check(ok, "C: DATA packet = AA BB without length bytes");
        wait_ms(1);
        wr(8'h5C, 32'h0000_0004);           // W3 rewrite: phase 0, period 4
        m1 = ms_count;
        wait_pk(n0 + 5, 12);
        wr(8'h50, 32'h0002_0100);           // T1 off
        check(n_pk == n0 + 5, "C: 2 more DATA packets after the W3 rewrite");
        check(mon_ms[n0+3] == m1 + 4 && mon_ms[n0+4] == m1 + 8, "C: W3 write restarts the countdown");
        exp_req = exp_req + 5;
        wait_ms(10);
        check(n_pk == n0 + 5, "C: no run after EN = 0");
        rd(8'h24); check(rd_val == exp_ok, "C: DATA runs do not count RESP_OK");
        rd(8'h14); check(rd_val == 32'h0000_0002, "C: DATA runs leave HUB_LAST alone");

        // ================= D: period 0, RUN_NOW =================
        // T2: EN, dest 4 (SPI), XFER_REQ, arg 01, wlen 1 (9F), rlen 3, RECORD
        n0 = n_pk;
        wr(8'h64, 32'h0000_009F);
        wr(8'h6C, 32'h0000_0000);
        wr(8'h60, 32'h0131_0119);
        wait_ms(30);
        check(n_pk == n0, "D: period 0 task never runs by itself");
        wr(8'h00, 32'h0000_0201);           // RUN_NOW task 1 (disabled)
        wait_ms(3);
        check(n_pk == n0, "D: RUN_NOW of a disabled task is ignored");
        rsp_delay = 2000;
        wr(8'h00, 32'h0000_0401);           // RUN_NOW task 2
        wait_pk(n0 + 1, 5);
        repeat (20) @(posedge clk);
        rd(8'h04); check(rd_val == 32'h0000_0105, "D: HUB_STATUS busy, task 2, waiting");
        rd(8'h00); check(rd_val == 32'h0000_0001, "D: RUN_NOW bit cleared when the task starts");
        wait_pk(n0 + 2, 10);
        check(p_type[n0] == 3'd1 && p_dest[n0] == 3'd4 && p_arg[n0] == 8'h01 && p_len[n0] == 6'd3 &&
              p_pay[n0*64] == 8'd1 && p_pay[n0*64+1] == 8'd3 && p_pay[n0*64+2] == 8'h9F,
              "D: XFER_REQ to SPI [01][03][9F]");
        check_rec(n0 + 1, n0, 3'd2, 3'd2, 8'd2, 8'd3, exp_rec + 1, 64'h0000_0000_0019_BA20);
        exp_req = exp_req + 1; exp_ok = exp_ok + 1; exp_rec = exp_rec + 1;
        wait_idle(5);
        rd(8'h14); check(rd_val == 32'h0002_0003, "D: HUB_LAST = task 2, status 0, length 3");
        rd(8'h18); check(rd_val == 32'h0019_BA20, "D: HUB_LAST_LO = 20 BA 19");
        rd(8'h10); check(rd_val == 32'd5, "D: HUB_SEQ = 5");
        rsp_delay = 300;

        // ================= E: several due tasks, index order =================
        // T3: XFER_REQ to UART (3), wlen 2 (55 66), rlen 1, SEND_LAST
        // T5: XFER_REQ to I2C (5) 0x4B, wlen 1 (00), rlen 2, RECORD
        // T6: DATA to SE (0), wlen 0, SEND_LAST
        n0 = n_pk;
        wr(8'h74, 32'h0000_6655); wr(8'h7C, 32'h0); wr(8'h70, 32'h0212_0017);
        wr(8'h94, 32'h0000_0000); wr(8'h9C, 32'h0); wr(8'h90, 32'h0121_4B1B);
        wr(8'hA4, 32'h0000_0000); wr(8'hAC, 32'h0); wr(8'hA0, 32'h0200_0001);
        wr(8'h00, 32'h0000_6800);           // EN = 0, RUN_NOW 3, 5, 6
        wait_ms(3);
        check(n_pk == n0 && mon_n == n0, "E: HUB_CTRL.EN = 0 starts nothing");
        rd(8'h00); check(rd_val == 32'h0000_6800, "E: HUB_CTRL shows tasks 3, 5, 6 due");
        wr(8'h00, 32'h0000_0400);           // RUN_NOW task 2 as well ...
        rd(8'h00); check(rd_val == 32'h0000_6C00, "E: task 2 due too");
        wr(8'h60, 32'h0131_0118);           // ... then switch task 2 off and on
        wr(8'h60, 32'h0131_0119);
        rd(8'h00); check(rd_val == 32'h0000_6800, "E: EN = 0 on a task clears its due flag");
        rsp_delay = 600;
        wr(8'h00, 32'h0000_0001);           // EN = 1
        wait_pk(n0 + 1, 3);
        repeat (20) @(posedge clk);
        rd(8'h04); check(rd_val == 32'h0000_0107, "E: task 3 running and waiting");
        rd(8'h00); check(rd_val == 32'h0000_6001, "E: tasks 5 and 6 still due");
        wait_pk(n0 + 4, 10);
        check(n_pk == n0 + 4, "E: 4 packets (req 3, req 5, record, DATA 6)");
        check(p_tag[n0][2:0] == 3'd3 && p_tag[n0+1][2:0] == 3'd5 && p_type[n0+2] == 3'd3 &&
              p_tag[n0+3][2:0] == 3'd6, "E: tasks run in index order 3, 5, 6");
        check(q_sent[rsp_q_of[n0]] < mon_cyc[n0+1], "E: task 5 starts after task 3's reply");
        check(p_dest[n0] == 3'd3 && p_len[n0] == 6'd7 && p_pay[n0*64] == 8'd5 &&
              p_pay[n0*64+1] == 8'd1 && p_pay[n0*64+2] == 8'h55 && p_pay[n0*64+3] == 8'h66 &&
              p_pay[n0*64+4] == 8'h20 && p_pay[n0*64+5] == 8'hBA && p_pay[n0*64+6] == 8'h19,
              "E: SEND_LAST request [05][01][55 66][20 BA 19]");
        check(p_dest[n0+1] == 3'd5 && p_len[n0+1] == 6'd3, "E: task 5 request");
        check_rec(n0 + 2, n0 + 1, 3'd2, 3'd5, 8'd3, 8'd2, exp_rec + 1, 64'h800C);
        check(p_type[n0+3] == 3'd0 && p_dest[n0+3] == 3'd0 && p_len[n0+3] == 6'd2 &&
              p_pay[(n0+3)*64] == 8'h0C && p_pay[(n0+3)*64+1] == 8'h80,
              "E: SEND_LAST DATA = last value 0C 80 (no length bytes)");
        exp_req = exp_req + 3; exp_ok = exp_ok + 2; exp_rec = exp_rec + 1;

        // E2: EN = 0 while a task runs: it finishes, the next does not start
        n0 = n_pk;
        wr(8'h00, 32'h0000_2001);           // RUN_NOW task 5
        wait_pk(n0 + 1, 3);
        wr(8'h00, 32'h0000_4000);           // EN = 0, RUN_NOW task 6
        wait_pk(n0 + 2, 5);
        wait_ms(3);
        check(n_pk == n0 + 2 && p_type[n0+1] == 3'd3, "E2: running task finishes with EN = 0");
        rd(8'h00); check(rd_val == 32'h0000_4000, "E2: task 6 waits for EN");
        wr(8'h00, 32'h0000_0001);
        wait_pk(n0 + 3, 3);
        check(n_pk == n0 + 3 && p_tag[n0+2][2:0] == 3'd6, "E2: task 6 runs after EN = 1");
        check({p_pay[(n0+1)*64+7], p_pay[(n0+1)*64+6]} == exp_rec + 1, "E2: record seq 6");
        exp_req = exp_req + 2; exp_ok = exp_ok + 1; exp_rec = exp_rec + 1;
        wait_idle(5);
        rsp_delay = 300;
        wr(8'h70, 0); wr(8'h90, 0); wr(8'hA0, 0);     // T3, T5, T6 off

        // ================= F: flash logging =================
        // T0: slide-18 task again, period 3 ms (makes records)
        // T4 "A": DATA to SPI, wlen 1 (06 = WREN), TRIG_RECORDS, period 2
        // T7 "B": XFER_REQ to SPI, wlen 4 (02 A2 A1 A0), rlen 0, ADDR_INC,
        //         APPEND_REC, TRIG_RECORDS, period 2.  Address starts at 0x01FE00.
        n0 = n_pk;
        wr(8'h84, 32'h0000_0006); wr(8'h8C, 32'h0000_0002); wr(8'h80, 32'h0801_0009);
        wr(8'hB4, 32'h00FE_0102); wr(8'hBC, 32'h0000_0002); wr(8'hB0, 32'h1C04_0019);
        wr(8'h4C, 32'h0000_0003); wr(8'h40, 32'h0121_4B1B);
        wait_pk(n0 + 18, 30);
        wr(8'h40, 32'h0121_4B1A);           // T0 off
        check(n_pk == n0 + 18, "F: 6 records, 3 x (WREN + page write)");
        for (i = 0; i < 3; i = i + 1) begin
            k = n0 + 6 * i;
            check(p_tag[k][2:0] == 3'd0 && p_type[k+1] == 3'd3 &&
                  p_tag[k+2][2:0] == 3'd0 && p_type[k+3] == 3'd3,
                  "F: two records before the triggered tasks");
            check(p_tag[k+4][2:0] == 3'd4 && p_tag[k+5][2:0] == 3'd7,
                  "F: TRIG_RECORDS tasks run after 2 records, in index order");
            check(p_type[k+4] == 3'd0 && p_dest[k+4] == 3'd4 && p_len[k+4] == 6'd1 &&
                  p_pay[(k+4)*64] == 8'h06, "F: task A sends WREN (06)");
            check(p_type[k+5] == 3'd1 && p_dest[k+5] == 3'd4 && p_len[k+5] == 6'd22 &&
                  p_pay[(k+5)*64] == 8'd20 && p_pay[(k+5)*64+1] == 8'd0 &&
                  p_pay[(k+5)*64+2] == 8'h02, "F: task B = [20][00][02 ...] + 16 bytes");
            check(same_bytes(k + 5, 6, k + 3, 16), "F: APPEND_REC = the last record sent");
        end
        check(p_pay[(n0+5)*64+3] == 8'h01 && p_pay[(n0+5)*64+4] == 8'hFE &&
              p_pay[(n0+5)*64+5] == 8'h00, "F: run 1 address 01 FE 00");
        check(p_pay[(n0+11)*64+3] == 8'h01 && p_pay[(n0+11)*64+4] == 8'hFF &&
              p_pay[(n0+11)*64+5] == 8'h00, "F: run 2 address 01 FF 00 (+256)");
        check(p_pay[(n0+17)*64+3] == 8'h02 && p_pay[(n0+17)*64+4] == 8'h00 &&
              p_pay[(n0+17)*64+5] == 8'h00, "F: run 3 address 02 00 00 (carry)");
        exp_req = exp_req + 12; exp_ok = exp_ok + 9; exp_rec = exp_rec + 6;
        wait_idle(5);
        rd(8'hB4); check(rd_val == 32'h0001_0202, "F: W1 of task B holds the next address 02 01 00");
        rd(8'h14); check(rd_val == 32'h0007_0002, "F: HUB_LAST = task 7, status 0, length 2");
        rd(8'h18); check(rd_val == 32'h0000_800C, "F: reply without data keeps the last value");
        wr(8'hB0, 32'h1C04_0018);           // T7 off

        // F2: TRIG_RECORDS: phase in records, and "records since the task last ran"
        n0 = n_pk;
        wr(8'h8C, 32'h0001_0002);           // T4: phase 1 record, then period 2
        wr(8'h00, 32'h0000_0401);           // T2 -> record: phase over, T4 runs
        wait_pk(n0 + 3, 5); wait_idle(5);
        check(n_pk == n0 + 3 && p_type[n0+1] == 3'd3 && p_tag[n0+2][2:0] == 3'd4,
              "F2: phase 1 -> T4 runs after the first record");
        wr(8'h00, 32'h0000_0401);           // T2 -> record (count 1 of 2)
        wait_pk(n0 + 5, 5); wait_idle(5);
        check(n_pk == n0 + 5, "F2: then the period (2 records) applies");
        wr(8'h00, 32'h0000_1001);           // RUN_NOW T4: runs, count starts again
        wait_pk(n0 + 6, 5); wait_idle(5);
        wr(8'h00, 32'h0000_0401);           // T2 -> record (count 1): T4 must not run
        wait_pk(n0 + 8, 5); wait_idle(5);
        check(n_pk == n0 + 8, "F2: one record after T4 ran does not trigger it");
        wr(8'h00, 32'h0000_0401);           // T2 -> record (count 2): T4 runs
        wait_pk(n0 + 11, 5); wait_idle(5);
        check(n_pk == n0 + 11 && p_tag[n0+5][2:0] == 3'd4 && p_tag[n0+10][2:0] == 3'd4 &&
              p_type[n0+9] == 3'd3, "F2: second record since the last run triggers T4");
        exp_req = exp_req + 7; exp_ok = exp_ok + 4; exp_rec = exp_rec + 4;
        wr(8'h80, 32'h0801_0008);           // T4 off

        // ================= G: wrong tags and junk are ignored =================
        n0 = n_pk;
        rsp_junk = 1'b1;
        rsp_delay = 2000;
        wr(8'h00, 32'h0000_0401);           // RUN_NOW T2
        wait_pk(n0 + 1, 3);
        rsp_junk = 1'b0;
        i = 0;
        while (q_rd < q_wr - 1 && i < 3000) begin @(posedge clk); i = i + 1; end
        repeat (50) @(posedge clk);
        check(q_rd == q_wr - 1, "G: 7 junk packets consumed by the hub");
        rd(8'h04); check(rd_val == 32'h0000_0105, "G: still waiting for the right tag");
        rd(8'h24); check(rd_val == exp_ok, "G: junk not counted as a response");
        check(n_pk == n0 + 1, "G: no record from a wrong-tag response");
        wait_pk(n0 + 2, 5);
        check_rec(n0 + 1, n0, 3'd2, 3'd2, 8'd2, 8'd3, exp_rec + 1, 64'h0019_BA20);
        exp_req = exp_req + 1; exp_ok = exp_ok + 1; exp_rec = exp_rec + 1;
        wait_idle(5);
        rd(8'h18); check(rd_val == 32'h0019_BA20, "G: last value from the right response");
        rsp_delay = 300;

        // ================= H: time-out =================
        n0 = n_pk;
        k0 = n_everr;
        wr(8'h0C, 32'd10);                  // HUB_TIMEOUT = 10 ms
        rsp_delay = 15 * CPM;               // reply after 15 ms: too late
        wr(8'h00, 32'h0000_0401);
        wait_pk(n0 + 1, 3);
        i = 0;
        while (n_everr == k0 && i < 20 * CPM) begin @(posedge clk); i = i + 1; end
        check(n_everr == k0 + 1, "H: ev_error on time-out");
        check(everr_cyc - mon_cyc[n0] >= 10 * CPM && everr_cyc - mon_cyc[n0] <= 11 * CPM + 20,
              "H: time-out after 10..11 ms");
        repeat (5) @(posedge clk);
        rd(8'h2C); check(rd_val == 32'd1, "H: TMO = 1");
        rd(8'h28); check(rd_val == 32'd0, "H: ERR unchanged");
        rd(8'h04); check(rd_val[0] == 1'b0, "H: hub idle after the time-out");
        rd(8'h14); check(rd_val == 32'h0002_FF03, "H: HUB_LAST status FF, task 2");
        wait_idle(10);                      // the late reply arrives
        check(n_pk == n0 + 1, "H: no record, late reply ignored");
        rd(8'h24); check(rd_val == exp_ok, "H: late reply not counted");
        rd(8'h10); check(rd_val == exp_rec + 1, "H: HUB_SEQ unchanged");
        exp_req = exp_req + 1; exp_tmo = exp_tmo + 1;
        wr(8'h0C, 32'd50);
        rsp_delay = 300;
        wr(8'h00, 32'h0000_0401);           // the hub goes on
        wait_pk(n0 + 3, 5);
        check_rec(n0 + 2, n0 + 1, 3'd2, 3'd2, 8'd2, 8'd3, exp_rec + 1, 64'h0019_BA20);
        exp_req = exp_req + 1; exp_ok = exp_ok + 1; exp_rec = exp_rec + 1;
        wait_idle(5);

        // ================= I: status NACK =================
        n0 = n_pk;
        k0 = n_everr;
        rsp_status = 8'd1;
        wr(8'h90, 32'h0121_4B1B);           // T5 on again (period 0)
        wr(8'h00, 32'h0000_2001);
        wait_pk(n0 + 1, 3);
        wait_idle(5);
        rsp_status = 8'd0;
        check(n_everr == k0 + 1, "I: ev_error on NACK");
        check(n_pk == n0 + 1, "I: no record after NACK");
        rd(8'h28); check(rd_val == 32'd1, "I: ERR = 1");
        rd(8'h14); check(rd_val == 32'h0005_0103, "I: HUB_LAST status 1, task 5, length kept");
        rd(8'h18); check(rd_val == 32'h0019_BA20, "I: last value kept");
        rd(8'h10); check(rd_val == exp_rec + 1, "I: HUB_SEQ unchanged");
        exp_req = exp_req + 1; exp_err = exp_err + 1;
        wr(8'h90, 0);

        // ================= J: long response, REC_DEST =================
        // T6: XFER_REQ to UART (3), wlen 0, rlen 9 (one byte more than kept), RECORD
        n0 = n_pk;
        wr(8'h08, 32'hFFFF_FFF7); rd(8'h08); check(rd_val == 32'd7, "J: HUB_REC_DEST = 7");
        wr(8'hA0, 32'h0190_0017);
        wr(8'h00, 32'h0000_4001);
        wait_pk(n0 + 2, 5);
        check(p_len[n0] == 6'd2 && p_pay[n0*64] == 8'd0 && p_pay[n0*64+1] == 8'd9,
              "J: request [00][09]");
        check_rec(n0 + 1, n0, 3'd7, 3'd6, 8'd1, 8'd8, exp_rec + 1, 64'h7766_5544_3322_115A);
        exp_req = exp_req + 1; exp_ok = exp_ok + 1; exp_rec = exp_rec + 1;
        wait_idle(5);
        rd(8'h14); check(rd_val == 32'h0006_0008, "J: HUB_LAST length 8 (first 8 kept)");
        rd(8'h18); check(rd_val == 32'h3322_115A, "J: HUB_LAST_LO");
        rd(8'h1C); check(rd_val == 32'h7766_5544, "J: HUB_LAST_HI");
        wr(8'h08, 32'd2);
        wr(8'hA0, 0);

        // ================= L: ALARM type and the prio bit =================
        // T1: ALARM (type 4) to node 0, arg 05, wlen 1 (A5), period 0
        n0 = n_pk;
        wr(8'h54, 32'h0000_00A5); wr(8'h5C, 32'h0); wr(8'h50, 32'h0001_0541);
        wr(8'h00, 32'h0000_0201);
        wait_pk(n0 + 1, 3);
        rd(8'h04); check(rd_val[0] == 1'b0, "L: ALARM does not wait for a reply");
        check(p_type[n0] == 3'd4 && p_prio[n0] == 1'b1 && p_dest[n0] == 3'd0 &&
              p_arg[n0] == 8'h05 && p_len[n0] == 6'd1 && p_pay[n0*64] == 8'hA5,
              "L: ALARM packet sent with prio 1, raw bytes");
        wr(8'h50, 32'h0001_0581);           // DATA with the prio bit set
        wr(8'h00, 32'h0000_0201);
        wait_pk(n0 + 2, 3);
        check(p_type[n0+1] == 3'd0 && p_prio[n0+1] == 1'b1 && p_len[n0+1] == 6'd1,
              "L: TASK_CFG prio bit sets the head prio");
        exp_req = exp_req + 2;
        wait_idle(5);
        wr(8'h50, 0);

        // ================= K: totals =================
        rd(8'h20); check(rd_val == exp_req, "K: REQ counter");
        rd(8'h24); check(rd_val == exp_ok,  "K: RESP_OK counter");
        rd(8'h28); check(rd_val == exp_err, "K: ERR counter");
        rd(8'h2C); check(rd_val == exp_tmo, "K: TMO counter");
        rd(8'h30); check(rd_val == exp_rec, "K: REC counter");
        rd(8'h10); check(rd_val == exp_rec + 1, "K: HUB_SEQ = records + 1");
        check(n_evrec == exp_rec, "K: one ev_record per record");
        check(n_everr == exp_err + exp_tmo, "K: one ev_error per error/time-out");
        check(bad_pulse == 0, "K: events are 1-clock pulses");
        check(n_run == exp_req && n_run > 32, "K: every run sent one request (roll wrapped)");
        check(bad_tag == 0, "K: tag = {roll, task}, roll +1 per request");
        check(bad_src == 0, "K: every packet from my_id");
        check(bad_len == 0, "K: XFER_REQ length = wtotal + 2");
        check(mon_n == n_pk, "K: every packet on the link was received");
        check(u_hub.u_tx.cred == 5'd4 && u_ttx.cred == 5'd4, "K: all credits returned");
        $display("hub: %0d packets, %0d requests, %0d records, %0d XFER_REQ answered",
                 n_pk, n_run, exp_rec, n_xreq);

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end
endmodule

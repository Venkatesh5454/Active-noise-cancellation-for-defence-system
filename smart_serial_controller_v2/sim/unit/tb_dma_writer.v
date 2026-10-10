// =============================================================================
// tb_dma_writer.v  -  unit test: DMA writer (node 2) + AXI memory model
// -----------------------------------------------------------------------------
// Set-up:  noc_pkt_tx --link--> dma_writer --AXI4--> axi_mem_model
// The sender is wired straight to the DMA writer's NoC input (one NoC hop).
// The memory model gives random awready/wready/bvalid delays and checks the
// AXI protocol.  The testbench keeps:
//   * a model of the DUT registers (wr, rd, COUNT, OVERFLOW, ...),
//   * a list of the bursts it expects (address + 16 bytes, in order),
//   * a shadow copy of the whole memory.
// A monitor on the AXI bus checks every burst against the list, the fixed
// AXI fields (awlen 3, awsize 2, INCR, awcache 0011, awprot 0, wstrb F) and
// that only one burst is outstanding.  Comparing the memory with the shadow
// copy proves every record landed at BASE + i*16 and nothing else changed.
//
//   T0  reset values of every register
//   T1  EN = 0: packets are consumed and counted in IGNORED, no bus traffic
//   T2  32 records: ev_batch exactly after record 32, PENDING back to 0
//   T3  wrap-around of the default 64-slot ring
//   T4  ring full: records dropped, OVERFLOW, nothing overwritten; moving
//       RD_IDX lets writing resume
//   T5  BRESP SLVERR / DECERR -> AXI_ERR, record still counts
//   T6  other packet types -> IGNORED, the stream stays in step
//   T7  short (0, 1, 5, 15) and long (17, 40, 63) RECORD payloads
//   T8  RESET, also while a burst is stuck on the bus (W before AW)
//   T9  SIZE_LOG2 clamp, rings of 4 and 2 slots, BATCH = 0, lowering BATCH
//   T10 time-out batch with a fast ms_tick, TIMEOUT = 0 disables it
//   T11 random traffic with random back-pressure (stress)
//   M   self-test of the memory model's protocol checker (second instance)
// =============================================================================
`timescale 1ns / 1ps
module tb_dma_writer;
    localparam MEMB = 16384;                     // bytes in the memory model
    localparam [31:0] MBASE = 32'h1000_0000;     // model address (like DDR)
    localparam [7:0]  FILLV = 8'hEE;             // memory contents at start

    // register offsets (SPEC section 8)
    localparam [7:0] R_CTRL = 8'h00, R_BASE = 8'h04, R_SIZE = 8'h08, R_WR = 8'h0C,
                     R_RD = 8'h10, R_BATCH = 8'h14, R_TMO = 8'h18, R_COUNT = 8'h1C,
                     R_OVF = 8'h20, R_AERR = 8'h24, R_PEND = 8'h28, R_STAT = 8'h2C,
                     R_IGN = 8'h30;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0;
    integer errors = 0;
    task check(input cond, input [8*72-1:0] what);
        begin
            checks = checks + 1;
            if (cond !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    integer seed;

    // ---------------- fast millisecond tick ----------------
    integer ms_period = 200;                     // clocks per "millisecond"
    integer ms_cnt = 0;
    reg     ms_tick = 1'b0;
    always @(posedge clk) begin
        if (!rst_n) begin
            ms_cnt  <= 0;
            ms_tick <= 1'b0;
        end else if (ms_cnt >= ms_period - 1) begin
            ms_cnt  <= 0;
            ms_tick <= 1'b1;
        end else begin
            ms_cnt  <= ms_cnt + 1;
            ms_tick <= 1'b0;
        end
    end

    // ---------------- register port ----------------
    reg         reg_we = 1'b0, reg_re = 1'b0;
    reg  [7:0]  reg_addr = 8'd0;
    reg  [31:0] reg_wdata = 32'd0;
    wire [31:0] reg_rdata;

    // ---------------- packet sender (one NoC hop) ----------------
    reg        start = 1'b0;
    wire       hdr_ready;
    reg  [2:0] t_type = 3'd0;
    reg  [5:0] t_len = 6'd0;
    reg  [7:0] t_tag = 8'd0, t_arg = 8'd0;
    reg        tb_valid = 1'b0;
    reg  [7:0] tb_data = 8'd0;
    wire       tb_ready;
    wire       link_valid;
    wire [33:0] link_flit;
    wire       link_credit;

    noc_pkt_tx #(.DEPTH(4)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(start), .hdr_ready(hdr_ready),
        .ptype(t_type), .dest(3'd2), .src(3'd1), .prio(1'b0),
        .len(t_len), .tag(t_tag), .arg(t_arg),
        .b_valid(tb_valid), .b_data(tb_data), .b_ready(tb_ready),
        .out_valid(link_valid), .out_flit(link_flit), .out_credit(link_credit));

    // ---------------- DUT ----------------
    wire        dut_out_valid;
    wire [33:0] dut_out_flit;
    wire [31:0] awaddr;
    wire [7:0]  awlen;
    wire [2:0]  awsize;
    wire [1:0]  awburst;
    wire [3:0]  awcache;
    wire [2:0]  awprot;
    wire        awvalid, awready;
    wire [31:0] wdata;
    wire [3:0]  wstrb;
    wire        wlast, wvalid, wready;
    wire [1:0]  bresp;
    wire        bvalid, bready;
    wire        ev_batch, ev_overflow, ev_axi_err;

    dma_writer u_dut (
        .clk(clk), .rst_n(rst_n), .ms_tick(ms_tick), .my_id(3'd2),
        .reg_we(reg_we), .reg_re(reg_re), .reg_addr(reg_addr),
        .reg_wdata(reg_wdata), .reg_rdata(reg_rdata),
        .in_valid(link_valid), .in_flit(link_flit), .in_credit(link_credit),
        .out_valid(dut_out_valid), .out_flit(dut_out_flit), .out_credit(1'b0),
        .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
        .m_axi_awburst(awburst), .m_axi_awcache(awcache), .m_axi_awprot(awprot),
        .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .ev_batch(ev_batch), .ev_overflow(ev_overflow), .ev_axi_err(ev_axi_err));

    axi_mem_model #(.MEM_BYTES(MEMB), .BASE_ADDR(MBASE), .SEED(11),
                    .AW_READY_PCT(60), .W_READY_PCT(70), .B_MAX_DELAY(4),
                    .FILL(FILLV), .NAME("axi_mem")) u_mem (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(awaddr), .s_axi_awlen(awlen), .s_axi_awsize(awsize),
        .s_axi_awburst(awburst), .s_axi_awcache(awcache), .s_axi_awprot(awprot),
        .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wlast(wlast),
        .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready));

    // ---------------- second memory model, driven by hand (section M) ----------------
    reg  [31:0] c_awaddr = 32'd0;
    reg  [7:0]  c_awlen = 8'd0;
    reg  [2:0]  c_awsize = 3'd2;
    reg  [1:0]  c_awburst = 2'b01;
    reg         c_awvalid = 1'b0;
    wire        c_awready;
    reg  [31:0] c_wdata = 32'd0;
    reg  [3:0]  c_wstrb = 4'h0;
    reg         c_wlast = 1'b0;
    reg         c_wvalid = 1'b0;
    wire        c_wready;
    wire [1:0]  c_bresp;
    wire        c_bvalid;
    reg         c_bready = 1'b0;

    axi_mem_model #(.MEM_BYTES(8192), .BASE_ADDR(32'h0), .SEED(3),
                    .FILL(8'h00), .NAME("axi_mem_selftest")) u_chk (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(c_awaddr), .s_axi_awlen(c_awlen), .s_axi_awsize(c_awsize),
        .s_axi_awburst(c_awburst), .s_axi_awcache(4'b0011), .s_axi_awprot(3'b000),
        .s_axi_awvalid(c_awvalid), .s_axi_awready(c_awready),
        .s_axi_wdata(c_wdata), .s_axi_wstrb(c_wstrb), .s_axi_wlast(c_wlast),
        .s_axi_wvalid(c_wvalid), .s_axi_wready(c_wready),
        .s_axi_bresp(c_bresp), .s_axi_bvalid(c_bvalid), .s_axi_bready(c_bready));

    // ---------------- reference model of the DUT ----------------
    reg [7:0]   shadow [0:MEMB-1];       // what the memory must contain
    reg [31:0]  ex_addr [0:2047];        // expected bursts, in order
    reg [127:0] ex_data [0:2047];
    integer     ex_n = 0;

    reg  [31:0] m_base = 32'd0;
    integer m_en = 0, m_log2 = 6, m_wr = 0, m_rd = 0, m_batch = 32;
    integer m_count = 0, m_ovf = 0, m_err = 0, m_ign = 0, m_pend = 0, m_nbatch = 0;
    reg  [31:0] last_addr;               // slot address of the last modelled record

    task model_reset;                    // what DMA_CTRL.RESET does
        begin
            m_wr = 0; m_rd = 0; m_count = 0; m_ovf = 0; m_err = 0; m_ign = 0; m_pend = 0;
        end
    endtask

    // a RECORD reaches the DUT with EN = 1
    //   will_err: the memory answers with an error (data not stored)
    //   killed:   a RESET will come while the burst is on the bus
    task model_record(input [127:0] r, input will_err, input killed);
        integer size, used, i;
        reg [31:0] a;
        begin
            size = 1 << m_log2;
            used = (m_wr - m_rd) & (size - 1);
            if (used == size - 1) begin
                m_ovf = m_ovf + 1;                       // ring full: dropped
            end else begin
                a = {m_base[31:4], 4'd0} + m_wr * 16;
                last_addr = a;
                ex_addr[ex_n] = a;
                ex_data[ex_n] = r;
                ex_n = ex_n + 1;
                if (!will_err)
                    for (i = 0; i < 16; i = i + 1) shadow[a - MBASE + i] = r[8*i +: 8];
                if (!killed) begin
                    m_wr    = (m_wr + 1) % size;
                    m_count = m_count + 1;
                    if (will_err) m_err = m_err + 1;
                    m_pend  = m_pend + 1;
                    if (m_batch != 0 && m_pend >= m_batch) begin
                        m_pend   = 0;
                        m_nbatch = m_nbatch + 1;
                    end
                end
            end
        end
    endtask

    // ---------------- AXI / event monitor ----------------
    integer cyc = 0;
    integer mon_bad = 0;
    integer aw_i = 0, w_i = 0, w_beat = 0, b_cnt = 0;
    integer n_batch = 0, n_ovf = 0, n_aerr = 0;
    integer last_b_cyc = 0, last_batch_cyc = 0, batch_gap = 0;
    integer b_since_batch = 0, last_batch_b = 0;
    integer exp_batch_b = 0;             // >0: every ev_batch must come after this many records
    integer batch_bad = 0;
    reg     prev_batch = 1'b0, prev_ovf = 1'b0, prev_aerr = 1'b0;
    reg [127:0] mon_rec;

    task mon_error(input [8*64-1:0] msg);
        begin
            mon_bad = mon_bad + 1;
            if (mon_bad <= 10) $display("MONITOR: %0s (t=%0t)", msg, $time);
        end
    endtask

    always @(posedge clk) begin
        if (rst_n) begin
            cyc = cyc + 1;
            if (dut_out_valid !== 1'b0 || dut_out_flit !== 34'd0)
                mon_error("the DMA writer must never send (out_valid/out_flit)");
            if (awvalid && (awlen !== 8'd3 || awsize !== 3'd2 || awburst !== 2'b01 ||
                            awcache !== 4'b0011 || awprot !== 3'b000))
                mon_error("AW fields are not len 3 / size 2 / INCR / cache 0011 / prot 0");
            if (wvalid && wstrb !== 4'hF)
                mon_error("wstrb is not F");

            if (awvalid && awready) begin
                if (aw_i != b_cnt) mon_error("a second AW before the B of the first burst");
                if (aw_i >= ex_n)  mon_error("a burst nobody expected");
                else if (awaddr !== ex_addr[aw_i]) begin
                    mon_error("burst to the wrong address");
                    $display("         awaddr %h, expected %h", awaddr, ex_addr[aw_i]);
                end
                aw_i = aw_i + 1;
            end

            if (wvalid && wready) begin
                if (w_i != b_cnt) mon_error("W beats of a new burst before the last B");
                if (w_i >= ex_n) begin
                    mon_error("W beat nobody expected");
                end else begin
                    mon_rec = ex_data[w_i];
                    if (wdata !== mon_rec[32*w_beat +: 32]) begin
                        mon_error("wrong write data");
                        $display("         burst %0d beat %0d: %h, expected %h", w_i, w_beat,
                                 wdata, mon_rec[32*w_beat +: 32]);
                    end
                end
                if (wlast !== (w_beat == 3)) mon_error("wlast not exactly on beat 3");
                if (w_beat == 3) begin
                    w_beat = 0;
                    w_i    = w_i + 1;
                end else begin
                    w_beat = w_beat + 1;
                end
            end

            if (ev_batch) begin
                if (prev_batch) mon_error("ev_batch longer than one clock");
                n_batch        = n_batch + 1;
                batch_gap      = cyc - last_b_cyc;
                last_batch_cyc = cyc;
                last_batch_b   = b_since_batch;
                if (exp_batch_b != 0 && (b_since_batch != exp_batch_b || batch_gap != 1)) begin
                    batch_bad = batch_bad + 1;
                    $display("MONITOR: ev_batch after %0d records, %0d clocks after the last B",
                             b_since_batch, batch_gap);
                end
                b_since_batch = 0;
            end
            if (ev_overflow) begin
                if (prev_ovf) mon_error("ev_overflow longer than one clock");
                n_ovf = n_ovf + 1;
            end
            if (ev_axi_err) begin
                if (prev_aerr) mon_error("ev_axi_err longer than one clock");
                n_aerr = n_aerr + 1;
            end
            prev_batch = ev_batch;
            prev_ovf   = ev_overflow;
            prev_aerr  = ev_axi_err;

            if (bvalid && bready) begin
                b_cnt         = b_cnt + 1;
                last_b_cyc    = cyc;
                b_since_batch = b_since_batch + 1;
            end
        end
    end

    // ---------------- register access ----------------
    task wreg(input [7:0] a, input [31:0] d);
        begin
            @(negedge clk);
            reg_we = 1'b1; reg_addr = a; reg_wdata = d;
            @(negedge clk);
            reg_we = 1'b0;
        end
    endtask

    task rreg(input [7:0] a, output [31:0] d);
        begin
            @(negedge clk);
            reg_re = 1'b1; reg_addr = a;
            #1 d = reg_rdata;
            @(negedge clk);
            reg_re = 1'b0;
        end
    endtask

    reg [31:0] rv;
    task expect_reg(input [7:0] a, input [31:0] v, input [8*72-1:0] what);
        begin
            rreg(a, rv);
            if (rv !== v) $display("       reg %h = %h, expected %h", a, rv, v);
            check(rv === v, what);
        end
    endtask

    // ---------------- packet sending ----------------
    reg [7:0] pay [0:63];
    integer   gap_pct = 0;               // chance of an idle clock between bytes

    task send_pkt(input [2:0] ptype, input [5:0] len);
        integer k;
        begin
            @(negedge clk);
            while (!hdr_ready) @(negedge clk);
            start = 1'b1; t_type = ptype; t_len = len;
            t_tag = $random(seed); t_arg = $random(seed);
            @(negedge clk);
            start = 1'b0;
            for (k = 0; k < len; k = k + 1) begin
                while (({$random(seed)} % 100) < gap_pct) begin
                    tb_valid = 1'b0;
                    @(negedge clk);
                end
                tb_valid = 1'b1; tb_data = pay[k];
                while (!tb_ready) @(negedge clk);
                @(negedge clk);                  // taken on the clock edge just passed
            end
            tb_valid = 1'b0;
        end
    endtask

    // random payload; returns the 16-byte record the DMA writer must store
    reg [127:0] rec_v;
    task make_payload(input [5:0] len);
        integer k;
        begin
            rec_v = 128'd0;
            for (k = 0; k < 64; k = k + 1) pay[k] = $random(seed);
            for (k = 0; k < 16; k = k + 1) if (k < len) rec_v[8*k +: 8] = pay[k];
        end
    endtask

    task send_record(input [5:0] len);
        begin
            make_payload(len);
            if (m_en != 0) model_record(rec_v, 1'b0, 1'b0);
            else           m_ign = m_ign + 1;
            send_pkt(3'd3, len);
        end
    endtask

    task send_record_flags(input [5:0] len, input will_err, input killed);
        begin
            make_payload(len);
            model_record(rec_v, will_err, killed);
            send_pkt(3'd3, len);
        end
    endtask

    task send_other(input [2:0] ptype, input [5:0] len);
        begin
            make_payload(len);
            m_ign = m_ign + 1;
            send_pkt(ptype, len);
        end
    endtask

    // wait until every packet sent so far has been handled
    reg [31:0] wc, wo, wg, ws;
    task wait_done;
        integer n;
        begin
            n = 0;
            wc = 0; wo = 0; wg = 0; ws = 1;
            while (n < 5000 && !((wc + wo + wg == m_count + m_ovf + m_ign) && ws[0] == 1'b0)) begin
                rreg(R_COUNT, wc); rreg(R_OVF, wo); rreg(R_IGN, wg); rreg(R_STAT, ws);
                n = n + 1;
            end
            check(n < 5000, "all packets handled (wait_done)");
            repeat (3) @(negedge clk);
        end
    endtask

    // compare the whole memory with the shadow copy
    task cmp_mem(input [8*72-1:0] what);
        integer i, bad;
        begin
            bad = 0;
            for (i = 0; i < MEMB; i = i + 1)
                if (u_mem.mem[i] !== shadow[i]) begin
                    if (bad < 5) $display("       mem[%h] = %h, expected %h", MBASE + i,
                                          u_mem.mem[i], shadow[i]);
                    bad = bad + 1;
                end
            check(bad == 0, what);
        end
    endtask

    // registers against the reference model
    task check_model;
        begin
            expect_reg(R_COUNT, m_count, "COUNT matches the model");
            expect_reg(R_OVF,   m_ovf,   "OVERFLOW matches the model");
            expect_reg(R_AERR,  m_err,   "AXI_ERR matches the model");
            expect_reg(R_IGN,   m_ign,   "IGNORED matches the model");
            expect_reg(R_WR,    m_wr,    "WR_IDX matches the model");
            expect_reg(R_PEND,  m_pend,  "PENDING matches the model");
        end
    endtask

    // ---------------- hand-driven master for the model self-test ----------------
    task c_aw(input [31:0] a, input [7:0] l, input [2:0] s, input [1:0] b);
        begin
            @(negedge clk);
            c_awvalid = 1'b1; c_awaddr = a; c_awlen = l; c_awsize = s; c_awburst = b;
            while (!c_awready) @(negedge clk);
            @(negedge clk);
            c_awvalid = 1'b0;
        end
    endtask

    task c_w(input [31:0] d, input [3:0] s, input l);
        begin
            @(negedge clk);
            c_wvalid = 1'b1; c_wdata = d; c_wstrb = s; c_wlast = l;
            while (!c_wready) @(negedge clk);
            @(negedge clk);
            c_wvalid = 1'b0;
        end
    endtask

    reg [1:0] c_got;
    reg       c_seen;
    task c_b;                                // take one B response (or time out)
        integer n;
        begin
            @(negedge clk);
            c_bready = 1'b1;
            n = 0;
            while (!c_bvalid && n < 50) begin @(negedge clk); n = n + 1; end
            c_seen = c_bvalid;
            c_got  = c_bresp;
            @(negedge clk);
            c_bready = 1'b0;
        end
    endtask

    // ---------------- watchdog ----------------
    initial begin
        #(8_000_000);
        $display("WATCHDOG: the test did not finish in time");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end

    // ---------------- test sequence ----------------
    integer i, j, n, nb0, aw0, e0, r0, t_first, size, used, round, npk, ty, ln;
    reg [31:0] v;

    initial begin
        seed = 20261010;
        for (i = 0; i < MEMB; i = i + 1) shadow[i] = FILLV;
        #33 rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ================= T0: reset values =================
        $display("T0: reset values");
        expect_reg(R_CTRL,  32'd0,    "T0: DMA_CTRL resets to 0 (EN off)");
        expect_reg(R_BASE,  32'd0,    "T0: DMA_BASE resets to 0");
        expect_reg(R_SIZE,  32'd6,    "T0: DMA_SIZE_LOG2 resets to 6");
        expect_reg(R_WR,    32'd0,    "T0: DMA_WR_IDX resets to 0");
        expect_reg(R_RD,    32'd0,    "T0: DMA_RD_IDX resets to 0");
        expect_reg(R_BATCH, 32'd32,   "T0: DMA_BATCH resets to 32");
        expect_reg(R_TMO,   32'd1000, "T0: DMA_TIMEOUT resets to 1000");
        expect_reg(R_COUNT, 32'd0,    "T0: COUNT resets to 0");
        expect_reg(R_OVF,   32'd0,    "T0: OVERFLOW resets to 0");
        expect_reg(R_AERR,  32'd0,    "T0: AXI_ERR resets to 0");
        expect_reg(R_PEND,  32'd0,    "T0: PENDING resets to 0");
        expect_reg(R_STAT,  32'd4,    "T0: STATUS = empty");
        expect_reg(R_IGN,   32'd0,    "T0: IGNORED resets to 0");
        expect_reg(8'h34,   32'd0,    "T0: unused address 0x34 reads 0");
        expect_reg(8'hFC,   32'd0,    "T0: unused address 0xFC reads 0");

        // ================= T1: EN = 0 =================
        $display("T1: EN = 0, packets are consumed and ignored");
        aw0 = u_mem.n_aw;
        send_record(16);
        send_other(3'd0, 6'd4);
        wait_done;
        expect_reg(R_IGN,   32'd2, "T1: EN=0: RECORD and DATA counted in IGNORED");
        expect_reg(R_COUNT, 32'd0, "T1: EN=0: nothing written (COUNT 0)");
        check(u_mem.n_aw == aw0, "T1: EN=0: no AXI traffic");

        // set-up: BASE (low bits ignored), no time-out, EN on
        wreg(R_BASE, 32'h1000_100C);
        expect_reg(R_BASE, 32'h1000_1000, "BASE[3:0] are ignored (read back 0)");
        m_base = 32'h1000_1000;
        wreg(R_TMO, 32'd0);
        wreg(R_CTRL, 32'd1);
        m_en = 1;
        expect_reg(R_CTRL, 32'd1, "DMA_CTRL reads EN = 1");

        // ================= T2: batch of 32 =================
        $display("T2: 32 records -> one ev_batch right after record 32");
        exp_batch_b = 32;
        for (i = 0; i < 32; i = i + 1) send_record(16);
        wait_done;
        check(n_batch == 1, "T2: exactly one ev_batch after 32 records");
        check(last_batch_b == 32 && batch_gap == 1,
              "T2: ev_batch came the clock after the B of record 32");
        expect_reg(R_PEND,  32'd0,  "T2: PENDING back to 0 after the batch");
        expect_reg(R_COUNT, 32'd32, "T2: COUNT = 32");
        expect_reg(R_WR,    32'd32, "T2: WR_IDX = 32");
        expect_reg(R_STAT,  32'd0,  "T2: STATUS: not busy, not full, not empty");
        for (i = 0; i < 8; i = i + 1) send_record(16);
        wait_done;
        expect_reg(R_PEND, 32'd8, "T2: PENDING = 8 after 8 more records");
        check(n_batch == 1, "T2: no second ev_batch yet");
        check(u_mem.rd32(m_base) === ex_data[0][31:0] &&
              u_mem.rd32(m_base + 39*16 + 12) === ex_data[39][127:96],
              "T2: record 0 at BASE, record 39 at BASE + 39*16");
        cmp_mem("T2: records 0..39 at BASE + i*16, rest of memory untouched");
        check_model;

        // ================= T3: wrap-around =================
        $display("T3: wrap-around of the 64-slot ring");
        wreg(R_RD, 32'd40);
        m_rd = 40;
        expect_reg(R_RD, 32'd40, "T3: RD_IDX written by software");
        for (i = 0; i < 30; i = i + 1) send_record(16);
        wait_done;
        expect_reg(R_WR, 32'd6, "T3: WR_IDX wrapped: (40 + 30) mod 64 = 6");
        check(u_mem.rd32(m_base) === ex_data[64][31:0],
              "T3: record 64 landed in slot 0 (BASE)");
        check(u_mem.rd32(m_base + 5*16 + 12) === ex_data[69][127:96],
              "T3: record 69 landed in slot 5");
        check(u_mem.rd32(m_base + 6*16) === ex_data[6][31:0],
              "T3: slot 6 still holds record 6");
        check(n_batch == 2 && m_nbatch == 2, "T3: second ev_batch after 64 records");
        cmp_mem("T3: memory matches after the wrap");
        check_model;

        // ================= T4: ring full =================
        $display("T4: ring full -> overflow; moving RD_IDX resumes writing");
        // rd = 40, wr = 6: 30 used, 33 free slots, so 3 of 36 records are dropped
        for (i = 0; i < 36; i = i + 1) send_record(16);
        wait_done;
        expect_reg(R_OVF, 32'd3,  "T4: OVERFLOW = 3");
        check(n_ovf == 3, "T4: three ev_overflow pulses");
        expect_reg(R_WR,  32'd39, "T4: WR_IDX stopped at rd - 1 = 39");
        expect_reg(R_STAT, 32'd2, "T4: STATUS = full");
        check(u_mem.rd32(m_base + 39*16) === ex_data[39][31:0],
              "T4: slot 39 (rd - 1) still holds its old record");
        cmp_mem("T4: nothing overwritten while the ring was full");
        wreg(R_RD, 32'd45);
        m_rd = 45;
        expect_reg(R_STAT, 32'd0, "T4: not full after software moved RD_IDX");
        send_record(16);
        send_record(16);
        wait_done;
        expect_reg(R_WR,  32'd41, "T4: writing resumed: WR_IDX = 41");
        expect_reg(R_OVF, 32'd3,  "T4: no new overflow");
        cmp_mem("T4: records after the RD_IDX move are in slots 39, 40");
        check(n_batch == m_nbatch, "T4: ev_batch count matches the model");
        check_model;
        wreg(R_RD, 32'd41);                      // software catches up: ring empty
        m_rd = 41;
        expect_reg(R_STAT, 32'd4, "T4: STATUS = empty after RD_IDX = WR_IDX");

        // ================= T5: BRESP errors =================
        $display("T5: BRESP errors -> AXI_ERR");
        exp_batch_b = 0;
        u_mem.inject_bresp(1, 2'b10);            // SLVERR
        send_record_flags(6'd16, 1'b1, 1'b0);
        wait_done;
        expect_reg(R_AERR, 32'd1, "T5: SLVERR counted in AXI_ERR");
        check(n_aerr == 1, "T5: one ev_axi_err pulse");
        u_mem.inject_bresp(1, 2'b11);            // DECERR
        send_record_flags(6'd16, 1'b1, 1'b0);
        send_record(16);                         // a good one again
        wait_done;
        expect_reg(R_AERR, 32'd2, "T5: DECERR counted, OKAY not counted");
        check(n_aerr == 2, "T5: two ev_axi_err pulses");
        cmp_mem("T5: memory matches (failed bursts are not stored by the model)");
        check_model;

        // ================= T6: other packet types =================
        $display("T6: other packet types are consumed and counted in IGNORED");
        aw0 = u_mem.n_aw;
        send_other(3'd0, 6'd0);
        send_other(3'd1, 6'd7);
        send_other(3'd2, 6'd20);
        send_other(3'd4, 6'd63);
        send_other(3'd5, 6'd0);
        send_other(3'd6, 6'd4);
        send_other(3'd7, 6'd33);
        send_record(16);
        wait_done;
        expect_reg(R_IGN, 32'd9, "T6: IGNORED = 2 (T1) + 7");
        check(u_mem.n_aw == aw0 + 1, "T6: only the RECORD made a burst");
        cmp_mem("T6: the RECORD after them is stored correctly");
        check_model;

        // ================= T7: short and long payloads =================
        $display("T7: short and long RECORD payloads");
        send_record(0);
        v = last_addr;
        send_record(5);
        r0 = last_addr;
        send_record(1);
        send_record(15);
        send_record(17);
        send_record(40);
        send_record(63);
        send_record(16);
        wait_done;
        check(u_mem.rd32(v) === 0 && u_mem.rd32(v + 4) === 0 &&
              u_mem.rd32(v + 8) === 0 && u_mem.rd32(v + 12) === 0,
              "T7: length-0 RECORD stored as 16 zero bytes");
        check(u_mem.rd8(r0 + 5) === 8'h00 && u_mem.rd32(r0 + 12) === 0 &&
              u_mem.rd8(r0 + 4) === ex_data[ex_n - 7][39:32],
              "T7: length-5 RECORD: 5 bytes then zeros");
        cmp_mem("T7: padded / cut records stored exactly");
        check_model;

        // ================= T8: RESET =================
        $display("T8: DMA_CTRL.RESET");
        wreg(R_CTRL, 32'h0000_0101);             // RESET, keep EN
        model_reset;
        expect_reg(R_CTRL,  32'd1, "T8: EN still 1 after RESET (bit 8 reads 0)");
        expect_reg(R_COUNT, 32'd0, "T8: COUNT cleared");
        expect_reg(R_OVF,   32'd0, "T8: OVERFLOW cleared");
        expect_reg(R_AERR,  32'd0, "T8: AXI_ERR cleared");
        expect_reg(R_IGN,   32'd0, "T8: IGNORED cleared");
        expect_reg(R_PEND,  32'd0, "T8: PENDING cleared");
        expect_reg(R_WR,    32'd0, "T8: WR_IDX cleared");
        expect_reg(R_RD,    32'd0, "T8: RD_IDX cleared (ring empty)");
        expect_reg(R_STAT,  32'd4, "T8: STATUS = empty");
        send_record(16);
        wait_done;
        check(u_mem.rd32(m_base) === ex_data[ex_n - 1][31:0], "T8: first record after RESET at BASE");
        expect_reg(R_WR, 32'd1, "T8: WR_IDX = 1");

        // RESET while a burst is stuck: AW never ready, W accepted (W before AW)
        u_mem.set_ready(0, 100, 0);
        aw0 = u_mem.n_aw;
        send_record_flags(6'd16, 1'b0, 1'b1);
        n = 0;
        while (n < 300 && !(awvalid && u_mem.wq_n == 4)) begin @(negedge clk); n = n + 1; end
        check(awvalid === 1'b1 && u_mem.wq_n == 4 && u_mem.n_aw == aw0,
              "T8: all 4 W beats accepted while AW still waits");
        expect_reg(R_STAT, 32'd1, "T8: STATUS = busy during the burst");
        wreg(R_CTRL, 32'h0000_0101);
        model_reset;
        u_mem.set_ready(60, 70, 4);
        n = 0;
        ws = 1;
        while (n < 300 && ws[0]) begin rreg(R_STAT, ws); n = n + 1; end
        check(ws[0] == 1'b0, "T8: stuck burst finished after AW was accepted");
        check(u_mem.n_aw == aw0 + 1, "T8: the burst was completed on the bus");
        expect_reg(R_COUNT, 32'd0, "T8: burst cut by RESET is not counted");
        expect_reg(R_WR,    32'd0, "T8: WR_IDX stays 0 after the cut burst");
        expect_reg(R_PEND,  32'd0, "T8: PENDING stays 0 after the cut burst");
        cmp_mem("T8: memory matches (the cut burst still reached slot 1)");
        b_since_batch = 0;
        send_record(16);
        wait_done;
        check(u_mem.rd32(m_base) === ex_data[ex_n - 1][31:0], "T8: next record at BASE again");
        check_model;

        // ================= T9: small rings =================
        $display("T9: SIZE_LOG2 clamp, rings of 4 and 2 slots, BATCH 0");
        wreg(R_CTRL, 32'h0000_0101);
        model_reset;
        wreg(R_SIZE, 32'd0);
        expect_reg(R_SIZE, 32'd1,  "T9: SIZE_LOG2 = 0 clamps to 1");
        wreg(R_SIZE, 32'd15);
        expect_reg(R_SIZE, 32'd12, "T9: SIZE_LOG2 = 15 clamps to 12");
        wreg(R_SIZE, 32'd13);
        expect_reg(R_SIZE, 32'd12, "T9: SIZE_LOG2 = 13 clamps to 12");
        wreg(R_SIZE, 32'd2);
        m_log2 = 2;
        expect_reg(R_SIZE, 32'd2,  "T9: SIZE_LOG2 = 2 (4 slots)");
        wreg(R_BATCH, 32'd0);
        m_batch = 0;
        nb0 = n_batch;
        for (i = 0; i < 5; i = i + 1) send_record(16);
        wait_done;
        expect_reg(R_WR,  32'd3, "T9: 4-slot ring holds 3 records");
        expect_reg(R_OVF, 32'd2, "T9: records 4 and 5 dropped");
        expect_reg(R_STAT, 32'd2, "T9: STATUS = full");
        wreg(R_RD, 32'd3);
        m_rd = 3;
        send_record(5);                          // short record right after a dropped one:
        send_record(16);                         // its bytes 5..15 must still be 0
        send_record(16);
        wait_done;
        expect_reg(R_WR, 32'd2, "T9: WR_IDX wrapped 3 -> 0 -> 1 -> 2");
        check(u_mem.rd32(m_base + 16) === ex_data[ex_n - 1][31:0],
              "T9: last record in slot 1");
        cmp_mem("T9: 4-slot ring contents");
        check(n_batch == nb0, "T9: BATCH = 0: no ev_batch");
        expect_reg(R_PEND, 32'd6, "T9: PENDING = 6 with BATCH = 0");
        check_model;
        wreg(R_BATCH, 32'd3);                    // lower than PENDING: fires at once
        repeat (3) @(negedge clk);
        check(n_batch == nb0 + 1, "T9: BATCH lowered under PENDING -> ev_batch");
        m_batch = 3;
        m_pend  = 0;
        expect_reg(R_PEND, 32'd0, "T9: PENDING = 0 after that batch");
        // 2 slots: only one record fits
        wreg(R_CTRL, 32'h0000_0101);
        model_reset;
        wreg(R_SIZE, 32'd1);
        m_log2 = 1;
        send_record(16);
        send_record(16);
        wait_done;
        expect_reg(R_OVF, 32'd1, "T9: 2-slot ring: second record dropped");
        wreg(R_RD, 32'd1);
        m_rd = 1;
        send_record(16);
        wait_done;
        expect_reg(R_WR, 32'd0, "T9: 2-slot ring wrapped to 0");
        check(u_mem.rd32(m_base + 16) === ex_data[ex_n - 1][31:0], "T9: record in slot 1");
        cmp_mem("T9: 2-slot ring contents");
        check_model;

        // ================= T10: time-out =================
        $display("T10: time-out batch (1 ms = %0d clocks)", ms_period);
        wreg(R_CTRL, 32'h0000_0101);
        model_reset;
        wreg(R_SIZE, 32'd6);
        m_log2 = 6;
        wreg(R_BATCH, 32'd32);
        m_batch = 32;
        wreg(R_TMO, 32'd3);
        expect_reg(R_TMO, 32'd3, "T10: TIMEOUT = 3 ms");
        nb0 = n_batch;
        send_record(16);
        wait_done;
        t_first = last_b_cyc;                    // the oldest pending record was written here
        repeat (250) @(negedge clk);             // a second record must not restart the timer
        send_record(16);
        n = 0;
        while (n < 4 * ms_period && n_batch == nb0) begin @(negedge clk); n = n + 1; end
        check(n_batch == nb0 + 1, "T10: ev_batch from the time-out");
        $display("    time-out ev_batch %0d clocks after the oldest record", last_batch_cyc - t_first);
        check(last_batch_cyc - t_first > 2 * ms_period &&
              last_batch_cyc - t_first <= 3 * ms_period + 1,
              "T10: ev_batch on the 3rd ms_tick after the oldest record");
        m_pend = 0;
        expect_reg(R_PEND, 32'd0, "T10: PENDING = 0 after the time-out");
        repeat (5 * ms_period) @(negedge clk);
        check(n_batch == nb0 + 1, "T10: no ev_batch while PENDING = 0");
        // TIMEOUT = 0 switches the time-out off
        wreg(R_TMO, 32'd0);
        send_record(16);
        wait_done;
        repeat (10 * ms_period) @(negedge clk);
        check(n_batch == nb0 + 1, "T10: TIMEOUT = 0: no ev_batch after 10 ms");
        expect_reg(R_PEND, 32'd1, "T10: PENDING = 1 still waiting");
        wreg(R_TMO, 32'd2);                      // record already older than 2 ms
        n = 0;
        while (n < ms_period + 5 && n_batch == nb0 + 1) begin @(negedge clk); n = n + 1; end
        check(n_batch == nb0 + 2, "T10: TIMEOUT set again -> ev_batch at the next ms_tick");
        m_pend = 0;
        expect_reg(R_PEND, 32'd0, "T10: PENDING = 0");
        cmp_mem("T10: memory matches");
        check_model;

        // ================= T11: random stress =================
        $display("T11: random traffic, random back-pressure, 8-slot ring, BATCH 7");
        wreg(R_CTRL, 32'h0000_0101);
        model_reset;
        wreg(R_SIZE, 32'd3);
        m_log2 = 3;
        wreg(R_BATCH, 32'd7);
        m_batch = 7;
        wreg(R_TMO, 32'd0);
        b_since_batch = 0;
        exp_batch_b = 7;
        nb0 = n_batch;
        m_nbatch = 0;
        gap_pct = 15;
        e0 = 0;
        for (round = 0; round < 30; round = round + 1) begin
            case (round % 3)
                0: u_mem.set_ready(100, 100, 0);   // no back-pressure at all
                1: u_mem.set_ready(25, 35, 9);     // heavy back-pressure
                default: u_mem.set_ready(60, 70, 3);
            endcase
            if (round == 13) begin wreg(R_CTRL, 32'd0); m_en = 0; end   // one round with EN off
            if (round == 14) begin wreg(R_CTRL, 32'd1); m_en = 1; end
            npk = 1 + ({$random(seed)} % 12);
            for (j = 0; j < npk; j = j + 1) begin
                ty = {$random(seed)} % 10;
                if (({$random(seed)} % 4) == 0) ln = {$random(seed)} % 64;
                else                            ln = 16;
                if (ty < 7) send_record(ln[5:0]);
                else begin
                    ty = {$random(seed)} % 7;              // 0 1 2 4 5 6 7 (not 3)
                    send_other((ty < 3) ? ty : ty + 1, ln[5:0]);
                end
                repeat ({$random(seed)} % 4) @(negedge clk);
            end
            wait_done;
            rreg(R_COUNT, wc); rreg(R_OVF, wo); rreg(R_IGN, wg); rreg(R_WR, v); rreg(R_PEND, ws);
            if (wc != m_count || wo != m_ovf || wg != m_ign || v != m_wr || ws != m_pend) begin
                e0 = e0 + 1;
                $display("    round %0d: COUNT %0d/%0d OVF %0d/%0d IGN %0d/%0d WR %0d/%0d PEND %0d/%0d",
                         round, wc, m_count, wo, m_ovf, wg, m_ign, v, m_wr, ws, m_pend);
            end
            // software reads some of the records
            size = 1 << m_log2;
            used = (m_wr - m_rd) & (size - 1);
            m_rd = (m_rd + ({$random(seed)} % (used + 1))) & (size - 1);
            wreg(R_RD, m_rd);
        end
        gap_pct = 0;
        check(e0 == 0, "T11: registers match the model after every round");
        check(m_ovf > 0 && m_ign > 0, "T11: the random rounds included overflows and ignored packets");
        check(n_batch - nb0 == m_nbatch, "T11: one ev_batch per 7 records");
        cmp_mem("T11: memory matches after the stress run");
        u_mem.set_ready(60, 70, 4);

        // ================= whole-run checks =================
        $display("Whole run");
        check(mon_bad == 0, "monitor: fixed AXI fields, one burst at a time, data as expected");
        check(aw_i == ex_n && w_i == ex_n && b_cnt == ex_n, "every expected burst happened, no extra ones");
        check(batch_bad == 0, "every count-triggered ev_batch came right after record BATCH");
        check(u_mem.proto_errors == 0, "memory model: zero AXI protocol errors");
        check(u_mem.range_errors == 0, "memory model: no access outside the memory");
        u_mem.report;
        $display("    %0d bursts expected, %0d records dropped in total", ex_n, n_ovf);

        // ================= M: memory model self-test =================
        $display("M: memory model self-test (the PROTOCOL ERROR lines below are deliberate)");
        e0 = u_chk.proto_errors;
        // M1a: AW first, 3 beats: no B yet; 4th beat -> B OKAY
        c_aw(32'h100, 8'd3, 3'd2, 2'b01);
        c_w(32'h0302_0100, 4'hF, 1'b0);
        c_w(32'h0706_0504, 4'hF, 1'b0);
        c_w(32'h0B0A_0908, 4'hF, 1'b0);
        repeat (5) @(negedge clk);
        check(c_bvalid === 1'b0, "M1: no B before the last W beat");
        c_w(32'h0F0E_0D0C, 4'hF, 1'b1);
        c_b;
        check(c_seen && c_got == 2'b00, "M1: B OKAY after the last beat");
        check(u_chk.rd32(32'h100) == 32'h0302_0100 && u_chk.rd32(32'h10C) == 32'h0F0E_0D0C,
              "M1: burst data stored little-endian");
        // M1b: all W beats before the AW
        c_w(32'hA1A1_A1A1, 4'hF, 1'b0);
        c_w(32'hB2B2_B2B2, 4'hF, 1'b1);
        repeat (5) @(negedge clk);
        check(c_bvalid === 1'b0, "M1: no B while the AW is missing");
        c_aw(32'h200, 8'd1, 3'd2, 2'b01);
        c_b;
        check(c_seen && c_got == 2'b00 && u_chk.rd32(32'h204) == 32'hB2B2_B2B2,
              "M1: W before AW handled");
        // narrow (byte) INCR burst, unaligned start
        c_aw(32'h301, 8'd1, 3'd0, 2'b01);
        c_w(32'h0000_5500, 4'b0010, 1'b0);
        c_w(32'h0066_0000, 4'b0100, 1'b1);
        c_b;
        check(u_chk.rd32(32'h300) == 32'h0066_5500, "M1: narrow byte burst lands in the right lanes");
        check(u_chk.proto_errors == e0, "M1: correct bursts give no protocol error");
        // M2: wlast missing
        c_aw(32'h400, 8'd1, 3'd2, 2'b01);
        c_w(32'h1, 4'hF, 1'b0);
        c_w(32'h2, 4'hF, 1'b0);
        c_b;
        check(u_chk.proto_errors == e0 + 1, "M2: missing wlast detected");
        // M3: wlast early (2 beats for awlen 3)
        c_aw(32'h500, 8'd3, 3'd2, 2'b01);
        c_w(32'h1, 4'hF, 1'b0);
        c_w(32'h2, 4'hF, 1'b1);
        c_b;
        check(u_chk.proto_errors == e0 + 2, "M3: early wlast (wrong beat count) detected");
        // M4: FIXED burst
        c_aw(32'h600, 8'd0, 3'd2, 2'b00);
        c_w(32'h1, 4'hF, 1'b1);
        c_b;
        check(u_chk.proto_errors == e0 + 3, "M4: non-INCR burst detected");
        // M5: 4 KB crossing
        c_aw(32'hFF8, 8'd3, 3'd2, 2'b01);
        c_w(32'h1, 4'hF, 1'b0);
        c_w(32'h2, 4'hF, 1'b0);
        c_w(32'h3, 4'hF, 1'b0);
        c_w(32'h4, 4'hF, 1'b1);
        c_b;
        check(u_chk.proto_errors == e0 + 4, "M5: 4 KB boundary crossing detected");
        // M6: awvalid dropped before awready
        u_chk.set_ready(0, 100, 0);
        repeat (2) @(negedge clk);
        c_awvalid = 1'b1; c_awaddr = 32'h700; c_awlen = 8'd0; c_awburst = 2'b01; c_awsize = 3'd2;
        repeat (3) @(negedge clk);
        c_awvalid = 1'b0;
        repeat (2) @(negedge clk);
        check(u_chk.proto_errors == e0 + 5, "M6: awvalid dropped before awready detected");
        // M7: W data changed while waiting for wready
        u_chk.set_ready(100, 0, 0);
        repeat (2) @(negedge clk);
        c_wvalid = 1'b1; c_wdata = 32'h1111_1111; c_wstrb = 4'hF; c_wlast = 1'b1;
        @(negedge clk);
        c_wdata = 32'h2222_2222;
        repeat (2) @(negedge clk);
        u_chk.set_ready(100, 100, 0);
        while (!c_wready) @(negedge clk);
        @(negedge clk);
        c_wvalid = 1'b0;
        c_aw(32'h700, 8'd0, 3'd2, 2'b01);
        c_b;
        check(u_chk.proto_errors == e0 + 6, "M7: W changed while waiting detected");
        // M8: strobe outside the active lane of a narrow beat
        c_aw(32'h800, 8'd0, 3'd0, 2'b01);
        c_w(32'hFFFF_FFFF, 4'b0011, 1'b1);
        c_b;
        check(u_chk.proto_errors == e0 + 7, "M8: wstrb outside the byte lane detected");
        // M9: injected SLVERR and an address outside the memory
        u_chk.inject_bresp(1, 2'b10);
        c_aw(32'h900, 8'd0, 3'd2, 2'b01);
        c_w(32'h1234_5678, 4'hF, 1'b1);
        c_b;
        check(c_seen && c_got == 2'b10 && u_chk.rd32(32'h900) == 32'h0,
              "M9: injected SLVERR answered, data not stored");
        c_aw(32'h0000_4000, 8'd0, 3'd2, 2'b01);
        c_w(32'h1, 4'hF, 1'b1);
        c_b;
        check(c_seen && c_got == 2'b11 && u_chk.range_errors == 1,
              "M9: address outside the memory -> DECERR");
        check(u_chk.proto_errors == e0 + 7, "M9: no extra protocol errors");
        u_chk.report;

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end
endmodule

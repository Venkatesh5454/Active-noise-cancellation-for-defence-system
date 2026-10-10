// =============================================================================
// tb_xbar.v  -  unit test: pin crossbar (SPEC 6, slide 14)
// -----------------------------------------------------------------------------
// Five TB "engines" stand in for UART, SPI, I2C, SM0 and SM1.  An engine is
// busy (src_idle = 0) during a frame and only changes its outputs while busy;
// at the end of a frame it goes back to its own idle pattern.
//
// A reference model written from SPEC 6 runs next to the DUT.  EVERY clock
// the TB checks:
//   * DUT switch state (current source, IDLE/WAIT/FLOAT/SETTLE) == model
//   * every pad nibble == out/oe of exactly the selected source (FLOAT: 0)
//   * every src_in nibble == pad_in of the lowest connected port that
//     selects the source, otherwise src_in_idle (settling ports excluded)
//   * ev_switch == "a port is in its hand-over clock"
//   * the register read this clock (random address) == model
// and some properties straight on the DUT, without the model:
//   * NO SWITCH MID-FRAME: a port only floats away from a source that was
//     idle in the last clock it drove the pins (unless FORCE / OFF / GPIO)
//   * FLOAT is exactly 1 clock with oe = 0, and the current source changes
//     only right after a FLOAT clock
//   * ev_switch pulses exactly in the clocks where a port hands over
//
// Parts:
//   A  reset values             F  write of the current value does nothing
//   B  idle switch (2 clocks)   G  one source on several ports: input priority
//   C  SW_EDGE                  H  GPIO registers, GPIO_IN 2-clock sync
//   D  FORCE (1 clock)          I  source 7 = OFF
//   E  wait for a frame end     J  slide-14 demo: JD from SPI to SM0
//   K  random: frames at random times, random PIN_SEL writes (+-FORCE),
//      random GPIO / other writes, random pad_in and src_in_idle
//   L  reset in the middle of activity
// The log prints the measured switch times ("SWITCH TIME" lines) and how
// often each case happened in the random part ("COVERAGE" line).
// =============================================================================
`timescale 1ns / 1ps
module tb_xbar;
    localparam NP    = 6;
    parameter  NRAND = 60000;              // clocks of random testing

    localparam [1:0] S_IDLE = 2'd0, S_WAIT = 2'd1, S_FLOAT = 2'd2, S_SETTLE = 2'd3;
    localparam [7:0] A_STATUS = 8'h20, A_SWC = 8'h40, A_SWE = 8'h60,
                     A_GOUT = 8'h80, A_GOE = 8'h84, A_GIN = 8'h88;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer seed   = 32'h5EED0042;
    integer cyc    = 0;                    // clock number (one per tick)
    integer checks = 0;
    integer errors = 0;

    task check(input cond, input [8*96-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                if (errors <= 30)
                    $display("FAIL: %0s (clock %0d, t=%0t)", what, cyc, $time);
            end
        end
    endtask

    // ---------------- DUT ----------------
    reg         reg_we = 1'b0, reg_re = 1'b0;
    reg  [7:0]  reg_addr = 8'd0;
    reg  [31:0] reg_wdata = 32'd0;
    wire [31:0] reg_rdata;
    reg  [19:0] src_out = 20'd0, src_oe = 20'd0, src_in_idle = 20'd0;
    reg  [4:0]  src_idle = 5'h1F;
    wire [19:0] src_in;
    reg  [4*NP-1:0] pad_in = {(4*NP){1'b0}};
    wire [4*NP-1:0] pad_out, pad_oe;
    wire        ev_switch;

    xbar #(.NP(NP)) dut (
        .clk(clk), .rst_n(rst_n),
        .reg_we(reg_we), .reg_re(reg_re), .reg_addr(reg_addr),
        .reg_wdata(reg_wdata), .reg_rdata(reg_rdata),
        .src_out(src_out), .src_oe(src_oe), .src_in(src_in),
        .src_idle(src_idle), .src_in_idle(src_in_idle),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .ev_switch(ev_switch));

    // peek at the switch state of every port (for the DUT-only properties)
    wire [3*NP-1:0] pk_cur;
    wire [2*NP-1:0] pk_st;
    genvar gp;
    generate
        for (gp = 0; gp < NP; gp = gp + 1) begin : g_peek
            assign pk_cur[3*gp +: 3] = dut.g_port[gp].cur;
            assign pk_st[2*gp +: 2]  = dut.g_port[gp].st;
        end
    endgenerate

    // ---------------- stimulus staging ----------------
    // Directed code never touches the DUT inputs directly: it writes these
    // "next clock" values, and tick applies them at the start of the clock.
    reg [4:0]  manual = 5'd0;              // 1: directed code drives source s
    reg [19:0] man_out = 20'd0, man_oe = 20'd0;
    reg [4:0]  man_idle = 5'h1F;
    reg        pad_rand = 1'b0, sid_rand = 1'b0, auto_frames = 1'b0;
    reg [4*NP-1:0] man_pad = {(4*NP){1'b0}};
    reg [19:0] man_sid = 20'd0;
    reg        bus_set = 1'b0, bus_we = 1'b0;
    reg [7:0]  bus_addr = 8'd0;
    reg [31:0] bus_wdata = 32'd0;
    reg [31:0] last_rdata;

    integer    flen [0:4];                 // remaining busy clocks per engine
    reg [3:0]  idle_out [0:4];
    reg [3:0]  idle_oe  [0:4];

    // ---------------- reference model ----------------
    integer m_cur [0:NP-1];
    integer m_req [0:NP-1];
    integer m_st  [0:NP-1];
    integer m_sc  [0:NP-1];
    integer m_c0  [0:NP-1];                // clock number of counter = 0
    integer m_swc [0:NP-1];
    integer m_swe [0:NP-1];
    integer m_arm [0:NP-1];
    integer m_frc [0:NP-1];
    integer m_peff[0:NP-1];
    reg [4*NP-1:0] m_gout, m_goe, pin_d1, pin_d2;

    // previous-clock copies for the DUT-only properties
    reg            prev_valid = 1'b0;
    reg [3*NP-1:0] prev_cur;
    reg [2*NP-1:0] prev_st;
    reg [4*NP-1:0] prev_pad_out, prev_pad_oe;
    reg [4:0]      prev_idle;

    // coverage counters (random part)
    integer n_req = 0, n_wait = 0, n_force_busy = 0, n_ignored = 0, n_same = 0;
    integer n_ho = 0, n_ev = 0, n_edge = 0, n_multi = 0, n_shadow = 0, n_race = 0;
    integer n_rd = 0;

    function integer rst_src(input integer p);
        begin
            case (p)
                0: rst_src = 1;  1: rst_src = 2;  2: rst_src = 3;  5: rst_src = 6;
                default: rst_src = 0;
            endcase
        end
    endfunction

    task model_reset;
        integer p;
        begin
            for (p = 0; p < NP; p = p + 1) begin
                m_cur[p] = rst_src(p);  m_req[p] = rst_src(p);
                m_st[p]  = S_IDLE;      m_sc[p]  = 0;  m_c0[p] = 0;
                m_swc[p] = 0;  m_swe[p] = 0;  m_arm[p] = 0;  m_frc[p] = 0;
                m_peff[p] = 0;
            end
            m_gout = 0;  m_goe = 0;  pin_d1 = 0;  pin_d2 = 0;
            prev_valid = 1'b0;
        end
    endtask

    // what pad nibble p must show this clock
    task exp_pad(input integer p, output [3:0] o, output [3:0] e);
        begin
            o = 4'd0;  e = 4'd0;
            if (m_st[p] != S_FLOAT) begin
                if (m_cur[p] >= 1 && m_cur[p] <= 5) begin
                    o = src_out[4*(m_cur[p]-1) +: 4];
                    e = src_oe[4*(m_cur[p]-1) +: 4];
                end else if (m_cur[p] == 6) begin
                    o = m_gout[4*p +: 4];
                    e = m_goe[4*p +: 4];
                end
            end
        end
    endtask

    // lowest connected port selecting source s (1..5), or -1
    function integer feed_port(input integer s);
        integer p;
        begin
            feed_port = -1;
            for (p = NP - 1; p >= 0; p = p - 1)
                if (m_cur[p] == s && (m_st[p] == S_IDLE || m_st[p] == S_WAIT))
                    feed_port = p;
        end
    endfunction

    function [31:0] exp_rdata(input [7:0] a);
        integer i;
        reg [31:0] r, q, c;
        begin
            i = a[4:2];
            exp_rdata = 32'd0;
            case (a[7:5])
                3'd0: if (i < NP) begin
                          q = m_req[i];  c = m_cur[i];
                          exp_rdata = {23'd0, (m_st[i] != S_IDLE), 1'b0, q[2:0], 1'b0, c[2:0]};
                      end
                3'd1: if (i == 0) begin
                          r = 32'd0;
                          for (i = 0; i < NP; i = i + 1) r[i] = (m_st[i] != S_IDLE);
                          exp_rdata = r;
                      end
                3'd2: if (i < NP) exp_rdata = m_swc[i];
                3'd3: if (i < NP) exp_rdata = m_swe[i];
                3'd4: case (i)
                          0: exp_rdata = m_gout;
                          1: exp_rdata = m_goe;
                          2: exp_rdata = pin_d2;
                          default: exp_rdata = 32'd0;
                      endcase
                default: exp_rdata = 32'd0;
            endcase
        end
    endfunction

    // the model's "posedge": next state from this clock's inputs
    task model_step;
        integer p, ap;
        reg [3:0] o, e;
        reg [7:0] eff;
        reg       wr_ps, ho_now;
        begin
            wr_ps = reg_we && (reg_addr[7:5] == 3'd0);
            ap    = reg_addr[4:2];
            for (p = 0; p < NP; p = p + 1) begin
                exp_pad(p, o, e);
                eff    = {e, e & o};
                ho_now = (m_st[p] == S_SETTLE) && (m_sc[p] == 0);
                if (m_arm[p] && !ho_now && (eff != m_peff[p])) begin
                    m_swe[p] = cyc - m_c0[p];
                    m_arm[p] = 0;
                    n_edge   = n_edge + 1;
                end
                m_peff[p] = eff;
                if (wr_ps && ap == p && m_st[p] != S_IDLE) n_ignored = n_ignored + 1;
                case (m_st[p])
                    S_IDLE: begin
                        if (wr_ps && ap == p && reg_wdata[2:0] != m_cur[p]) begin
                            m_req[p] = reg_wdata[2:0];
                            m_frc[p] = reg_wdata[8];
                            m_c0[p]  = cyc + 1;
                            m_swe[p] = 0;
                            m_arm[p] = 0;
                            n_req    = n_req + 1;
                            if (reg_wdata[8] || m_cur[p] == 0 || m_cur[p] >= 6) begin
                                m_st[p] = S_FLOAT;
                                if (reg_wdata[8] && m_cur[p] >= 1 && m_cur[p] <= 5 &&
                                    !src_idle[m_cur[p]-1])
                                    n_force_busy = n_force_busy + 1;
                            end else begin
                                m_st[p] = S_WAIT;
                            end
                        end else if (wr_ps && ap == p) begin
                            n_same = n_same + 1;
                        end
                    end
                    S_WAIT: begin
                        if (src_idle[m_cur[p]-1]) m_st[p] = S_FLOAT;
                        else if (cyc == m_c0[p]) n_wait = n_wait + 1;   // had to wait
                    end
                    S_FLOAT: begin
                        m_cur[p] = m_req[p];
                        m_st[p]  = S_SETTLE;
                        m_sc[p]  = 0;
                        m_swc[p] = cyc + 1 - m_c0[p];
                        m_arm[p] = 1;
                    end
                    default: begin
                        if (m_sc[p] == 7) m_st[p] = S_IDLE;
                        else              m_sc[p] = m_sc[p] + 1;
                    end
                endcase
            end
            if (reg_we && reg_addr[7:2] == 6'h20) m_gout = reg_wdata[4*NP-1:0];
            if (reg_we && reg_addr[7:2] == 6'h21) m_goe  = reg_wdata[4*NP-1:0];
            pin_d2 = pin_d1;
            pin_d1 = pad_in;
        end
    endtask

    // ---------------- the checks of one clock ----------------
    task check_cycle;
        integer p, s, fp, nsel;
        reg [3:0] o, e;
        reg       any_ho, dut_ho;
        reg [2:0] pc, old;
        begin
            any_ho = 1'b0;
            dut_ho = 1'b0;
            for (p = 0; p < NP; p = p + 1) begin
                pc = pk_cur[3*p +: 3];
                check(pc == m_cur[p] && pk_st[2*p +: 2] == m_st[p],
                      "DUT switch state == reference model");
                exp_pad(p, o, e);
                check(pad_out[4*p +: 4] == o && pad_oe[4*p +: 4] == e,
                      "pad shows exactly the selected source");
                if (m_st[p] == S_SETTLE && m_sc[p] == 0) any_ho = 1'b1;

                // ---- DUT-only properties ----
                if (pk_st[2*p +: 2] == S_FLOAT)
                    check(pad_oe[4*p +: 4] == 4'd0 && pad_out[4*p +: 4] == 4'd0,
                          "FLOAT: all 4 pins oe = 0");
                if (prev_valid) begin
                    if (pc != prev_cur[3*p +: 3]) begin
                        dut_ho = 1'b1;
                        check(prev_st[2*p +: 2] == S_FLOAT && prev_pad_oe[4*p +: 4] == 4'd0,
                              "source changes only right after a FLOAT clock");
                    end
                    if (prev_st[2*p +: 2] == S_FLOAT)
                        check(pk_st[2*p +: 2] != S_FLOAT, "FLOAT lasts exactly 1 clock");
                    if (pk_st[2*p +: 2] == S_FLOAT && prev_st[2*p +: 2] != S_FLOAT) begin
                        old = pc;
                        if (old >= 3'd1 && old <= 3'd5 && !m_frc[p])
                            check(prev_idle[old-1],
                                  "no switch mid-frame: old source idle in its last driven clock");
                    end
                    // old source started a frame in its FLOAT clock (it never reaches the pins)
                    if (prev_st[2*p +: 2] == S_FLOAT && pc != prev_cur[3*p +: 3]) begin
                        old = prev_cur[3*p +: 3];
                        if (old >= 3'd1 && old <= 3'd5 && !prev_idle[old-1] && !m_frc[p])
                            n_race = n_race + 1;
                    end
                end
            end
            // inputs of every source
            for (s = 1; s <= 5; s = s + 1) begin
                fp = feed_port(s);
                if (fp >= 0)
                    check(src_in[4*(s-1) +: 4] == pad_in[4*fp +: 4],
                          "src_in = pad_in of the lowest connected port");
                else
                    check(src_in[4*(s-1) +: 4] == src_in_idle[4*(s-1) +: 4],
                          "src_in = src_in_idle when not connected / settling");
                nsel = 0;
                for (p = 0; p < NP; p = p + 1)
                    if (m_cur[p] == s && m_st[p] != S_FLOAT) nsel = nsel + 1;
                if (nsel >= 2) n_multi = n_multi + 1;
                for (p = 0; p < NP; p = p + 1)
                    if (m_cur[p] == s && m_st[p] == S_SETTLE && fp > p) n_shadow = n_shadow + 1;
            end
            check(ev_switch == any_ho, "ev_switch == hand-over clock (model)");
            if (prev_valid) check(ev_switch == dut_ho, "ev_switch pulses once per hand-over");
            if (dut_ho) n_ho = n_ho + 1;
            if (ev_switch) n_ev = n_ev + 1;
            check(reg_rdata == exp_rdata(reg_addr), "register read == model");
            n_rd = n_rd + 1;
        end
    endtask

    // ---------------- engines ----------------
    task engines_step;
        integer s;
        reg [3:0] o, e;
        begin
            for (s = 0; s < 5; s = s + 1) begin
                if (manual[s]) begin
                    src_out[4*s +: 4] = man_out[4*s +: 4];
                    src_oe[4*s +: 4]  = man_oe[4*s +: 4];
                    src_idle[s]       = man_idle[s];
                end else begin
                    if (flen[s] == 0 && auto_frames && ({$random(seed)} % 160 == 0)) begin
                        if ({$random(seed)} % 8 == 0) flen[s] = 2 + {$random(seed)} % 700;
                        else                          flen[s] = 2 + {$random(seed)} % 100;
                    end
                    if (flen[s] > 0) begin
                        src_idle[s] = 1'b0;            // mid-frame
                        o = src_out[4*s +: 4];
                        e = src_oe[4*s +: 4];
                        if (flen[s] == 1) begin        // last frame clock: back to idle
                            o = idle_out[s];  e = idle_oe[s];
                        end else if ({$random(seed)} % 3 == 0) begin
                            o = $random(seed);  e = $random(seed);
                        end
                        src_out[4*s +: 4] = o;
                        src_oe[4*s +: 4]  = e;
                        flen[s] = flen[s] - 1;
                    end else begin
                        src_idle[s] = 1'b1;            // outputs hold their idle pattern
                    end
                end
            end
        end
    endtask

    // pick a random register read when the directed code did not ask for one
    task pick_read;
        integer r;
        begin
            r = {$random(seed)} % 8;
            case (r)
                0: bus_addr = {$random(seed)} % 256;
                1: bus_addr = A_STATUS;
                2: bus_addr = 8'h80 + 4 * ({$random(seed)} % 3);
                3, 4: bus_addr = A_SWC + 4 * ({$random(seed)} % 8);
                5: bus_addr = A_SWE + 4 * ({$random(seed)} % 8);
                default: bus_addr = 4 * ({$random(seed)} % 8);          // PIN_SEL
            endcase
            bus_addr[1:0] = $random(seed);                             // must be ignored
        end
    endtask

    // ---------------- one clock ----------------
    task tick_body;
        begin
            cyc = cyc + 1;
            engines_step;
            pad_in = pad_rand ? $random(seed) : man_pad;
            if (sid_rand) begin
                if ({$random(seed)} % 400 == 0) src_in_idle = $random(seed);
            end else begin
                src_in_idle = man_sid;
            end
            if (!bus_set) begin
                bus_we = 1'b0;
                pick_read;
            end
            reg_we    = bus_we;
            reg_re    = ~bus_we;
            reg_addr  = bus_addr;
            reg_wdata = bus_wdata;
            #1;
            check_cycle;
            last_rdata = reg_rdata;
            model_step;
            prev_valid   = 1'b1;
            prev_cur     = pk_cur;
            prev_st      = pk_st;
            prev_pad_out = pad_out;
            prev_pad_oe  = pad_oe;
            prev_idle    = src_idle;
            bus_set = 1'b0;
            bus_we  = 1'b0;
        end
    endtask

    task tick;
        begin
            @(negedge clk);
            tick_body;
        end
    endtask

    task wr(input [7:0] a, input [31:0] d);
        begin
            bus_set = 1'b1;  bus_we = 1'b1;  bus_addr = a;  bus_wdata = d;
            tick;
        end
    endtask

    task rd(input [7:0] a, output [31:0] d);
        begin
            bus_set = 1'b1;  bus_we = 1'b0;  bus_addr = a;
            tick;
            d = last_rdata;
        end
    endtask

    // write PIN_SEL[p] and wait until the switch is over (sources idle)
    task sel(input integer p, input [2:0] v);
        integer n;
        begin
            wr(4 * p, {23'd0, 1'b0, 5'd0, v});
            n = 0;
            while (m_st[p] != S_IDLE && n < 2000) begin
                tick;
                n = n + 1;
            end
            check(m_cur[p] == v, "directed switch finished");
        end
    endtask

    task do_reset;
        begin
            @(negedge clk);
            rst_n = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;              // released at a falling edge: no race
            model_reset;
            tick_body;                 // first clock after reset
        end
    endtask

    // ---------------- test sequence ----------------
    integer   k, s, w_cyc, c0, i_cyc, h_cyc, e_cyc, n, b, ph;
    integer   t_idle, t_force, t_off, demo_swc, demo_swe, demo_wait;
    reg [31:0] d, d2;
    reg [7:0]  spi_byte;
    reg        sck, mosi, csn, sq, frame_ok, seen_float;

    initial begin
        for (s = 0; s < 5; s = s + 1) begin
            flen[s] = 0;
            idle_out[s] = 4'd0;
            idle_oe[s]  = 4'd0;
        end
        model_reset;

        // ========== A: reset values ==========
        manual   = 5'h1F;
        man_idle = 5'h1F;
        //          SM1     SM0     I2C     SPI     UART
        man_out  = {4'h3,   4'h6,   4'h0,   4'h9,   4'hA};
        man_oe   = {4'h7,   4'hF,   4'h4,   4'hB,   4'hA};
        man_pad  = 24'h9C5A3E;
        man_sid  = 20'h1E2D4;
        do_reset;
        tick;
        check(pad_out[3:0] == 4'hA && pad_oe[3:0] == 4'hA, "reset: JA = UART");
        check(pad_out[7:4] == 4'h9 && pad_oe[7:4] == 4'hB, "reset: JB = SPI");
        check(pad_out[11:8] == 4'h0 && pad_oe[11:8] == 4'h4, "reset: JC = I2C");
        check(pad_oe[15:12] == 4'h0 && pad_oe[19:16] == 4'h0, "reset: JD, OLED = OFF");
        check(pad_oe[23:20] == 4'h0 && pad_out[23:20] == 4'h0, "reset: LED = GPIO (0)");
        check(src_in[3:0] == 4'hE && src_in[7:4] == 4'h3 && src_in[11:8] == 4'hA,
              "reset: UART/SPI/I2C see JA/JB/JC");
        check(src_in[19:12] == 8'h1E, "reset: SM0/SM1 see src_in_idle");
        rd(8'h00, d);  check(d == 32'h011, "reset: PIN_SEL[JA] = 0x011");
        rd(8'h04, d);  check(d == 32'h022, "reset: PIN_SEL[JB] = 0x022");
        rd(8'h08, d);  check(d == 32'h033, "reset: PIN_SEL[JC] = 0x033");
        rd(8'h0C, d);  check(d == 32'h000, "reset: PIN_SEL[JD] = 0");
        rd(8'h10, d);  check(d == 32'h000, "reset: PIN_SEL[OLED] = 0");
        rd(8'h14, d);  check(d == 32'h066, "reset: PIN_SEL[LED] = 0x066");
        rd(8'h18, d);  check(d == 32'h000, "PIN_SEL[6] (no port) reads 0");
        rd(A_STATUS, d); check(d == 32'h0, "reset: XBAR_STATUS = 0");
        for (k = 0; k < NP; k = k + 1) begin
            rd(A_SWC + 4 * k, d);  check(d == 0, "reset: SW_CYCLES = 0");
            rd(A_SWE + 4 * k, d);  check(d == 0, "reset: SW_EDGE = 0");
        end
        rd(A_GOUT, d); check(d == 0, "reset: GPIO_OUT = 0");
        rd(A_GOE, d);  check(d == 0, "reset: GPIO_OE = 0");
        rd(A_GIN, d);  check(d == 32'h009C5A3E, "GPIO_IN = pad_in");
        rd(8'h24, d);  check(d == 0, "unused 0x24 reads 0");
        rd(8'h8C, d);  check(d == 0, "unused 0x8C reads 0");
        rd(8'hFC, d);  check(d == 0, "unused 0xFC reads 0");

        // ========== B: idle switch, JA from UART (idle) to SM1 ==========
        man_pad[3:0] = 4'h5;  man_sid[19:16] = 4'hA;     // JA pins != SM1 idle level
        wr(8'h00, 32'h0000_0005);  w_cyc = cyc;
        rd(8'h00, d);
        check(d == 32'h151, "B clock 0: WAIT_IDLE, PIN_SEL = switching, req 5, cur 1");
        check(pad_out[3:0] == 4'hA && pad_oe[3:0] == 4'hA, "B clock 0: UART still drives JA");
        rd(A_STATUS, d);
        check(d == 32'h1 && pad_oe[3:0] == 4'h0, "B clock 1: FLOAT, oe = 0, XBAR_STATUS[0] = 1");
        wr(8'h00, 32'h0000_0002);                         // during SETTLE: ignored
        check(ev_switch && pad_out[3:0] == 4'h3 && pad_oe[3:0] == 4'h7,
              "B clock 2: hand-over, ev_switch, SM1 drives JA");
        check(src_in[19:16] == 4'hA, "B settle clock 1: SM1 sees src_in_idle");
        for (k = 2; k <= 8; k = k + 1) begin
            tick;
            check(src_in[19:16] == 4'hA && !ev_switch, "B settle: SM1 sees src_in_idle");
        end
        rd(8'h00, d);
        check(d == 32'h055, "B after settle: PIN_SEL = cur 5, req 5 (write in SETTLE ignored)");
        check(src_in[19:16] == 4'h5, "B after 8 settle clocks: SM1 sees JA");
        check(src_in[3:0] == 4'h4, "B: UART (no port) sees src_in_idle");
        rd(A_SWC, d);
        t_idle = d;
        check(d == 2, "B: SW_CYCLES = 2 for an idle source");

        // ========== C: SW_EDGE on JA (SM1 oe = 0111) ==========
        c0 = w_cyc + 1;
        repeat (10) tick;
        rd(A_SWE, d);  check(d == 0, "C: SW_EDGE = 0 while SM1 does not move");
        man_out[19:16] = 4'hB;             // pin 3 changes but oe[3] = 0: not an edge
        repeat (5) tick;
        rd(A_SWE, d);  check(d == 0, "C: an undriven pin change is not an edge");
        man_out[19:16] = 4'h9;             // pin 1 (driven) goes 1 -> 0
        tick;  e_cyc = cyc;
        tick;
        rd(A_SWE, d);  check(d == e_cyc - c0, "C: SW_EDGE = clocks from request to first edge");
        man_oe[19:16] = 4'hF;              // later changes do not move SW_EDGE
        repeat (3) tick;
        rd(A_SWE, d2); check(d2 == d, "C: SW_EDGE keeps the FIRST edge");
        man_out[19:16] = 4'h3;  man_oe[19:16] = 4'h7;

        // ========== D: FORCE, JB from SPI (busy) to SM0 ==========
        man_idle[1] = 1'b0;                // SPI in the middle of a frame
        repeat (3) tick;
        wr(8'h04, 32'h0000_0104);  w_cyc = cyc;
        tick;
        check(pk_st[3:2] == S_FLOAT && pad_oe[7:4] == 4'h0, "D clock 0: FORCE goes straight to FLOAT");
        tick;
        check(ev_switch && pad_out[7:4] == 4'h6 && pad_oe[7:4] == 4'hF, "D clock 1: SM0 drives JB");
        rd(A_SWC + 4, d);
        t_force = d;
        check(d == 1, "D: SW_CYCLES = 1 with FORCE although SPI is busy");
        man_idle[1] = 1'b1;
        repeat (8) tick;

        // ========== E: JC from I2C (busy) to UART: wait for the frame end ==========
        man_idle[2] = 1'b0;
        repeat (2) tick;
        wr(8'h08, 32'h0000_0001);  w_cyc = cyc;
        frame_ok = 1'b1;
        for (k = 0; k < 30; k = k + 1) begin
            man_oe[11:8] = {k[0], k[1], 2'b00};          // I2C keeps toggling
            if (k == 29) man_oe[11:8] = 4'h4;            // last frame clock: idle level
            if (k == 10) begin bus_set = 1'b1; bus_we = 1'b1; bus_addr = 8'h08; bus_wdata = 32'h4; end
            if (k == 12) begin bus_set = 1'b1; bus_we = 1'b1; bus_addr = 8'h08; bus_wdata = 32'h105; end
            tick;
            if (pad_oe[11:8] != src_oe[11:8] || pk_st[5:4] != S_WAIT) frame_ok = 1'b0;
        end
        check(frame_ok, "E: I2C keeps JC while it is busy (WAIT_IDLE)");
        rd(8'h08, d);
        check(d == 32'h113, "E: writes during WAIT_IDLE ignored (req stays 1)");
        man_idle[2] = 1'b1;
        tick;  i_cyc = cyc;                               // first idle clock
        tick;  check(pk_st[5:4] == S_FLOAT && pad_oe[11:8] == 4'h0, "E: FLOAT right after the frame");
        tick;  check(ev_switch && pad_oe[11:8] == 4'hA, "E: UART drives JC");
        rd(A_SWC + 8, d);
        check(d == i_cyc + 2 - (w_cyc + 1), "E: SW_CYCLES = wait + 2");
        $display("E: JC I2C -> UART requested mid-frame: waited %0d clocks, SW_CYCLES = %0d",
                 i_cyc - w_cyc, d);
        repeat (8) tick;

        // ========== F: writing the current value does nothing ==========
        man_out[3:0] = 4'h8;               // make an edge on JC so SW_EDGE != 0
        repeat (2) tick;
        rd(A_SWE + 8, d);
        check(d != 0, "F: SW_EDGE of JC latched");
        wr(8'h08, 32'h0000_0001);
        rd(8'h08, d2);
        check(d2 == 32'h011, "F: same-value write does not start a switch");
        check(!ev_switch, "F: no ev_switch");
        rd(A_SWE + 8, d2);
        check(d2 == d, "F: same-value write keeps SW_EDGE");
        wr(8'h08, 32'h0000_0101);          // also with FORCE
        rd(A_STATUS, d2);
        check(d2 == 0, "F: same value + FORCE does nothing");

        // ========== G: one source on several ports ==========
        // now: JA SM1, JB SM0, JC UART, JD OFF, OLED OFF, LED GPIO
        man_pad = 24'h654321;              // port p pins = p + 1
        sel(3, 3'd4);                      // JD -> SM0
        sel(4, 3'd4);                      // OLED -> SM0
        tick;
        check(pad_out[7:4] == 4'h6 && pad_out[15:12] == 4'h6 && pad_out[19:16] == 4'h6 &&
              pad_oe[7:4] == 4'hF && pad_oe[15:12] == 4'hF && pad_oe[19:16] == 4'hF,
              "G: SM0 outputs on JB, JD and OLED");
        check(src_in[15:12] == 4'h2, "G: SM0 input from JB (lowest port)");
        wr(8'h04, 32'h0);                  // JB -> OFF (SM0 is idle)
        tick;
        check(pk_st[3:2] == S_WAIT && src_in[15:12] == 4'h2, "G: JB in WAIT_IDLE still feeds SM0");
        tick;
        check(pk_st[3:2] == S_FLOAT && src_in[15:12] == 4'h4, "G: JB floats: SM0 input from JD");
        repeat (10) tick;
        check(src_in[15:12] == 4'h4, "G: JB off: SM0 input from JD");
        wr(8'h0C, 32'h5);                  // JD -> SM1 (SM0 idle)
        tick;
        check(pk_st[7:6] == S_WAIT && src_in[15:12] == 4'h4, "G: JD in WAIT_IDLE still feeds SM0");
        tick;
        check(src_in[15:12] == 4'h5 && src_in[19:16] == 4'h1, "G: JD floats: SM0 from OLED, SM1 from JA");
        tick;
        check(src_in[19:16] == 4'h1, "G: JD settling on SM1: SM1 keeps JA");
        repeat (10) tick;
        wr(8'h04, 32'h4);                  // JB back to SM0 (from OFF)
        for (k = 0; k < 9; k = k + 1) begin
            tick;
            check(src_in[15:12] == 4'h5, "G: JB floating/settling: SM0 keeps OLED");
        end
        tick;
        check(src_in[15:12] == 4'h2, "G: JB settled: SM0 input from JB again");
        sel(0, 3'd0);                      // JA -> OFF
        repeat (2) tick;
        check(src_in[19:16] == 4'h4, "G: SM1 now from JD");

        // ========== H: GPIO ==========
        wr(A_GOUT, 32'hFF5A_C3E1);         // bits above 4*NP are ignored
        wr(A_GOE,  32'h00F0_0F0F);
        rd(A_GOUT, d);  check(d == 32'h005A_C3E1, "H: GPIO_OUT = 4 bits per port");
        rd(A_GOE, d);   check(d == 32'h00F0_0F0F, "H: GPIO_OE");
        check(pad_out[23:20] == 4'h5 && pad_oe[23:20] == 4'hF, "H: LED shows GPIO nibble 5");
        sel(3, 3'd6);                      // JD -> GPIO
        tick;
        check(pad_out[15:12] == 4'hC && pad_oe[15:12] == 4'h0, "H: JD shows GPIO nibble 3");
        man_pad = 24'hABCDEF;
        repeat (3) tick;
        man_pad = 24'h123456;
        rd(A_GIN, d);  check(d == 32'h00ABCDEF, "H: GPIO_IN 2 clocks late (old value)");
        rd(A_GIN, d);  check(d == 32'h00ABCDEF, "H: GPIO_IN 2 clocks late (still old)");
        rd(A_GIN, d);  check(d == 32'h00123456, "H: GPIO_IN 2 clocks late (new value)");

        // ========== I: source 7 = OFF ==========
        wr(8'h0C, 32'h7);  w_cyc = cyc;    // JD GPIO -> 7: no wait (old is GPIO)
        tick;
        check(pk_st[7:6] == S_FLOAT, "I: from GPIO straight to FLOAT");
        tick;
        check(ev_switch && pad_oe[15:12] == 4'h0 && pad_out[15:12] == 4'h0, "I: JD = 7 is OFF");
        rd(A_SWC + 12, d);
        t_off = d;
        check(d == 1, "I: SW_CYCLES = 1 from GPIO");
        repeat (8) tick;
        rd(8'h0C, d);  check(d == 32'h077, "I: PIN_SEL[JD] reads 7");
        sel(3, 3'd0);
        rd(8'h0C, d);  check(d == 32'h000, "I: 7 -> 0 is a (harmless) switch");

        // ========== J: slide-14 demo, JD from SPI to SM0 ==========
        sel(3, 3'd2);                      // JD -> SPI (from OFF: 1 clock)
        repeat (4) tick;
        spi_byte   = 8'h9F;                // JEDEC ID command
        h_cyc      = -1;
        w_cyc      = -1;
        frame_ok   = 1'b1;
        seen_float = 1'b0;
        e_cyc      = -1;
        for (k = 0; k < 120; k = k + 1) begin
            // SPI master, mode 0, SCK = clk/8: CS# low 2 clocks, 8 bits, CS# high
            if (k < 68) begin
                csn = (k == 67);
                if (k >= 2 && k < 66) begin
                    b = (k - 2) / 8;  ph = (k - 2) % 8;
                    sck = (ph >= 4);  mosi = spi_byte[7 - b];
                end else begin
                    sck = 1'b0;  mosi = 1'b0;
                end
                man_idle[1] = 1'b0;
            end else begin
                csn = 1'b1;  sck = 1'b0;  mosi = 1'b0;
                man_idle[1] = 1'b1;
            end
            man_out[7:4] = {sck, 1'b0, mosi, csn};
            man_oe[7:4]  = 4'b1011;
            // SM0 is already running: a square wave on pin 1, period 24 clocks
            sq = ((k / 12) % 2 == 1);
            man_out[15:12] = {2'b00, sq, 1'b1};
            man_oe[15:12]  = 4'b0011;
            if (k == 25) begin             // request in the middle of the SPI frame
                bus_set = 1'b1;  bus_we = 1'b1;  bus_addr = 8'h0C;  bus_wdata = 32'h4;
            end
            tick;
            if (k == 25) w_cyc = cyc;
            if (k == 68) i_cyc = cyc;      // first SPI idle clock
            if (k < 68 && (pad_out[15:12] != man_out[7:4] || pad_oe[15:12] != 4'b1011))
                frame_ok = 1'b0;           // the whole SPI frame must reach JD
            if (pk_st[7:6] == S_FLOAT) seen_float = 1'b1;
            if (ev_switch && h_cyc < 0) h_cyc = cyc;
            if (h_cyc >= 0 && cyc > h_cyc && e_cyc < 0 && k % 12 == 0) e_cyc = cyc;
        end
        check(frame_ok, "J: the SPI frame on JD was never cut");
        check(seen_float && h_cyc == i_cyc + 2, "J: FLOAT then hand-over right after the frame");
        rd(A_SWC + 12, d);
        demo_swc  = d;
        demo_wait = i_cyc - w_cyc - 1;
        check(d == h_cyc - (w_cyc + 1), "J: SW_CYCLES = clocks from request to hand-over");
        rd(A_SWE + 12, d);
        demo_swe = d;
        check(d == e_cyc - (w_cyc + 1), "J: SW_EDGE = clocks from request to first SM0 edge");
        check(pad_out[15:12] == man_out[15:12] && pad_oe[15:12] == 4'b0011, "J: SM0 drives JD");

        $display("SWITCH TIME: idle old source (JA UART -> SM1): SW_CYCLES = %0d clocks (%0d ns);",
                 t_idle, 10 * t_idle);
        $display("             the new source sees its pins after 8 more settle clocks.");
        $display("SWITCH TIME: FORCE (JB SPI busy -> SM0) = %0d clock, from GPIO/OFF = %0d clock", t_force, t_off);
        $display("SWITCH TIME: slide-14 demo, JD SPI -> SM0 written in clock 25 of a 68-clock SPI");
        $display("             frame (0x9F, SCK = clk/8): waited %0d clocks for the frame end, SW_CYCLES = %0d,",
                 demo_wait, demo_swc);
        $display("             first SM0 edge on JD at SW_EDGE = %0d (%0d clocks after hand-over)",
                 demo_swe, demo_swe - demo_swc);

        // ========== K: random ==========
        manual      = 5'd0;
        for (s = 0; s < 5; s = s + 1) begin
            idle_out[s] = $random(seed);
            idle_oe[s]  = $random(seed);
        end
        auto_frames = 1'b1;
        pad_rand    = 1'b1;
        sid_rand    = 1'b1;
        n_req = 0;  n_wait = 0;  n_force_busy = 0;  n_ignored = 0;  n_same = 0;
        n_ho = 0;   n_ev = 0;    n_edge = 0;  n_multi = 0;  n_shadow = 0;  n_race = 0;
        for (k = 0; k < NRAND; k = k + 1) begin
            n = {$random(seed)} % 1000;
            if (n < 40) begin                               // PIN_SEL write
                bus_set = 1'b1;  bus_we = 1'b1;
                if ({$random(seed)} % 12 == 0) b = 6 + {$random(seed)} % 2;   // no such port
                else                           b = {$random(seed)} % NP;
                bus_addr  = 4 * b + ({$random(seed)} % 4);
                bus_wdata = $random(seed) & 32'hFFFF_FEF8;  // junk in the unused bits
                if ({$random(seed)} % 6 == 0) bus_wdata[2:0] = $random(seed);
                else                          bus_wdata[2:0] = 1 + {$random(seed)} % 6;
                bus_wdata[8] = ({$random(seed)} % 5 == 0);  // FORCE
            end else if (n < 44) begin                      // GPIO_OUT / GPIO_OE
                bus_set = 1'b1;  bus_we = 1'b1;
                bus_addr  = ({$random(seed)} % 2) ? A_GOUT : A_GOE;
                bus_wdata = $random(seed);
            end else if (n < 48) begin                      // any address (RO ones too)
                bus_set = 1'b1;  bus_we = 1'b1;
                bus_addr  = $random(seed);
                bus_wdata = $random(seed);
            end
            tick;
        end
        $display("COVERAGE: %0d switches (%0d waited for a frame end, %0d forced while busy), %0d hand-overs, %0d ev_switch clocks",
                 n_req, n_wait, n_force_busy, n_ho, n_ev);
        $display("COVERAGE: %0d writes ignored while switching, %0d same-value writes, %0d SW_EDGE latches",
                 n_ignored, n_same, n_edge);
        $display("COVERAGE: %0d source-clocks on 2+ ports, %0d settle-shadow clocks, %0d frames started in a FLOAT clock",
                 n_multi, n_shadow, n_race);
        check(n_req > 300 && n_wait > 30 && n_force_busy > 10 && n_ignored > 50 &&
              n_same > 10 && n_edge > 100 && n_multi > 1000 && n_shadow > 20,
              "random part covered every case");

        // ========== L: reset in the middle of activity ==========
        do_reset;
        check(pad_out[3:0] == src_out[3:0] && pad_oe[3:0] == src_oe[3:0], "L: after reset JA = UART");
        rd(8'h00, d);  check(d == 32'h011, "L: PIN_SEL[JA] back to UART");
        rd(8'h14, d);  check(d == 32'h066, "L: PIN_SEL[LED] back to GPIO");
        rd(A_STATUS, d); check(d == 0, "L: nothing switching after reset");
        rd(A_SWC + 12, d); check(d == 0, "L: SW_CYCLES cleared by reset");
        repeat (200) tick;

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end

    // watchdog
    initial begin
        #10_000_000;
        $display("FAIL: watchdog time-out");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule

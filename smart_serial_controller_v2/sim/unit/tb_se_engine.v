// =============================================================================
// tb_se_engine.v  -  unit test: serial engine (se_engine + 2 x se_sm)
// -----------------------------------------------------------------------------
// Everything is done through the register port, the way the CPU would do it:
// write the program into PROG[], configure the SM, then run instructions one
// at a time with SMs_EXEC or run whole programs with the SM enabled.
//
//   A. reset values, register read-back, unused addresses, PROG memory
//   B. TX FIFO: levels, full, drop when full, PULL block / noblock (copies X)
//   C. SET x / y and every JMP condition; EXEC does not advance the PC
//   D. OUT and IN, both shift directions (directed + 48 random cases checked
//      against a bit-by-bit model), every IN source, OUT isr / pc / null
//   E. pins: SET pins + SET_COUNT + wrap, SET/OUT pindirs, OUT pins, OD_MASK,
//      the OD instruction, side-set (also while stalled, and its priority)
//   F. PUSH: RX levels, PUSH noblock overflow flag (clear on read), PUSH
//      block stall, rx_not_empty, SM1 overflow bit
//   G. CRC: CRC-16/CCITT-FALSE("123456789") = 0x29B1 with IN left, IN right,
//      OUT crc left and right; CRC reset; another POLY/INIT (0xFEE8);
//      CRC 32 against a model; CRC n stalls the SM for n clocks
//   H. running SMs: delays, side-set + delay, clock divider (108.5 average),
//      WAIT + 2-clock input synchroniser, stall status, sm_idle, freeze on
//      SM_EN = 0, EXEC on a running SM, RESTART, two SMs in lock-step
//   I. ni_* FIFO ports, CPU priority, out/in_shift_left
//   J. programs/uart_tx.hex (assembler output, loaded with $readmemh) sends
//      bytes that a TB UART receiver decodes: frame 0 1000 0010 1 for 'A',
//      bit time 8.68 us +-1 %, back-to-back bytes, sm_idle at the end
// Ends with "ALL n CHECKS PASSED" or "m OF n CHECKS FAILED".
// =============================================================================
`timescale 1ns / 1ps
module tb_se_engine;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0;
    integer errors = 0;
    integer seed = 32'h5E5E_2026;

    task check(input cond, input [8*64-1:0] what);
        begin
            checks = checks + 1;
            if (cond !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    task check_eq(input [31:0] got, input [31:0] exp, input [8*64-1:0] what);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL: %0s: got %h, expected %h (t=%0t)", what, got, exp, $time);
            end
        end
    endtask

    // ---------------- DUT ----------------
    reg  [31:0] time_us = 32'd0;
    reg         reg_we = 1'b0, reg_re = 1'b0;
    reg  [8:0]  reg_addr = 9'd0;
    reg  [31:0] reg_wdata = 32'd0;
    wire [31:0] reg_rdata;
    reg  [1:0]  ni_tx_push = 2'b00;
    reg  [31:0] ni_tx_data0 = 32'd0, ni_tx_data1 = 32'd0;
    wire [1:0]  ni_tx_full;
    reg  [1:0]  ni_rx_pop = 2'b00;
    wire [31:0] ni_rx_data0, ni_rx_data1;
    wire [1:0]  ni_rx_empty;
    wire [1:0]  out_shift_left, in_shift_left;
    wire [3:0]  sm0_out, sm0_oe, sm1_out, sm1_oe;
    reg  [3:0]  sm0_in = 4'hF, sm1_in = 4'hF;
    wire [1:0]  sm_idle, rx_not_empty;

    se_engine dut (
        .clk(clk), .rst_n(rst_n), .time_us(time_us),
        .reg_we(reg_we), .reg_re(reg_re), .reg_addr(reg_addr),
        .reg_wdata(reg_wdata), .reg_rdata(reg_rdata),
        .ni_tx_push(ni_tx_push), .ni_tx_data0(ni_tx_data0), .ni_tx_data1(ni_tx_data1),
        .ni_tx_full(ni_tx_full),
        .ni_rx_pop(ni_rx_pop), .ni_rx_data0(ni_rx_data0), .ni_rx_data1(ni_rx_data1),
        .ni_rx_empty(ni_rx_empty),
        .out_shift_left(out_shift_left), .in_shift_left(in_shift_left),
        .sm0_out(sm0_out), .sm0_oe(sm0_oe), .sm0_in(sm0_in),
        .sm1_out(sm1_out), .sm1_oe(sm1_oe), .sm1_in(sm1_in),
        .sm_idle(sm_idle), .rx_not_empty(rx_not_empty)
    );

    // ---------------- register map ----------------
    localparam [8:0] A_CTRL = 9'h000, A_FSTAT = 9'h004, A_CRCCFG = 9'h008, A_TIME = 9'h00C;
    localparam [8:0] O_CLKDIV = 9'h010, O_PINCTRL = 9'h014, O_SHIFT = 9'h018, O_TXF = 9'h01C,
                     O_RXF = 9'h020, O_EXEC = 9'h024, O_STATE = 9'h028, O_X = 9'h02C,
                     O_Y = 9'h030, O_CRC = 9'h034, O_PINS = 9'h038;

    function [8:0] SA(input integer s, input [8:0] off);
        SA = off + ((s == 1) ? 9'h040 : 9'h000);
    endfunction

    function [31:0] pinctrl_f(input [1:0] out_base, input [1:0] set_base, input [2:0] set_cnt,
                              input [1:0] in_base, input [1:0] side_pin, input side_en,
                              input [1:0] jmp_pin, input [3:0] od, input [3:0] init_out,
                              input [3:0] init_oe);
        pinctrl_f = {4'd0, init_oe, init_out, od, 1'b0, jmp_pin, side_en, side_pin,
                     in_base, 1'b0, set_cnt, set_base, out_base};
    endfunction

    // ---------------- instruction encoders (SPEC 5.1) ----------------
    localparam [2:0] C_ALW = 0, C_NX = 1, C_XDEC = 2, C_NY = 3, C_YDEC = 4, C_XNEY = 5,
                     C_PIN = 6, C_NOSRE = 7;
    localparam [2:0] S_PINS = 0, S_X = 1, S_Y = 2, S_NULL = 3, S_TIME = 4, S_CRC = 5,
                     S_ISR = 6, S_OSR = 7;
    localparam [2:0] D_PINS = 0, D_X = 1, D_Y = 2, D_NULL = 3, D_PINDIRS = 4, D_PC = 5,
                     D_ISR = 6, D_CRC = 7;

    function [15:0] i_jmp(input [2:0] c, input [4:0] a, input [4:0] ds);
        i_jmp = {3'd0, ds, c, a};
    endfunction
    function [15:0] i_wait(input lvl, input [1:0] pin, input [4:0] ds);
        i_wait = {3'd1, ds, lvl, 5'd0, pin};
    endfunction
    function [15:0] i_in(input [2:0] src, input [5:0] n, input [4:0] ds);
        i_in = {3'd2, ds, src, n[4:0]};
    endfunction
    function [15:0] i_out(input [2:0] dst, input [5:0] n, input [4:0] ds);
        i_out = {3'd3, ds, dst, n[4:0]};
    endfunction
    function [15:0] i_push(input blk, input [4:0] ds);
        i_push = {3'd4, ds, 1'b0, blk, 6'd0};
    endfunction
    function [15:0] i_pull(input blk, input [4:0] ds);
        i_pull = {3'd4, ds, 1'b1, blk, 6'd0};
    endfunction
    function [15:0] i_set(input [2:0] dst, input [4:0] v, input [4:0] ds);
        i_set = {3'd5, ds, dst, v};
    endfunction
    function [15:0] i_od(input z, input [1:0] pin, input [4:0] ds);
        i_od = {3'd6, ds, z, 5'd0, pin};
    endfunction
    function [15:0] i_crc(input [5:0] n, input [4:0] ds);
        i_crc = {3'd7, ds, 3'd0, n[4:0]};
    endfunction
    function [15:0] i_crcrst(input [4:0] ds);
        i_crcrst = {3'd7, ds, 8'h80};
    endfunction

    // ---------------- reference models ----------------
    function [31:0] m_outdata(input [31:0] osr, input [5:0] n, input left);
        integer i;
        begin
            m_outdata = 32'd0;
            for (i = 0; i < 32; i = i + 1)
                if (i < n) begin
                    if (left) m_outdata[i] = osr[32 - n + i];
                    else      m_outdata[i] = osr[i];
                end
        end
    endfunction
    function [31:0] m_osrshift(input [31:0] osr, input [5:0] n, input left);
        integer i;
        begin
            m_osrshift = 32'd0;
            for (i = 0; i < 32; i = i + 1) begin
                if (left) begin
                    if (i >= n) m_osrshift[i] = osr[i - n];
                end else begin
                    if (i + n < 32) m_osrshift[i] = osr[i + n];
                end
            end
        end
    endfunction
    function [31:0] m_isrin(input [31:0] isr, input [31:0] d, input [5:0] n, input left);
        integer i;
        begin
            m_isrin = 32'd0;
            for (i = 0; i < 32; i = i + 1) begin
                if (left) begin
                    if (i >= n) m_isrin[i] = isr[i - n];
                    else        m_isrin[i] = d[i];
                end else begin
                    if (i < 32 - n) m_isrin[i] = isr[i + n];
                    else            m_isrin[i] = d[i - (32 - n)];
                end
            end
        end
    endfunction
    function [15:0] crc_bit(input [15:0] c, input b, input [15:0] poly);
        crc_bit = {c[14:0], 1'b0} ^ ((c[15] ^ b) ? poly : 16'h0000);
    endfunction
    function [7:0] rev8(input [7:0] v);
        integer i;
        for (i = 0; i < 8; i = i + 1) rev8[i] = v[7 - i];
    endfunction

    // ---------------- register access tasks ----------------
    reg [31:0] rv;

    task wr(input [8:0] a, input [31:0] d);
        begin
            @(negedge clk);
            reg_addr = a; reg_wdata = d; reg_we = 1'b1;
            @(negedge clk);
            reg_we = 1'b0;
        end
    endtask

    task rd(input [8:0] a);
        begin
            @(negedge clk);
            reg_addr = a; reg_re = 1'b1;
            #1 rv = reg_rdata;
            @(negedge clk);
            reg_re = 1'b0;
        end
    endtask

    task rchk(input [8:0] a, input [31:0] exp, input [8*64-1:0] what);
        begin
            rd(a);
            check_eq(rv, exp, what);
        end
    endtask

    task rchkm(input [8:0] a, input [31:0] mask, input [31:0] exp, input [8*64-1:0] what);
        begin
            rd(a);
            check_eq(rv & mask, exp, what);
        end
    endtask

    task pl(input integer i, input [15:0] w);          // program word i
        wr(9'h100 + i * 4, {16'd0, w});
    endtask

    task ex(input integer s, input [15:0] ins);        // EXEC, do not wait
        wr(SA(s, O_EXEC), {16'd0, ins});
    endtask

    task exw(input integer s, input [15:0] ins);       // EXEC and wait until done
        integer n;
        begin
            ex(s, ins);
            n = 0;
            rd(SA(s, O_STATE));
            while (rv[10] && n < 6000) begin
                rd(SA(s, O_STATE));
                n = n + 1;
            end
            if (rv[10]) begin
                errors = errors + 1;
                $display("FAIL: EXEC %h on SM%0d never finished (t=%0t)", ins, s, $time);
            end
        end
    endtask

    task smcfg(input integer s, input [31:0] clkdiv, input [31:0] pctl, input [31:0] shift);
        begin
            wr(SA(s, O_CLKDIV), clkdiv);
            wr(SA(s, O_PINCTRL), pctl);
            wr(SA(s, O_SHIFT), shift);
        end
    endtask

    task drain_rx(input integer s);
        integer n;
        begin
            n = 0;
            rd(A_FSTAT);
            while (((s == 0) ? rv[6:4] : rv[14:12]) != 3'd0 && n < 8) begin
                rd(SA(s, O_RXF));
                rd(A_FSTAT);
                n = n + 1;
            end
        end
    endtask

    task clks(input integer n);
        repeat (n) @(posedge clk);
    endtask

    // ---------------- edge monitor (pin timing) ----------------
    reg          mon_en = 1'b0;
    reg  [2:0]   mon_sel = 3'd0;            // 0..3 sm0_out[i], 4..7 sm1_out[i]
    integer      edge_n = 0;
    reg  [31:0]  edge_t [0:1023];           // time of each edge, ns
    wire         mon_sig = mon_sel[2] ? sm1_out[mon_sel[1:0]] : sm0_out[mon_sel[1:0]];

    always @(mon_sig) begin
        if (mon_en && edge_n < 1024) begin
            edge_t[edge_n] = $time;
            edge_n = edge_n + 1;
        end
    end

    task mon_start(input [2:0] sel);
        begin
            mon_en = 1'b0;
            mon_sel = sel;
            #1;
            edge_n = 0;
            mon_en = 1'b1;
        end
    endtask

    task wait_edges(input integer n, input integer tmo);
        integer c;
        begin
            c = 0;
            while (edge_n < n && c < tmo) begin
                @(posedge clk);
                c = c + 1;
            end
            check(edge_n >= n, "enough pin edges seen");
        end
    endtask

    // ---------------- lock-step monitor (H8) ----------------
    reg     lock_chk = 1'b0;
    integer lock_err = 0;
    always @(posedge clk)
        if (lock_chk && (sm0_out[0] !== sm1_out[0])) lock_err = lock_err + 1;

    // ---------------- TB UART receiver (J) ----------------
    localparam real UBIT = 8680.556;         // 115200 baud, ns
    reg        uart_on = 1'b0;
    wire       txd = sm0_oe[1] ? sm0_out[1] : 1'b1;      // pull-up when released
    reg  [7:0] urx_byte  [0:15];
    reg  [9:0] urx_frame [0:15];
    real       urx_start [0:15];
    integer    urx_n = 0;
    real       ut0;
    reg  [9:0] ufr;
    integer    ub;
    real       uedge [0:15];                   // edges of the first frame
    integer    uedge_n = 0;

    always @(negedge txd) begin
        if (uart_on) begin
            ut0 = $realtime;
            for (ub = 0; ub < 10; ub = ub + 1) begin
                #(UBIT * (ub + 0.5) - ($realtime - ut0));
                ufr[ub] = txd;                       // sample in the middle of each bit
            end
            if (urx_n < 16) begin
                urx_frame[urx_n] = ufr;
                urx_byte[urx_n]  = ufr[8:1];
                urx_start[urx_n] = ut0;
            end
            urx_n = urx_n + 1;
        end
    end

    always @(txd) begin
        if (uart_on && urx_n == 0 && uedge_n < 16) begin
            uedge[uedge_n] = $realtime;
            uedge_n = uedge_n + 1;
        end
    end

    // sm_idle must stay 0 while bytes are queued back to back
    reg     idle_watch = 1'b0;
    integer idle_bad = 0;
    always @(posedge clk)
        if (idle_watch && sm_idle[0]) idle_bad = idle_bad + 1;

    // ---------------- watchdog ----------------
    initial begin
        #20_000_000;
        $display("FAIL: watchdog time-out, the test hung");
        errors = errors + 1;
        $display("%0d OF %0d CHECKS FAILED", errors, checks + 1);
        $finish;
    end

    // ---------------- test sequence ----------------
    reg  [15:0] pg [0:31];
    reg  [15:0] hexmem [0:31];
    reg  [15:0] exp_uart [0:6];
    reg  [7:0]  msg [0:8];
    reg  [7:0]  ubytes [0:5];
    reg  [31:0] w, mx, my, mi, mo, t0, t1;
    reg  [15:0] mc;
    reg  [5:0]  n1, n2, n3;
    reg         ol, il;
    integer     i, j, k, it, lo, hi, sum, bad;
    real        bt, dt;

    initial begin
        msg[0] = "1"; msg[1] = "2"; msg[2] = "3"; msg[3] = "4"; msg[4] = "5";
        msg[5] = "6"; msg[6] = "7"; msg[7] = "8"; msg[8] = "9";

        repeat (5) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        clks(3);

        // =================================================================
        // A. reset values and registers
        // =================================================================
        $display("A: reset values and registers");
        rchk(A_CTRL,   32'h0000_0000, "SE_CTRL reset");
        rchk(A_FSTAT,  32'h000A_0000, "SE_FSTAT reset (RX FIFOs empty)");
        rchk(A_CRCCFG, 32'hFFFF_1021, "SE_CRC_CFG reset");
        time_us = 32'h1234_5678;
        rchk(A_TIME,   32'h1234_5678, "SE_TIME = time_us");
        for (i = 0; i < 2; i = i + 1) begin
            rchk(SA(i, O_CLKDIV),  32'h0001_0000, "CLKDIV reset INT=1 FRAC=0");
            rchk(SA(i, O_PINCTRL), 32'h0, "PINCTRL reset");
            rchk(SA(i, O_SHIFT),   32'h0, "SHIFTCTRL reset");
            rchk(SA(i, O_STATE),   32'h0, "STATE reset");
            rchk(SA(i, O_X),       32'h0, "X reset");
            rchk(SA(i, O_Y),       32'h0, "Y reset");
            rchk(SA(i, O_CRC),     32'h0000_FFFF, "CRC reset = INIT");
            rchk(SA(i, O_PINS),    32'h0000_000F, "PINS reset: in = F, out = oe = 0");
            rchk(SA(i, O_RXF),     32'h0, "RXF reads 0 when empty");
        end
        check(sm_idle == 2'b11, "sm_idle = 11 while both SMs are disabled");
        check(rx_not_empty == 2'b00 && ni_rx_empty == 2'b11 && ni_tx_full == 2'b00,
              "FIFO status ports after reset");
        check(ni_rx_data0 == 32'd0 && ni_rx_data1 == 32'd0, "ni_rx_data = 0 when empty");

        wr(SA(0, O_CLKDIV), 32'hFFFF_FFFF);
        rchk(SA(0, O_CLKDIV), 32'hFFFF_FF00, "CLKDIV read-back");
        wr(SA(0, O_PINCTRL), 32'hFFFF_FFFF);
        rchk(SA(0, O_PINCTRL), 32'h0FFF_7F7F, "PINCTRL read-back (unused bits 0)");
        wr(SA(0, O_SHIFT), 32'hFFFF_FFFF);
        rchk(SA(0, O_SHIFT), 32'h0000_1F03, "SHIFTCTRL read-back");
        check(out_shift_left == 2'b01 && in_shift_left == 2'b01, "shift_left ports follow SM0 SHIFTCTRL");
        wr(SA(1, O_SHIFT), 32'h0000_0002);
        check(out_shift_left == 2'b01 && in_shift_left == 2'b11, "shift_left ports follow SM1 SHIFTCTRL");
        rchk(SA(1, O_SHIFT), 32'h0000_0002, "SM1 SHIFTCTRL read-back");
        wr(SA(1, O_CLKDIV), 32'h1234_5600);
        rchk(SA(1, O_CLKDIV), 32'h1234_5600, "SM1 CLKDIV read-back");
        rchk(SA(0, O_CLKDIV), 32'hFFFF_FF00, "SM0 CLKDIV unchanged by SM1 write");
        wr(A_CRCCFG, 32'h1D0F_8005);
        rchk(A_CRCCFG, 32'h1D0F_8005, "SE_CRC_CFG read-back");
        wr(A_CRCCFG, 32'hFFFF_1021);
        wr(A_CTRL, 32'h0000_0303);
        rchk(A_CTRL, 32'h0000_0003, "SE_CTRL: EN bits read back, RESTART bits read 0");
        rchk(SA(1, O_STATE) & 32'h200, 32'h200, "STATE[9] enabled");
        wr(A_CTRL, 32'h0);
        // unused / write-only addresses read 0
        rchk(9'h03C, 32'h0, "0x03C reads 0");
        rchk(9'h040, 32'h0, "0x040 reads 0");
        rchk(9'h07C, 32'h0, "0x07C reads 0");
        rchk(9'h080, 32'h0, "0x080 reads 0");
        rchk(9'h0FC, 32'h0, "0x0FC reads 0");
        rchk(SA(0, O_TXF),  32'h0, "TXF (write only) reads 0");
        rchk(SA(1, O_EXEC), 32'h0, "EXEC (write only) reads 0");
        // program memory
        for (i = 0; i < 32; i = i + 1) begin
            pg[i] = $random(seed);
            wr(9'h100 + i * 4, {16'hDEAD, pg[i]});
        end
        rchk(9'h180, 32'h0, "0x180 (beyond PROG) reads 0");
        bad = 0;
        for (i = 0; i < 32; i = i + 1) begin
            rd(9'h100 + i * 4);
            if (rv !== {16'd0, pg[i]}) bad = bad + 1;
        end
        check(bad == 0, "PROG[0..31] write / read-back");
        smcfg(0, 32'h0001_0000, 32'h0, 32'h0);
        smcfg(1, 32'h0001_0000, 32'h0, 32'h0);

        // =================================================================
        // B. TX FIFO and PULL
        // =================================================================
        $display("B: TX FIFO, PULL block / noblock");
        for (i = 0; i < 5; i = i + 1)
            wr(SA(0, O_TXF), 32'h1000_0000 + i);          // the 5th is dropped
        rchk(A_FSTAT, 32'h000B_0004, "FSTAT: TX0 level 4 + full");
        check(ni_tx_full == 2'b01, "ni_tx_full[0] when TX0 holds 4 words");
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_out(D_X, 6'd32, 5'd0));
            rd(SA(0, O_X));
            if (rv !== 32'h1000_0000 + i) bad = bad + 1;
        end
        check(bad == 0, "PULL returns the TX words in order");
        rchk(A_FSTAT, 32'h000A_0000, "FSTAT: TX0 empty again (5th word was dropped)");
        ex(0, i_pull(1'b1, 5'd0));                       // TX empty: stalls
        clks(3);
        rchk(SA(0, O_STATE), 32'h0000_0500, "PULL block on empty FIFO: stalled + exec pending");
        rchkm(A_FSTAT, 32'h0300_0000, 32'h0100_0000, "FSTAT[24] SM0 stalled");
        wr(SA(0, O_TXF), 32'hCAFE_F00D);
        rchk(SA(0, O_STATE), 32'h0000_0000, "PULL completes when a word arrives");
        exw(0, i_out(D_X, 6'd32, 5'd0));
        rchk(SA(0, O_X), 32'hCAFE_F00D, "OSR = the word that ended the stall");
        exw(0, i_set(D_X, 5'd19, 5'd0));
        exw(0, i_pull(1'b0, 5'd0));                      // noblock, empty: OSR = X
        exw(0, i_out(D_Y, 6'd32, 5'd0));
        rchk(SA(0, O_Y), 32'd19, "PULL noblock on empty FIFO copies X into OSR");

        // =================================================================
        // C. SET and JMP conditions (EXEC on the stopped SM0)
        // =================================================================
        $display("C: SET, JMP conditions");
        exw(0, i_set(D_X, 5'd21, 5'd0));
        rchk(SA(0, O_X), 32'd21, "SET x, 21");
        exw(0, i_set(D_Y, 5'd31, 5'd0));
        rchk(SA(0, O_Y), 32'd31, "SET y, 31");
        exw(0, i_jmp(C_ALW, 5'd17, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd17, "JMP always");
        exw(0, i_set(D_X, 5'd2, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd17, "EXEC SET does not advance the PC");
        exw(0, i_set(D_X, 5'd0, 5'd0));
        exw(0, i_jmp(C_NX, 5'd3, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd3, "JMP !x taken when X = 0");
        exw(0, i_set(D_X, 5'd1, 5'd0));
        exw(0, i_jmp(C_NX, 5'd9, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd3, "JMP !x not taken when X != 0");
        exw(0, i_set(D_X, 5'd2, 5'd0));
        exw(0, i_jmp(C_XDEC, 5'd5, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd5, "JMP x-- taken (X = 2)");
        rchk(SA(0, O_X), 32'd1, "JMP x-- decrements X");
        exw(0, i_jmp(C_XDEC, 5'd6, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd6, "JMP x-- taken (X = 1)");
        rchk(SA(0, O_X), 32'd0, "X = 0");
        exw(0, i_jmp(C_XDEC, 5'd7, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd6, "JMP x-- not taken (X = 0)");
        rchk(SA(0, O_X), 32'hFFFF_FFFF, "JMP x-- still decrements X (wraps)");
        exw(0, i_set(D_Y, 5'd0, 5'd0));
        exw(0, i_jmp(C_NY, 5'd8, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd8, "JMP !y taken");
        exw(0, i_set(D_Y, 5'd1, 5'd0));
        exw(0, i_jmp(C_NY, 5'd9, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd8, "JMP !y not taken");
        exw(0, i_jmp(C_YDEC, 5'd10, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd10, "JMP y-- taken (Y = 1)");
        exw(0, i_jmp(C_YDEC, 5'd11, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd10, "JMP y-- not taken (Y = 0)");
        rchk(SA(0, O_Y), 32'hFFFF_FFFF, "JMP y-- decrements Y");
        exw(0, i_set(D_X, 5'd4, 5'd0));
        exw(0, i_set(D_Y, 5'd4, 5'd0));
        exw(0, i_jmp(C_XNEY, 5'd12, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd10, "JMP x!=y not taken (X = Y)");
        exw(0, i_set(D_Y, 5'd5, 5'd0));
        exw(0, i_jmp(C_XNEY, 5'd12, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd12, "JMP x!=y taken");
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 0, 0, 0, 0, 0, 2'd2, 0, 0, 0));   // JMP_PIN = 2
        sm0_in = 4'b1011;
        clks(3);
        exw(0, i_jmp(C_PIN, 5'd13, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd12, "JMP pin not taken (pin 2 = 0)");
        sm0_in = 4'b0100;
        clks(3);
        exw(0, i_jmp(C_PIN, 5'd13, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd13, "JMP pin taken (pin 2 = 1)");
        wr(SA(0, O_TXF), 32'h0000_00FF);
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_jmp(C_NOSRE, 5'd14, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd14, "JMP !osre taken after PULL (count 0)");
        exw(0, i_out(D_NULL, 6'd16, 5'd0));
        exw(0, i_jmp(C_NOSRE, 5'd15, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd15, "JMP !osre taken (count 16)");
        exw(0, i_out(D_NULL, 6'd20, 5'd0));            // 36 -> saturates at 32
        exw(0, i_jmp(C_NOSRE, 5'd16, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd15, "JMP !osre not taken (count saturated at 32)");
        exw(0, i_pull(1'b0, 5'd0));                     // noblock pull resets the count too
        exw(0, i_out(D_NULL, 6'd31, 5'd0));
        exw(0, i_jmp(C_NOSRE, 5'd18, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd18, "JMP !osre taken (count 31)");
        exw(0, i_out(D_NULL, 6'd1, 5'd0));
        exw(0, i_jmp(C_NOSRE, 5'd19, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd18, "JMP !osre not taken (count 32)");

        // =================================================================
        // D. OUT and IN
        // =================================================================
        $display("D: OUT / IN shifting");
        wr(SA(0, O_PINCTRL), 32'h0);
        wr(SA(0, O_SHIFT), 32'h0);                       // OUT right, IN right
        wr(SA(0, O_TXF), 32'hA5C3_0F96);
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_X, 6'd4, 5'd0));
        rchk(SA(0, O_X), 32'h0000_0006, "OUT right: x, 4 = OSR[3:0]");
        exw(0, i_out(D_Y, 6'd8, 5'd0));
        rchk(SA(0, O_Y), 32'h0000_00F9, "OUT right: y, 8 = next byte");
        exw(0, i_out(D_X, 6'd20, 5'd0));
        rchk(SA(0, O_X), 32'h000A_5C30, "OUT right: x, 20");
        wr(SA(0, O_SHIFT), 32'h1);                       // OUT left
        wr(SA(0, O_TXF), 32'hA5C3_0F96);
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_X, 6'd4, 5'd0));
        rchk(SA(0, O_X), 32'h0000_000A, "OUT left: x, 4 = OSR[31:28]");
        exw(0, i_out(D_Y, 6'd8, 5'd0));
        rchk(SA(0, O_Y), 32'h0000_005C, "OUT left: y, 8");
        exw(0, i_out(D_X, 6'd32, 5'd0));
        rchk(SA(0, O_X), 32'h30F9_6000, "OUT left: x, 32 = shifted OSR");
        // IN right puts 8 bits in ISR[31:24]; IN left in ISR[7:0]
        drain_rx(0);
        wr(SA(0, O_SHIFT), 32'h0);
        wr(SA(0, O_TXF), 32'h0000_00A5);
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_X, 6'd32, 5'd0));
        exw(0, i_in(S_X, 6'd8, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'hA500_0000, "IN right: 8 bits end in ISR[31:24]");
        wr(SA(0, O_SHIFT), 32'h2);
        exw(0, i_in(S_X, 6'd8, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'h0000_00A5, "IN left: 8 bits end in ISR[7:0]");

        // random OUT / IN against the model
        bad = 0;
        mi = 32'd0;                                     // ISR is 0 after the PUSH
        for (it = 0; it < 48; it = it + 1) begin
            ol = $random(seed);
            il = $random(seed);
            wr(SA(0, O_SHIFT), {30'd0, il, ol});
            w  = $random(seed);
            n1 = ({$random(seed)} % 32) + 1;
            n2 = ({$random(seed)} % 32) + 1;
            wr(SA(0, O_TXF), w);
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_out(D_X, n1, 5'd0));
            exw(0, i_out(D_Y, n2, 5'd0));
            mo = w;
            mx = m_outdata(mo, n1, ol);  mo = m_osrshift(mo, n1, ol);
            my = m_outdata(mo, n2, ol);  mo = m_osrshift(mo, n2, ol);
            rd(SA(0, O_X));
            if (rv !== mx) begin
                bad = bad + 1;
                $display("  OUT x: w=%h n=%0d left=%b got %h exp %h", w, n1, ol, rv, mx);
            end
            rd(SA(0, O_Y));
            if (rv !== my) begin
                bad = bad + 1;
                $display("  OUT y: n=%0d left=%b got %h exp %h", n2, ol, rv, my);
            end
            n1 = ({$random(seed)} % 32) + 1;
            n2 = ({$random(seed)} % 32) + 1;
            n3 = ({$random(seed)} % 32) + 1;
            exw(0, i_in(S_X, n1, 5'd0));
            exw(0, i_in(S_Y, n2, 5'd0));
            exw(0, i_in(S_OSR, n3, 5'd0));
            exw(0, i_push(1'b1, 5'd0));
            mi = m_isrin(32'd0, mx, n1, il);
            mi = m_isrin(mi, my, n2, il);
            mi = m_isrin(mi, mo, n3, il);
            rd(SA(0, O_RXF));
            if (rv !== mi) begin
                bad = bad + 1;
                $display("  IN: n=%0d,%0d,%0d left=%b got %h exp %h", n1, n2, n3, il, rv, mi);
            end
        end
        check(bad == 0, "48 random OUT x/y + IN x/y/osr cases match the model");

        // every IN source
        wr(SA(0, O_SHIFT), 32'h2);                       // IN left
        exw(0, i_set(D_X, 5'b10110, 5'd0));
        exw(0, i_in(S_X, 6'd5, 5'd0));
        exw(0, i_in(S_NULL, 6'd3, 5'd0));               // ISR = 1011 0000
        exw(0, i_in(S_ISR, 6'd8, 5'd0));                // ISR = B0B0
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'h0000_B0B0, "IN x / IN null / IN isr");
        time_us = 32'hDEAD_BEEF;
        exw(0, i_in(S_TIME, 6'd32, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'hDEAD_BEEF, "IN time, 32");
        wr(SA(0, O_SHIFT), 32'h0);                       // IN right
        exw(0, i_in(S_TIME, 6'd16, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'hBEEF_0000, "IN time, 16 (right)");
        exw(0, i_crcrst(5'd0));
        exw(0, i_in(S_CRC, 6'd32, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'h0000_FFFF, "IN crc, 32 = {16'b0, crc}");
        wr(SA(0, O_TXF), 32'h1234_5678);
        exw(0, i_pull(1'b1, 5'd0));
        wr(SA(0, O_SHIFT), 32'h2);
        exw(0, i_in(S_OSR, 6'd12, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'h0000_0678, "IN osr, 12 (left)");
        // IN pins: IN_BASE = 3, pins 0110 -> bit j = pin (3 + j) mod 4 = 1100
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 0, 0, 2'd3, 0, 0, 0, 0, 0, 0));
        sm0_in = 4'b0110;
        clks(3);
        exw(0, i_in(S_PINS, 6'd4, 5'd0));
        exw(0, i_in(S_PINS, 6'd4, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'h0000_00CC, "IN pins, 4 twice (IN_BASE 3, wraps)");
        exw(0, i_in(S_PINS, 6'd32, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'hCCCC_CCCC, "IN pins, 32 repeats the 4 pins");
        wr(SA(0, O_SHIFT), 32'h0);
        exw(0, i_in(S_PINS, 6'd3, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'h8000_0000, "IN pins, 3 (right) = ISR[31:29]");
        rchk(SA(0, O_PINS), 32'h0000_0006, "PINS[3:0] shows the synchronised inputs");
        // OUT isr / OUT pc / OUT y
        wr(SA(0, O_TXF), 32'h89AB_CDF3);
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_PC, 6'd8, 5'd0));                // data = F3 -> PC = 0x13
        rchkm(SA(0, O_STATE), 32'h1F, 32'd19, "OUT pc, 8 jumps to data[4:0]");
        exw(0, i_out(D_ISR, 6'd12, 5'd0));              // data = BCD
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'h0000_0BCD, "OUT isr, 12 (ISR = data)");
        exw(0, i_out(D_Y, 6'd12, 5'd0));
        rchk(SA(0, O_Y), 32'h0000_089A, "OUT y after OUT pc + OUT isr");

        // =================================================================
        // E. pins: SET / OUT / pindirs / OD / OD_MASK / side-set
        // =================================================================
        $display("E: pins, OD, side-set");
        // SET_BASE 3, SET_COUNT 2 -> pins 3 and 0
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 2'd3, 3'd2, 0, 0, 0, 0, 0, 0, 0));
        wr(A_CTRL, 32'h100);                             // RESTART SM0: pins = INIT = 0
        rchk(SA(0, O_PINS), 32'h0000_0006, "RESTART: out = oe = 0");
        exw(0, i_set(D_PINS, 5'b01, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h080, "SET pins, 01 (base 3, count 2): pin3 = 1");
        exw(0, i_set(D_PINS, 5'b10, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h010, "SET pins, 10: pin0 = 1 (wraps)");
        exw(0, i_set(D_PINS, 5'b11111, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h090, "SET pins, 11111 writes only 2 pins");
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 2'd3, 3'd0, 0, 0, 0, 0, 0, 0, 0));  // count 0 = 4
        exw(0, i_set(D_PINS, 5'b00110, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h030, "SET pins, 0110 (count 0 = 4, base 3)");
        exw(0, i_set(D_PINDIRS, 5'b01001, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hC30, "SET pindirs, 1001 (base 3) -> oe = 1100");
        // OUT pins: OUT_BASE 1, 8 bits -> only bits 3:0 reach pins 1,2,3,0
        wr(SA(0, O_PINCTRL), pinctrl_f(2'd1, 0, 0, 0, 0, 0, 0, 0, 0, 0));
        wr(SA(0, O_SHIFT), 32'h0);
        wr(SA(0, O_TXF), 32'h0000_E6A5);
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_PINS, 6'd8, 5'd0));              // A5: 0101 -> pin1 1 pin2 0 pin3 1 pin0 0
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hCA0, "OUT pins, 8 (base 1): out = 1010");
        exw(0, i_out(D_PINDIRS, 6'd3, 5'd0));           // E6 low 3 bits 110: pin1 0 pin2 1 pin3 1
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hCA0, "OUT pindirs, 3 (base 1): oe = 1100");
        wr(SA(0, O_PINCTRL), pinctrl_f(2'd3, 0, 0, 0, 0, 0, 0, 0, 0, 0));
        exw(0, i_out(D_PINS, 6'd2, 5'd0));              // next bits 00 -> pin3 0, pin0 0
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hC20, "OUT pins, 2 (base 3, wraps to pin 0)");
        // OD_MASK pin 2, all 4 SET pins, INIT_OUT = F, INIT_OE = 0
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 0, 3'd0, 0, 0, 0, 0, 4'b0100, 4'hF, 4'h0));
        wr(A_CTRL, 32'h100);
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h0F0, "RESTART: out = INIT_OUT, oe = INIT_OE");
        exw(0, i_set(D_PINS, 5'b0000, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h400, "SET pins 0: OD pin 2 pulls low (oe=1,out=0)");
        exw(0, i_set(D_PINS, 5'b1111, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h0B0, "SET pins 1: OD pin 2 released (oe=0)");
        exw(0, i_set(D_PINDIRS, 5'b1011, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hBB0, "SET pindirs 1011");
        exw(0, i_od(1'b0, 2'd0, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hBA0, "OD pin 0, 0: drive low");
        exw(0, i_od(1'b1, 2'd0, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hAA0, "OD pin 0, Z: release");
        exw(0, i_od(1'b0, 2'd2, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hEA0, "OD pin 2, 0");
        // OUT pins on an OD pin: OUT_BASE 2, data 01 -> pin2 release, pin3 out = 0
        wr(SA(0, O_PINCTRL), pinctrl_f(2'd2, 0, 0, 0, 0, 0, 0, 4'b0100, 4'hF, 4'h0));
        wr(SA(0, O_TXF), 32'h0000_0001);
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_PINS, 6'd2, 5'd0));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hAA0 & 32'hBF0 & 32'hF70, "OUT pins on OD pin 2 + normal pin 3");
        // side-set: SIDE_EN, SIDE_PIN 3, SET pins on pin 0, all outputs
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 0, 3'd1, 0, 2'd3, 1'b1, 0, 4'b0000, 4'h0, 4'hF));
        wr(A_CTRL, 32'h100);
        exw(0, i_set(D_X, 5'd5, 5'b1_0000));            // side 1
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hF80, "side 1 drives SIDE_PIN 3 high");
        rchk(SA(0, O_X), 32'd5, "the side-set instruction still runs (SET x, 5)");
        exw(0, i_set(D_PINS, 5'd1, 5'b0_0011));         // side 0, delay 3
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hF10, "side 0 + SET pins 1 in one instruction");
        ex(0, i_pull(1'b1, 5'b1_0000));                 // stalls (TX empty) but side-set happens
        clks(4);
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hF90, "side-set is driven even when the instruction stalls");
        rchk(SA(0, O_STATE) & 32'h500, 32'h500, "... and the PULL is stalled");
        exw(0, i_set(D_X, 5'd7, 5'b0_0000));            // a new EXEC replaces the stalled one
        rchk(SA(0, O_STATE) & 32'h500, 32'h000, "a new EXEC replaces a stalled EXEC");
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hF10, "side 0 after the replacement");
        rchk(SA(0, O_X), 32'd7, "replacement EXEC ran");
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 0, 3'd1, 0, 2'd0, 1'b1, 0, 4'b0000, 4'h0, 4'hF));
        exw(0, i_set(D_PINS, 5'd0, 5'b1_0000));         // SET pin 0 = 0, side pin 0 = 1
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hF10, "side-set wins over SET pins on the same pin");
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 0, 3'd1, 0, 2'd3, 1'b1, 0, 4'b1000, 4'h0, 4'hF));
        exw(0, i_set(D_X, 5'd0, 5'b0_0000));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'hF10, "side 0 on an OD pin: pull low");
        exw(0, i_set(D_X, 5'd0, 5'b1_0000));
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h710, "side 1 on an OD pin: release (oe = 0)");

        // =================================================================
        // F. PUSH, RX FIFO levels, overflow
        // =================================================================
        $display("F: PUSH, RX levels, overflow");
        drain_rx(0);
        wr(SA(0, O_PINCTRL), 32'h0);
        wr(SA(0, O_SHIFT), 32'h0);
        for (i = 1; i <= 4; i = i + 1) begin
            exw(0, i_set(D_X, i, 5'd0));
            exw(0, i_in(S_X, 6'd32, 5'd0));
            exw(0, i_push(1'b1, 5'd0));
            rchkm(A_FSTAT, 32'h0002_0070, {29'd0, i[2:0]} << 4, "FSTAT RX0 level counts up");
        end
        check(rx_not_empty == 2'b01 && ni_rx_empty == 2'b10, "rx_not_empty[0] / ni_rx_empty");
        check(ni_rx_data0 == 32'd1, "ni_rx_data0 shows the oldest RX word");
        exw(0, i_set(D_X, 5'd30, 5'd0));
        exw(0, i_in(S_X, 6'd32, 5'd0));
        exw(0, i_push(1'b0, 5'd0));                     // noblock, full: dropped
        rchkm(A_FSTAT, 32'h3000_0070, 32'h1000_0040, "PUSH noblock on full FIFO: overflow flag");
        rchkm(A_FSTAT, 32'h3000_0000, 32'h0, "overflow flag cleared by the read");
        ex(0, i_push(1'b1, 5'd0));                      // block, full: stalls
        clks(3);
        rchk(SA(0, O_STATE) & 32'h500, 32'h500, "PUSH block on full FIFO stalls");
        rchk(SA(0, O_RXF), 32'd1, "RXF pop (1)");
        rchk(SA(0, O_STATE) & 32'h500, 32'h000, "PUSH completes after a pop");
        rchk(SA(0, O_RXF), 32'd2, "RXF pop (2)");
        rchk(SA(0, O_RXF), 32'd3, "RXF pop (3)");
        rchk(SA(0, O_RXF), 32'd4, "RXF pop (4)");
        rchk(SA(0, O_RXF), 32'd0, "word pushed after noblock drop is 0 (ISR was cleared)");
        rchk(SA(0, O_RXF), 32'd0, "RXF reads 0 when empty");
        rchk(A_FSTAT, 32'h000A_0000, "FSTAT: all empty");
        check(rx_not_empty == 2'b00, "rx_not_empty = 0 again");
        for (i = 0; i < 5; i = i + 1)
            exw(1, i_push(1'b0, 5'd0));                 // SM1: 4 stored, 1 dropped
        rchkm(A_FSTAT, 32'h300F_7000, 32'h2002_4000, "SM1: RX1 level 4, RX overflow bit 29");
        rchkm(A_FSTAT, 32'h3000_0000, 32'h0, "SM1 overflow cleared by the read");
        drain_rx(1);
        rchk(A_FSTAT, 32'h000A_0000, "FSTAT: SM1 RX drained");

        // =================================================================
        // G. CRC
        // =================================================================
        $display("G: CRC");
        wr(A_CRCCFG, 32'hFFFF_1021);
        wr(SA(0, O_SHIFT), 32'h2);                       // IN left
        exw(0, i_crcrst(5'd0));
        rchk(SA(0, O_CRC), 32'h0000_FFFF, "CRC reset loads INIT");
        for (i = 0; i < 9; i = i + 1) begin
            wr(SA(0, O_TXF), {24'd0, msg[i]});
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_in(S_OSR, 6'd8, 5'd0));
            exw(0, i_crc(6'd8, 5'd0));
        end
        rchk(SA(0, O_CRC), 32'h0000_29B1, "CCITT-FALSE(123456789), IN left + CRC 8");
        wr(SA(0, O_SHIFT), 32'h0);                       // IN right: bit-reversed bytes
        exw(0, i_crcrst(5'd0));
        for (i = 0; i < 9; i = i + 1) begin
            wr(SA(0, O_TXF), {24'd0, rev8(msg[i])});
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_in(S_OSR, 6'd8, 5'd0));
            exw(0, i_crc(6'd8, 5'd0));
        end
        rchk(SA(0, O_CRC), 32'h0000_29B1, "CCITT-FALSE, IN right (reversed bytes) + CRC 8");
        wr(SA(0, O_SHIFT), 32'h1);                       // OUT left, byte in [31:24]
        exw(0, i_crcrst(5'd0));
        for (i = 0; i < 9; i = i + 1) begin
            wr(SA(0, O_TXF), {msg[i], 24'h00_5A5A});
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_out(D_CRC, 6'd8, 5'd0));
        end
        rchk(SA(0, O_CRC), 32'h0000_29B1, "CCITT-FALSE, OUT crc, 8 (left)");
        wr(SA(0, O_SHIFT), 32'h0);                       // OUT right, reversed byte in [7:0]
        exw(0, i_crcrst(5'd0));
        for (i = 0; i < 9; i = i + 1) begin
            wr(SA(0, O_TXF), {24'hA5A5_A5, rev8(msg[i])});
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_out(D_CRC, 6'd8, 5'd0));
        end
        rchk(SA(0, O_CRC), 32'h0000_29B1, "CCITT-FALSE, OUT crc, 8 (right)");
        wr(A_CRCCFG, 32'h0000_8005);                     // CRC-16/UMTS: poly 8005, init 0
        wr(SA(0, O_SHIFT), 32'h2);
        exw(0, i_crcrst(5'd0));
        rchk(SA(0, O_CRC), 32'h0000_0000, "CRC reset loads the new INIT");
        for (i = 0; i < 9; i = i + 1) begin
            wr(SA(0, O_TXF), {24'd0, msg[i]});
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_in(S_OSR, 6'd8, 5'd0));
            exw(0, i_crc(6'd8, 5'd0));
        end
        rchk(SA(0, O_CRC), 32'h0000_FEE8, "CRC-16/UMTS(123456789) = FEE8 (POLY / INIT used)");
        wr(A_CRCCFG, 32'hFFFF_1021);
        // CRC 32 and CRC 5 against the model, both IN directions
        for (k = 0; k < 2; k = k + 1) begin
            wr(SA(0, O_SHIFT), (k == 0) ? 32'h2 : 32'h0);
            exw(0, i_crcrst(5'd0));
            w = $random(seed);
            wr(SA(0, O_TXF), w);
            exw(0, i_pull(1'b1, 5'd0));
            exw(0, i_in(S_OSR, 6'd32, 5'd0));
            exw(0, i_crc(6'd32, 5'd0));
            exw(0, i_crc(6'd5, 5'd0));
            mc = 16'hFFFF;
            for (i = 0; i < 32; i = i + 1)
                mc = crc_bit(mc, (k == 0) ? w[31 - i] : w[i], 16'h1021);
            for (i = 0; i < 5; i = i + 1)
                mc = crc_bit(mc, (k == 0) ? w[4 - i] : w[27 + i], 16'h1021);
            rchk(SA(0, O_CRC), {16'd0, mc}, (k == 0) ? "CRC 32 + CRC 5, IN left (model)"
                                                     : "CRC 32 + CRC 5, IN right (model)");
        end
        // CRC n stalls the SM for n clocks: SET pins 1 / CRC n / SET pins 0
        wr(SA(0, O_SHIFT), 32'h0);
        pl(0, i_set(D_PINS, 5'd1, 5'd0));
        pl(1, i_crc(6'd8, 5'd0));
        pl(2, i_set(D_PINS, 5'd0, 5'd0));
        pl(3, i_crc(6'd32, 5'd0));
        pl(4, i_jmp(C_ALW, 5'd0, 5'd0));
        smcfg(0, 32'h0001_0000, pinctrl_f(0, 0, 3'd1, 0, 0, 0, 0, 0, 0, 4'h1), 32'h0);
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(5, 2000);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        check_eq(edge_t[1] - edge_t[0], 32'd100, "high = SET + CRC 8 (1 + 8 stall) + 1 = 10 clocks");
        check_eq(edge_t[2] - edge_t[1], 32'd360, "low = SET + CRC 32 (1 + 32 stall) + JMP + 1 = 36");
        smcfg(0, 32'h0014_0000, pinctrl_f(0, 0, 3'd1, 0, 0, 0, 0, 0, 0, 4'h1), 32'h0);
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(3, 3000);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        check_eq(edge_t[1] - edge_t[0], 32'd400, "INT=20: CRC 8 ends before the next tick (2 ticks)");
        check_eq(edge_t[2] - edge_t[1], 32'd800, "INT=20: CRC 32 costs 2 ticks (4 ticks low)");
        // a running program: PULL / IN osr, 8 / CRC 8 / JMP, fed from the TX FIFO
        pl(0, i_pull(1'b1, 5'd0));
        pl(1, i_in(S_OSR, 6'd8, 5'd0));
        pl(2, i_crc(6'd8, 5'd0));
        pl(3, i_jmp(C_ALW, 5'd0, 5'd0));
        smcfg(0, 32'h0001_0000, 32'h0, 32'h2);
        wr(A_CTRL, 32'h100);
        exw(0, i_crcrst(5'd0));
        wr(A_CTRL, 32'h001);
        for (i = 0; i < 9; i = i + 1) begin
            rd(A_FSTAT);
            while (rv[16]) rd(A_FSTAT);                  // wait while TX0 is full
            wr(SA(0, O_TXF), {24'd0, msg[i]});
        end
        clks(100);
        check(sm_idle[0] == 1'b1, "sm_idle[0]: running SM stalled on PULL with TX empty");
        rchk(SA(0, O_STATE), 32'h0000_0300, "STATE: PC 0, stalled, enabled");
        rchk(SA(0, O_CRC), 32'h0000_29B1, "CCITT-FALSE computed by a running program");
        wr(A_CTRL, 32'h0);

        // =================================================================
        // H. running SMs: timing
        // =================================================================
        $display("H: delays, side-set, divider, WAIT, EXEC, RESTART");
        // H1 delays: SET pins 1 [3] / SET pins 0 [5] / JMP 0
        pl(0, i_set(D_PINS, 5'd1, 5'd3));
        pl(1, i_set(D_PINS, 5'd0, 5'd5));
        pl(2, i_jmp(C_ALW, 5'd0, 5'd0));
        smcfg(0, 32'h0001_0000, pinctrl_f(0, 0, 3'd1, 0, 0, 0, 0, 0, 0, 4'h1), 32'h0);
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(9, 500);
        bad = 0;
        for (i = 1; i < 9; i = i + 1)
            if (edge_t[i] - edge_t[i-1] != ((i % 2) ? 32'd40 : 32'd70)) bad = bad + 1;
        check(bad == 0, "delays: high 1+3 = 4 ticks, low 1+5+1 = 7 ticks (INT = 1)");
        wr(SA(0, O_CLKDIV), 32'h0003_0000);              // INT = 3 while running
        clks(100);
        mon_start(3'd0);
        wait_edges(9, 500);
        bad = 0;
        for (i = 2; i < 9; i = i + 1)
            if (edge_t[i] - edge_t[i-1] != ((edge_t[i] - edge_t[i-1] == 32'd120) ? 32'd120 : 32'd210))
                bad = bad + 1;
        check(bad == 0 && (edge_t[3] - edge_t[2]) + (edge_t[4] - edge_t[3]) == 32'd330,
              "delays at INT = 3: 12 + 21 clocks");
        wr(A_CTRL, 32'h0);
        pl(0, i_set(D_PINS, 5'd1, 5'd31));
        wr(SA(0, O_CLKDIV), 32'h0001_0000);
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(3, 500);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        check_eq(edge_t[1] - edge_t[0], 32'd320, "delay 31: high 1 + 31 = 32 ticks");
        // H2 side-set + delay (delay field is only 4 bits)
        pl(0, i_jmp(C_ALW, 5'd1, 5'b1_0010));            // JMP 1 side 1 [2]
        pl(1, i_jmp(C_ALW, 5'd0, 5'b0_0100));            // JMP 0 side 0 [4]
        smcfg(0, 32'h0001_0000, pinctrl_f(0, 0, 0, 0, 2'd0, 1'b1, 0, 0, 0, 4'h1), 32'h0);
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(6, 500);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        check(edge_t[1] - edge_t[0] == 32'd30 && edge_t[2] - edge_t[1] == 32'd50 &&
              edge_t[3] - edge_t[2] == 32'd30, "side-set: high 1+2 = 3, low 1+4 = 5 ticks");
        // H3 clock divider: JMP 1 side 1 / JMP 0 side 0 toggles every tick
        pl(0, i_jmp(C_ALW, 5'd1, 5'b1_0000));
        pl(1, i_jmp(C_ALW, 5'd0, 5'b0_0000));
        wr(SA(0, O_CLKDIV), 32'h006C_8000);              // 108.5
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(201, 30000);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        lo = 0; hi = 0; bad = 0;
        for (i = 1; i < 201; i = i + 1) begin
            if (edge_t[i] - edge_t[i-1] == 32'd1080) lo = lo + 1;
            else if (edge_t[i] - edge_t[i-1] == 32'd1090) hi = hi + 1;
            else bad = bad + 1;
        end
        check(bad == 0 && lo == 100 && hi == 100, "INT=108 FRAC=128: periods 108 / 109 alternate");
        check_eq(edge_t[200] - edge_t[0], 32'd2170000, "200 ticks = 21700 clocks (108.5 average)");
        wr(SA(0, O_CLKDIV), 32'h0002_4000);              // 2.25
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(401, 3000);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        check_eq(edge_t[400] - edge_t[0], 32'd9000, "INT=2 FRAC=64: 400 ticks = 900 clocks");
        wr(SA(0, O_CLKDIV), 32'h0001_8000);              // 1.5
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(201, 3000);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        check_eq(edge_t[200] - edge_t[0], 32'd3000, "INT=1 FRAC=128: 200 ticks = 300 clocks");
        wr(SA(0, O_CLKDIV), 32'h0000_0000);              // INT = 0 counts as 1
        mon_start(3'd0);
        wr(A_CTRL, 32'h101);
        wait_edges(41, 500);
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        check_eq(edge_t[40] - edge_t[0], 32'd400, "INT=0 is treated as 1: a tick every clock");
        // H4 WAIT, input synchroniser latency, stall status, sm_idle
        pl(0, i_wait(1'b1, 2'd2, 5'd0));
        pl(1, i_set(D_PINS, 5'd1, 5'd0));
        pl(2, i_wait(1'b0, 2'd2, 5'd0));
        pl(3, i_set(D_PINS, 5'd0, 5'd0));
        pl(4, i_jmp(C_ALW, 5'd0, 5'd0));
        smcfg(0, 32'h0001_0000, pinctrl_f(0, 0, 3'd1, 0, 0, 0, 0, 0, 0, 4'h1), 32'h0);
        sm0_in = 4'b0000;
        clks(3);
        wr(A_CTRL, 32'h101);
        clks(10);
        rchk(SA(0, O_STATE), 32'h0000_0300, "WAIT 1, pin 2: stalled at PC 0, enabled");
        rchkm(A_FSTAT, 32'h0300_0000, 32'h0100_0000, "FSTAT[24]: SM0 stalled in WAIT");
        check(sm_idle[0] == 1'b0, "sm_idle[0] = 0 while stalled in WAIT (not a PULL)");
        @(negedge clk);
        sm0_in = 4'b0100;
        t0 = $time;
        @(posedge sm0_out[0]);
        check_eq($time - t0, 32'd35, "WAIT: 2 sync clocks + WAIT tick + SET tick = 35 ns");
        clks(5);
        @(negedge clk);
        sm0_in = 4'b0000;
        t0 = $time;
        @(negedge sm0_out[0]);
        check_eq($time - t0, 32'd35, "WAIT 0 releases after the same latency");
        // H5 freeze: SM_EN = 0 holds everything
        wr(A_CTRL, 32'h0);
        rd(SA(0, O_STATE));
        t1 = rv;
        mon_start(3'd0);
        sm0_in = 4'b0100;                                // would release the WAIT
        clks(50);
        rchk(SA(0, O_STATE), t1 & 32'hFFFF_FDFF, "SM_EN = 0 freezes PC and stall state");
        check(edge_n == 0 && sm0_out[0] == 1'b0 && sm0_oe == 4'h1, "pins hold while disabled");
        check(sm_idle[0] == 1'b1, "sm_idle[0] = 1 while disabled");
        wr(A_CTRL, 32'h001);
        wait_edges(1, 50);
        check(sm0_out[0] == 1'b1, "re-enabled SM continues (WAIT released)");
        wr(A_CTRL, 32'h0);
        mon_en = 1'b0;
        // H6 EXEC on a running SM (slow divider: INT = 1000)
        pl(10, i_jmp(C_ALW, 5'd10, 5'd0));               // spin
        pl(12, i_set(D_X, 5'd3, 5'd0));
        pl(13, i_jmp(C_ALW, 5'd13, 5'd0));
        pl(14, i_wait(1'b1, 2'd3, 5'd0));
        pl(16, i_set(D_X, 5'd1, 5'd31));                 // long delay
        pl(17, i_jmp(C_ALW, 5'd17, 5'd0));
        smcfg(0, 32'h03E8_0000, 32'h0, 32'h0000_0A00);   // START_PC = 10
        wr(A_CTRL, 32'h101);
        clks(20);
        ex(0, i_set(D_Y, 5'd7, 5'd0));
        rchk(SA(0, O_STATE) & 32'h400, 32'h400, "EXEC on a running SM waits for the next tick");
        exw(0, i_set(D_Y, 5'd7, 5'd0));
        rchk(SA(0, O_Y), 32'd7, "EXEC SET y, 7 ran on the running SM");
        rchkm(SA(0, O_STATE), 32'h71F, 32'h20A, "PC still 10, running, nothing pending");
        exw(0, i_jmp(C_ALW, 5'd12, 5'd0));
        clks(2200);
        rchk(SA(0, O_X), 32'd3, "EXEC JMP 12: the program continued from 12");
        rchkm(SA(0, O_STATE), 32'h1F, 32'd13, "PC = 13");
        sm0_in = 4'b0000;
        exw(0, i_jmp(C_ALW, 5'd14, 5'd0));
        clks(2200);
        rchk(SA(0, O_STATE), 32'h0000_030E, "SM stalled in WAIT at 14");
        exw(0, i_jmp(C_ALW, 5'd13, 5'd0));
        rchkm(SA(0, O_STATE), 32'h51F, 32'd13, "EXEC JMP replaces the stalled WAIT");
        exw(0, i_jmp(C_ALW, 5'd16, 5'd0));
        clks(2200);
        rchk(SA(0, O_X), 32'd1, "SET x, 1 [31] ran, now in a 31-tick delay");
        t0 = $time;
        exw(0, i_set(D_Y, 5'd9, 5'd0));
        check($time - t0 < 32'd12000, "EXEC runs at the next tick, not after the delay");
        rchk(SA(0, O_Y), 32'd9, "EXEC during a delay ran");
        // H7 RESTART: PC, X, Y, ISR, OSR, counts, delay, stall, pins; FIFOs kept
        wr(A_CTRL, 32'h0);
        drain_rx(0);
        wr(SA(0, O_SHIFT), 32'h0000_0B00);               // START_PC = 11
        wr(SA(0, O_PINCTRL), pinctrl_f(0, 0, 0, 0, 0, 0, 0, 0, 4'b1010, 4'b0110));
        exw(0, i_set(D_X, 5'd5, 5'd0));
        exw(0, i_set(D_Y, 5'd6, 5'd0));
        exw(0, i_in(S_Y, 6'd32, 5'd0));
        exw(0, i_push(1'b1, 5'd0));                      // RX: one word (6)
        exw(0, i_in(S_X, 6'd32, 5'd0));                  // ISR = 5
        wr(SA(0, O_TXF), 32'h1111_1111);
        wr(SA(0, O_TXF), 32'h2222_2222);
        wr(SA(0, O_TXF), 32'h3333_3333);
        exw(0, i_pull(1'b1, 5'd0));                      // OSR = 1111_1111
        exw(0, i_jmp(C_ALW, 5'd3, 5'd0));
        ex(0, i_wait(1'b1, 2'd3, 5'd0));                 // stalled EXEC
        clks(3);
        rchk(SA(0, O_STATE), 32'h0000_0503, "before RESTART: PC 3, stalled, EXEC pending");
        wr(A_CTRL, 32'h100);
        rchk(SA(0, O_STATE), 32'h0000_000B, "RESTART: PC = START_PC, stall / EXEC cleared");
        rchk(SA(0, O_X), 32'd0, "RESTART: X = 0");
        rchk(SA(0, O_Y), 32'd0, "RESTART: Y = 0");
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h6A0, "RESTART: out = INIT_OUT, oe = INIT_OE");
        rchkm(A_FSTAT, 32'h77, 32'h12, "RESTART keeps the FIFOs (TX0 2, RX0 1)");
        exw(0, i_jmp(C_NOSRE, 5'd20, 5'd0));
        rchkm(SA(0, O_STATE), 32'h1F, 32'd11, "RESTART: OSR empty (out count 32)");
        exw(0, i_out(D_X, 6'd32, 5'd0));
        rchk(SA(0, O_X), 32'd0, "RESTART: OSR = 0");
        exw(0, i_push(1'b1, 5'd0));
        rchk(SA(0, O_RXF), 32'd6, "old RX word kept");
        rchk(SA(0, O_RXF), 32'd0, "RESTART: ISR = 0");
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_X, 6'd32, 5'd0));
        rchk(SA(0, O_X), 32'h2222_2222, "old TX words kept");
        exw(0, i_pull(1'b1, 5'd0));
        // RESTART clears a running delay
        pl(11, i_set(D_X, 5'd2, 5'd0));
        pl(12, i_jmp(C_ALW, 5'd12, 5'd0));
        pl(16, i_set(D_X, 5'd1, 5'd31));
        pl(17, i_jmp(C_ALW, 5'd17, 5'd0));
        smcfg(0, 32'h0032_0000, 32'h0, 32'h0000_1000);   // INT 50, START_PC 16
        wr(A_CTRL, 32'h101);
        clks(200);
        rchk(SA(0, O_X), 32'd1, "SM in a 31 x 50 clock delay");
        wr(SA(0, O_SHIFT), 32'h0000_0B00);               // START_PC = 11
        wr(A_CTRL, 32'h101);
        clks(60);
        rchk(SA(0, O_X), 32'd2, "RESTART cleared the delay: START_PC ran at once");
        wr(A_CTRL, 32'h0);
        // H8 two SMs share the program and run in lock-step
        pl(0, i_jmp(C_ALW, 5'd1, 5'b1_0001));
        pl(1, i_jmp(C_ALW, 5'd0, 5'b0_0000));
        smcfg(0, 32'h0003_4000, pinctrl_f(0, 0, 0, 0, 0, 1'b1, 0, 0, 0, 4'h1), 32'h0);
        smcfg(1, 32'h0003_4000, pinctrl_f(0, 0, 0, 0, 0, 1'b1, 0, 0, 0, 4'h1), 32'h0);
        mon_start(3'd4);
        lock_err = 0;
        @(negedge clk);
        lock_chk = 1'b1;
        wr(A_CTRL, 32'h303);
        clks(600);
        wr(A_CTRL, 32'h0);
        @(negedge clk);
        lock_chk = 1'b0;
        mon_en = 1'b0;
        check(lock_err == 0 && edge_n > 100, "SM0 and SM1 run the same program in lock-step");

        // =================================================================
        // I. ni_* FIFO ports
        // =================================================================
        $display("I: ni_* ports");
        smcfg(1, 32'h0001_0000, 32'h0, 32'h0);
        wr(A_CTRL, 32'h200);                             // RESTART SM1
        for (i = 0; i < 4; i = i + 1) begin
            @(negedge clk);
            ni_tx_push = 2'b10;
            ni_tx_data1 = 32'h5000_0000 + i;
            @(negedge clk);
            ni_tx_push = 2'b00;
        end
        check(ni_tx_full == 2'b10, "ni_tx_full[1] after 4 NI pushes");
        rchkm(A_FSTAT, 32'h0004_0700, 32'h0004_0400, "FSTAT TX1 level 4 + full");
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            exw(1, i_pull(1'b1, 5'd0));
            exw(1, i_out(D_X, 6'd32, 5'd0));
            rd(SA(1, O_X));
            if (rv !== 32'h5000_0000 + i) bad = bad + 1;
        end
        check(bad == 0, "SM1 pulls the NI words in order");
        @(negedge clk);                                  // CPU and NI push in the same clock
        reg_addr = SA(1, O_TXF); reg_wdata = 32'hC0C0_C0C0; reg_we = 1'b1;
        ni_tx_push = 2'b10; ni_tx_data1 = 32'h0BAD_0BAD;
        @(negedge clk);
        reg_we = 1'b0; ni_tx_push = 2'b00;
        rchkm(A_FSTAT, 32'h0000_0700, 32'h0000_0100, "same-clock CPU + NI push: one word");
        exw(1, i_pull(1'b1, 5'd0));
        exw(1, i_out(D_X, 6'd32, 5'd0));
        rchk(SA(1, O_X), 32'hC0C0_C0C0, "CPU data has priority");
        @(negedge clk);
        ni_tx_push = 2'b01; ni_tx_data0 = 32'h0A0B_0C0D;
        @(negedge clk);
        ni_tx_push = 2'b00;
        exw(0, i_pull(1'b1, 5'd0));
        exw(0, i_out(D_Y, 6'd32, 5'd0));
        rchk(SA(0, O_Y), 32'h0A0B_0C0D, "ni_tx_push[0] / ni_tx_data0 reach SM0");
        exw(1, i_set(D_X, 5'd11, 5'd0));
        exw(1, i_in(S_X, 6'd32, 5'd0));
        exw(1, i_push(1'b1, 5'd0));
        exw(1, i_set(D_X, 5'd22, 5'd0));
        exw(1, i_in(S_X, 6'd32, 5'd0));
        exw(1, i_push(1'b1, 5'd0));
        check(rx_not_empty == 2'b10 && ni_rx_empty == 2'b01, "rx_not_empty[1] / ni_rx_empty[1]");
        check_eq(ni_rx_data1, 32'd11, "ni_rx_data1 = front of RX1 (fall-through)");
        @(negedge clk);
        ni_rx_pop = 2'b10;
        @(negedge clk);
        ni_rx_pop = 2'b00;
        check_eq(ni_rx_data1, 32'd22, "ni_rx_pop[1] pops one word");
        @(negedge clk);
        ni_rx_pop = 2'b10;
        @(negedge clk);
        ni_rx_pop = 2'b00;
        check(ni_rx_empty == 2'b11 && ni_rx_data1 == 32'd0 && rx_not_empty == 2'b00,
              "RX1 empty after two NI pops");
        exw(0, i_set(D_X, 5'd17, 5'd0));
        exw(0, i_in(S_X, 6'd32, 5'd0));
        exw(0, i_push(1'b1, 5'd0));
        check(ni_rx_data0 == 32'd17 && rx_not_empty == 2'b01, "ni_rx_data0 shows SM0's word");
        @(negedge clk);
        ni_rx_pop = 2'b01;
        @(negedge clk);
        ni_rx_pop = 2'b00;
        check(ni_rx_empty == 2'b11, "ni_rx_pop[0] pops SM0's word");
        wr(SA(0, O_SHIFT), 32'h0000_0002);
        wr(SA(1, O_SHIFT), 32'h0000_0001);
        check(out_shift_left == 2'b10 && in_shift_left == 2'b01, "out/in_shift_left per SM");

        // =================================================================
        // J. the assembled UART program (programs/uart_tx.hex)
        // =================================================================
        $display("J: programs/uart_tx.hex at 115200 baud");
        for (i = 0; i < 32; i = i + 1) hexmem[i] = 16'hxxxx;
        $readmemh("programs/uart_tx.hex", hexmem, 0, 6);
        exp_uart[0] = i_pull(1'b1, 5'd0);                // loop: PULL block
        exp_uart[1] = i_set(D_X, 5'd7, 5'd0);            //       SET x, 7
        exp_uart[2] = i_set(D_PINS, 5'd0, 5'd7);         //       SET pins, 0 [7]
        exp_uart[3] = i_out(D_PINS, 6'd1, 5'd6);         // bit:  OUT pins, 1 [6]
        exp_uart[4] = i_jmp(C_XDEC, 5'd3, 5'd0);         //       JMP x--, bit
        exp_uart[5] = i_set(D_PINS, 5'd1, 5'd7);         //       SET pins, 1 [7]
        exp_uart[6] = i_jmp(C_ALW, 5'd0, 5'd0);          //       JMP loop
        bad = 0;
        for (i = 0; i < 7; i = i + 1)
            if (hexmem[i] !== exp_uart[i]) begin
                bad = bad + 1;
                $display("  uart_tx.hex word %0d = %h, expected %h", i, hexmem[i], exp_uart[i]);
            end
        check(bad == 0, "assembler output uart_tx.hex = the slide-13 encoding");
        check_eq(pinctrl_f(2'd1, 2'd1, 3'd1, 0, 0, 0, 0, 0, 4'b0010, 4'b0010), 32'h0220_0015,
                 "UART PINCTRL value (TX on pin 1)");
        for (i = 0; i < 7; i = i + 1)
            pl(i, hexmem[i]);
        drain_rx(0);
        smcfg(0, 32'h006C_8000, 32'h0220_0015, 32'h0);
        rd(A_FSTAT);
        check(rv[2:0] == 3'd0, "TX0 empty before the UART test");
        wr(A_CTRL, 32'h100);
        rchkm(SA(0, O_PINS), 32'hFF0, 32'h220, "RESTART: TXD (pin 1) driven high");
        uart_on = 1'b1;
        wr(A_CTRL, 32'h101);
        clks(3000);
        check(sm_idle[0] == 1'b1 && urx_n == 0 && txd == 1'b1, "UART idle: PULL stalled, line high");
        ubytes[0] = 8'h41; ubytes[1] = 8'h00; ubytes[2] = 8'hFF;
        ubytes[3] = 8'h55; ubytes[4] = 8'hA5; ubytes[5] = 8'h3C;
        for (i = 0; i < 6; i = i + 1) begin
            rd(A_FSTAT);
            while (rv[16]) rd(A_FSTAT);
            wr(SA(0, O_TXF), {24'hFFFF_FF, ubytes[i]});  // upper bits are ignored
            if (i == 0) begin
                clks(250);                               // 2 ticks
                check(sm_idle[0] == 1'b0, "sm_idle[0] = 0 once a byte is queued");
                idle_watch = 1'b1;
            end
        end
        k = 0;
        while (urx_n < 6 && k < 100000) begin
            @(posedge clk);
            if (urx_n >= 5) idle_watch = 1'b0;          // last frame started
            k = k + 1;
        end
        idle_watch = 1'b0;
        k = 0;
        while (!sm_idle[0] && k < 5000) begin
            @(posedge clk);
            k = k + 1;
        end
        dt = $realtime - urx_start[5];
        check(sm_idle[0] == 1'b1 && dt >= 10.0 * UBIT, "sm_idle[0] = 1 only after the last stop bit");
        check(idle_bad == 0, "sm_idle[0] stayed 0 while bytes were queued");
        check(urx_n == 6, "6 UART frames received");
        bad = 0;
        for (i = 0; i < 6; i = i + 1)
            if (urx_byte[i] !== ubytes[i] || urx_frame[i][0] !== 1'b0 || urx_frame[i][9] !== 1'b1) begin
                bad = bad + 1;
                $display("  frame %0d: %b, expected byte %h", i, urx_frame[i], ubytes[i]);
            end
        check(bad == 0, "received bytes 41 00 FF 55 A5 3C with start / stop bits");
        check(urx_frame[0] == 10'b10_1000_0010, "'A' on the wire: 0 1000 0010 1");
        // bit time from the edges of the 'A' frame: 0 | 1 0 0 0 0 0 1 0 | 1
        check(uedge_n >= 6, "edges of the first frame recorded");
        bt = (uedge[5] - uedge[0]) / 9.0;
        $display("  measured bit time %0.1f ns (ideal %0.1f)", bt, UBIT);
        check(bt > UBIT * 0.99 && bt < UBIT * 1.01, "bit time 8.68 us +-1 %");
        bad = 0;
        if ((uedge[1] - uedge[0]) / 1.0 < 0.99 * UBIT || (uedge[1] - uedge[0]) > 1.01 * UBIT) bad = bad + 1;
        if ((uedge[2] - uedge[0]) < 1.98 * UBIT || (uedge[2] - uedge[0]) > 2.02 * UBIT) bad = bad + 1;
        if ((uedge[3] - uedge[0]) < 6.93 * UBIT || (uedge[3] - uedge[0]) > 7.07 * UBIT) bad = bad + 1;
        if ((uedge[4] - uedge[0]) < 7.92 * UBIT || (uedge[4] - uedge[0]) > 8.08 * UBIT) bad = bad + 1;
        check(bad == 0, "every edge of the 'A' frame at k bit times (+-1 %)");
        // back to back: 8 ticks x 9 bits + 11-tick stop (PULL, SET x) = 83 ticks
        bad = 0;
        for (i = 1; i < 6; i = i + 1) begin
            dt = urx_start[i] - urx_start[i-1];
            if (dt < 90045.0 || dt > 90065.0) begin
                bad = bad + 1;
                $display("  frame %0d starts %0.1f ns after the previous one", i, dt);
            end
        end
        check(bad == 0, "frames back to back: start every 83 ticks (90.055 us)");
        check(txd == 1'b1 && sm0_oe[1] == 1'b1, "TXD idles high after the last byte");
        wr(A_CTRL, 32'h0);
        uart_on = 1'b0;

        // =================================================================
        if (errors == 0)
            $display("ALL %0d CHECKS PASSED", checks);
        else
            $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end
endmodule

// =============================================================================
// tb_se_programs.v  -  unit test: the serial-engine programs vs the v1 engines
// -----------------------------------------------------------------------------
// Slide 23: "I run the UART, SPI and I2C programs in simulation and compare
// the pin waveforms with my v1 engines, which act as reference models.
// Pass: waveforms match the v1 engines bit for bit."
//
// The assembled programs (programs/*.hex, written by tools/se_asm.py) are
// loaded into se_engine through the register port, exactly as the ARM
// software does it, and configured with the .clkdiv / .pinctrl / .shiftctrl
// values of each .se file.  Next to the serial engine runs a v1 engine with
// the same job; both talk to their own copy of the same behavioural device.
//
//   A. UART (115200 8N1): SM0 uart_tx and v1 ssc_uart send the same 8 bytes;
//      both lines are sampled in the middle of every bit and the 10-bit
//      frames must be equal bit for bit; every SE edge must sit at a whole
//      number of bit times (8.68 us).  At the same time the PC terminal
//      model sends 8 bytes (one with a bad stop bit) to SM1 uart_rx and to
//      the v1 receiver: both must keep the same 7 good bytes.
//   B. SPI (mode 0, 1 MHz): SM0 spi_master and v1 ssc_spi each run the
//      frames [9F 00 00 00] [06] [05 00] against a PmodSF3 flash model:
//      MOSI sampled on every SCK rise, the CS frame of every bit and the
//      received bytes (20 BA 19, status 02) must match.  SCK = 1 MHz, MOSI
//      never changes while SCK is high, a count word of 0 does nothing.
//   C. I2C (100 kHz): SM0 i2c_master and v1 ssc_i2c each do the PmodTMP2
//      read (S 96 A 00 A Sr 97 A 0C A 80 N P) against the I2C slave model;
//      the bus monitors must log the same event list.  Then a wrong address
//      (both: S 90 N P, the SE skips the rest of the transaction), a good
//      read again, and a read with clock stretching (20 us after every
//      slave ACK).  SE SCL high >= 4.0 us and low >= 4.7 us all the time.
//   D. Manchester: SM0 manchester_tx -> SM1 manchester_rx.  40 random bytes
//      (fast clock), a TB decoder checks the IEEE 802.3 waveform (1 = low ->
//      high in the middle of the bit), SM1's CRC-16 (CRC 8 instruction) must
//      equal a TB model; then 3 bytes at the real 100 kbit/s setting, and
//      16 bytes each with the RX clock 5 % slower and 5 % faster.
//   E. OLED: SM1 oled_spi sends a reset word, commands and data; the bits
//      seen on SCLK rises (SDIN, DC, RES#) must equal the expected stream;
//      SDIN/DC/RES# never change while SCLK is high; SCLK = 5 MHz.
// Ends with "ALL n CHECKS PASSED" or "m OF n CHECKS FAILED".
// =============================================================================
`timescale 1ns / 1ps
module tb_se_programs;
    // Folder with the assembled programs: "programs/" when the simulator runs
    // in the project folder (Icarus, sim/run_*.sh), six levels up when it runs
    // in Vivado's build/sim/ssc2_sim.sim/sim_1/behav/xsim folder
    // (vivado/run_sim.tcl).  +define+PROG_DIR="..." overrides both.
    function [8*64-1:0] prog_dir;
        input dummy;
        integer fdp;
        begin
`ifdef PROG_DIR
            prog_dir = `PROG_DIR;
`else
            fdp = $fopen("programs/uart_tx.hex", "r");
            if (fdp != 0) begin
                $fclose(fdp);
                prog_dir = "programs/";
            end else begin
                prog_dir = "../../../../../../programs/";
            end
`endif
        end
    endfunction
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0;
    integer errors = 0;
    integer seed = 32'h5E9_2023;

    task check(input cond, input [8*100-1:0] what);
        begin
            checks = checks + 1;
            if (cond !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    task check_eq(input [31:0] got, input [31:0] exp, input [8*100-1:0] what);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL: %0s: got %h, expected %h (t=%0t)", what, got, exp, $time);
            end
        end
    endtask

    // ---------------- the programs (values of the .se directives) ----------------
    localparam UTX_ORG = 0,  UTX_LEN = 7;
    localparam [31:0] UTX_CLK = 32'h006C_8000, UTX_PIN = 32'h0220_0015, UTX_SH = 32'h0000_0000;
    localparam URX_ORG = 7,  URX_LEN = 11;
    localparam [31:0] URX_CLK = 32'h006C_8000, URX_PIN = 32'h0000_4200, URX_SH = 32'h0000_0700;
    localparam SPI_ORG = 18, SPI_LEN = 14;
    localparam [31:0] SPI_CLK = 32'h0019_0000, SPI_PIN = 32'h0B10_1E11, SPI_SH = 32'h0000_1203;
    localparam I2C_ORG = 0,  I2C_LEN = 32;
    localparam [31:0] I2C_CLK = 32'h007D_0000, I2C_PIN = 32'h000C_7B1F, I2C_SH = 32'h0000_0003;
    localparam MTX_ORG = 7,  MTX_LEN = 13;
    localparam [31:0] MTX_CLK = 32'h003E_8000, MTX_PIN = 32'h0200_1400, MTX_SH = 32'h0000_0700;
    localparam MRX_ORG = 20, MRX_LEN = 12;
    localparam [31:0] MRX_CLK = 32'h003E_8000, MRX_PIN = 32'h0000_4200, MRX_SH = 32'h0000_1400;
    localparam OLD_ORG = 26, OLD_LEN = 6;
    localparam [31:0] OLD_CLK = 32'h000A_0000, OLD_PIN = 32'h0F40_1C01, OLD_SH = 32'h0000_1A01;

    // which test owns the serial-engine pins
    localparam [2:0] T_NONE = 3'd0, T_UART = 3'd1, T_SPI = 3'd2, T_I2C = 3'd3,
                     T_MAN = 3'd4, T_OLED = 3'd5;
    reg [2:0] tsel = T_NONE;

    // ---------------- serial engine (DUT) ----------------
    reg  [31:0] time_us = 32'd0;
    reg         reg_we = 1'b0, reg_re = 1'b0;
    reg  [8:0]  reg_addr = 9'd0;
    reg  [31:0] reg_wdata = 32'd0;
    wire [31:0] reg_rdata;
    wire [1:0]  ni_tx_full, ni_rx_empty, out_shift_left, in_shift_left, sm_idle, rx_not_empty;
    wire [31:0] ni_rx_data0, ni_rx_data1;
    wire [3:0]  sm0_out, sm0_oe, sm1_out, sm1_oe;
    wire [3:0]  sm0_in, sm1_in;

    se_engine dut (
        .clk(clk), .rst_n(rst_n), .time_us(time_us),
        .reg_we(reg_we), .reg_re(reg_re), .reg_addr(reg_addr),
        .reg_wdata(reg_wdata), .reg_rdata(reg_rdata),
        .ni_tx_push(2'b00), .ni_tx_data0(32'd0), .ni_tx_data1(32'd0), .ni_tx_full(ni_tx_full),
        .ni_rx_pop(2'b00), .ni_rx_data0(ni_rx_data0), .ni_rx_data1(ni_rx_data1),
        .ni_rx_empty(ni_rx_empty),
        .out_shift_left(out_shift_left), .in_shift_left(in_shift_left),
        .sm0_out(sm0_out), .sm0_oe(sm0_oe), .sm0_in(sm0_in),
        .sm1_out(sm1_out), .sm1_oe(sm1_oe), .sm1_in(sm1_in),
        .sm_idle(sm_idle), .rx_not_empty(rx_not_empty)
    );

    // ---------------- SE pin wiring per test ----------------
    // UART: SM0 pin 1 = TXD, SM1 pin 2 = RXD (from the PC terminal model)
    wire term_tx;
    wire se_txd = (tsel == T_UART && sm0_oe[1]) ? sm0_out[1] : 1'b1;
    // SPI: SM0 pins 0 CS#, 1 MOSI, 2 MISO, 3 SCK
    wire se_cs   = (tsel == T_SPI && sm0_oe[0]) ? sm0_out[0] : 1'b1;
    wire se_mosi = (tsel == T_SPI && sm0_oe[1]) ? sm0_out[1] : 1'b0;
    wire se_sck  = (tsel == T_SPI && sm0_oe[3]) ? sm0_out[3] : 1'b0;
    wire se_miso;
    pullup (se_miso);
    // I2C: SM0 pins 2 SCL, 3 SDA, open drain with pull-ups
    wire scl_se, sda_se;
    pullup (scl_se);
    pullup (sda_se);
    assign scl_se = (tsel == T_I2C && sm0_oe[2]) ? sm0_out[2] : 1'bz;
    assign sda_se = (tsel == T_I2C && sm0_oe[3]) ? sm0_out[3] : 1'bz;
    // Manchester: SM0 pin 1 -> SM1 pin 2 (what SE_LOOP does in the core)
    wire man_line = (tsel == T_MAN && sm0_oe[1]) ? sm0_out[1] : 1'b0;
    // OLED: SM1 pins 0 DC, 1 SDIN, 2 RES#, 3 SCLK
    wire ol_dc   = (tsel == T_OLED) ? sm1_out[0] : 1'b0;
    wire ol_sdin = (tsel == T_OLED) ? sm1_out[1] : 1'b0;
    wire ol_res  = (tsel == T_OLED) ? sm1_out[2] : 1'b0;
    wire ol_sclk = (tsel == T_OLED) ? sm1_out[3] : 1'b0;

    assign sm0_in = (tsel == T_SPI) ? {1'b1, se_miso, 2'b11} :
                    (tsel == T_I2C) ? {sda_se, scl_se, 2'b11} : 4'hF;
    assign sm1_in = (tsel == T_UART) ? {1'b1, term_tx, 2'b11} :
                    (tsel == T_MAN)  ? {1'b1, man_line, 2'b11} : 4'hF;

    // ---------------- v1 reference engines ----------------
    // UART: 115200 baud = 100 MHz / (16 x 54.25), 8N1
    reg        v1u_push = 1'b0;
    reg  [7:0] v1u_wdata = 8'd0;
    reg        v1u_pop = 1'b0;
    wire [7:0] v1u_rdata;
    wire       v1u_rx_empty, v1u_ferr;
    wire       v1_txd;

    ssc_uart v1_uart (
        .clk(clk), .rst_n(rst_n),
        .tx_en(1'b1), .rx_en(1'b1), .data_bits(2'd3), .parity_en(1'b0), .parity_odd(1'b0),
        .stop2(1'b0), .flow_en(1'b0), .loopback(1'b0), .baud_int(16'd54), .baud_frac(4'd4),
        .tx_flush(1'b0), .rx_flush(1'b0),
        .tx_push(v1u_push), .tx_data(v1u_wdata), .rx_pop(v1u_pop), .rx_data(v1u_rdata),
        .tx_empty(), .tx_full(), .rx_empty(v1u_rx_empty), .rx_full(), .tx_count(), .rx_count(),
        .tx_busy(), .rx_busy(), .cts_ok(),
        .ev_parity_err(), .ev_frame_err(v1u_ferr), .ev_break(), .ev_rx_ovf(), .ev_tx_ovf(),
        .rxd(term_tx), .txd(v1_txd), .cts_n(1'b0), .rts_n()
    );

    // SPI master: mode 0, 8 bit, MSB first, automatic CS, SCK = 100 MHz / (2 x 50) = 1 MHz
    reg         v1s_push = 1'b0;
    reg  [7:0]  v1s_wdata = 8'd0;
    reg         v1s_pop = 1'b0;
    wire [31:0] v1s_rdata;
    wire        v1s_rx_empty, v1s_busy;
    wire        v1_sck, v1_mosi, v1_miso;
    wire [3:0]  v1_csn;
    pullup (v1_miso);

    ssc_spi v1_spi (
        .clk(clk), .rst_n(rst_n),
        .en(1'b1), .cpol(1'b0), .cpha(1'b0), .lsb_first(1'b0), .len_m1(5'd7),
        .cs_sel(2'd0), .cs_manual(1'b0), .cs_level(1'b0), .slave_mode(1'b0), .loopback(1'b0),
        .clk_div(16'd49), .tx_flush(1'b0), .rx_flush(1'b0), .abort(1'b0),
        .tx_push(v1s_push), .tx_data({24'd0, v1s_wdata}), .rx_pop(v1s_pop), .rx_data(v1s_rdata),
        .tx_empty(), .tx_full(), .rx_empty(v1s_rx_empty), .rx_full(), .tx_count(), .rx_count(),
        .busy(v1s_busy), .ev_done(), .ev_rx_ovf(), .ev_tx_ovf(),
        .sclk(v1_sck), .mosi(v1_mosi), .miso(v1_miso), .cs_n(v1_csn),
        .s_sclk(1'b0), .s_mosi(1'b0), .s_cs_n(1'b1), .s_miso(), .s_miso_oe()
    );

    // I2C master: 100 kHz (prescale 249)
    reg        v1i_cmd = 1'b0;
    reg  [7:0] v1i_len = 8'd0;
    reg        v1i_read = 1'b0, v1i_stop = 1'b0;
    reg  [9:0] v1i_addr = 10'h04B;
    reg        v1i_flush = 1'b0;
    reg        v1i_push = 1'b0;
    reg  [7:0] v1i_wdata = 8'd0;
    reg        v1i_pop = 1'b0;
    wire [7:0] v1i_rdata;
    wire       v1i_rx_empty, v1i_nack, v1i_done;
    wire       v1_scl_oe, v1_sda_oe;
    wire       scl_v1, sda_v1;
    pullup (scl_v1);
    pullup (sda_v1);
    assign scl_v1 = v1_scl_oe ? 1'b0 : 1'bz;
    assign sda_v1 = v1_sda_oe ? 1'b0 : 1'bz;

    ssc_i2c v1_i2c (
        .clk(clk), .rst_n(rst_n),
        .en(1'b1), .auto_wr(1'b0), .prescale(16'd249), .addr(v1i_addr), .ten_bit(1'b0),
        .cmd_valid(v1i_cmd), .cmd_len(v1i_len), .cmd_read(v1i_read), .cmd_stop(v1i_stop),
        .cmd_stop_only(1'b0), .tx_flush(v1i_flush), .rx_flush(1'b0), .abort(1'b0),
        .tx_push(v1i_push), .tx_data(v1i_wdata), .rx_pop(v1i_pop), .rx_data(v1i_rdata),
        .tx_empty(), .tx_full(), .rx_empty(v1i_rx_empty), .rx_full(), .tx_count(), .rx_count(),
        .busy(), .holding(), .bus_busy(), .nack_flag(v1i_nack), .arb_flag(),
        .scl_state(), .sda_state(),
        .ev_done(v1i_done), .ev_nack(), .ev_arb_lost(), .ev_tx_ovf(),
        .scl_in(scl_v1), .scl_oe(v1_scl_oe), .sda_in(sda_v1), .sda_oe(v1_sda_oe)
    );

    // ---------------- behavioural devices ----------------
    tb_uart_term   term     (.rx(se_txd), .tx(term_tx));
    tb_spi_flash   flash_se (.cs_n(se_cs), .sclk(se_sck), .mosi(se_mosi), .miso(se_miso));
    tb_spi_flash   flash_v1 (.cs_n(v1_csn[0]), .sclk(v1_sck), .mosi(v1_mosi), .miso(v1_miso));
    tb_i2c_slave   slave_se (.scl(scl_se), .sda(sda_se));
    tb_i2c_slave   slave_v1 (.scl(scl_v1), .sda(sda_v1));
    tb_i2c_monitor mon_se   (.scl(scl_se), .sda(sda_se));
    tb_i2c_monitor mon_v1   (.scl(scl_v1), .sda(sda_v1));

    // ---------------- register map ----------------
    localparam [8:0] A_CTRL = 9'h000, A_FSTAT = 9'h004;
    localparam [8:0] O_CLKDIV = 9'h010, O_PINCTRL = 9'h014, O_SHIFT = 9'h018, O_TXF = 9'h01C,
                     O_RXF = 9'h020, O_EXEC = 9'h024, O_STATE = 9'h028, O_CRC = 9'h034,
                     O_PINS = 9'h038;

    function [8:0] SA(input integer s, input [8:0] off);
        SA = off + ((s == 1) ? 9'h040 : 9'h000);
    endfunction

    // ---------------- register access (like the CPU) ----------------
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

    task clks(input integer n);
        repeat (n) @(posedge clk);
    endtask

    // program words read from programs/<name>.hex
    reg [15:0] hexmem [0:31];

    task load_prog(input integer org, input integer len);
        integer i, bad;
        begin
            bad = 0;
            for (i = 0; i < len; i = i + 1) begin
                if (^hexmem[i] === 1'bx) bad = bad + 1;
                wr(9'h100 + (org + i) * 4, {16'd0, hexmem[i]});
            end
            check(bad == 0, "the .hex file has all the program words");
        end
    endtask

    task smcfg(input integer s, input [31:0] clkdiv, input [31:0] pctl, input [31:0] shift);
        begin
            wr(SA(s, O_CLKDIV), clkdiv);
            wr(SA(s, O_PINCTRL), pctl);
            wr(SA(s, O_SHIFT), shift);
        end
    endtask

    // push txq[0..ntx-1] into SM ts and pop nrx words of SM rs into rxq[]
    reg [31:0] txq [0:255];
    reg [31:0] rxq [0:255];
    integer    xfer_tx, xfer_rx;

    task se_xfer(input integer ts, input integer ntx, input integer rs, input integer nrx,
                 input integer tmo);
        integer c;
        reg full, empty;
        begin
            xfer_tx = 0;
            xfer_rx = 0;
            c = 0;
            while ((xfer_tx < ntx || xfer_rx < nrx) && c < tmo) begin
                rd(A_FSTAT);
                full  = (ts == 1) ? rv[18] : rv[16];
                empty = (rs == 1) ? rv[19] : rv[17];
                if (xfer_tx < ntx && !full) begin
                    wr(SA(ts, O_TXF), txq[xfer_tx]);
                    xfer_tx = xfer_tx + 1;
                end
                if (xfer_rx < nrx && !empty) begin
                    rd(SA(rs, O_RXF));
                    rxq[xfer_rx] = rv;
                    xfer_rx = xfer_rx + 1;
                end
                c = c + 1;
            end
            if (xfer_tx < ntx || xfer_rx < nrx)
                $display("  se_xfer time-out: %0d of %0d words sent, %0d of %0d received",
                         xfer_tx, ntx, xfer_rx, nrx);
        end
    endtask

    // wait until SM s is stalled on an empty TX FIFO (all words used)
    task wait_sm_idle(input integer s, input integer tmo);
        integer c;
        begin
            c = 0;
            while (!sm_idle[s] && c < tmo) begin
                @(posedge clk);
                c = c + 1;
            end
        end
    endtask

    // =========================================================================
    // A. UART monitors
    // =========================================================================
    localparam real UBIT = 8680.556;                 // 115200 baud, ns
    reg uart_rec = 1'b0;

    // sample a UART line in the middle of every bit: frame = {stop, d7..d0, start}
    reg  [9:0] fr_se [0:15];
    reg  [9:0] fr_v1 [0:15];
    real       fr_se_t [0:15];
    integer    fr_se_n = 0, fr_v1_n = 0;
    real       ta0, tb0;
    integer    ia, ib;
    reg  [9:0] fa, fb;

    always @(negedge se_txd) if (uart_rec) begin
        ta0 = $realtime;
        for (ia = 0; ia < 10; ia = ia + 1) begin
            #(UBIT * (ia + 0.5) - ($realtime - ta0));
            fa[ia] = se_txd;
        end
        if (fr_se_n < 16) begin
            fr_se[fr_se_n]   = fa;
            fr_se_t[fr_se_n] = ta0;
        end
        fr_se_n = fr_se_n + 1;
    end

    always @(negedge v1_txd) if (uart_rec) begin
        tb0 = $realtime;
        for (ib = 0; ib < 10; ib = ib + 1) begin
            #(UBIT * (ib + 0.5) - ($realtime - tb0));
            fb[ib] = v1_txd;
        end
        if (fr_v1_n < 16) fr_v1[fr_v1_n] = fb;
        fr_v1_n = fr_v1_n + 1;
    end

    // every edge of the SE TX line, for the bit-time check
    real    se_edge [0:255];
    integer se_edge_n = 0;
    always @(se_txd) if (uart_rec && se_edge_n < 256) begin
        se_edge[se_edge_n] = $realtime;
        se_edge_n = se_edge_n + 1;
    end

    // v1 UART receiver: take every byte as soon as it arrives
    reg [7:0] v1u_rx [0:31];
    integer   v1u_rx_n = 0;
    integer   v1u_ferr_n = 0;
    always @(posedge clk) begin
        if (v1u_ferr) v1u_ferr_n <= v1u_ferr_n + 1;
        if (v1u_pop) v1u_pop <= 1'b0;
        else if (!v1u_rx_empty) begin
            v1u_rx[v1u_rx_n] <= v1u_rdata;
            v1u_rx_n <= v1u_rx_n + 1;
            v1u_pop <= 1'b1;
        end
    end

    // =========================================================================
    // B. SPI monitors: MOSI on every SCK rise while CS# is low, per CS frame
    // =========================================================================
    reg     spi_rec = 1'b0;
    reg     sb_se [0:255];
    reg     sb_v1 [0:255];
    integer sf_se [0:255];
    integer sf_v1 [0:255];
    integer sn_se = 0, sn_v1 = 0;            // bits seen
    integer scs_se = 0, scs_v1 = 0;          // CS# frames seen
    integer spi_bad_mosi = 0, spi_bad_cs = 0;
    real    sck_t [0:15];
    integer sck_n = 0;

    always @(negedge se_cs) if (spi_rec) begin
        scs_se = scs_se + 1;
        if (se_sck !== 1'b0) spi_bad_cs = spi_bad_cs + 1;
    end
    always @(posedge se_cs) if (spi_rec && se_sck !== 1'b0) spi_bad_cs = spi_bad_cs + 1;
    always @(negedge v1_csn[0]) if (spi_rec) scs_v1 = scs_v1 + 1;

    always @(posedge se_sck) if (spi_rec && se_cs === 1'b0) begin
        sb_se[sn_se] = se_mosi;
        sf_se[sn_se] = scs_se;
        sn_se = sn_se + 1;
        if (sck_n < 16) begin
            sck_t[sck_n] = $realtime;
            sck_n = sck_n + 1;
        end
    end
    always @(posedge v1_sck) if (spi_rec && v1_csn[0] === 1'b0) begin
        sb_v1[sn_v1] = v1_mosi;
        sf_v1[sn_v1] = scs_v1;
        sn_v1 = sn_v1 + 1;
    end
    // mode 0: MOSI may only change while SCK is low
    always @(se_mosi) if (spi_rec) begin
        #1;
        if (se_sck === 1'b1 && se_cs === 1'b0) spi_bad_mosi = spi_bad_mosi + 1;
    end

    // v1 SPI receiver
    reg [7:0] v1s_rx [0:31];
    integer   v1s_rx_n = 0;
    always @(posedge clk) begin
        if (v1s_pop) v1s_pop <= 1'b0;
        else if (!v1s_rx_empty) begin
            v1s_rx[v1s_rx_n] <= v1s_rdata[7:0];
            v1s_rx_n <= v1s_rx_n + 1;
            v1s_pop <= 1'b1;
        end
    end

    reg [7:0] spq [0:15];                    // bytes of the v1 frames

    task v1s_frame(input integer first, input integer n);
        integer i, c;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk);
                v1s_wdata = spq[first + i];
                v1s_push  = 1'b1;
            end
            @(negedge clk);
            v1s_push = 1'b0;
            c = 0;
            while (v1s_rx_n < first + n && c < 200000) begin
                @(posedge clk);
                c = c + 1;
            end
            c = 0;
            while (v1s_busy && c < 10000) begin          // CS# back high
                @(posedge clk);
                c = c + 1;
            end
        end
    endtask

    // =========================================================================
    // C. I2C: v1 command helper and SCL timing of the SE bus
    // =========================================================================
    integer v1i_done_n = 0;
    always @(posedge clk) if (v1i_done) v1i_done_n <= v1i_done_n + 1;

    reg [7:0] v1i_rx [0:15];
    integer   v1i_rx_n = 0;
    always @(posedge clk) begin
        if (v1i_pop) v1i_pop <= 1'b0;
        else if (!v1i_rx_empty) begin
            v1i_rx[v1i_rx_n] <= v1i_rdata;
            v1i_rx_n <= v1i_rx_n + 1;
            v1i_pop <= 1'b1;
        end
    end

    task v1i_command(input [7:0] len, input rdn, input stp);
        integer d0, c;
        begin
            d0 = v1i_done_n;
            @(negedge clk);
            v1i_len = len; v1i_read = rdn; v1i_stop = stp; v1i_cmd = 1'b1;
            @(negedge clk);
            v1i_cmd = 1'b0;
            c = 0;
            while (v1i_done_n == d0 && c < 400000) begin
                @(posedge clk);
                c = c + 1;
            end
        end
    endtask

    task v1i_tmp2_read(input [6:0] addr);
        begin
            v1i_addr = {3'b000, addr};
            @(negedge clk);
            v1i_wdata = 8'h00; v1i_push = 1'b1;          // register pointer 0
            @(negedge clk);
            v1i_push = 1'b0;
            v1i_command(8'd1, 1'b0, 1'b0);               // S addr+W, 00, keep the bus
            if (!v1i_nack)
                v1i_command(8'd2, 1'b1, 1'b1);           // Sr addr+R, 2 bytes, P
            else begin
                @(negedge clk);
                v1i_flush = 1'b1;                        // the 00 was never sent
                @(negedge clk);
                v1i_flush = 1'b0;
            end
        end
    endtask

    // SE words of the TMP2 read (SE format: [31] START [30] STOP [29:22] byte [21] 9th bit)
    localparam [31:0] I2C_S = 32'h8000_0000, I2C_P = 32'h4000_0000;
    function [31:0] i2c_wr(input [7:0] b);
        i2c_wr = {2'b00, b, 1'b1, 21'd0};
    endfunction
    function [31:0] i2c_rd(input nack);
        i2c_rd = {2'b00, 8'hFF, nack, 21'd0};
    endfunction

    task se_tmp2_words(input [6:0] addr);
        begin
            txq[0] = I2C_S | i2c_wr({addr, 1'b0});
            txq[1] = i2c_wr(8'h00);
            txq[2] = I2C_S | i2c_wr({addr, 1'b1});
            txq[3] = i2c_rd(1'b0);
            txq[4] = I2C_P | i2c_rd(1'b1);
        end
    endtask

    // SCL high / low times on the SE bus
    reg     i2c_rec = 1'b0;
    real    scl_rise_t = 0.0, scl_fall_t = 0.0;
    real    scl_min_hi = 1.0e9, scl_min_lo = 1.0e9;
    always @(posedge scl_se) if (i2c_rec) begin
        if ($realtime - scl_fall_t < scl_min_lo) scl_min_lo = $realtime - scl_fall_t;
        scl_rise_t = $realtime;
    end
    always @(negedge scl_se) if (i2c_rec) begin
        if ($realtime - scl_rise_t < scl_min_hi) scl_min_hi = $realtime - scl_rise_t;
        scl_fall_t = $realtime;
    end

    // compare the two bus logs, and both with the expected list
    integer exp_ev [0:15];
    integer exp_n;

    task cmp_i2c_logs(input [8*48-1:0] what);
        integer i, bad;
        begin
            bad = 0;
            if (mon_se.nev != exp_n || mon_v1.nev != exp_n) bad = bad + 1;
            for (i = 0; i < exp_n; i = i + 1)
                if (mon_se.ev[i] != exp_ev[i] || mon_v1.ev[i] != exp_ev[i]) bad = bad + 1;
            if (bad != 0) begin
                $display("  %0s: expected %0d events", what, exp_n);
                mon_v1.show;
                mon_se.show;
            end
            check(bad == 0, {what, ": SE bus events == v1 bus events"});
        end
    endtask

    // =========================================================================
    // D. Manchester: IEEE 802.3 decoder on the line + CRC model
    // =========================================================================
    real      man_T = 640.0;                 // bit time, ns
    reg       man_rec = 1'b0;
    reg [7:0] man_dec [0:63];
    integer   man_dn = 0, man_bad = 0;
    real      m0;
    integer   mk;
    reg       ma, mb;
    reg [7:0] mbyte;

    // a frame starts with the rising middle edge of the start bit (line idle low)
    always @(posedge man_line) if (man_rec) begin
        m0 = $realtime;
        mbyte = 8'd0;
        #(man_T / 4.0);
        if (man_line !== 1'b1) man_bad = man_bad + 1;      // start bit 2nd half high
        for (mk = 1; mk <= 8; mk = mk + 1) begin
            #(m0 + mk * man_T - man_T / 4.0 - $realtime);
            ma = man_line;                                  // first half
            #(man_T / 2.0);
            mb = man_line;                                  // second half
            if (ma === mb) man_bad = man_bad + 1;           // no middle edge
            mbyte[mk - 1] = mb;                             // 1 = low -> high
        end
        if (man_dn < 64) man_dec[man_dn] = mbyte;
        man_dn = man_dn + 1;
    end

    function [15:0] crc_byte(input [15:0] c, input [7:0] b);     // LSB first (wire order)
        integer i;
        reg [15:0] t;
        begin
            t = c;
            for (i = 0; i < 8; i = i + 1)
                t = {t[14:0], 1'b0} ^ ((t[15] ^ b[i]) ? 16'h1021 : 16'h0000);
            crc_byte = t;
        end
    endfunction

    // =========================================================================
    // E. OLED: bits on SCLK rises, changes while SCLK is high
    // =========================================================================
    reg       ol_rec = 1'b0;
    reg [2:0] ol_bit [0:255];                // {RES#, DC, SDIN}
    integer   ol_n = 0, ol_bad_hi = 0, ol_bad_setup = 0, ol_bad_hold = 0;
    real      ol_rise_t = 0.0, ol_chg_t = 0.0;
    real      ol_min_hi = 1.0e9, ol_min_per = 1.0e9;

    always @(posedge ol_sclk) if (ol_rec) begin
        if (ol_n < 256) ol_bit[ol_n] = {ol_res, ol_dc, ol_sdin};
        ol_n = ol_n + 1;
        if ($realtime - ol_rise_t < ol_min_per) ol_min_per = $realtime - ol_rise_t;
        if ($realtime - ol_chg_t < 100.0) ol_bad_setup = ol_bad_setup + 1;
        ol_rise_t = $realtime;
    end
    always @(negedge ol_sclk) if (ol_rec) begin
        if ($realtime - ol_rise_t < ol_min_hi) ol_min_hi = $realtime - ol_rise_t;
    end
    always @(ol_dc or ol_res or ol_sdin) if (ol_rec) begin
        ol_chg_t = $realtime;
        #1;
        if (ol_sclk === 1'b1) ol_bad_hi = ol_bad_hi + 1;
        if ($realtime - ol_rise_t < 50.0) ol_bad_hold = ol_bad_hold + 1;
    end

    // ---------------- watchdog ----------------
    initial begin
        #30_000_000;
        $display("FAIL: watchdog time-out, the test hung");
        errors = errors + 1;
        $display("%0d OF %0d CHECKS FAILED", errors, checks + 1);
        $finish;
    end

    // =========================================================================
    // test sequence
    // =========================================================================
    reg  [7:0]  txb [0:15];
    reg  [7:0]  rxb [0:15];
    reg  [7:0]  mexp [0:63];
    reg  [31:0] w;
    reg  [15:0] crc_m;
    integer     i, j, k, bad, n, n0, d0, d1;
    real        dt, per;

    initial begin
        for (i = 0; i < 32; i = i + 1) hexmem[i] = 16'hxxxx;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // =================================================================
        // A. UART: uart_tx (SM0) + uart_rx (SM1) against the v1 UART
        // =================================================================
        $display("A: UART 115200 8N1 - uart_tx / uart_rx vs v1 ssc_uart");
        tsel = T_UART;
        $readmemh({prog_dir(1'b0), "uart_tx.hex"}, hexmem, 0, UTX_LEN - 1);
        load_prog(UTX_ORG, UTX_LEN);
        $readmemh({prog_dir(1'b0), "uart_rx.hex"}, hexmem, 0, URX_LEN - 1);
        load_prog(URX_ORG, URX_LEN);
        smcfg(0, UTX_CLK, UTX_PIN, UTX_SH);
        smcfg(1, URX_CLK, URX_PIN, URX_SH);
        wr(A_CTRL, 32'h300);                             // RESTART both
        rd(SA(0, O_PINS));
        check_eq(rv & 32'hFF0, 32'h220, "uart_tx after RESTART: TXD (pin 1) driven high");
        rd(SA(1, O_PINS));
        check_eq(rv & 32'hF00, 32'h000, "uart_rx after RESTART: drives no pin");
        wr(A_CTRL, 32'h003);                             // run SM0 and SM1
        clks(2000);
        check(se_txd === 1'b1 && v1_txd === 1'b1, "both TX lines idle high");
        rd(A_FSTAT);
        check(rv[14:12] == 3'd0, "uart_rx: nothing received while the line is idle");

        txb[0] = 8'h41; txb[1] = 8'h00; txb[2] = 8'hFF; txb[3] = 8'h55;
        txb[4] = 8'hA5;
        for (i = 5; i < 8; i = i + 1) txb[i] = $random(seed);
        rxb[0] = 8'h5A; rxb[1] = 8'h00; rxb[2] = 8'hFF; rxb[3] = 8'h81;
        rxb[4] = 8'h3C;                                  // sent with a bad stop bit
        for (i = 5; i < 8; i = i + 1) rxb[i] = $random(seed);
        for (i = 0; i < 8; i = i + 1) txq[i] = {24'h00_0000, txb[i]};
        uart_rec = 1'b1;
        fork
            begin : a_v1_tx                              // v1: queue all 8 bytes
                for (j = 0; j < 8; j = j + 1) begin
                    @(negedge clk);
                    v1u_wdata = txb[j];
                    v1u_push  = 1'b1;
                end
                @(negedge clk);
                v1u_push = 1'b0;
            end
            begin : a_term                               // the PC sends 8 bytes
                for (k = 0; k < 8; k = k + 1) begin
                    term.send(rxb[k], 1'b0, (k == 4));
                    if (k == 4) #(2.0 * UBIT);           // idle after the bad frame
                end
            end
            se_xfer(0, 8, 1, 7, 200000);                 // SE: 8 bytes out, 7 good bytes in
        join
        k = 0;
        while ((fr_se_n < 8 || fr_v1_n < 8) && k < 200000) begin
            @(posedge clk);
            k = k + 1;
        end
        clks(2000);
        uart_rec = 1'b0;

        // --- TX: bit for bit ---
        check(fr_se_n == 8 && fr_v1_n == 8, "UART TX: 8 frames on both lines");
        bad = 0;
        for (i = 0; i < 8; i = i + 1)
            if (fr_se[i] !== fr_v1[i] || fr_se[i] !== {1'b1, txb[i], 1'b0}) begin
                bad = bad + 1;
                $display("  frame %0d: SE %b  v1 %b  (byte %h)", i, fr_se[i], fr_v1[i], txb[i]);
            end
        check(bad == 0, "UART TX: SE frames == v1 frames bit for bit (start, 8 data, stop)");
        check(fr_se[0] == 10'b10_1000_0010, "UART TX: 'A' on the wire = 0 1000 0010 1");
        // every SE edge at a whole number of bit times after its frame's start edge
        bad = 0;
        for (i = 0; i < se_edge_n; i = i + 1) begin
            k = -1;
            for (j = 0; j < 8; j = j + 1)
                if (fr_se_t[j] <= se_edge[i] + 0.5) k = j;
            if (k >= 0) begin
                dt = se_edge[i] - fr_se_t[k];
                n = $rtoi(dt / UBIT + 0.5);
                if (dt - n * UBIT > 20.0 || dt - n * UBIT < -20.0) begin
                    bad = bad + 1;
                    $display("  SE edge at %0.1f ns = %0.3f bit times after frame %0d", se_edge[i],
                             dt / UBIT, k);
                end
            end
        end
        check(se_edge_n > 30 && bad == 0, "UART TX: every SE edge at k x 8680.6 ns (+-20 ns)");
        // --- RX ---
        check(xfer_rx == 7 && v1u_rx_n == 7, "UART RX: 7 good bytes on SE and on v1");
        check(v1u_ferr_n == 1, "UART RX: v1 saw one framing error");
        bad = 0;
        j = 0;
        for (i = 0; i < 8; i = i + 1)
            if (i != 4) begin
                if (rxq[j] !== {rxb[i], 24'd0} || v1u_rx[j] !== rxb[i]) begin
                    bad = bad + 1;
                    $display("  byte %0d: SE word %h  v1 %h  expected %h", j, rxq[j], v1u_rx[j], rxb[i]);
                end
                j = j + 1;
            end
        check(bad == 0, "UART RX: SE bytes ([31:24]) == v1 bytes, bad-stop byte dropped by both");
        rd(A_FSTAT);
        check(rv[14:12] == 3'd0 && rv[29] == 1'b0, "UART RX: RX1 empty, no overflow");
        check(term.rx_cnt >= 8 && term.rx_mem[0] == 8'h41, "UART TX: PC terminal decoded the SE bytes");
        wr(A_CTRL, 32'h0);

        // =================================================================
        // B. SPI: spi_master (SM0) against the v1 SPI master, PmodSF3 flash
        // =================================================================
        $display("B: SPI mode 0 - spi_master vs v1 ssc_spi, flash READ ID / WREN / RDSR");
        tsel = T_SPI;
        $readmemh({prog_dir(1'b0), "spi_master.hex"}, hexmem, 0, SPI_LEN - 1);
        load_prog(SPI_ORG, SPI_LEN);
        smcfg(0, SPI_CLK, SPI_PIN, SPI_SH);
        wr(A_CTRL, 32'h100);
        rd(SA(0, O_PINS));
        check_eq(rv & 32'hFF0, 32'hB10, "spi_master after RESTART: CS# high, SCK low, oe = 1011");
        spi_rec = 1'b1;
        wr(A_CTRL, 32'h001);
        // v1 frames: [9F 00 00 00] [06] [05 00]
        spq[0] = 8'h9F; spq[1] = 8'h00; spq[2] = 8'h00; spq[3] = 8'h00;
        spq[4] = 8'h06; spq[5] = 8'h05; spq[6] = 8'h00;
        // SE words: a count word, then the bytes in [31:24]
        txq[0]  = 32'h0000_0000;                         // count 0: ignored
        txq[1]  = 32'h0400_0000;
        txq[2]  = 32'h9F00_0000; txq[3] = 32'h0; txq[4] = 32'h0; txq[5] = 32'h0;
        txq[6]  = 32'h0100_0000; txq[7] = 32'h0600_0000;
        txq[8]  = 32'h0200_0000; txq[9] = 32'h0500_0000; txq[10] = 32'h0;
        fork
            begin : b_v1
                v1s_frame(0, 4);
                v1s_frame(4, 1);
                v1s_frame(5, 2);
            end
            se_xfer(0, 11, 0, 7, 100000);
        join
        wait_sm_idle(0, 10000);
        clks(200);
        spi_rec = 1'b0;
        check(scs_se == 3 && scs_v1 == 3, "SPI: 3 CS# frames on both buses (count 0 ignored)");
        check(sn_se == 56 && sn_v1 == 56, "SPI: 56 SCK rises with CS# low on both buses");
        bad = 0;
        for (i = 0; i < 56; i = i + 1)
            if (sb_se[i] !== sb_v1[i] || sf_se[i] != sf_v1[i]) bad = bad + 1;
        check(bad == 0, "SPI: MOSI on every SCK rise and its CS frame: SE == v1 bit for bit");
        bad = 0;
        for (i = 0; i < 7; i = i + 1)
            if (rxq[i] !== {24'd0, v1s_rx[i]}) begin
                bad = bad + 1;
                $display("  byte %0d: SE %h  v1 %h", i, rxq[i], v1s_rx[i]);
            end
        check(xfer_rx == 7 && v1s_rx_n == 7 && bad == 0, "SPI: received bytes SE == v1 (one per byte sent)");
        check(rxq[1] == 32'h20 && rxq[2] == 32'hBA && rxq[3] == 32'h19, "SPI: flash ID 20 BA 19 in [7:0]");
        check(rxq[6] == 32'h02 && v1s_rx[6] == 8'h02, "SPI: status 02 = WEL set by the 1-byte 06 frame");
        per = (sck_t[7] - sck_t[0]) / 7.0;
        check(per > 990.0 && per < 1010.0, "SPI: SE SCK = 1 MHz (+-1 %)");
        check(spi_bad_mosi == 0, "SPI: MOSI only changes while SCK is low");
        check(spi_bad_cs == 0, "SPI: SCK is low whenever CS# changes");
        check(se_cs === 1'b1 && se_sck === 1'b0, "SPI: idle at the end: CS# high, SCK low");
        wr(A_CTRL, 32'h0);

        // =================================================================
        // C. I2C: i2c_master (SM0) against the v1 I2C master, PmodTMP2 model
        // =================================================================
        $display("C: I2C 100 kHz - i2c_master vs v1 ssc_i2c, PmodTMP2 at 0x4B");
        tsel = T_I2C;
        $readmemh({prog_dir(1'b0), "i2c_master.hex"}, hexmem, 0, I2C_LEN - 1);
        load_prog(I2C_ORG, I2C_LEN);
        smcfg(0, I2C_CLK, I2C_PIN, I2C_SH);
        wr(A_CTRL, 32'h100);
        rd(SA(0, O_PINS));
        check_eq(rv & 32'hF00, 32'h000, "i2c_master after RESTART: SCL and SDA released");
        wr(A_CTRL, 32'h001);
        clks(100);
        check(scl_se === 1'b1 && sda_se === 1'b1, "I2C: SE bus idle high");
        i2c_rec = 1'b1;
        mon_se.clear;
        mon_v1.clear;
        // C1: the TMP2 temperature read
        se_tmp2_words(7'h4B);
        fork
            v1i_tmp2_read(7'h4B);
            se_xfer(0, 5, 0, 5, 100000);
        join
        exp_ev[0] = 1000; exp_ev[1] = 8'h96; exp_ev[2] = 8'h00; exp_ev[3] = 1001;
        exp_ev[4] = 8'h97; exp_ev[5] = 8'h0C; exp_ev[6] = 9'h180; exp_ev[7] = 1002;
        exp_n = 8;
        wait_sm_idle(0, 10000);
        clks(1000);
        cmp_i2c_logs("TMP2 read: S 96 A 00 A Sr 97 A 0C A 80 N P");
        check(v1i_rx_n == 2 && v1i_rx[0] == 8'h0C && v1i_rx[1] == 8'h80, "v1 read 0C 80");
        check(xfer_rx == 5 && rxq[0] == 32'h12C && rxq[1] == 32'h000 && rxq[2] == 32'h12E,
              "SE RX words: {byte, ACK} = 12C 000 12E for the address/pointer bytes");
        check(rxq[3] == 32'h018 && rxq[4] == 32'h101, "SE RX words: 018 101 = 0C (ACK) 80 (NACK)");
        check(scl_se === 1'b1 && sda_se === 1'b1, "I2C: SE bus released after STOP");
        // C2: wrong address: S 90 N P on both buses
        mon_se.clear;
        mon_v1.clear;
        se_tmp2_words(7'h48);
        fork
            v1i_tmp2_read(7'h48);
            se_xfer(0, 5, 0, 1, 100000);
        join
        wait_sm_idle(0, 20000);
        clks(1000);
        exp_ev[0] = 1000; exp_ev[1] = 9'h190; exp_ev[2] = 1002;
        exp_n = 3;
        cmp_i2c_logs("wrong address 0x48: S 90 N P");
        check(v1i_nack === 1'b1, "v1 reports the NACK");
        check(xfer_rx == 1 && rxq[0] == 32'h121, "SE reports the NACK: RX word 121 = {90, 1}");
        rd(A_FSTAT);
        check(rv[2:0] == 3'd0 && rv[6:4] == 3'd0, "SE skipped the other 4 words, pushed nothing else");
        rd(SA(0, O_STATE));
        check(rv[4:0] == 5'd0 && rv[8] == 1'b1, "SE back at 'idle': PULL with the bus released");
        // C3: the good read works again straight after the NACK
        mon_se.clear;
        mon_v1.clear;
        se_tmp2_words(7'h4B);
        fork
            v1i_tmp2_read(7'h4B);
            se_xfer(0, 5, 0, 5, 100000);
        join
        wait_sm_idle(0, 10000);
        clks(1000);
        exp_ev[0] = 1000; exp_ev[1] = 8'h96; exp_ev[2] = 8'h00; exp_ev[3] = 1001;
        exp_ev[4] = 8'h97; exp_ev[5] = 8'h0C; exp_ev[6] = 9'h180; exp_ev[7] = 1002;
        exp_n = 8;
        cmp_i2c_logs("TMP2 read after the NACK");
        check(xfer_rx == 5 && rxq[3] == 32'h018 && rxq[4] == 32'h101, "SE reads 0C 80 again");
        // C4: clock stretching: the slave holds SCL low for 20 us after each ACK
        mon_se.clear;
        mon_v1.clear;
        slave_se.stretch_ns = 20000;
        slave_v1.stretch_ns = 20000;
        d0 = slave_se.stretch_count;
        d1 = slave_v1.stretch_count;
        se_tmp2_words(7'h4B);
        fork
            v1i_tmp2_read(7'h4B);
            se_xfer(0, 5, 0, 5, 100000);
        join
        wait_sm_idle(0, 10000);
        clks(1000);
        slave_se.stretch_ns = 0;
        slave_v1.stretch_ns = 0;
        cmp_i2c_logs("TMP2 read with clock stretching");
        check(slave_se.stretch_count - d0 == 3 && slave_v1.stretch_count - d1 == 3,
              "the slave stretched SCL 3 times on both buses");
        check(xfer_rx == 5 && rxq[3] == 32'h018 && rxq[4] == 32'h101, "SE reads 0C 80 with stretching");
        check(v1i_rx_n == 6 && v1i_rx[4] == 8'h0C && v1i_rx[5] == 8'h80, "v1 reads 0C 80 with stretching");
        i2c_rec = 1'b0;
        $display("  SE SCL: shortest high %0.0f ns, shortest low %0.0f ns", scl_min_hi, scl_min_lo);
        check(scl_min_hi >= 4000.0, "SE SCL high time >= 4.0 us (also after a stretch)");
        check(scl_min_lo >= 4700.0, "SE SCL low time >= 4.7 us");
        wr(A_CTRL, 32'h0);

        // =================================================================
        // D. Manchester: manchester_tx (SM0) -> manchester_rx (SM1)
        // =================================================================
        $display("D: Manchester IEEE 802.3 - manchester_tx -> manchester_rx, CRC-16");
        tsel = T_MAN;
        $readmemh({prog_dir(1'b0), "manchester_tx.hex"}, hexmem, 0, MTX_LEN - 1);
        load_prog(MTX_ORG, MTX_LEN);
        $readmemh({prog_dir(1'b0), "manchester_rx.hex"}, hexmem, 0, MRX_LEN - 1);
        load_prog(MRX_ORG, MRX_LEN);
        // D1: 40 random bytes, fast clock (INT 4: 16 ticks = 640 ns per bit)
        smcfg(0, 32'h0004_0000, MTX_PIN, MTX_SH);
        smcfg(1, 32'h0004_0000, MRX_PIN, MRX_SH);
        wr(A_CTRL, 32'h300);
        rd(SA(0, O_PINS));
        check_eq(rv & 32'hFF0, 32'h200, "manchester_tx after RESTART: pin 1 driven low (idle)");
        wr(SA(1, O_EXEC), 32'h0000_E080);                // CRC reset on SM1
        clks(5);
        rd(SA(1, O_CRC));
        check_eq(rv, 32'h0000_FFFF, "SM1 CRC = INIT after the EXEC'd 'CRC reset'");
        man_T = 640.0;
        man_dn = 0;
        man_bad = 0;
        man_rec = 1'b1;
        wr(A_CTRL, 32'h003);
        crc_m = 16'hFFFF;
        for (i = 0; i < 40; i = i + 1) begin
            mexp[i] = $random(seed);
            w = $random(seed);
            txq[i] = {w[31:8], mexp[i]};                 // bits [31:8] are ignored
            crc_m = crc_byte(crc_m, mexp[i]);
        end
        se_xfer(0, 40, 1, 40, 200000);
        wait_sm_idle(0, 10000);
        clks(200);
        man_rec = 1'b0;
        bad = 0;
        for (i = 0; i < 40; i = i + 1)
            if (rxq[i] !== {mexp[i], 24'd0}) begin
                bad = bad + 1;
                if (bad < 5) $display("  byte %0d: RX word %h, expected %h", i, rxq[i], mexp[i]);
            end
        check(xfer_rx == 40 && bad == 0, "Manchester: 40 random bytes arrive intact in [31:24]");
        bad = 0;
        for (i = 0; i < 40; i = i + 1)
            if (man_dec[i] !== mexp[i]) bad = bad + 1;
        check(man_dn == 40 && bad == 0 && man_bad == 0,
              "Manchester: TB decoder sees IEEE 802.3 (1 = low->high mid-bit), 640 ns bits");
        rd(SA(1, O_CRC));
        $display("  CRC-16 over the received bits: SE %h, model %h", rv[15:0], crc_m);
        check_eq(rv, {16'd0, crc_m}, "Manchester: SM1 CRC (CRC 8 per byte) == CRC-16 model");
        rd(A_FSTAT);
        check(rv[29] == 1'b0, "Manchester: no RX overflow");
        // D2: the real setting of the .se files: 100 kbit/s
        wr(A_CTRL, 32'h0);
        smcfg(0, MTX_CLK, MTX_PIN, MTX_SH);
        smcfg(1, MRX_CLK, MRX_PIN, MRX_SH);
        wr(A_CTRL, 32'h300);
        man_T = 10000.0;
        man_dn = 0;
        man_rec = 1'b1;
        wr(A_CTRL, 32'h003);
        mexp[0] = 8'hA7; mexp[1] = 8'h00; mexp[2] = 8'hFF;
        for (i = 0; i < 3; i = i + 1) txq[i] = {24'd0, mexp[i]};
        se_xfer(0, 3, 1, 3, 100000);
        man_rec = 1'b0;
        check(xfer_rx == 3 && rxq[0] == 32'hA700_0000 && rxq[1] == 32'h0 && rxq[2] == 32'hFF00_0000,
              "Manchester at 100 kbit/s (.clkdiv 62.5): A7 00 FF received");
        check(man_dn == 3 && man_dec[0] == 8'hA7 && man_dec[2] == 8'hFF, "100 kbit/s waveform: 10 us bits");
        // D3 / D4: receiver clock 5 % slower, then 5 % faster than the sender
        for (k = 0; k < 2; k = k + 1) begin
            wr(A_CTRL, 32'h0);
            smcfg(0, 32'h0004_0000, MTX_PIN, MTX_SH);
            smcfg(1, (k == 0) ? 32'h0004_3300 : 32'h0003_CD00, MRX_PIN, MRX_SH);  // 4.2 / 3.8
            wr(A_CTRL, 32'h300);
            wr(A_CTRL, 32'h003);
            for (i = 0; i < 16; i = i + 1) begin
                mexp[i] = $random(seed);
                txq[i] = {24'd0, mexp[i]};
            end
            se_xfer(0, 16, 1, 16, 100000);
            bad = 0;
            for (i = 0; i < 16; i = i + 1)
                if (rxq[i] !== {mexp[i], 24'd0}) bad = bad + 1;
            check(xfer_rx == 16 && bad == 0,
                  (k == 0) ? "Manchester: RX clock 5 % slow: 16 bytes intact"
                           : "Manchester: RX clock 5 % fast: 16 bytes intact");
        end
        wait_sm_idle(0, 10000);
        wr(A_CTRL, 32'h0);

        // =================================================================
        // E. OLED: oled_spi on SM1, SSD1306 word format
        // =================================================================
        $display("E: OLED SSD1306 write-only SPI - oled_spi on SM1");
        tsel = T_OLED;
        $readmemh({prog_dir(1'b0), "oled_spi.hex"}, hexmem, 0, OLD_LEN - 1);
        load_prog(OLD_ORG, OLD_LEN);
        smcfg(1, OLD_CLK, OLD_PIN, OLD_SH);
        wr(A_CTRL, 32'h200);
        rd(SA(1, O_PINS));
        check_eq(rv & 32'hFF0, 32'hF40, "oled_spi after RESTART: all 4 pins out, RES# high, SCLK low");
        ol_rec = 1'b1;
        wr(A_CTRL, 32'h002);
        txq[0] = 32'h0E30_0000;                          // RES# low (NOP E3 ignored)
        txq[1] = 32'h2E30_0000;                          // RES# high, command E3 (NOP)
        txq[2] = 32'h2AE0_0000;                          // command AE: display off
        txq[3] = 32'h28D0_0000;                          // command 8D 14: charge pump on
        txq[4] = 32'h2140_0000;
        txq[5] = 32'hA550_0000;                          // data 55 AA 0F 81
        txq[6] = 32'hAAA0_0000;
        txq[7] = 32'hA0F0_0000;
        txq[8] = 32'hA810_0000;
        for (i = 9; i < 16; i = i + 1) begin
            w = $random(seed);
            txq[i] = {w[31], 1'b0, 1'b1, 1'b0, w[7:0], 20'd0};   // random DC and byte
        end
        se_xfer(1, 16, 1, 0, 100000);
        wait_sm_idle(1, 10000);
        clks(50);
        ol_rec = 1'b0;
        check(ol_n == 128, "OLED: 16 words = 128 SCLK rising edges");
        bad = 0;
        for (i = 0; i < 16; i = i + 1)
            for (j = 0; j < 8; j = j + 1)
                if (ol_bit[8 * i + j] !== {txq[i][29], txq[i][31], txq[i][27 - j]}) begin
                    bad = bad + 1;
                    if (bad < 5)
                        $display("  word %0d bit %0d: {RES#,DC,SDIN} = %b", i, j, ol_bit[8 * i + j]);
                end
        check(bad == 0, "OLED: SDIN (MSB first), DC and RES# on every SCLK rise == expected stream");
        check(ol_bad_hi == 0, "OLED: SDIN, DC and RES# never change while SCLK is high");
        check(ol_bad_setup == 0 && ol_bad_hold == 0, "OLED: >= 100 ns set-up and >= 50 ns hold around SCLK rises");
        $display("  SCLK: shortest high %0.0f ns, shortest period %0.0f ns", ol_min_hi, ol_min_per);
        check(ol_min_hi >= 99.0 && ol_min_per >= 199.0 && ol_min_per <= 201.0,
              "OLED: SCLK 5 MHz (period 200 ns, high 100 ns)");
        check(ol_sclk === 1'b0 && ol_res === 1'b1 && ol_dc === txq[15][31], "OLED: idle: SCLK low, RES# high");
        rd(A_FSTAT);
        check(rv[19] == 1'b1, "OLED: nothing pushed to RX1");
        wr(A_CTRL, 32'h0);

        // =================================================================
        if (errors == 0)
            $display("ALL %0d CHECKS PASSED", checks);
        else
            $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end
endmodule

// =============================================================================
// tb_ssc_top.v  -  self-checking testbench for the whole controller
// -----------------------------------------------------------------------------
// The testbench plays the role of the ARM: it reads and writes the registers
// over APB (tasks apb_wr / apb_rd), exactly like the C driver will.  Fake
// devices sit on the pins:
//     UART : tb_uart_term   (a PC terminal)
//     SPI  : tb_spi_flash   on CS0 (PmodSF3)
//            tb_spi_slave   on CS1 (any mode / word length)
//            a behavioural SPI master drives the controller's slave port
//     I2C  : tb_i2c_slave   0x4B  (PmodTMP2, 25.0 C)
//            tb_i2c_slave   0x2A5 (10-bit address device)
//            tb_i2c_master  (a second master, for arbitration tests)
//            tb_i2c_monitor (logs S / Sr / P / bytes / ACK / NACK)
// Every check compares against an expected value; the run ends with
//     ALL n CHECKS PASSED   or   m OF n CHECKS FAILED
// =============================================================================
`timescale 1ns/1ps
module tb_ssc_top;
    // ---------------- register map ----------------
    localparam [11:0] ID = 12'h000, INT_STATUS = 12'h004, INT_ENABLE = 12'h008,
                      BRIDGE_CTRL = 12'h00C, BRIDGE_CNT = 12'h010, SCRATCH = 12'h014,
                      UART_CTRL = 12'h100, UART_BAUD = 12'h104, UART_STATUS = 12'h108,
                      UART_TX = 12'h10C, UART_RX = 12'h110,
                      SPI_CTRL = 12'h200, SPI_CLKDIV = 12'h204, SPI_STATUS = 12'h208,
                      SPI_TX = 12'h20C, SPI_RX = 12'h210,
                      I2C_CTRL = 12'h300, I2C_PRESC = 12'h304, I2C_ADDR = 12'h308,
                      I2C_CMD = 12'h30C, I2C_STATUS = 12'h310, I2C_TX = 12'h314,
                      I2C_RX = 12'h318;

    // ---------------- clock and reset ----------------
    reg clk = 1'b0;
    always #5 clk = ~clk;                   // 100 MHz
    reg rst_n = 1'b0;

    // ---------------- APB ----------------
    reg  [11:0] paddr   = 12'd0;
    reg         psel    = 1'b0;
    reg         penable = 1'b0;
    reg         pwrite  = 1'b0;
    reg  [31:0] pwdata  = 32'd0;
    wire [31:0] prdata;
    wire        pready, pslverr, irq;

    // ---------------- pins ----------------
    wire       uart_txd, uart_rxd, uart_rts_n;
    reg        term_cts_n = 1'b0;
    wire       spi_sclk, spi_mosi, spi_miso;
    wire [3:0] spi_cs_n;
    reg        m_sclk = 1'b0, m_mosi = 1'b0, m_cs_n = 1'b1;   // TB SPI master
    wire       spis_miso, spis_miso_oe, spi_slave_mode;
    wire       scl, sda, scl_oe, sda_oe;

    pullup (scl);
    pullup (sda);
    pullup (spi_miso);
    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;

    ssc_apb_top dut (
        .pclk(clk), .presetn(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .irq(irq),
        .uart_rxd(uart_rxd), .uart_txd(uart_txd),
        .uart_cts_n(term_cts_n), .uart_rts_n(uart_rts_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .spis_sclk(m_sclk), .spis_mosi(m_mosi), .spis_cs_n(m_cs_n),
        .spis_miso(spis_miso), .spis_miso_oe(spis_miso_oe),
        .spi_slave_mode(spi_slave_mode),
        .i2c_scl_in(scl), .i2c_scl_oe(scl_oe),
        .i2c_sda_in(sda), .i2c_sda_oe(sda_oe)
    );

    tb_uart_term  term  (.rx(uart_txd), .tx(uart_rxd));
    tb_spi_flash  flash (.cs_n(spi_cs_n[0]), .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso));
    tb_spi_slave  gslv  (.cs_n(spi_cs_n[1]), .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso));
    tb_i2c_slave  #(.ADDR(10'h04B), .TEN(0)) tmp2  (.scl(scl), .sda(sda));
    tb_i2c_slave  #(.ADDR(10'h2A5), .TEN(1)) dev10 (.scl(scl), .sda(sda));
    tb_i2c_master m2  (.scl(scl), .sda(sda));
    tb_i2c_monitor mon (.scl(scl), .sda(sda));

    // ---------------- bookkeeping ----------------
    integer errors = 0;
    integer checks = 0;

    task check;
        input        cond;
        input [8*80-1:0] what;
        begin
            checks = checks + 1;
            if (cond !== 1'b1) begin
                errors = errors + 1;
                $display("    FAIL  %0s   (t = %0t ns)", what, $time);
            end
        end
    endtask

    task check_eq;
        input [31:0] got;
        input [31:0] exp;
        input [8*80-1:0] what;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("    FAIL  %0s: got 0x%0h, expected 0x%0h   (t = %0t ns)",
                         what, got, exp, $time);
            end
        end
    endtask

    // ---------------- APB master (the "CPU") ----------------
    task apb_wr;
        input [11:0] a;
        input [31:0] d;
        begin
            @(posedge clk);
            paddr <= a; pwdata <= d; pwrite <= 1'b1; psel <= 1'b1; penable <= 1'b0;
            @(posedge clk);
            penable <= 1'b1;
            @(posedge clk);
            while (!pready) @(posedge clk);
            psel <= 1'b0; penable <= 1'b0; pwrite <= 1'b0;
        end
    endtask

    task apb_rd;
        input  [11:0] a;
        output [31:0] d;
        begin
            @(posedge clk);
            paddr <= a; pwrite <= 1'b0; psel <= 1'b1; penable <= 1'b0;
            @(posedge clk);
            penable <= 1'b1;
            @(posedge clk);
            while (!pready) @(posedge clk);
            d = prdata;
            psel <= 1'b0; penable <= 1'b0;
        end
    endtask

    // shared scratch variables
    reg [31:0] r, r2;
    integer    i, j, k, n, t0, t1;
    real       rt0, rt1;

    // wait until an INT_STATUS bit is set (polling), with a time-out in us
    task wait_int;
        input integer bitn;
        input integer timeout_us;
        integer tend;
        begin
            tend = $time + timeout_us * 1000;
            apb_rd(INT_STATUS, r);
            while (r[bitn] !== 1'b1 && $time < tend) begin
                #(200);
                apb_rd(INT_STATUS, r);
            end
            if (r[bitn] !== 1'b1) $display("    (time-out waiting for INT bit %0d)", bitn);
        end
    endtask

    task wait_term_rx;                     // wait until the terminal got n bytes
        input integer cnt;
        input integer timeout_us;
        integer tend;
        begin
            tend = $time + timeout_us * 1000;
            while (term.rx_cnt < cnt && $time < tend) #(100);
        end
    endtask

    // =================================================================
    // UART helpers
    // =================================================================
    integer div16;
    task uart_setup;                       // same format on DUT and terminal
        input integer baud;
        input integer bits;                // 5..8
        input         par_en;
        input         par_odd;
        input         stop2;
        input         flow;
        begin
            div16 = (100000000 + baud / 2) / baud;     // divisor x 16, rounded
            apb_wr(UART_BAUD, {12'd0, div16[3:0], div16[19:4]});
            apb_wr(UART_CTRL, 32'h0003_0003 | ((bits - 5) << 2) | (par_en << 4) |
                              (par_odd << 5) | (stop2 << 6) | (flow << 7));
            term.bit_ns  = 1.0e9 / baud;
            term.nbits   = bits;
            term.par_en  = par_en;
            term.par_odd = par_odd;
            term.stop2   = stop2;
        end
    endtask

    // =================================================================
    // SPI helpers
    // =================================================================
    reg [7:0] sbuf [0:63];                 // bytes to send
    reg [7:0] rbuf [0:63];                 // bytes received
    // one SPI frame with manual CS0 (flash): send n bytes, collect n bytes
    task spi_frame;
        input integer nbytes;
        input [31:0]  ctrl_base;           // EN, mode, 8 bit, CS_MANUAL, CS_SEL
        integer sent, got, chunk;
        begin
            apb_wr(SPI_CTRL, ctrl_base | 32'h0003_0000);       // flush
            apb_wr(SPI_CTRL, ctrl_base | (1 << 12));          // CS low
            sent = 0; got = 0;
            while (got < nbytes) begin
                chunk = 0;
                while (sent < nbytes && chunk < 16) begin
                    apb_wr(SPI_TX, {24'd0, sbuf[sent]});
                    sent = sent + 1; chunk = chunk + 1;
                end
                apb_rd(SPI_STATUS, r);
                while (r[20:16] < chunk || r[4]) apb_rd(SPI_STATUS, r);
                for (k = 0; k < chunk; k = k + 1) begin
                    apb_rd(SPI_RX, r);
                    rbuf[got] = r[7:0];
                    got = got + 1;
                end
            end
            apb_wr(SPI_CTRL, ctrl_base);                       // CS high
        end
    endtask

    task flash_wait_ready;
        input [31:0] ctrl_base;
        begin
            sbuf[0] = 8'h05; sbuf[1] = 8'h00;
            spi_frame(2, ctrl_base);
            while (rbuf[1][0]) spi_frame(2, ctrl_base);
        end
    endtask

    // the testbench as an SPI master talking to the DUT's slave port
    reg [31:0] mtx [0:3];
    reg [31:0] mrx [0:3];
    task tbm_xfer;
        input integer nwords;
        input integer nb;
        input cpol, cpha, lsb;
        integer w, b, bi;
        reg [31:0] rw;
        begin
            m_sclk = cpol;
            #(500);
            m_cs_n = 1'b0;
            #(400);
            for (w = 0; w < nwords; w = w + 1) begin
                rw = 32'd0;
                for (b = 0; b < nb; b = b + 1) begin
                    bi = lsb ? b : (nb - 1 - b);
                    if (!cpha) begin
                        m_mosi = mtx[w][bi];
                        #(250);
                        rw[bi] = spis_miso;
                        m_sclk = ~cpol;               // leading: both sample
                        #(250);
                        m_sclk = cpol;                // trailing: both change
                    end else begin
                        m_sclk = ~cpol;               // leading: both change
                        m_mosi = mtx[w][bi];
                        #(250);
                        rw[bi] = spis_miso;
                        m_sclk = cpol;                // trailing: both sample
                        #(250);
                    end
                end
                mrx[w] = rw;
            end
            #(400);
            m_cs_n = 1'b1;
            #(1000);
        end
    endtask

    // =================================================================
    // I2C helpers
    // =================================================================
    integer exp_ev [0:31];
    integer n_exp;

    task i2c_done;                         // wait for DONE, return INT_STATUS
        output [31:0] st;
        begin
            wait_int(9, 20000);
            st = r;
            apb_wr(INT_STATUS, 32'h0000_0E00);   // clear DONE/NACK/ARB
        end
    endtask

    task expect_bus;                       // compare the monitor log
        input [8*40-1:0] what;
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

    // =================================================================
    // TESTS
    // =================================================================
    task t_id;
        begin
            $display("\n[1] Register bank: ID and scratch register");
            apb_rd(ID, r);           check_eq(r, 32'h5353_4301, "ID register");
            apb_wr(SCRATCH, 32'hDEAD_BEEF);
            apb_rd(SCRATCH, r);      check_eq(r, 32'hDEAD_BEEF, "scratch read-back 1");
            apb_wr(SCRATCH, 32'h1234_5678);
            apb_rd(SCRATCH, r);      check_eq(r, 32'h1234_5678, "scratch read-back 2");
            apb_rd(UART_BAUD, r);    check_eq(r, 32'h0004_0036, "UART_BAUD reset = 54.25 (115200)");
        end
    endtask

    task t_uart_tx_A;
        begin
            $display("\n[2] UART TX: send 'A' (0x41) at 115200 8N1 - the slide 9 example");
            uart_setup(115200, 8, 0, 0, 0, 0);
            n = term.rx_cnt;
            apb_wr(UART_TX, 32'h41);
            wait_term_rx(n + 1, 200);
            check_eq(term.rx_mem[n], 8'h41, "terminal received 0x41");
            $display("        line: start %0d | %0d %0d %0d %0d %0d %0d %0d %0d | stop %0d",
                     term.frame_lvls[0], term.frame_lvls[1], term.frame_lvls[2],
                     term.frame_lvls[3], term.frame_lvls[4], term.frame_lvls[5],
                     term.frame_lvls[6], term.frame_lvls[7], term.frame_lvls[8],
                     term.frame_lvls[9]);
            check({term.frame_lvls[0], term.frame_lvls[1], term.frame_lvls[2],
                   term.frame_lvls[3], term.frame_lvls[4], term.frame_lvls[5],
                   term.frame_lvls[6], term.frame_lvls[7], term.frame_lvls[8],
                   term.frame_lvls[9]} == 10'b0_1000001_0_1,
                  "line levels: start 0, 1 0 0 0 0 0 1 0, stop 1");
        end
    endtask

    task t_uart_rx;
        reg [8*5-1:0] hello;
        begin
            $display("\n[3] UART RX: terminal types \"Hello\", CPU reads the RX FIFO");
            hello = "Hello";
            for (i = 4; i >= 0; i = i - 1) term.send(hello[8*i +: 8], 1'b0, 1'b0);
            #(20000);
            apb_rd(UART_STATUS, r);
            check_eq(r[20:16], 5, "RX_COUNT = 5");
            for (i = 4; i >= 0; i = i - 1) begin
                apb_rd(UART_RX, r);
                check_eq(r, {24'd0, hello[8*i +: 8]}, "received character");
            end
            apb_rd(UART_RX, r);
            check_eq(r[31], 1'b1, "RXDATA EMPTY flag when FIFO is empty");
        end
    endtask

    task uart_format_case;
        input integer bits;
        input par_en, par_odd, stop2;
        reg [7:0] msk, v;
        begin
            uart_setup(460800, bits, par_en, par_odd, stop2, 0);
            apb_wr(INT_STATUS, 32'hFFFF);
            msk = 8'hFF >> (8 - bits);
            n = term.rx_cnt;
            for (j = 0; j < 3; j = j + 1) apb_wr(UART_TX, 32'hA5 + j * 8'h3C);
            wait_term_rx(n + 3, 500);
            for (j = 0; j < 3; j = j + 1) begin
                v = 8'hA5 + j * 8'h3C;
                check_eq(term.rx_mem[n + j], v & msk, "DUT -> terminal byte");
                check(term.rx_perr[n + j] == 1'b0 && term.rx_ferr[n + j] == 1'b0,
                      "terminal sees correct parity and stop bit");
            end
            for (j = 0; j < 3; j = j + 1) term.send(8'h5A + j * 8'h21, 1'b0, 1'b0);
            #(20000);
            for (j = 0; j < 3; j = j + 1) begin
                v = 8'h5A + j * 8'h21;
                apb_rd(UART_RX, r);
                check_eq(r, {24'd0, v & msk}, "terminal -> DUT byte");
            end
            apb_rd(INT_STATUS, r);
            check(r[4:2] == 3'b000, "no parity / framing / break error");
            $display("        %0d data bits, parity %0s, %0d stop bit(s): ok",
                     bits, par_en ? (par_odd ? "odd" : "even") : "none", stop2 ? 2 : 1);
        end
    endtask

    task t_uart_formats;
        begin
            $display("\n[4] UART frame formats (both directions) at 460800 baud");
            uart_format_case(7, 1, 0, 0);   // 7E1
            uart_format_case(5, 0, 0, 1);   // 5N2
            uart_format_case(6, 1, 1, 0);   // 6O1
            uart_format_case(8, 1, 1, 1);   // 8O2
            uart_format_case(8, 1, 0, 0);   // 8E1
        end
    endtask

    task t_uart_errors;
        begin
            $display("\n[5] UART error detection: parity, framing, break");
            uart_setup(230400, 8, 1, 0, 0, 0);             // 8E1
            apb_wr(INT_STATUS, 32'hFFFF);
            term.send(8'h5A, 1'b1, 1'b0);                  // wrong parity bit
            #(10000);
            apb_rd(INT_STATUS, r);
            check(r[2] === 1'b1, "PARITY error flagged");
            apb_rd(UART_RX, r);
            check_eq(r, 32'h5A, "byte with bad parity is still stored");

            uart_setup(230400, 8, 0, 0, 0, 0);             // 8N1
            apb_wr(INT_STATUS, 32'hFFFF);
            term.send(8'h33, 1'b0, 1'b1);                  // stop bit = 0
            #(20000);
            apb_rd(INT_STATUS, r);
            check(r[3] === 1'b1, "FRAMING error flagged");
            check(r[4] === 1'b0, "... and not reported as BREAK");

            apb_wr(INT_STATUS, 32'hFFFF);
            term.send_break(25);                           // line low 25 bit times
            #(20000);
            apb_rd(INT_STATUS, r);
            check(r[4] === 1'b1, "BREAK detected");
            apb_rd(UART_STATUS, r);
            check(r[2] === 1'b1, "bad frames were not stored (RX FIFO empty)");
        end
    endtask

    task t_uart_overflow;
        begin
            $display("\n[6] UART FIFO overflow (RX and TX)");
            uart_setup(1000000, 8, 0, 0, 0, 0);
            apb_wr(INT_STATUS, 32'hFFFF);
            for (i = 0; i < 18; i = i + 1) term.send(i + 8'h30, 1'b0, 1'b0);
            #(5000);
            apb_rd(INT_STATUS, r);
            check(r[5] === 1'b1, "RX overflow flagged after 18 bytes");
            apb_rd(UART_STATUS, r);
            check_eq(r[20:16], 16, "RX FIFO holds 16 bytes");
            for (i = 0; i < 16; i = i + 1) begin
                apb_rd(UART_RX, r);
                check_eq(r, i + 8'h30, "oldest 16 bytes kept in order");
            end
            // TX overflow: transmitter off, write 17 bytes
            apb_wr(UART_CTRL, 32'h0003_000E);          // RX on, TX off, flush
            apb_wr(INT_STATUS, 32'hFFFF);
            for (i = 0; i < 17; i = i + 1) apb_wr(UART_TX, i);
            apb_rd(INT_STATUS, r);
            check(r[13] === 1'b1, "TX overflow flagged on the 17th write");
            apb_rd(UART_STATUS, r);
            check(r[1] === 1'b1 && r[12:8] == 16, "TX FIFO full with 16 bytes");
            apb_wr(UART_CTRL, 32'h0003_000F);          // flush, TX on again
        end
    endtask

    task t_uart_flow;
        begin
            $display("\n[7] UART RTS/CTS flow control");
            uart_setup(1000000, 8, 0, 0, 0, 1);
            term_cts_n = 1'b1;                           // PC says "wait"
            n = term.rx_cnt;
            apb_wr(UART_TX, 32'h77);
            #(40000);
            check(term.rx_cnt == n, "nothing sent while CTS# is high");
            apb_rd(UART_STATUS, r);
            check(r[6] === 1'b0 && r[0] === 1'b0, "CTS_OK=0, byte waiting in TX FIFO");
            term_cts_n = 1'b0;                           // PC ready
            wait_term_rx(n + 1, 100);
            check_eq(term.rx_mem[n], 8'h77, "byte sent once CTS# went low");
            for (i = 0; i < 14; i = i + 1) term.send(i, 1'b0, 1'b0);
            #(2000);
            check(uart_rts_n === 1'b1, "RTS# high (stop!) with 14 bytes in RX FIFO");
            apb_rd(UART_RX, r);
            #(100);
            check(uart_rts_n === 1'b0, "RTS# low again after reading one byte");
            apb_wr(UART_CTRL, 32'h0003_000F);            // flow off, flush
        end
    endtask

    task t_uart_glitch;
        begin
            $display("\n[8] UART glitch filter and start-bit vote");
            uart_setup(115200, 8, 0, 0, 0, 0);
            apb_wr(INT_STATUS, 32'hFFFF);
            term.tx = 1'b0; #(20);   term.tx = 1'b1; #(20000);   // 20 ns spike
            term.tx = 1'b0; #(60);   term.tx = 1'b1; #(20000);   // 60 ns spike
            term.tx = 1'b0; #(2500); term.tx = 1'b1; #(200000);  // 2.5 us pulse
            apb_rd(UART_STATUS, r);
            check(r[2] === 1'b1, "no byte received from the spikes");
            apb_rd(INT_STATUS, r);
            check(r[4:2] == 3'b000, "no framing or break error from the spikes");
        end
    endtask

    task t_baud;
        integer rates [0:7];
        integer ri, baud, idiv;
        real    ideal, meas, err, err_int, worst;
        begin
            $display("\n[9] Baud-rate accuracy at every standard speed (100 MHz clock)");
            $display("        baud     divisor        measured bit   error    integer-only divider");
            rates[0] = 9600;   rates[1] = 19200;  rates[2] = 38400;  rates[3] = 57600;
            rates[4] = 115200; rates[5] = 230400; rates[6] = 460800; rates[7] = 921600;
            worst = 0.0;
            for (ri = 0; ri < 8; ri = ri + 1) begin
                baud = rates[ri];
                uart_setup(baud, 8, 0, 0, 0, 0);
                n = term.rx_cnt;
                fork
                    apb_wr(UART_TX, 32'h55);       // 0x55: an edge every bit
                    begin
                        @(negedge uart_txd); rt0 = $realtime;
                        repeat (5) @(posedge uart_txd);
                        rt1 = $realtime;
                    end
                join
                wait_term_rx(n + 1, 2000);
                ideal   = 1.0e9 / baud;
                meas    = (rt1 - rt0) / 9.0;       // 9 bit times start->stop
                err     = (meas - ideal) / ideal * 100.0;
                idiv    = (100000000 + 8 * baud) / (16 * baud);
                err_int = (idiv * 160.0 - ideal) / ideal * 100.0;
                if (err < 0) err = -err;
                if (err_int < 0) err_int = -err_int;
                if (err > worst) worst = err;
                $display("        %6d   %3d + %2d/16   %9.2f ns   %5.3f %%   %2d -> %5.2f %%",
                         baud, div16 >> 4, div16 & 15, meas, err, idiv, err_int);
                check(err < 0.5, "baud error below 0.5 %");
                check_eq(term.rx_mem[n], 8'h55, "byte received correctly");
            end
            $display("        worst fractional-divider error: %5.3f %%", worst);
        end
    endtask

    task t_uart_random;
        reg [7:0] v, msk;
        integer   cfg;
        begin
            $display("\n[10] UART long random run: 240 bytes, internal loopback, random formats");
            for (cfg = 0; cfg < 12; cfg = cfg + 1) begin
                k = $random;
                uart_setup(1000000, 5 + (k & 3), (k >> 2) & 1, (k >> 3) & 1, (k >> 4) & 1, 0);
                apb_wr(UART_CTRL, 32'h0003_0103 | ((k & 3) << 2) | (((k >> 2) & 1) << 4) |
                                  (((k >> 3) & 1) << 5) | (((k >> 4) & 1) << 6));  // + LOOPBACK
                msk = 8'hFF >> (3 - (k & 3));
                for (j = 0; j < 20; j = j + 1) begin
                    v = $random;
                    apb_wr(UART_TX, v);
                    apb_rd(UART_RX, r);
                    while (r[31]) apb_rd(UART_RX, r);
                    check_eq(r, v & msk, "loopback byte");
                end
            end
            apb_wr(UART_CTRL, 32'h0003_000F);
            apb_rd(INT_STATUS, r);
            check(r[5:2] == 4'b0000, "no errors during the random run");
            apb_wr(INT_STATUS, 32'hFFFF);
        end
    endtask

    // ---------------- SPI ----------------
    localparam [31:0] SPI_M0 = 32'h0000_0871;   // EN, mode 0, 8 bit, manual CS, CS0
    localparam [31:0] SPI_M3 = 32'h0000_0877;   // same, mode 3

    task t_spi_id;
        begin
            $display("\n[11] SPI: read the PmodSF3 JEDEC ID (command 0x9F)");
            apb_wr(SPI_CLKDIV, 4);                       // 10 MHz
            sbuf[0] = 8'h9F; sbuf[1] = 0; sbuf[2] = 0; sbuf[3] = 0;
            spi_frame(4, SPI_M0);
            $display("        ID = %02X %02X %02X", rbuf[1], rbuf[2], rbuf[3]);
            check({rbuf[1], rbuf[2], rbuf[3]} == 24'h20BA19, "JEDEC ID = 20 BA 19");
            #(50);                                       // CS# output is registered
            check(spi_cs_n === 4'hF, "chip selects released afterwards");
        end
    endtask

    task t_spi_flash_rw;
        begin
            $display("\n[12] SPI: erase, page program and read back the flash");
            sbuf[0] = 8'h06; spi_frame(1, SPI_M0);                      // WREN
            sbuf[0] = 8'h05; sbuf[1] = 0; spi_frame(2, SPI_M0);
            check(rbuf[1][1] === 1'b1, "WEL bit set after WRITE ENABLE");
            sbuf[0] = 8'h20; sbuf[1] = 0; sbuf[2] = 0; sbuf[3] = 0;     // erase 4 KB
            spi_frame(4, SPI_M0);
            flash_wait_ready(SPI_M0);
            sbuf[0] = 8'h06; spi_frame(1, SPI_M0);
            sbuf[0] = 8'h02; sbuf[1] = 8'h00; sbuf[2] = 8'h01; sbuf[3] = 8'h00;  // PP @0x100
            for (i = 0; i < 20; i = i + 1) sbuf[4 + i] = 8'h11 * (i + 1);
            spi_frame(24, SPI_M0);                                      // > 16: two chunks
            flash_wait_ready(SPI_M0);
            sbuf[0] = 8'h03; sbuf[1] = 8'h00; sbuf[2] = 8'h01; sbuf[3] = 8'h00;  // READ
            for (i = 0; i < 20; i = i + 1) sbuf[4 + i] = 8'h00;
            spi_frame(24, SPI_M0);
            n = 0;
            for (i = 0; i < 20; i = i + 1) if (rbuf[4 + i] !== ((8'h11 * (i + 1)) & 8'hFF)) n = n + 1;
            check(n == 0, "20 bytes read back in mode 0");
            spi_frame(24, SPI_M3);
            n = 0;
            for (i = 0; i < 20; i = i + 1) if (rbuf[4 + i] !== ((8'h11 * (i + 1)) & 8'hFF)) n = n + 1;
            check(n == 0, "same 20 bytes read back in mode 3");
        end
    endtask

    task t_spi_modes;
        integer mode, li, lsbf, len;
        integer lens [0:5];
        reg [31:0] msk, txw, rsp;
        integer passed;
        begin
            $display("\n[13] SPI master: 4 modes x word sizes 4..32 x MSB/LSB first");
            lens[0] = 4; lens[1] = 8; lens[2] = 13; lens[3] = 16; lens[4] = 24; lens[5] = 32;
            apb_wr(SPI_CLKDIV, 9);                       // 5 MHz
            passed = 0;
            for (mode = 0; mode < 4; mode = mode + 1)
                for (li = 0; li < 6; li = li + 1)
                    for (lsbf = 0; lsbf < 2; lsbf = lsbf + 1) begin
                        len = lens[li];
                        msk = (len == 32) ? 32'hFFFF_FFFF : ((32'd1 << len) - 1);
                        txw = $random & msk;
                        rsp = $random & msk;
                        gslv.cpol = mode >> 1; gslv.cpha = mode & 1; gslv.lsb = lsbf;
                        gslv.nbits = len; gslv.resp = rsp; gslv.resp_inc = 0;
                        n = gslv.ngot;
                        // EN, CPOL, CPHA, LSB, LEN-1, CS_SEL=1, automatic CS, flush
                        apb_wr(SPI_CTRL, 32'h0003_0201 | ((mode >> 1) << 1) | ((mode & 1) << 2) |
                                         (lsbf << 3) | ((len - 1) << 4));
                        apb_wr(SPI_TX, txw);
                        apb_rd(SPI_STATUS, r);
                        while (r[2] || r[4]) apb_rd(SPI_STATUS, r);
                        apb_rd(SPI_RX, r);
                        if (r === rsp && gslv.ngot == n + 1 && gslv.got[n] === txw)
                            passed = passed + 1;
                        else
                            $display("    mode %0d len %0d lsb %0d: sent %h got-by-slave %h | expected %h got %h",
                                     mode, len, lsbf, txw, gslv.got[n], rsp, r);
                    end
            $display("        %0d of 48 combinations correct", passed);
            check(passed == 48, "all SPI mode / length / bit-order combinations");
        end
    endtask

    task t_spi_abort;
        begin
            $display("\n[14] SPI clean abort in the middle of a transfer");
            apb_wr(SPI_CLKDIV, 99);                      // 500 kHz, slow
            gslv.cpol = 1; gslv.cpha = 0; gslv.lsb = 0; gslv.nbits = 32;
            n = gslv.ngot;
            apb_wr(SPI_CTRL, 32'h0003_01F3 | (1 << 9));  // EN, CPOL=1, 32 bit, CS1 auto
            for (i = 0; i < 8; i = i + 1) apb_wr(SPI_TX, 32'hCAFE_0000 + i);
            #(150000);                                   // part-way through word 3
            apb_wr(SPI_CTRL, 32'h0004_01F3 | (1 << 9));  // ABORT
            #(100);
            apb_rd(SPI_STATUS, r);
            check(r[4] === 1'b0, "BUSY cleared");
            check(r[0] === 1'b1, "TX FIFO flushed");
            check(spi_cs_n === 4'hF, "chip select released");
            check(spi_sclk === 1'b1, "SCLK back at its idle level (CPOL=1)");
            $display("        slave received %0d complete words before the abort", gslv.ngot - n);
            check(gslv.ngot - n < 8, "transfer really was cut short");
            apb_wr(SPI_CTRL, 32'h0003_0000);             // disable, flush
        end
    endtask

    task t_spi_slave;
        integer mode, nb, ok;
        reg [31:0] msk;
        begin
            $display("\n[15] SPI slave mode: an outside master talks to the controller");
            ok = 0;
            for (mode = 0; mode < 4; mode = mode + 1)
                for (nb = 8; nb <= 16; nb = nb + 8) begin
                    msk = (32'd1 << nb) - 1;
                    // EN, mode, LEN-1, SLAVE, flush
                    apb_wr(SPI_CTRL, 32'h0003_2001 | ((mode >> 1) << 1) | ((mode & 1) << 2) |
                                     ((nb - 1) << 4));
                    apb_wr(INT_STATUS, 32'h0040);
                    apb_wr(SPI_TX, 32'h0000_C3A5 & msk);       // replies
                    apb_wr(SPI_TX, 32'h0000_5A3C & msk);
                    mtx[0] = $random & msk; mtx[1] = $random & msk;
                    tbm_xfer(2, nb, mode >> 1, mode & 1, 1'b0);
                    apb_rd(SPI_RX, r);
                    apb_rd(SPI_RX, r2);
                    if (r === mtx[0] && r2 === mtx[1] &&
                        mrx[0] === (32'h0000_C3A5 & msk) && mrx[1] === (32'h0000_5A3C & msk))
                        ok = ok + 1;
                    else
                        $display("    mode %0d %0d bit: DUT got %h %h (exp %h %h), master got %h %h",
                                 mode, nb, r, r2, mtx[0], mtx[1], mrx[0], mrx[1]);
                    apb_rd(INT_STATUS, r);
                    if (r[6] !== 1'b1) begin
                        ok = ok - 1;
                        $display("    mode %0d %0d bit: SPI_DONE not raised at CS# high", mode, nb);
                    end
                end
            $display("        %0d of 8 slave-mode cases correct (2 words per frame)", ok);
            check(ok == 8, "SPI slave mode, all 4 modes, 8 and 16 bit");
            apb_wr(SPI_CTRL, 32'h0003_0000);
        end
    endtask

    task t_spi_random;
        integer cnt, mode, len;
        reg [31:0] msk, v;
        begin
            $display("\n[16] SPI long random run: 200 words, internal loopback");
            apb_wr(SPI_CLKDIV, 3);                       // 12.5 MHz
            cnt = 0;
            for (i = 0; i < 200; i = i + 1) begin
                k = $random;
                mode = k & 3;
                len  = 4 + ((k >> 2) & 31) % 29;         // 4..32
                msk  = (len == 32) ? 32'hFFFF_FFFF : ((32'd1 << len) - 1);
                v    = $random & msk;
                // EN, mode, LSB random, LEN-1, CS3 (nothing attached), LOOPBACK
                apb_wr(SPI_CTRL, 32'h0000_4601 | (mode << 1) | (((k >> 8) & 1) << 3) |
                                 ((len - 1) << 4));
                apb_wr(SPI_TX, v);
                apb_rd(SPI_STATUS, r);
                while (r[2]) apb_rd(SPI_STATUS, r);
                apb_rd(SPI_RX, r);
                if (r === v) cnt = cnt + 1;
            end
            $display("        %0d of 200 words came back correctly", cnt);
            check(cnt == 200, "SPI random loopback run");
            apb_wr(SPI_CTRL, 32'h0003_0000);
        end
    endtask

    // ---------------- I2C ----------------
    task tmp2_read;                         // the worked example, result in r2[15:0]
        begin
            apb_wr(I2C_ADDR, 32'h4B);
            apb_wr(I2C_TX, 32'h00);                       // pointer = temperature
            apb_wr(I2C_CMD, 32'h0000_0001);               // write 1, keep the bus
            i2c_done(r);
            apb_wr(I2C_CMD, 32'h0000_0302);               // read 2, then STOP
            i2c_done(r2);
            apb_rd(I2C_RX, r);  r2[15:8] = r[7:0];
            apb_rd(I2C_RX, r);  r2[7:0]  = r[7:0];
        end
    endtask

    task t_i2c_temp;
        real temp;
        begin
            $display("\n[17] I2C worked example: read the PmodTMP2 temperature (100 kHz)");
            apb_wr(I2C_PRESC, 249);
            apb_wr(I2C_CTRL, 32'h0003_0001);
            apb_wr(INT_STATUS, 32'hFFFF);
            mon.clear;
            apb_wr(I2C_ADDR, 32'h4B);
            apb_wr(I2C_TX, 32'h00);
            apb_wr(I2C_CMD, 32'h0000_0001);
            i2c_done(r);
            check(r[10] === 1'b0, "pointer write ACKed");
            apb_rd(I2C_STATUS, r);
            check(r[5] === 1'b1, "bus kept (HOLDING) for the repeated START");
            apb_wr(I2C_CMD, 32'h0000_0302);
            i2c_done(r);
            apb_rd(I2C_RX, r);  r2[15:8] = r[7:0];
            apb_rd(I2C_RX, r);  r2[7:0]  = r[7:0];
            temp = (r2[15:3]) * 0.0625;
            $display("        bytes 0x%02X 0x%02X -> 0x%04X >> 3 = %0d -> %0.2f C",
                     r2[15:8], r2[7:0], r2[15:0], r2[15:3], temp);
            check_eq(r2[15:0], 16'h0C80, "temperature bytes 0x0C 0x80");
            check(temp == 25.0, "temperature = 25.0 C");
            exp_ev[0] = 1000; exp_ev[1] = 8'h96; exp_ev[2] = 8'h00; exp_ev[3] = 1001;
            exp_ev[4] = 8'h97; exp_ev[5] = 8'h0C; exp_ev[6] = 9'h180; exp_ev[7] = 1002;
            n_exp = 8;
            expect_bus("bus: S 96 A 00 A Sr 97 A 0C A 80 N P");
        end
    endtask

    task t_i2c_400k;
        real f;
        begin
            $display("\n[18] I2C at 400 kHz (prescale 62)");
            apb_wr(I2C_PRESC, 62);
            fork
                tmp2_read;
                begin
                    @(posedge scl); @(posedge scl); rt0 = $realtime;
                    @(posedge scl); rt1 = $realtime;
                end
            join
            f = 1.0e6 / (rt1 - rt0);
            $display("        measured SCL = %0.1f kHz, data = 0x%04X", f, r2[15:0]);
            check(f > 370.0 && f <= 400.0, "SCL between 370 and 400 kHz");
            check_eq(r2[15:0], 16'h0C80, "temperature bytes at 400 kHz");
        end
    endtask

    task t_i2c_nack;
        begin
            $display("\n[19] I2C NACK: nobody at address 0x50");
            apb_wr(I2C_CTRL, 32'h0003_0001);
            apb_wr(INT_STATUS, 32'hFFFF);
            mon.clear;
            apb_wr(I2C_ADDR, 32'h50);
            apb_wr(I2C_TX, 32'h00);
            apb_wr(I2C_CMD, 32'h0000_0201);               // write 1 + STOP
            wait_int(9, 2000);
            check(r[10] === 1'b1, "NACK interrupt flag");
            apb_rd(I2C_STATUS, r);
            check(r[6] === 1'b1, "status NACK bit");
            check(r[5] === 1'b0 && r[24] === 1'b0, "STOP sent, bus free");
            n_exp = 3;
            exp_ev[0] = 1000; exp_ev[1] = 9'h1A0; exp_ev[2] = 1002;
            expect_bus("bus: S A0 N P");
            apb_wr(I2C_CTRL, 32'h0003_0001);              // flush the unsent byte
            apb_wr(INT_STATUS, 32'hFFFF);
        end
    endtask

    task t_i2c_10bit;
        begin
            $display("\n[20] I2C 10-bit address 0x2A5: write two registers, read them back");
            mon.clear;
            apb_wr(I2C_ADDR, 32'h0000_82A5);              // TEN_BIT | 0x2A5
            apb_wr(I2C_TX, 32'h05);                       // pointer
            apb_wr(I2C_TX, 32'h77);
            apb_wr(I2C_TX, 32'h88);
            apb_wr(I2C_CMD, 32'h0000_0203);               // write 3 + STOP
            i2c_done(r);
            check(r[10] === 1'b0, "10-bit write ACKed");
            check(dev10.regs[5] === 8'h77 && dev10.regs[6] === 8'h88, "device registers written");
            n_exp = 7;
            exp_ev[0] = 1000; exp_ev[1] = 8'hF4; exp_ev[2] = 8'hA5; exp_ev[3] = 8'h05;
            exp_ev[4] = 8'h77; exp_ev[5] = 8'h88; exp_ev[6] = 1002;
            expect_bus("bus: S F4 A A5 A 05 A 77 A 88 A P");
            mon.clear;
            apb_wr(I2C_TX, 32'h05);
            apb_wr(I2C_CMD, 32'h0000_0001);               // write pointer, keep bus
            i2c_done(r);
            apb_wr(I2C_CMD, 32'h0000_0302);               // read 2 + STOP
            i2c_done(r);
            apb_rd(I2C_RX, r);  check_eq(r, 32'h77, "10-bit read byte 1");
            apb_rd(I2C_RX, r);  check_eq(r, 32'h88, "10-bit read byte 2");
            n_exp = 12;
            exp_ev[0] = 1000; exp_ev[1] = 8'hF4; exp_ev[2] = 8'hA5; exp_ev[3] = 8'h05;
            exp_ev[4] = 1001; exp_ev[5] = 8'hF4; exp_ev[6] = 8'hA5; exp_ev[7] = 1001;
            exp_ev[8] = 8'hF5; exp_ev[9] = 8'h77; exp_ev[10] = 9'h188; exp_ev[11] = 1002;
            expect_bus("bus: 10-bit read with two repeated STARTs");
        end
    endtask

    task t_i2c_stretch;
        integer d0, d1;
        begin
            $display("\n[21] I2C clock stretching: the sensor holds SCL low 30 us after each ACK");
            apb_wr(I2C_PRESC, 249);
            t0 = $time; tmp2_read; d0 = $time - t0;
            tmp2.stretch_ns = 30000;
            n = tmp2.stretch_count;
            t0 = $time; tmp2_read; d1 = $time - t0;
            tmp2.stretch_ns = 0;
            $display("        read took %0d us normally, %0d us with stretching (%0d stretches)",
                     d0 / 1000, d1 / 1000, tmp2.stretch_count - n);
            check(tmp2.stretch_count - n == 3, "sensor stretched after each of its 3 ACKs");
            // each 30 us stretch overlaps the 5 us the master holds SCL low anyway
            check(d1 > d0 + 70000, "controller waited for the stretched clock");
            check_eq(r2[15:0], 16'h0C80, "data still correct");
        end
    endtask

    task t_i2c_stall;
        integer good;
        begin
            $display("\n[22] I2C 20-byte read with a 16-entry RX FIFO (controller pauses SCL)");
            apb_wr(I2C_PRESC, 62);
            apb_wr(I2C_ADDR, 32'h4B);
            apb_wr(I2C_TX, 32'h00);
            apb_wr(I2C_CMD, 32'h0000_0001);
            i2c_done(r);
            apb_wr(I2C_CMD, 32'h0000_0314);               // read 20 + STOP
            apb_rd(I2C_STATUS, r);
            while (!r[3]) apb_rd(I2C_STATUS, r);          // wait for RX FIFO full
            #(100000);
            apb_rd(I2C_STATUS, r);
            check(r[4] === 1'b1 && r[25] === 1'b0, "busy, holding SCL low while FIFO is full");
            good = 0;
            for (i = 0; i < 20; i = i + 1) begin
                apb_rd(I2C_RX, r);
                while (r[31]) apb_rd(I2C_RX, r);
                if (r[7:0] === tmp2.regs[i & 15]) good = good + 1;
            end
            i2c_done(r);
            $display("        %0d of 20 bytes correct", good);
            check(good == 20, "all 20 bytes received in order");
        end
    endtask

    task t_i2c_arb;
        begin
            $display("\n[23] I2C arbitration: a second master starts at the same moment");
            apb_wr(I2C_PRESC, 249);
            apb_wr(INT_STATUS, 32'hFFFF);
            mon.clear;
            n = m2.done_count;
            apb_wr(I2C_ADDR, 32'h4B);
            apb_wr(I2C_TX, 32'h00);
            fork
                m2.join_start(8'h20);                     // other master: address 0x10
                apb_wr(I2C_CMD, 32'h0000_0201);           // we: address 0x4B
            join
            wait_int(9, 2000);
            check(r[11] === 1'b1, "ARB_LOST interrupt flag");
            apb_rd(I2C_STATUS, r);
            check(r[7] === 1'b1, "status ARB_LOST bit");
            check(m2.won === 1'b1 && m2.done_count == n + 1, "other master finished undisturbed");
            mon.show;
            apb_wr(I2C_CTRL, 32'h0003_0001);
            apb_wr(INT_STATUS, 32'hFFFF);
            tmp2_read;
            check_eq(r2[15:0], 16'h0C80, "next transaction works normally");
        end
    endtask

    task t_i2c_busy;
        begin
            $display("\n[24] I2C bus busy: we must wait for the other master's STOP");
            mon.clear;
            n = m2.done_count;
            apb_wr(I2C_ADDR, 32'h4B);
            apb_wr(I2C_TX, 32'h00);
            fork
                m2.transaction(8'h20);
                begin
                    #(15000);
                    apb_wr(I2C_CMD, 32'h0000_0201);       // issued mid-transaction
                end
            join
            i2c_done(r);
            check(r[10] === 1'b0 && r[11] === 1'b0, "our write completed (no NACK, no ARB)");
            check(m2.won === 1'b1, "other master was not disturbed");
            n_exp = 7;
            exp_ev[0] = 1000; exp_ev[1] = 9'h120; exp_ev[2] = 1002;
            exp_ev[3] = 1000; exp_ev[4] = 8'h96;  exp_ev[5] = 8'h00; exp_ev[6] = 1002;
            expect_bus("bus: their S 20 N P, then our S 96 A 00 A P");
        end
    endtask

    // ---------------- bridge ----------------
    task t_bridge_echo;
        begin
            $display("\n[25] Bridge pass-through UART -> UART (hardware echo, no CPU)");
            uart_setup(1000000, 8, 0, 0, 0, 0);
            apb_wr(BRIDGE_CNT, 0);
            apb_wr(BRIDGE_CTRL, 32'h0F);                  // ch0: UART(3) -> UART(3)
            n = term.rx_cnt;
            term.send("H", 1'b0, 1'b0);
            term.send("i", 1'b0, 1'b0);
            term.send("!", 1'b0, 1'b0);
            wait_term_rx(n + 3, 200);
            check({term.rx_mem[n], term.rx_mem[n + 1], term.rx_mem[n + 2]} == "Hi!",
                  "terminal got its own bytes back");
            apb_rd(BRIDGE_CNT, r);
            check_eq(r[15:0], 3, "bridge channel 0 moved 3 bytes");
            apb_wr(BRIDGE_CTRL, 32'h00);
        end
    endtask

    task t_bridge_uart_spi;
        begin
            $display("\n[26] Two-way bridge UART <-> SPI (ch0 UART->SPI, ch1 SPI->UART)");
            apb_wr(SPI_CLKDIV, 9);
            apb_wr(SPI_CTRL, 32'h0003_0271);              // EN, mode 0, 8 bit, CS1 auto
            gslv.cpol = 0; gslv.cpha = 0; gslv.lsb = 0; gslv.nbits = 8;
            gslv.resp = 32'hA5; gslv.resp_inc = 1;
            apb_wr(BRIDGE_CNT, 0);
            apb_wr(BRIDGE_CTRL, 32'hD7);                  // ch0 3->1, ch1 1->3
            n = term.rx_cnt;
            k = gslv.ngot;
            term.send(8'h11, 1'b0, 1'b0);
            term.send(8'h22, 1'b0, 1'b0);
            term.send(8'h33, 1'b0, 1'b0);
            wait_term_rx(n + 3, 200);
            check(gslv.ngot == k + 3 && gslv.got[k] == 8'h11 && gslv.got[k + 1] == 8'h22 &&
                  gslv.got[k + 2] == 8'h33, "SPI device received 11 22 33 from the UART");
            check(term.rx_mem[n] == 8'hA5 && term.rx_mem[n + 1] == 8'hA6 &&
                  term.rx_mem[n + 2] == 8'hA7, "terminal received the SPI replies A5 A6 A7");
            apb_rd(BRIDGE_CNT, r);
            check(r == {16'd3, 16'd3}, "3 bytes each way");
            apb_wr(BRIDGE_CTRL, 32'h00);
            gslv.resp_inc = 0;
            apb_wr(SPI_CTRL, 32'h0003_0000);
        end
    endtask

    task t_bridge_uart_i2c;
        begin
            $display("\n[27] Bridge UART -> I2C (AUTO_WR) and I2C -> UART");
            apb_wr(I2C_PRESC, 62);
            apb_wr(I2C_ADDR, 32'h4B);
            apb_wr(I2C_CTRL, 32'h0003_0003);              // EN + AUTO_WR
            apb_wr(BRIDGE_CNT, 0);
            apb_wr(BRIDGE_CTRL, 32'h0B);                  // ch0: UART(3) -> I2C(2)
            n = tmp2.nwlog;
            term.send(8'h03, 1'b0, 1'b0);
            term.send(8'h01, 1'b0, 1'b0);
            #(200000);
            check(tmp2.nwlog == n + 2 && tmp2.wlog[n] == 8'h03 && tmp2.wlog[n + 1] == 8'h01,
                  "bytes typed on the UART arrived on the I2C bus");
            apb_wr(BRIDGE_CTRL, 32'h00);
            apb_wr(I2C_CTRL, 32'h0003_0001);              // AUTO_WR off
            apb_wr(INT_STATUS, 32'hFFFF);
            apb_wr(BRIDGE_CTRL, 32'hE0);                  // ch1: I2C(2) -> UART(3)
            n = term.rx_cnt;
            apb_wr(I2C_TX, 32'h00);
            apb_wr(I2C_CMD, 32'h0000_0001);
            i2c_done(r);
            apb_wr(I2C_CMD, 32'h0000_0302);               // read 2 + STOP
            i2c_done(r);
            wait_term_rx(n + 2, 200);
            check(term.rx_mem[n] == 8'h0C && term.rx_mem[n + 1] == 8'h80,
                  "sensor bytes 0C 80 forwarded to the UART by the bridge");
            apb_rd(BRIDGE_CNT, r);
            check_eq(r[31:16], 2, "bridge channel 1 moved 2 bytes");
            apb_wr(BRIDGE_CTRL, 32'h00);
        end
    endtask

    // ---------------- interrupts ----------------
    task t_irq;
        begin
            $display("\n[28] Interrupt controller: enable mask, sticky flags, W1C, IRQ line");
            apb_wr(INT_STATUS, 32'hFFFF);
            apb_wr(INT_ENABLE, 32'h0200);                 // I2C_DONE only
            #(50);
            check(irq === 1'b0, "IRQ low when nothing pending");
            apb_wr(I2C_ADDR, 32'h50);
            apb_wr(I2C_TX, 32'h00);
            apb_wr(I2C_CMD, 32'h0000_0201);               // will NACK
            t0 = $time;
            while (irq !== 1'b1 && $time < t0 + 2000000) #(100);
            check(irq === 1'b1, "IRQ rises when I2C_DONE is set");
            apb_rd(INT_STATUS, r);
            check(r[9] === 1'b1 && r[10] === 1'b1, "DONE and NACK flags pending");
            apb_wr(INT_STATUS, 32'h0200);                 // clear DONE only
            #(50);
            check(irq === 1'b0, "IRQ drops after write-1-to-clear");
            apb_rd(INT_STATUS, r);
            check(r[10] === 1'b1, "other flags untouched by the clear");
            apb_wr(I2C_CTRL, 32'h0003_0001);
            // level source: UART RX not empty
            apb_wr(UART_CTRL, 32'h0003_000F);             // flush
            apb_wr(INT_STATUS, 32'hFFFF);
            apb_wr(INT_ENABLE, 32'h0001);
            term.send(8'h42, 1'b0, 1'b0);
            #(5000);
            check(irq === 1'b1, "IRQ for UART RX data");
            apb_wr(INT_STATUS, 32'h0001);
            #(50);
            check(irq === 1'b1, "level flag comes back while data is still waiting");
            apb_rd(UART_RX, r);
            apb_wr(INT_STATUS, 32'h0001);
            #(50);
            check(irq === 1'b0, "IRQ low once the byte has been read");
            apb_wr(INT_ENABLE, 32'h0000);
        end
    endtask

    // =================================================================
    initial begin
        $display("==============================================================");
        $display(" Smart Serial Controller - full system testbench");
        $display("==============================================================");
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        t_id;
        t_uart_tx_A;
        t_uart_rx;
        t_uart_formats;
        t_uart_errors;
        t_uart_overflow;
        t_uart_flow;
        t_uart_glitch;
        t_baud;
        t_uart_random;
        t_spi_id;
        t_spi_flash_rw;
        t_spi_modes;
        t_spi_abort;
        t_spi_slave;
        t_spi_random;
        t_i2c_temp;
        t_i2c_400k;
        t_i2c_nack;
        t_i2c_10bit;
        t_i2c_stretch;
        t_i2c_stall;
        t_i2c_arb;
        t_i2c_busy;
        t_bridge_echo;
        t_bridge_uart_spi;
        t_bridge_uart_i2c;
        t_irq;

        $display("\n==============================================================");
        if (errors == 0) $display(" ALL %0d CHECKS PASSED", checks);
        else             $display(" %0d OF %0d CHECKS FAILED", errors, checks);
        $display("==============================================================");
        $finish;
    end

    initial begin
        #(400_000_000);
        errors = errors + 1;                     // a time-out is a failure
        $display("GLOBAL TIME-OUT - %0d OF %0d CHECKS FAILED (counting the time-out)", errors, checks + 1);
        $finish;
    end

    // optional waveform dump: run with +vcd
    initial if ($test$plusargs("vcd")) begin
        $dumpfile("tb_ssc_top.vcd");
        $dumpvars(0, tb_ssc_top);
    end
endmodule

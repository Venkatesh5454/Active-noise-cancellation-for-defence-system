// =============================================================================
// tb_ssc2_system.v  -  the whole v2 controller as the ARM sees it (Way 2)
// -----------------------------------------------------------------------------
// ssc2_axi_top is driven over AXI4-Lite exactly like sw/ssc2_driver.c does,
// its DMA master writes into an AXI memory model (standing in for DDR), and
// its pins go through a copy of the board pads to behavioural devices:
//
//     JA : PC terminal (tb_uart_term)          JB : SPI flash (tb_spi_flash)
//     JC : PmodTMP2 at 0x4B (tb_i2c_slave)     JD : logic-analyser decoder
//
// The tests follow the slides:
//   [1] registers: ID, VERSION, v1 scratch, crossbar reset values
//   [2] slide 17 flow A: the PC sends 01 9F 03 and gets 20 BA 19 through the NoC
//   [3] slides 18-20: the hub reads the TMP2, records land in "DDR", one
//       interrupt after 32 records, and the CPU makes 0 bus accesses meanwhile
//   [4] flow A again while flow B runs (both cross router (1,1))
//   [5] slide 14: JD switches from the SPI engine to SM0 running the slide-13
//       UART program; the crossbar waits for the SPI frame to finish
//   [6] protocol conversion PC UART -> SM0 UART on JD (RAW mode to node 0)
//   [7] v1 path still works: v1-style TMP2 read with the CPU (counts APB
//       accesses and interrupts per reading for slide 20 / measurement 4)
//
// One simulated "millisecond" is 10 us (US_PER_MS = 10) so the 32 readings
// take milliseconds instead of seconds.  Everything else is real time.
// =============================================================================
`timescale 1ns/1ps
// folder with the assembled serial-engine programs (vivado/run_sim.tcl passes
// an absolute path; Icarus runs from the project folder)
`ifndef PROG_DIR
`define PROG_DIR "programs/"
`endif
module tb_ssc2_system;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg aresetn = 1'b0;

    integer errors = 0, checks = 0;

    // ---------------- AXI4-Lite master signals ----------------
    reg  [11:0] awaddr = 0;  reg awvalid = 0;  wire awready;
    reg  [31:0] wdata  = 0;  reg wvalid  = 0;  wire wready;
    wire [1:0]  bresp;       wire bvalid;      reg bready = 0;
    reg  [11:0] araddr = 0;  reg arvalid = 0;  wire arready;
    wire [31:0] rdata;       wire [1:0] rresp; wire rvalid;  reg rready = 0;
    wire        irq;

    // ---------------- AXI4 master (DMA) signals ----------------
    wire [31:0] m_awaddr, m_wdata;
    wire [7:0]  m_awlen;
    wire [2:0]  m_awsize, m_awprot;
    wire [1:0]  m_awburst, m_bresp;
    wire [3:0]  m_awcache, m_wstrb;
    wire        m_awvalid, m_awready, m_wlast, m_wvalid, m_wready, m_bvalid, m_bready;

    // ---------------- pins ----------------
    wire [23:0] pad_out, pad_oe, pad_in;
    wire [2:0]  spi_cs_hi_n;
    wire [3:0]  mon, status_led;
    wire [1:0]  oled_pwr;
    wire [3:0]  ja, jb, jc, jd, oled;
    reg  [7:0]  sw  = 8'h00;
    reg  [4:0]  btn = 5'h00;

    ssc2_axi_top #(.US_PER_MS(10)) dut (
        .S_AXI_ACLK(clk), .S_AXI_ARESETN(aresetn),
        .S_AXI_AWADDR(awaddr), .S_AXI_AWPROT(3'd0), .S_AXI_AWVALID(awvalid), .S_AXI_AWREADY(awready),
        .S_AXI_WDATA(wdata), .S_AXI_WSTRB(4'hF), .S_AXI_WVALID(wvalid), .S_AXI_WREADY(wready),
        .S_AXI_BRESP(bresp), .S_AXI_BVALID(bvalid), .S_AXI_BREADY(bready),
        .S_AXI_ARADDR(araddr), .S_AXI_ARPROT(3'd0), .S_AXI_ARVALID(arvalid), .S_AXI_ARREADY(arready),
        .S_AXI_RDATA(rdata), .S_AXI_RRESP(rresp), .S_AXI_RVALID(rvalid), .S_AXI_RREADY(rready),
        .M_AXI_AWADDR(m_awaddr), .M_AXI_AWLEN(m_awlen), .M_AXI_AWSIZE(m_awsize),
        .M_AXI_AWBURST(m_awburst), .M_AXI_AWLOCK(), .M_AXI_AWCACHE(m_awcache),
        .M_AXI_AWPROT(m_awprot), .M_AXI_AWQOS(), .M_AXI_AWVALID(m_awvalid),
        .M_AXI_AWREADY(m_awready),
        .M_AXI_WDATA(m_wdata), .M_AXI_WSTRB(m_wstrb), .M_AXI_WLAST(m_wlast),
        .M_AXI_WVALID(m_wvalid), .M_AXI_WREADY(m_wready),
        .M_AXI_BRESP(m_bresp), .M_AXI_BVALID(m_bvalid), .M_AXI_BREADY(m_bready),
        .M_AXI_ARADDR(), .M_AXI_ARLEN(), .M_AXI_ARSIZE(), .M_AXI_ARBURST(), .M_AXI_ARLOCK(),
        .M_AXI_ARCACHE(), .M_AXI_ARPROT(), .M_AXI_ARQOS(), .M_AXI_ARVALID(),
        .M_AXI_ARREADY(1'b0), .M_AXI_RDATA(32'd0), .M_AXI_RRESP(2'd0), .M_AXI_RLAST(1'b0),
        .M_AXI_RVALID(1'b0), .M_AXI_RREADY(),
        .irq(irq),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .spi_cs_hi_n(spi_cs_hi_n), .mon(mon),
        .board_sw(sw), .board_btn(btn), .oled_pwr(oled_pwr), .status_led(status_led)
    );

    // the board pads, as in zed2_top_ps.v
    wire [3:0] jb_lo, jd_lo;
    wire       oled_vdd, oled_vbat;
    wire [7:0] led, board_sw_unused;
    wire [4:0] board_btn_unused;
    zed2_pads pads (
        .clk(clk),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .spi_cs_hi_n(spi_cs_hi_n), .mon(mon), .oled_pwr(oled_pwr), .status_led(status_led),
        .board_sw(board_sw_unused), .board_btn(board_btn_unused),
        .ja(ja), .jb(jb), .jb_lo(jb_lo), .jc(jc), .jd(jd), .jd_lo(jd_lo),
        .oled(oled), .oled_vdd(oled_vdd), .oled_vbat(oled_vbat),
        .led(led), .sw(sw), .btn(btn)
    );

    // pull-ups on every Pmod pin (zed2_pins.xdc), like the real board
    pullup (ja[0]); pullup (ja[1]); pullup (ja[2]); pullup (ja[3]);
    pullup (jb[0]); pullup (jb[1]); pullup (jb[2]); pullup (jb[3]);
    pullup (jc[0]); pullup (jc[1]); pullup (jc[2]); pullup (jc[3]);
    pullup (jd[0]); pullup (jd[1]); pullup (jd[2]); pullup (jd[3]);

    // ---------------- devices on the Pmods ----------------
    wire pc_tx;
    tb_uart_term pc (.rx(ja[1]), .tx(pc_tx));            // JA2 = our TXD, JA3 = our RXD
    assign ja[2] = pc_tx;
    tb_spi_flash flash (.cs_n(jb[0]), .sclk(jb[3]), .mosi(jb[1]), .miso(jb[2]));
    tb_i2c_slave #(.ADDR(10'h04B), .TEN(0)) tmp2 (.scl(jc[2]), .sda(jc[3]));
    wire la_tx_unused;
    tb_uart_term la (.rx(jd[1]), .tx(la_tx_unused));     // decodes UART frames on JD2

    // ---------------- DDR stand-in ----------------
    axi_mem_model mem (
        .clk(clk), .rst_n(aresetn),
        .awaddr(m_awaddr), .awlen(m_awlen), .awsize(m_awsize), .awburst(m_awburst),
        .awcache(m_awcache), .awprot(m_awprot), .awvalid(m_awvalid), .awready(m_awready),
        .wdata(m_wdata), .wstrb(m_wstrb), .wlast(m_wlast), .wvalid(m_wvalid), .wready(m_wready),
        .bresp(m_bresp), .bvalid(m_bvalid), .bready(m_bready)
    );

    // ---------------- helpers ----------------
    task check;
        input cond;
        input [8*72-1:0] what;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("    FAIL  %0s", what);
            end else
                $display("    ok    %0s", what);
        end
    endtask

    task check_eq;
        input [31:0] got, exp;
        input [8*72-1:0] what;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("    FAIL  %0s: got 0x%0h expected 0x%0h", what, got, exp);
            end else
                $display("    ok    %0s = 0x%0h", what, got);
        end
    endtask

    task axi_wr;
        input [11:0] a;
        input [31:0] d;
        begin
            fork
                begin
                    @(posedge clk); awaddr <= a; awvalid <= 1'b1;
                    @(posedge clk); while (!awready) @(posedge clk);
                    awvalid <= 1'b0;
                end
                begin
                    @(posedge clk); wdata <= d; wvalid <= 1'b1;
                    @(posedge clk); while (!wready) @(posedge clk);
                    wvalid <= 1'b0;
                end
            join
            bready <= 1'b1;
            @(posedge clk); while (!bvalid) @(posedge clk);
            bready <= 1'b0;
        end
    endtask

    task axi_rd;
        input  [11:0] a;
        output [31:0] d;
        begin
            @(posedge clk); araddr <= a; arvalid <= 1'b1;
            @(posedge clk); while (!arready) @(posedge clk);
            arvalid <= 1'b0;
            rready  <= 1'b1;
            @(posedge clk); while (!rvalid) @(posedge clk);
            d = rdata;
            rready <= 1'b0;
        end
    endtask

    // wait until the PC terminal has received n bytes in total (or time out)
    task pc_wait_bytes;
        input integer n;
        input integer max_us;
        integer t;
        begin
            t = 0;
            while (pc.rx_cnt < n && t < max_us) begin #1000; t = t + 1; end
        end
    endtask

    task la_wait_bytes;
        input integer n;
        input integer max_us;
        integer t;
        begin
            t = 0;
            while (la.rx_cnt < n && t < max_us) begin #1000; t = t + 1; end
        end
    endtask

    // record field helpers (little-endian bytes from the memory model)
    function [7:0] mb;
        input [31:0] addr;
        begin
            mb = mem.read_byte(addr);
        end
    endfunction

    // ---------------- the serial-engine program from the assembler ----------------
    reg [15:0] prog_uart_tx [0:31];
    integer    prog_len;

    // ---------------- test sequence ----------------
    localparam [31:0] RING = 32'h0010_0000;        // DMA ring buffer in "DDR"
    reg [31:0] r, r2, ts_prev, apb0, irq0;
    integer    i, k, n0, good, base_cnt;

    initial begin
        $display("==============================================================");
        $display(" Smart Serial Controller v2 - system test (Way 2 hardware path)");
        $display("==============================================================");
        for (i = 0; i < 32; i = i + 1) prog_uart_tx[i] = 16'hFFFF;
        $readmemh({`PROG_DIR, "uart_tx.hex"}, prog_uart_tx);
        prog_len = 0;
        for (i = 0; i < 32; i = i + 1) if (prog_uart_tx[i] !== 16'hFFFF && prog_uart_tx[i] !== 16'hxxxx) prog_len = i + 1;

        repeat (5) @(posedge clk);
        aresetn = 1'b1;
        repeat (5) @(posedge clk);

        // ------------------------------------------------------------------
        $display("[1] registers");
        axi_rd(12'h000, r);  check_eq(r, 32'h5353_4302, "ID (v1 block, version 2)");
        axi_rd(12'hA00, r);  check_eq(r, 32'h0002_0000, "VERSION");
        axi_wr(12'h014, 32'hCAFE_F00D);
        axi_rd(12'h014, r);  check_eq(r, 32'hCAFE_F00D, "v1 SCRATCH register");
        axi_rd(12'h400, r);  check_eq(r[2:0], 3'd1, "PIN_SEL[JA] = UART after reset");
        axi_rd(12'h404, r);  check_eq(r[2:0], 3'd2, "PIN_SEL[JB] = SPI after reset");
        axi_rd(12'h408, r);  check_eq(r[2:0], 3'd3, "PIN_SEL[JC] = I2C after reset");
        axi_rd(12'h40C, r);  check_eq(r[2:0], 3'd0, "PIN_SEL[JD] = OFF after reset");
        axi_rd(12'hA18, r);  check_eq(r, 32'd3, "OLED supplies off after reset");
        axi_rd(12'hA04, r);  r2 = r;
        #20000;
        axi_rd(12'hA04, r);  check((r - r2) >= 19 && (r - r2) <= 21, "TIME_US counts microseconds");

        // ------------------------------------------------------------------
        $display("[2] slide 17, flow A: PC -> UART NI -> SPI flash -> PC");
        axi_wr(12'h100, 32'h0000_000F);           // UART: TX, RX, 8N1, 115200
        axi_wr(12'h200, 32'h0000_0071);           // SPI: EN, mode 0, 8 bit, 1 MHz
        axi_wr(12'h300, 32'h0000_0001);           // I2C: EN
        axi_wr(12'h304, 32'd62);                  // I2C 400 kHz
        axi_wr(12'h50C, 32'h0000_0043);           // UART NI: EN, FRAME, dest 4 (SPI)
        axi_wr(12'h510, 32'h0000_0001);           // SPI NI: EN
        axi_wr(12'h514, 32'h0000_0001);           // I2C NI: EN
        n0 = pc.rx_cnt;
        pc.send(8'h01, 0, 0); pc.send(8'h9F, 0, 0); pc.send(8'h03, 0, 0);
        pc_wait_bytes(n0 + 3, 2000);
        check(pc.rx_cnt == n0 + 3, "PC got 3 reply bytes");
        check_eq({8'd0, pc.rx_mem[n0], pc.rx_mem[n0+1], pc.rx_mem[n0+2]}, 32'h0020_BA19,
                 "flash ID through the NoC = 20 BA 19");
        axi_rd(12'h52C, r);  check_eq(r, 32'h0001_0001, "NI_STAT[UART]: 1 packet out, 1 in");
        axi_rd(12'h530, r);  check_eq(r, 32'h0001_0001, "NI_STAT[SPI]: 1 packet in, 1 out");

        // ------------------------------------------------------------------
        $display("[3] slides 18-20, flow B: hub -> I2C -> hub -> DMA -> DDR, 1 IRQ per 32");
        axi_wr(12'h704, RING);                    // DMA_BASE
        axi_wr(12'h708, 32'd6);                   // 64 records
        axi_wr(12'h714, 32'd32);                  // batch of 32
        axi_wr(12'h700, 32'h0000_0001);           // DMA EN
        axi_wr(12'h60C, 32'd200);                 // hub timeout 200 "ms"
        axi_wr(12'h640, 32'h0121_4B1B);           // task 0: EN, I2C, XFER_REQ, 0x4B, w1 r2, RECORD
        axi_wr(12'h644, 32'h0000_0000);           //   pointer byte 0x00
        axi_wr(12'h64C, 32'd20);                  //   every 20 "ms" (200 us here)
        axi_wr(12'hA08, 32'hFF);                  // clear INT2
        axi_wr(12'hA0C, 32'h0000_0001);           // INT2_ENABLE: DMA_BATCH
        axi_wr(12'h600, 32'h0000_0001);           // HUB EN -> the CPU now "sleeps"
        axi_wr(12'hA10, 32'd0);                   // clear APB_COUNT
        axi_wr(12'hA14, 32'd0);                   // clear IRQ_COUNT

        // ---- [4] at the same time: flow A again (shares router (1,1)) ----
        n0 = pc.rx_cnt;
        pc.send(8'h01, 0, 0); pc.send(8'h9F, 0, 0); pc.send(8'h03, 0, 0);

        k = 0;
        while (!irq && k < 20000) begin #1000; k = k + 1; end   // up to 20 ms
        check(irq, "batch interrupt arrived");
        axi_rd(12'hA10, apb0);
        axi_rd(12'hA14, irq0);
        $display("    measured: %0d APB accesses and %0d interrupt(s) for 32 readings", apb0, irq0);
        check_eq(apb0, 32'd0, "CPU made 0 bus accesses during the 32 readings");
        check_eq(irq0, 32'd1, "exactly 1 interrupt for 32 readings");
        axi_rd(12'hA08, r);  check(r[0], "INT2_STATUS.DMA_BATCH set");
        axi_rd(12'h71C, r);  check(r >= 32, "DMA COUNT >= 32");

        // the records in memory
        good = 0;
        ts_prev = 0;
        for (i = 0; i < 32; i = i + 1) begin
            r = {mb(RING + i*16 + 3), mb(RING + i*16 + 2), mb(RING + i*16 + 1), mb(RING + i*16)};
            if (mb(RING + i*16 + 4) == 8'd3 &&                        // source I2C
                mb(RING + i*16 + 5) == 8'd2 &&                        // 2 data bytes
                {mb(RING + i*16 + 7), mb(RING + i*16 + 6)} == i + 1 &&  // sequence
                mb(RING + i*16 + 8) == 8'h0C && mb(RING + i*16 + 9) == 8'h80 &&
                mb(RING + i*16 + 10) == 8'h00 && mb(RING + i*16 + 15) == 8'h00 &&
                (i == 0 || (r - ts_prev >= 150 && r - ts_prev <= 250)))
                good = good + 1;
            else
                $display("    record %0d: ts %0d src %0d len %0d seq %0d data %h %h", i, r,
                         mb(RING + i*16 + 4), mb(RING + i*16 + 5),
                         {mb(RING + i*16 + 7), mb(RING + i*16 + 6)},
                         mb(RING + i*16 + 8), mb(RING + i*16 + 9));
            ts_prev = r;
        end
        check_eq(good, 32, "32 records: ts every 200 us, source 3, len 2, seq 1..32, 0C 80");
        r = {mb(RING + 3), mb(RING + 2), mb(RING + 1), mb(RING)};
        $display("    first record: timestamp %0d us, temperature 0x0C80 = 25.00 C", r);

        axi_wr(12'hA08, 32'h0000_0001);           // clear the batch flag
        @(posedge clk); @(posedge clk);
        check(!irq, "IRQ line low after W1C");

        $display("[4] flow A while flow B was running");
        pc_wait_bytes(n0 + 3, 2000);
        check_eq({8'd0, pc.rx_mem[n0], pc.rx_mem[n0+1], pc.rx_mem[n0+2]}, 32'h0020_BA19,
                 "flash ID during the hub traffic = 20 BA 19");

        axi_wr(12'h600, 32'h0000_0000);           // stop the hub
        #400000;                                  // let the last reading finish

        // ------------------------------------------------------------------
        $display("[5] slide 14: JD switches from the SPI engine to SM0 (UART program)");
        check(prog_len >= 7, "programs/uart_tx.hex loaded");
        for (i = 0; i < prog_len; i = i + 1) axi_wr(12'h900 + 4*i, {16'd0, prog_uart_tx[i]});
        axi_wr(12'h810, 32'h006C_8000);           // SM0 CLKDIV 108.5
        axi_wr(12'h814, 32'h0220_0015);           // SM0 PINCTRL: TX on pin 1, idle high
        axi_wr(12'h818, 32'h0000_0000);           // SM0 SHIFTCTRL
        axi_wr(12'h800, 32'h0000_0101);           // SM0 restart + enable
        axi_wr(12'h40C, 32'd2);                   // JD = SPI (copy of the flash lines)
        k = 0; r = 32'h100;
        while (r[8] && k < 100) begin axi_rd(12'h40C, r); k = k + 1; end
        check_eq(r[2:0], 3'd2, "JD now shows the SPI engine");
        // start a long SPI read through the NoC: 01 03 00 00 00 28 = read 40 bytes
        n0 = pc.rx_cnt;
        pc.send(8'h04, 0, 0); pc.send(8'h03, 0, 0); pc.send(8'h00, 0, 0);
        pc.send(8'h00, 0, 0); pc.send(8'h00, 0, 0); pc.send(8'd40, 0, 0);
        // wait until the SPI frame is running (CS# low on JD1), then ask to switch
        k = 0;
        while (jd[0] !== 1'b0 && k < 100000) begin #100; k = k + 1; end
        check(jd[0] === 1'b0, "SPI frame visible on JD (CS# low)");
        axi_wr(12'h40C, 32'd4);                   // JD -> SM0, not forced
        axi_rd(12'h40C, r);
        check(r[8] && r[2:0] == 3'd2, "switch is waiting while the SPI frame runs");
        k = 0;
        while (r[8] && k < 2000) begin #1000; axi_rd(12'h40C, r); k = k + 1; end
        check_eq(r[2:0], 3'd4, "JD now driven by SM0");
        check(jb[0] === 1'b1, "the SPI frame had finished before the hand-over");
        axi_rd(12'h44C, r);
        $display("    SW_CYCLES[JD] = %0d clocks (waited for the SPI frame)", r);
        check(r > 100, "SW_CYCLES counts the wait for the idle SPI engine");
        pc_wait_bytes(n0 + 40, 3000);
        check(pc.rx_cnt == n0 + 40, "PC still got all 40 flash bytes");
        // now send 'A' with the slide-13 program and decode it on JD2
        n0 = la.rx_cnt;
        axi_wr(12'h81C, 32'h41);
        axi_wr(12'h81C, 32'h42);
        la_wait_bytes(n0 + 2, 1000);
        check(la.rx_cnt == n0 + 2 && la.rx_mem[n0] == 8'h41 && la.rx_mem[n0+1] == 8'h42,
              "slide-13 program sends 'A' 'B' on JD2 at 115200 baud");
        axi_rd(12'h46C, r);
        $display("    SW_EDGE[JD] = %0d clocks", r);

        // a quick switch from an idle source: JD -> SM0 is idle -> OFF -> SM0
        axi_wr(12'h40C, 32'd0);
        k = 0; r = 32'h100;
        while (r[8] && k < 100) begin axi_rd(12'h40C, r); k = k + 1; end
        axi_rd(12'h44C, r);
        $display("    switch time from an idle source: %0d clocks", r);
        check(r < 20, "switching from an idle source takes a few clocks");

        // ------------------------------------------------------------------
        $display("[6] protocol conversion: PC UART -> NoC -> SM0 UART on JD");
        axi_wr(12'h40C, 32'd4);                   // JD = SM0 again
        k = 0; r = 32'h100;
        while (r[8] && k < 100) begin axi_rd(12'h40C, r); k = k + 1; end
        axi_wr(12'h500, 32'h0000_0001);           // SE NI: EN
        axi_wr(12'h50C, 32'h0000_0001);           // UART NI: EN, RAW, dest 0 (SE), arg 0 = SM0
        n0 = la.rx_cnt;
        pc.send("H", 0, 0); pc.send("i", 0, 0); pc.send("!", 0, 0);
        la_wait_bytes(n0 + 3, 4000);
        check(la.rx_cnt == n0 + 3 && la.rx_mem[n0] == "H" && la.rx_mem[n0+1] == "i" &&
              la.rx_mem[n0+2] == "!", "\"Hi!\" from the PC came out of SM0 on JD");
        axi_rd(12'h520, r);  check(r[15:0] >= 1, "NI_STAT[SE] counted the packet");

        // ------------------------------------------------------------------
        $display("[7] v1 path: CPU reads the TMP2 itself (slide 20, left side)");
        axi_wr(12'h514, 32'h0000_0000);           // I2C NI off: the CPU owns the engine
        axi_wr(12'h008, 32'h0000_0200);           // INT_ENABLE: I2C_DONE
        axi_wr(12'h004, 32'hFFFF);                // clear v1 flags
        axi_wr(12'hA0C, 32'h0000_0000);           // INT2 off
        axi_wr(12'hA10, 32'd0);
        axi_wr(12'hA14, 32'd0);
        // the v1 driver sequence (sw/ssc2_driver.c ssc_i2c_write_read)
        axi_wr(12'h308, 32'h0000_004B);           // I2C_ADDR
        axi_wr(12'h314, 32'h0000_0000);           // pointer = 0
        axi_wr(12'h30C, 32'h0000_0001);           // write 1 byte, no STOP
        k = 0; while (!irq && k < 1000) begin #1000; k = k + 1; end
        axi_rd(12'h004, r);                       // see why
        axi_wr(12'h004, 32'h0000_0200);           // clear
        axi_wr(12'h30C, 32'h0000_0302);           // read 2 + STOP
        k = 0; while (!irq && k < 1000) begin #1000; k = k + 1; end
        axi_rd(12'h004, r);
        axi_wr(12'h004, 32'h0000_0200);
        axi_rd(12'h318, r);  r2[15:8] = r[7:0];
        axi_rd(12'h318, r);  r2[7:0]  = r[7:0];
        axi_rd(12'hA10, apb0);
        axi_rd(12'hA14, irq0);
        check_eq(r2[15:0], 16'h0C80, "v1-style read gives 0x0C80");
        $display("    measured v1 style: %0d APB accesses and %0d interrupts for 1 reading", apb0, irq0);
        check(apb0 >= 10 && irq0 == 2, "v1 style costs ~10-20 accesses + 2 IRQs per reading");

        // ------------------------------------------------------------------
        check(mem.errors == 0, "AXI memory model saw no protocol errors");
        $display("==============================================================");
        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $display("==============================================================");
        $finish;
    end

    // watchdog
    initial begin
        #200_000_000;
        $display("FAIL: watchdog - simulation took too long");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule

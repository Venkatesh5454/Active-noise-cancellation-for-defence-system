// =============================================================================
// zed2_top_standalone.v  -  "Way 1": the v2 controller on the FPGA alone
// -----------------------------------------------------------------------------
// No ARM, no software.  ssc2_boot plays the CPU's set-up role over APB, then
// the controller runs on its own:
//   * PC (PmodUSBUART on JA) sends 01 9F 03 -> the UART NI turns it into a
//     packet for the SPI node -> PmodSF3 on JB answers 20 BA 19 -> back to the PC
//   * the sensor hub reads the PmodTMP2 on JC every 100 ms, the DMA writer
//     stores the 16-byte records in a 4 KB block RAM (instead of DDR)
//   * SW2..0 pick the source of the JD pins (0 off, 1 UART, 2 SPI, 3 I2C,
//     4 SM0 = slide-13 UART program, 5 SM1, 6 GPIO): the crossbar demo
//   * SW7: UART NI mode, 0 = FRAME (to the SPI flash), 1 = ADDRESSED frames
//   * LD0..3 = records written / 4, LD4 heartbeat, LD5 DMA batch,
//     LD6 hub error, LD7 interrupt line
//   * BTNC = reset
// =============================================================================
`timescale 1ns / 1ps
module zed2_top_standalone (
    input  wire        clk_100m,
    input  wire        btn_reset,     // BTNC, active high
    input  wire [4:1]  btn,           // BTND, BTNL, BTNR, BTNU
    input  wire [7:0]  sw,
    inout  wire [3:0]  ja,
    inout  wire [3:0]  jb,
    output wire [3:0]  jb_lo,
    inout  wire [3:0]  jc,
    inout  wire [3:0]  jd,
    output wire [3:0]  jd_lo,
    inout  wire [3:0]  oled,
    output wire        oled_vdd,
    output wire        oled_vbat,
    output wire [7:0]  led
);
    wire clk = clk_100m;

    // reset: asynchronous on, synchronous off (2 flip-flops)
    reg [1:0] rst_q;
    always @(posedge clk or posedge btn_reset) begin
        if (btn_reset) rst_q <= 2'b00;
        else           rst_q <= {rst_q[0], 1'b1};
    end
    wire rst_n = rst_q[1];

    // ---- set-up sequencer (APB master) ----
    wire [11:0] paddr;
    wire        psel, penable, pwrite, pready, pslverr, irq, running;
    wire [31:0] pwdata, prdata;

    ssc2_boot #(.CLK_HZ(100_000_000)) u_boot (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .running(running)
    );

    // ---- the controller ----
    wire [31:0] awaddr, wdata;
    wire [7:0]  awlen;
    wire [3:0]  wstrb;
    wire        awvalid, awready, wlast, wvalid, wready, bvalid, bready;
    wire [1:0]  bresp;
    wire [23:0] pad_out, pad_oe, pad_in;
    wire [2:0]  spi_cs_hi_n;
    wire [3:0]  mon, status_led;
    wire [1:0]  oled_pwr;
    wire [7:0]  board_sw;
    wire [4:0]  board_btn;

    ssc2_core u_core (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .irq(irq),
        .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(),
        .m_axi_awburst(), .m_axi_awcache(), .m_axi_awprot(),
        .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .spi_cs_hi_n(spi_cs_hi_n), .mon(mon),
        .board_sw(board_sw), .board_btn(board_btn),
        .oled_pwr(oled_pwr), .status_led(status_led)
    );

    // ---- record memory (stands in for DDR) ----
    ssc2_axi_bram #(.AW(10)) u_mem (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(awaddr), .s_axi_awlen(awlen),
        .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wlast(wlast),
        .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .rd_addr(10'd0), .rd_data()
    );

    // ---- pads ----
    zed2_pads u_pads (
        .clk(clk),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .spi_cs_hi_n(spi_cs_hi_n), .mon(mon), .oled_pwr(oled_pwr),
        .status_led(status_led),
        .board_sw(board_sw), .board_btn(board_btn),
        .ja(ja), .jb(jb), .jb_lo(jb_lo), .jc(jc), .jd(jd), .jd_lo(jd_lo),
        .oled(oled), .oled_vdd(oled_vdd), .oled_vbat(oled_vbat),
        .led(led), .sw(sw), .btn({btn, 1'b0})
    );
endmodule

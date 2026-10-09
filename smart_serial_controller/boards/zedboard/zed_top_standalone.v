// =============================================================================
// zed_top_standalone.v  -  "Way 1": FPGA only, no ARM, no software
// -----------------------------------------------------------------------------
//   GCLK 100 MHz ---> ssc_cmd_fsm --APB--> ssc_apb_top ---> Pmod pins
//
// Open a terminal (PuTTY / Tera Term / Vivado's none) on the PmodUSBUART COM
// port at 115200 8N1, press BTNC (reset) and you get a menu:
//     t -> temperature from the PmodTMP2 (JC)
//     f -> JEDEC ID of the PmodSF3 flash (JB)
// =============================================================================
`timescale 1ns / 1ps
module zed_top_standalone (
    input  wire       clk_100m,     // GCLK  (Y9)
    input  wire       btn_reset,    // BTNC  (P16), active high
    // JA: PmodUSBUART
    input  wire       uart_cts_n,   // JA1
    output wire       uart_txd,     // JA2  FPGA -> PC
    input  wire       uart_rxd,     // JA3  PC -> FPGA
    output wire       uart_rts_n,   // JA4
    // JB: PmodSF3 SPI flash
    inout  wire       spi_cs_n,     // JB1
    inout  wire       spi_mosi,     // JB2
    inout  wire       spi_miso,     // JB3
    inout  wire       spi_sclk,     // JB4
    output wire       spi_cs1_n,    // JB7  spare CS1# (NC on the PmodSF3)
    output wire       spi_rst_n,    // JB8  PmodSF3 RST#  (held high)
    output wire       spi_wp_n,     // JB9  PmodSF3 WP#   (held high)
    output wire       spi_hold_n,   // JB10 PmodSF3 HOLD# (held high)
    // JC: PmodTMP2 I2C temperature sensor
    inout  wire       i2c_scl,      // JC3
    inout  wire       i2c_sda,      // JC4
    // JD: logic analyser
    output wire [7:0] la,
    // LEDs
    output wire [7:0] led
);
    // ---------------- reset: power-on + button, released in sync ----------------
    reg [3:0] por = 4'd0;
    always @(posedge clk_100m)
        if (por != 4'hF) por <= por + 4'd1;
    wire rst_req = btn_reset | (por != 4'hF);

    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync = 2'b00;
    always @(posedge clk_100m or posedge rst_req)
        if (rst_req) rst_sync <= 2'b00;
        else         rst_sync <= {rst_sync[0], 1'b1};
    wire rst_n = rst_sync[1];

    // ---------------- APB master (stands in for the CPU) ----------------
    wire [11:0] paddr;
    wire        psel, penable, pwrite, pready, pslverr;
    wire [31:0] pwdata, prdata;
    wire        err_flag;

    ssc_cmd_fsm u_cmd (
        .clk(clk_100m), .rst_n(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready),
        .err_flag(err_flag)
    );

    // ---------------- the controller ----------------
    wire       c_spi_sclk, c_spi_mosi, c_spi_miso;
    wire [3:0] c_spi_cs_n;
    wire       c_spis_sclk, c_spis_mosi, c_spis_cs_n, c_spis_miso, c_spis_miso_oe;
    wire       c_slave_mode;
    wire       c_scl_in, c_scl_oe, c_sda_in, c_sda_oe;
    wire       irq;

    ssc_apb_top u_ssc (
        .pclk(clk_100m), .presetn(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .irq(irq),
        .uart_rxd(uart_rxd), .uart_txd(uart_txd),
        .uart_cts_n(uart_cts_n), .uart_rts_n(uart_rts_n),
        .spi_sclk(c_spi_sclk), .spi_mosi(c_spi_mosi), .spi_miso(c_spi_miso),
        .spi_cs_n(c_spi_cs_n),
        .spis_sclk(c_spis_sclk), .spis_mosi(c_spis_mosi), .spis_cs_n(c_spis_cs_n),
        .spis_miso(c_spis_miso), .spis_miso_oe(c_spis_miso_oe),
        .spi_slave_mode(c_slave_mode),
        .i2c_scl_in(c_scl_in), .i2c_scl_oe(c_scl_oe),
        .i2c_sda_in(c_sda_in), .i2c_sda_oe(c_sda_oe)
    );

    // ---------------- pads, logic analyser, LEDs ----------------
    zed_pmod_pads u_pads (
        .clk(clk_100m),
        .c_uart_txd(uart_txd), .c_uart_rxd(uart_rxd),
        .c_spi_sclk(c_spi_sclk), .c_spi_mosi(c_spi_mosi), .c_spi_miso(c_spi_miso),
        .c_spi_cs_n(c_spi_cs_n),
        .c_spis_sclk(c_spis_sclk), .c_spis_mosi(c_spis_mosi), .c_spis_cs_n(c_spis_cs_n),
        .c_spis_miso(c_spis_miso), .c_spis_miso_oe(c_spis_miso_oe),
        .c_slave_mode(c_slave_mode),
        .c_scl_in(c_scl_in), .c_scl_oe(c_scl_oe),
        .c_sda_in(c_sda_in), .c_sda_oe(c_sda_oe),
        .aux({rst_n, err_flag}),         // LD7 = running, LD6 = last command failed
        .pad_spi_cs_n(spi_cs_n), .pad_spi_mosi(spi_mosi),
        .pad_spi_miso(spi_miso), .pad_spi_sclk(spi_sclk),
        .pad_spi_cs1_n(spi_cs1_n), .pad_spi_rst_n(spi_rst_n),
        .pad_spi_wp_n(spi_wp_n), .pad_spi_hold_n(spi_hold_n),
        .pad_i2c_scl(i2c_scl), .pad_i2c_sda(i2c_sda),
        .la(la), .led(led)
    );
endmodule

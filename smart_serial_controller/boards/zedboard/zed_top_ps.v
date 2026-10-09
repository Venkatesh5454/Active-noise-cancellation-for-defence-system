// =============================================================================
// zed_top_ps.v  -  "Way 2": ARM (Zynq PS) + controller in the FPGA fabric
// -----------------------------------------------------------------------------
// The block design "system" (made by vivado/build_ps_system.tcl) contains:
//     Zynq PS  --M_AXI_GP0-->  AXI interconnect  -->  ssc_axi_top (module ref)
//     ssc_axi_top/irq  -->  PS IRQ_F2P[0]   (interrupt ID 61)
// Its generated wrapper "system_wrapper" brings the controller's pins out as
// plain ports; this file adds the tri-state pads, the logic-analyser copy and
// the LEDs, exactly like the stand-alone top level.
// =============================================================================
`timescale 1ns / 1ps
module zed_top_ps (
    // ---- Zynq PS: DDR memory and fixed MIO (do not touch) ----
    inout  wire [14:0] DDR_addr,
    inout  wire [2:0]  DDR_ba,
    inout  wire        DDR_cas_n,
    inout  wire        DDR_ck_n,
    inout  wire        DDR_ck_p,
    inout  wire        DDR_cke,
    inout  wire        DDR_cs_n,
    inout  wire [3:0]  DDR_dm,
    inout  wire [31:0] DDR_dq,
    inout  wire [3:0]  DDR_dqs_n,
    inout  wire [3:0]  DDR_dqs_p,
    inout  wire        DDR_odt,
    inout  wire        DDR_ras_n,
    inout  wire        DDR_reset_n,
    inout  wire        DDR_we_n,
    inout  wire        FIXED_IO_ddr_vrn,
    inout  wire        FIXED_IO_ddr_vrp,
    inout  wire [53:0] FIXED_IO_mio,
    inout  wire        FIXED_IO_ps_clk,
    inout  wire        FIXED_IO_ps_porb,
    inout  wire        FIXED_IO_ps_srstb,
    // ---- JA: PmodUSBUART ----
    input  wire        uart_cts_n,   // JA1
    output wire        uart_txd,     // JA2
    input  wire        uart_rxd,     // JA3
    output wire        uart_rts_n,   // JA4
    // ---- JB: PmodSF3 ----
    inout  wire        spi_cs_n,     // JB1
    inout  wire        spi_mosi,     // JB2
    inout  wire        spi_miso,     // JB3
    inout  wire        spi_sclk,     // JB4
    output wire        spi_cs1_n,    // JB7  spare CS1# (NC on the PmodSF3)
    output wire        spi_rst_n,    // JB8  PmodSF3 RST#  (held high)
    output wire        spi_wp_n,     // JB9  PmodSF3 WP#   (held high)
    output wire        spi_hold_n,   // JB10 PmodSF3 HOLD# (held high)
    // ---- JC: PmodTMP2 ----
    inout  wire        i2c_scl,      // JC3
    inout  wire        i2c_sda,      // JC4
    // ---- JD + LEDs ----
    output wire [7:0]  la,
    output wire [7:0]  led
);
    wire       fclk, irq_out;
    wire       c_spi_sclk, c_spi_mosi, c_spi_miso;
    wire [3:0] c_spi_cs_n;
    wire       c_spis_sclk, c_spis_mosi, c_spis_cs_n, c_spis_miso, c_spis_miso_oe;
    wire       c_slave_mode;
    wire       c_scl_in, c_scl_oe, c_sda_in, c_sda_oe;

    system_wrapper u_system (
        .DDR_addr(DDR_addr), .DDR_ba(DDR_ba), .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n), .DDR_ck_p(DDR_ck_p), .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n), .DDR_dm(DDR_dm), .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n), .DDR_dqs_p(DDR_dqs_p), .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n), .DDR_reset_n(DDR_reset_n), .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn), .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio), .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb), .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
        .fclk(fclk),
        .irq_out(irq_out),
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

    zed_pmod_pads u_pads (
        .clk(fclk),
        .c_uart_txd(uart_txd), .c_uart_rxd(uart_rxd),
        .c_spi_sclk(c_spi_sclk), .c_spi_mosi(c_spi_mosi), .c_spi_miso(c_spi_miso),
        .c_spi_cs_n(c_spi_cs_n),
        .c_spis_sclk(c_spis_sclk), .c_spis_mosi(c_spis_mosi), .c_spis_cs_n(c_spis_cs_n),
        .c_spis_miso(c_spis_miso), .c_spis_miso_oe(c_spis_miso_oe),
        .c_slave_mode(c_slave_mode),
        .c_scl_in(c_scl_in), .c_scl_oe(c_scl_oe),
        .c_sda_in(c_sda_in), .c_sda_oe(c_sda_oe),
        .aux({1'b1, irq_out}),            // LD7 = bitstream loaded, LD6 = IRQ
        .pad_spi_cs_n(spi_cs_n), .pad_spi_mosi(spi_mosi),
        .pad_spi_miso(spi_miso), .pad_spi_sclk(spi_sclk),
        .pad_spi_cs1_n(spi_cs1_n), .pad_spi_rst_n(spi_rst_n),
        .pad_spi_wp_n(spi_wp_n), .pad_spi_hold_n(spi_hold_n),
        .pad_i2c_scl(i2c_scl), .pad_i2c_sda(i2c_sda),
        .la(la), .led(led)
    );
endmodule

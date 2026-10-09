// =============================================================================
// ssc_axi_top.v  -  the controller as seen from the Zynq block design
// -----------------------------------------------------------------------------
//   ARM (AXI GP0) --AXI4-Lite--> ssc_axi_apb_bridge --APB--> ssc_apb_top --> pins
//                                                               \--> irq -> IRQ_F2P
//
// Add this file to a Vivado block design with "Add Module" (module reference).
// The S_AXI_* names and the X_INTERFACE attributes let Vivado recognise the
// AXI port (as interface "S_AXI"), its clock and reset, and the interrupt, so
// "Run Connection Automation" can hook it to the processor and give it an
// address (0x43C0_0000, 4 KB, in the provided script).  The prefix is upper
// case on purpose: the inferred interface is then called S_AXI in every
// Vivado version, matching ASSOCIATED_BUSIF below.
//
// Note: the pin ports carry X_INTERFACE_IGNORE so Vivado keeps them as plain
// wires (they become block-design ports that the board top level wires to
// tri-state pads).
// =============================================================================
`timescale 1ns / 1ps
module ssc_axi_top (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 S_AXI_ACLK CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET S_AXI_ARESETN" *)
    input  wire        S_AXI_ACLK,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 S_AXI_ARESETN RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        S_AXI_ARESETN,
    // AXI4-Lite slave
    input  wire [11:0] S_AXI_AWADDR,
    input  wire [2:0]  S_AXI_AWPROT,
    input  wire        S_AXI_AWVALID,
    output wire        S_AXI_AWREADY,
    input  wire [31:0] S_AXI_WDATA,
    input  wire [3:0]  S_AXI_WSTRB,
    input  wire        S_AXI_WVALID,
    output wire        S_AXI_WREADY,
    output wire [1:0]  S_AXI_BRESP,
    output wire        S_AXI_BVALID,
    input  wire        S_AXI_BREADY,
    input  wire [11:0] S_AXI_ARADDR,
    input  wire [2:0]  S_AXI_ARPROT,
    input  wire        S_AXI_ARVALID,
    output wire        S_AXI_ARREADY,
    output wire [31:0] S_AXI_RDATA,
    output wire [1:0]  S_AXI_RRESP,
    output wire        S_AXI_RVALID,
    input  wire        S_AXI_RREADY,
    // interrupt
    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq INTERRUPT" *)
    (* X_INTERFACE_PARAMETER = "SENSITIVITY LEVEL_HIGH" *)
    output wire        irq,
    // UART pins
    (* X_INTERFACE_IGNORE = "true" *) input  wire       uart_rxd,
    (* X_INTERFACE_IGNORE = "true" *) output wire       uart_txd,
    (* X_INTERFACE_IGNORE = "true" *) input  wire       uart_cts_n,
    (* X_INTERFACE_IGNORE = "true" *) output wire       uart_rts_n,
    // SPI master pins
    (* X_INTERFACE_IGNORE = "true" *) output wire       spi_sclk,
    (* X_INTERFACE_IGNORE = "true" *) output wire       spi_mosi,
    (* X_INTERFACE_IGNORE = "true" *) input  wire       spi_miso,
    (* X_INTERFACE_IGNORE = "true" *) output wire [3:0] spi_cs_n,
    // SPI slave pins
    (* X_INTERFACE_IGNORE = "true" *) input  wire       spis_sclk,
    (* X_INTERFACE_IGNORE = "true" *) input  wire       spis_mosi,
    (* X_INTERFACE_IGNORE = "true" *) input  wire       spis_cs_n,
    (* X_INTERFACE_IGNORE = "true" *) output wire       spis_miso,
    (* X_INTERFACE_IGNORE = "true" *) output wire       spis_miso_oe,
    (* X_INTERFACE_IGNORE = "true" *) output wire       spi_slave_mode,
    // I2C pins
    (* X_INTERFACE_IGNORE = "true" *) input  wire       i2c_scl_in,
    (* X_INTERFACE_IGNORE = "true" *) output wire       i2c_scl_oe,
    (* X_INTERFACE_IGNORE = "true" *) input  wire       i2c_sda_in,
    (* X_INTERFACE_IGNORE = "true" *) output wire       i2c_sda_oe
);
    wire [11:0] paddr;
    wire        psel, penable, pwrite, pready, pslverr;
    wire [31:0] pwdata, prdata;

    ssc_axi_apb_bridge #(.AW(12)) u_axi2apb (
        .aclk(S_AXI_ACLK), .aresetn(S_AXI_ARESETN),
        .s_axi_awaddr(S_AXI_AWADDR), .s_axi_awvalid(S_AXI_AWVALID),
        .s_axi_awready(S_AXI_AWREADY),
        .s_axi_wdata(S_AXI_WDATA), .s_axi_wvalid(S_AXI_WVALID),
        .s_axi_wready(S_AXI_WREADY),
        .s_axi_bresp(S_AXI_BRESP), .s_axi_bvalid(S_AXI_BVALID),
        .s_axi_bready(S_AXI_BREADY),
        .s_axi_araddr(S_AXI_ARADDR), .s_axi_arvalid(S_AXI_ARVALID),
        .s_axi_arready(S_AXI_ARREADY),
        .s_axi_rdata(S_AXI_RDATA), .s_axi_rresp(S_AXI_RRESP),
        .s_axi_rvalid(S_AXI_RVALID), .s_axi_rready(S_AXI_RREADY),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr)
    );

    ssc_apb_top u_ssc (
        .pclk(S_AXI_ACLK), .presetn(S_AXI_ARESETN),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .irq(irq),
        .uart_rxd(uart_rxd), .uart_txd(uart_txd),
        .uart_cts_n(uart_cts_n), .uart_rts_n(uart_rts_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .spis_sclk(spis_sclk), .spis_mosi(spis_mosi), .spis_cs_n(spis_cs_n),
        .spis_miso(spis_miso), .spis_miso_oe(spis_miso_oe),
        .spi_slave_mode(spi_slave_mode),
        .i2c_scl_in(i2c_scl_in), .i2c_scl_oe(i2c_scl_oe),
        .i2c_sda_in(i2c_sda_in), .i2c_sda_oe(i2c_sda_oe)
    );
endmodule

// =============================================================================
// ssc_axi_top.v  -  the controller as seen from the Zynq block design
// -----------------------------------------------------------------------------
//   ARM (AXI GP0) --AXI4-Lite--> ssc_axi_apb_bridge --APB--> ssc_apb_top --> pins
//                                                               \--> irq -> IRQ_F2P
//
// Add this file to a Vivado block design with "Add Module" (module reference).
// The s_axi_* names and the X_INTERFACE attributes let Vivado recognise the
// AXI port, its clock and reset, and the interrupt, so "Run Connection
// Automation" can hook it to the processor and give it an address
// (0x43C0_0000, 4 KB, in the provided script).
//
// Note: the pin ports carry X_INTERFACE_IGNORE so Vivado keeps them as plain
// wires (they become block-design ports that the board top level wires to
// tri-state pads).
// =============================================================================
`timescale 1ns / 1ps
module ssc_axi_top (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET s_axi_aresetn" *)
    input  wire        s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        s_axi_aresetn,
    // AXI4-Lite slave
    input  wire [11:0] s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [11:0] s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,
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
        .aclk(s_axi_aclk), .aresetn(s_axi_aresetn),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr)
    );

    ssc_apb_top u_ssc (
        .pclk(s_axi_aclk), .presetn(s_axi_aresetn),
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

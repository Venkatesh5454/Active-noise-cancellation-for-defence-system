// =============================================================================
// ssc2_axi_top.v  -  the v2 controller as seen from the Zynq block design
// -----------------------------------------------------------------------------
//   ARM (M_AXI_GP0) --AXI4-Lite--> S_AXI --> ssc_axi_apb_bridge --APB--> ssc2_core
//   ssc2_core DMA writer --AXI4 (write only)--> M_AXI --> Zynq S_AXI_HP0 --> DDR
//   ssc2_core irq --> IRQ_F2P[0]
//
// Add this file to a block design with "Add Module" (module reference).  The
// upper-case S_AXI_* / M_AXI_* names and the X_INTERFACE attributes let Vivado
// recognise both AXI ports, their clock and reset, and the interrupt, so
// connection automation can wire them (vivado/build_ps_system.tcl does it).
//
// M_AXI is a full AXI4 master port.  The DMA writer only writes, so the read
// channel is tied off (ARVALID = 0, RREADY = 1).  Bursts are 4 beats of 32
// bits, so the port also fits the AXI3 HP0 port (Vivado adds the converter).
//
// The pin ports carry X_INTERFACE_IGNORE so Vivado keeps them as plain wires;
// they become block-design ports and boards/zedboard/zed2_top_ps.v adds the
// tri-state pads.
// =============================================================================
`timescale 1ns / 1ps
module ssc2_axi_top #(
    parameter US_PER_MS = 1000           // simulations may shorten a millisecond
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 S_AXI_ACLK CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI:M_AXI, ASSOCIATED_RESET S_AXI_ARESETN" *)
    input  wire        S_AXI_ACLK,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 S_AXI_ARESETN RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        S_AXI_ARESETN,
    // ---------------- AXI4-Lite slave (registers, 4 KB) ----------------
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
    // ---------------- AXI4 master (DMA writer -> DDR) ----------------
    output wire [31:0] M_AXI_AWADDR,
    output wire [7:0]  M_AXI_AWLEN,
    output wire [2:0]  M_AXI_AWSIZE,
    output wire [1:0]  M_AXI_AWBURST,
    output wire        M_AXI_AWLOCK,
    output wire [3:0]  M_AXI_AWCACHE,
    output wire [2:0]  M_AXI_AWPROT,
    output wire [3:0]  M_AXI_AWQOS,
    output wire        M_AXI_AWVALID,
    input  wire        M_AXI_AWREADY,
    output wire [31:0] M_AXI_WDATA,
    output wire [3:0]  M_AXI_WSTRB,
    output wire        M_AXI_WLAST,
    output wire        M_AXI_WVALID,
    input  wire        M_AXI_WREADY,
    input  wire [1:0]  M_AXI_BRESP,
    input  wire        M_AXI_BVALID,
    output wire        M_AXI_BREADY,
    output wire [31:0] M_AXI_ARADDR,
    output wire [7:0]  M_AXI_ARLEN,
    output wire [2:0]  M_AXI_ARSIZE,
    output wire [1:0]  M_AXI_ARBURST,
    output wire        M_AXI_ARLOCK,
    output wire [3:0]  M_AXI_ARCACHE,
    output wire [2:0]  M_AXI_ARPROT,
    output wire [3:0]  M_AXI_ARQOS,
    output wire        M_AXI_ARVALID,
    input  wire        M_AXI_ARREADY,
    input  wire [31:0] M_AXI_RDATA,
    input  wire [1:0]  M_AXI_RRESP,
    input  wire        M_AXI_RLAST,
    input  wire        M_AXI_RVALID,
    output wire        M_AXI_RREADY,
    // ---------------- interrupt ----------------
    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq INTERRUPT" *)
    (* X_INTERFACE_PARAMETER = "SENSITIVITY LEVEL_HIGH" *)
    output wire        irq,
    // ---------------- pins (see ssc2_core.v) ----------------
    (* X_INTERFACE_IGNORE = "true" *) output wire [23:0] pad_out,
    (* X_INTERFACE_IGNORE = "true" *) output wire [23:0] pad_oe,
    (* X_INTERFACE_IGNORE = "true" *) input  wire [23:0] pad_in,
    (* X_INTERFACE_IGNORE = "true" *) output wire [2:0]  spi_cs_hi_n,
    (* X_INTERFACE_IGNORE = "true" *) output wire [3:0]  mon,
    (* X_INTERFACE_IGNORE = "true" *) input  wire [7:0]  board_sw,
    (* X_INTERFACE_IGNORE = "true" *) input  wire [4:0]  board_btn,
    (* X_INTERFACE_IGNORE = "true" *) output wire [1:0]  oled_pwr,
    (* X_INTERFACE_IGNORE = "true" *) output wire [3:0]  status_led
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

    ssc2_core #(.CLK_HZ(100_000_000), .US_PER_MS(US_PER_MS)) u_core (
        .clk(S_AXI_ACLK), .rst_n(S_AXI_ARESETN),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .irq(irq),
        .m_axi_awaddr(M_AXI_AWADDR), .m_axi_awlen(M_AXI_AWLEN), .m_axi_awsize(M_AXI_AWSIZE),
        .m_axi_awburst(M_AXI_AWBURST), .m_axi_awcache(M_AXI_AWCACHE), .m_axi_awprot(M_AXI_AWPROT),
        .m_axi_awvalid(M_AXI_AWVALID), .m_axi_awready(M_AXI_AWREADY),
        .m_axi_wdata(M_AXI_WDATA), .m_axi_wstrb(M_AXI_WSTRB), .m_axi_wlast(M_AXI_WLAST),
        .m_axi_wvalid(M_AXI_WVALID), .m_axi_wready(M_AXI_WREADY),
        .m_axi_bresp(M_AXI_BRESP), .m_axi_bvalid(M_AXI_BVALID), .m_axi_bready(M_AXI_BREADY),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .spi_cs_hi_n(spi_cs_hi_n), .mon(mon),
        .board_sw(board_sw), .board_btn(board_btn),
        .oled_pwr(oled_pwr), .status_led(status_led)
    );

    // write-only master: fixed signals and a tied-off read channel
    assign M_AXI_AWLOCK  = 1'b0;
    assign M_AXI_AWQOS   = 4'd0;
    assign M_AXI_ARADDR  = 32'd0;
    assign M_AXI_ARLEN   = 8'd0;
    assign M_AXI_ARSIZE  = 3'd2;
    assign M_AXI_ARBURST = 2'd1;
    assign M_AXI_ARLOCK  = 1'b0;
    assign M_AXI_ARCACHE = 4'b0011;
    assign M_AXI_ARPROT  = 3'd0;
    assign M_AXI_ARQOS   = 4'd0;
    assign M_AXI_ARVALID = 1'b0;
    assign M_AXI_RREADY  = 1'b1;
endmodule

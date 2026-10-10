// =============================================================================
// zed2_top_ps.v  -  "Way 2": ARM (Zynq PS) + v2 controller in the FPGA fabric
// -----------------------------------------------------------------------------
// The block design "system" (made by vivado/build_ps_system.tcl) contains:
//     Zynq PS  --M_AXI_GP0-->  AXI interconnect  -->  ssc_0/S_AXI   (registers)
//     ssc_0/M_AXI  -->  AXI interconnect  -->  Zynq S_AXI_HP0 --> DDR (DMA writer)
//     ssc_0/irq  -->  PS IRQ_F2P[0]   (interrupt ID 61)
// ssc_0 is a module reference to rtl/top/ssc2_axi_top.v.  The generated
// "system_wrapper" brings the controller's pins out as plain ports; this file
// adds the pads (zed2_pads.v).
// =============================================================================
`timescale 1ns / 1ps
module zed2_top_ps (
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
    // ---- Pmods, OLED, LEDs, switches, buttons ----
    inout  wire [3:0]  ja,           // crossbar port 0 (PmodUSBUART)
    inout  wire [3:0]  jb,           // crossbar port 1 (PmodSF3)
    output wire [3:0]  jb_lo,        // JB7..10
    inout  wire [3:0]  jc,           // crossbar port 2 (PmodTMP2 on JC3/JC4)
    inout  wire [3:0]  jd,           // crossbar port 3 (logic analyser / demo)
    output wire [3:0]  jd_lo,        // JD7..10 probe copies
    inout  wire [3:0]  oled,         // crossbar port 4: DC, SDIN, RES, SCLK
    output wire        oled_vdd,
    output wire        oled_vbat,
    output wire [7:0]  led,
    input  wire [7:0]  sw,
    input  wire [4:0]  btn
);
    wire        fclk, irq_out;
    wire [23:0] pad_out, pad_oe, pad_in;
    wire [2:0]  spi_cs_hi_n;
    wire [3:0]  mon, status_led;
    wire [1:0]  oled_pwr;
    wire [7:0]  board_sw;
    wire [4:0]  board_btn;

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
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .spi_cs_hi_n(spi_cs_hi_n), .mon(mon),
        .board_sw(board_sw), .board_btn(board_btn),
        .oled_pwr(oled_pwr), .status_led(status_led)
    );

    zed2_pads u_pads (
        .clk(fclk),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .spi_cs_hi_n(spi_cs_hi_n), .mon(mon), .oled_pwr(oled_pwr),
        .status_led(status_led),
        .board_sw(board_sw), .board_btn(board_btn),
        .ja(ja), .jb(jb), .jb_lo(jb_lo), .jc(jc), .jd(jd), .jd_lo(jd_lo),
        .oled(oled), .oled_vdd(oled_vdd), .oled_vbat(oled_vbat),
        .led(led), .sw(sw), .btn(btn)
    );
endmodule

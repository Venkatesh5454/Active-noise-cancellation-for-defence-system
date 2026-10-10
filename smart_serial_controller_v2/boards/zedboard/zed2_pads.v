// =============================================================================
// zed2_pads.v  -  ZedBoard pads for the v2 controller (used by both top levels)
// -----------------------------------------------------------------------------
// The crossbar works with three plain buses per port (out, oe, in).  This file
// turns them into real tri-state pins:
//
//     pin = oe ? out : 'z'        in = pin            (JA, JB, JC, JD, OLED)
//
// An open-drain line (I2C) is simply "oe = 1, out = 0" to pull low and
// "oe = 0" to let the pull-up raise it, so no special pad is needed.
// The four LEDs of crossbar port 5 are outputs only: oe = 0 means "off".
//
// Switches and buttons go through two flip-flops (they are read over APB).
// =============================================================================
`timescale 1ns / 1ps
module zed2_pads (
    input  wire        clk,
    // from / to the controller
    input  wire [23:0] pad_out,
    input  wire [23:0] pad_oe,
    output wire [23:0] pad_in,
    input  wire [2:0]  spi_cs_hi_n,
    input  wire [3:0]  mon,
    input  wire [1:0]  oled_pwr,
    input  wire [3:0]  status_led,
    output reg  [7:0]  board_sw,
    output reg  [4:0]  board_btn,
    // board pins
    inout  wire [3:0]  ja,          // JA1..4
    inout  wire [3:0]  jb,          // JB1..4
    output wire [3:0]  jb_lo,       // JB7..10: CS1#, RST#, WP#, HOLD# of the PmodSF3
    inout  wire [3:0]  jc,          // JC1..4
    inout  wire [3:0]  jd,          // JD1..4
    output wire [3:0]  jd_lo,       // JD7..10: probe copies TXD, RXD, SCL, SDA
    inout  wire [3:0]  oled,        // DC, SDIN, RES, SCLK
    output wire        oled_vdd,    // 1 = OLED logic supply off
    output wire        oled_vbat,   // 1 = OLED panel supply off
    output wire [7:0]  led,         // LD0..3 crossbar port 5, LD4..7 status
    input  wire [7:0]  sw,
    input  wire [4:0]  btn          // [0] C [1] D [2] L [3] R [4] U
);
    // ---- tri-state ports 0..4 ----
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_pin
            assign ja[i]   = pad_oe[0*4+i]  ? pad_out[0*4+i]  : 1'bz;
            assign jb[i]   = pad_oe[1*4+i]  ? pad_out[1*4+i]  : 1'bz;
            assign jc[i]   = pad_oe[2*4+i]  ? pad_out[2*4+i]  : 1'bz;
            assign jd[i]   = pad_oe[3*4+i]  ? pad_out[3*4+i]  : 1'bz;
            assign oled[i] = pad_oe[4*4+i]  ? pad_out[4*4+i]  : 1'bz;
        end
    endgenerate

    // ---- port 5: LEDs (output only) ----
    wire [3:0] led_lo = pad_oe[23:20] & pad_out[23:20];

    assign pad_in = {led_lo, oled, jd, jc, jb, ja};

    // ---- fixed pins ----
    assign jb_lo     = {1'b1, 1'b1, 1'b1, spi_cs_hi_n[0]};   // HOLD#, WP#, RST# high
    assign jd_lo     = mon;
    assign oled_vdd  = oled_pwr[0];
    assign oled_vbat = oled_pwr[1];
    assign led       = {status_led, led_lo};

    // ---- switches and buttons: 2-flop synchronisers ----
    reg [7:0] sw_q;
    reg [4:0] btn_q;
    always @(posedge clk) begin
        sw_q      <= sw;
        board_sw  <= sw_q;
        btn_q     <= btn;
        board_btn <= btn_q;
    end
endmodule

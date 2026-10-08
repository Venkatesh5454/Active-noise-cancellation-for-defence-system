// =============================================================================
// zed_pmod_pads.v  -  pin logic shared by both ZedBoard top levels
// -----------------------------------------------------------------------------
//   JB  SPI:  master mode -> we drive CS#, MOSI, SCLK and read MISO
//             slave mode  -> an outside master drives CS#, MOSI, SCLK and we
//                            drive MISO (only while our CS# is low)
//             JB7/JB8 = WP#/HOLD# of the PmodSF3 flash, tied high (inactive)
//   JC  I2C:  open drain: drive 0 or let go (high-Z); the pull-ups make the 1.
//             "assign pad = oe ? 1'b0 : 1'bz" makes Vivado use an IOBUF.
//   JD  copy of all bus lines for a logic analyser
//   LD0..LD7  heartbeat + activity lights
// =============================================================================
`timescale 1ns / 1ps
module zed_pmod_pads (
    input  wire       clk,
    // ---- controller side ----
    input  wire       c_uart_txd,
    input  wire       c_uart_rxd,
    input  wire       c_spi_sclk,
    input  wire       c_spi_mosi,
    output wire       c_spi_miso,
    input  wire [3:0] c_spi_cs_n,
    output wire       c_spis_sclk,
    output wire       c_spis_mosi,
    output wire       c_spis_cs_n,
    input  wire       c_spis_miso,
    input  wire       c_spis_miso_oe,
    input  wire       c_slave_mode,
    output wire       c_scl_in,
    input  wire       c_scl_oe,
    output wire       c_sda_in,
    input  wire       c_sda_oe,
    input  wire [1:0] aux,            // two extra status lights (LD6, LD7)
    // ---- pads ----
    inout  wire       pad_spi_cs_n,   // JB1
    inout  wire       pad_spi_mosi,   // JB2
    inout  wire       pad_spi_miso,   // JB3
    inout  wire       pad_spi_sclk,   // JB4
    output wire       pad_spi_wp_n,   // JB7
    output wire       pad_spi_hold_n, // JB8
    output wire       pad_spi_cs1_n,  // JB9
    output wire       pad_spi_cs2_n,  // JB10
    inout  wire       pad_i2c_scl,    // JC3
    inout  wire       pad_i2c_sda,    // JC4
    output wire [7:0] la,             // JD1-4, JD7-10
    output wire [7:0] led             // LD0..LD7
);
    // ---------------- SPI ----------------
    assign pad_spi_cs_n = c_slave_mode ? 1'bz : c_spi_cs_n[0];
    assign pad_spi_mosi = c_slave_mode ? 1'bz : c_spi_mosi;
    assign pad_spi_sclk = c_slave_mode ? 1'bz : c_spi_sclk;
    assign pad_spi_miso = (c_slave_mode & c_spis_miso_oe) ? c_spis_miso : 1'bz;

    assign c_spi_miso   = pad_spi_miso;
    assign c_spis_cs_n  = c_slave_mode ? pad_spi_cs_n : 1'b1;
    assign c_spis_mosi  = pad_spi_mosi;
    assign c_spis_sclk  = pad_spi_sclk;

    assign pad_spi_wp_n   = 1'b1;
    assign pad_spi_hold_n = 1'b1;
    assign pad_spi_cs1_n  = c_spi_cs_n[1];
    assign pad_spi_cs2_n  = c_spi_cs_n[2];

    // ---------------- I2C (open drain) ----------------
    assign pad_i2c_scl = c_scl_oe ? 1'b0 : 1'bz;
    assign pad_i2c_sda = c_sda_oe ? 1'b0 : 1'bz;
    assign c_scl_in    = pad_i2c_scl;
    assign c_sda_in    = pad_i2c_sda;

    // ---------------- logic analyser on JD ----------------
    //  JD1 UART TX   JD2 UART RX   JD3 SPI SCLK  JD4 SPI MOSI
    //  JD7 SPI MISO  JD8 SPI CS#   JD9 I2C SCL   JD10 I2C SDA
    assign la = {pad_i2c_sda, pad_i2c_scl, pad_spi_cs_n, pad_spi_miso,
                 pad_spi_mosi, pad_spi_sclk, c_uart_rxd, c_uart_txd};

    // ---------------- LEDs ----------------
    reg [26:0] heartbeat = 27'd0;
    always @(posedge clk) heartbeat <= heartbeat + 27'd1;

    wire act_utx, act_urx, act_spi, act_i2c;
    zed_led_pulse u_l1 (.clk(clk), .sig(c_uart_txd),   .led(act_utx));
    zed_led_pulse u_l2 (.clk(clk), .sig(c_uart_rxd),   .led(act_urx));
    zed_led_pulse u_l3 (.clk(clk), .sig(pad_spi_sclk), .led(act_spi));
    zed_led_pulse u_l4 (.clk(clk), .sig(pad_i2c_scl),  .led(act_i2c));

    assign led = {aux, c_slave_mode, act_i2c, act_spi, act_urx, act_utx,
                  heartbeat[26]};
endmodule

// -----------------------------------------------------------------------------
// keeps an LED on for ~40 ms after any edge on sig, so short bursts are visible
// -----------------------------------------------------------------------------
module zed_led_pulse #(
    parameter W = 22
) (
    input  wire clk,
    input  wire sig,
    output wire led
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] s = 3'b000;
    reg [W-1:0] cnt = {W{1'b0}};
    always @(posedge clk) begin
        s <= {s[1:0], sig};
        if (s[2] != s[1])     cnt <= {W{1'b1}};
        else if (cnt != 0)    cnt <= cnt - 1'b1;
    end
    assign led = (cnt != 0);
endmodule

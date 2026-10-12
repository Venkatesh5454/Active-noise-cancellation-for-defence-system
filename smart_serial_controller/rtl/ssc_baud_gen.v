// =============================================================================
// ssc_baud_gen.v  -  fractional baud-rate generator for the UART
// -----------------------------------------------------------------------------
// The UART samples every bit 16 times, so it needs a tick at 16 x baud.
//
//      divisor = f_clk / (16 x baud)       e.g. 100 MHz / (16 x 115200) = 54.25
//
// A plain counter can only divide by a whole number (54), which makes the
// line 0.47 % too fast.  Here the divisor is split into
//
//      div_int  = whole part      (54)
//      div_frac = fraction / 16   (0.25 x 16 = 4)
//
// Every tick the fraction is added to a 4-bit accumulator.  When it overflows
// past 16, the next tick period is one clock longer.  With div_frac = 4 that
// happens every fourth tick: 54, 54, 54, 55, 54, 54, 54, 55 ...
// The average is exactly 54.25, so the error at 115200 baud drops to 0.007 %.
//
// div_int must be at least 2 (anything smaller is treated as 2).
// =============================================================================
`timescale 1ns / 1ps
module ssc_baud_gen (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire [15:0] div_int,
    input  wire [3:0]  div_frac,
    output reg         tick          // one clock wide, 16 per bit
);
    wire [15:0] div   = (div_int < 16'd2) ? 16'd2 : div_int;
    reg  [15:0] cnt;
    reg  [3:0]  acc;                 // fractional accumulator (1/16 units)
    reg         extra;               // this period gets one more clock

    wire [16:0] last     = {1'b0, div} - 17'd1 + {16'd0, extra};
    wire [4:0]  acc_next = {1'b0, acc} + {1'b0, div_frac};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt   <= 16'd0;
            acc   <= 4'd0;
            extra <= 1'b0;
            tick  <= 1'b0;
        end else if (!en) begin
            cnt   <= 16'd0;
            acc   <= 4'd0;
            extra <= 1'b0;
            tick  <= 1'b0;
        end else if ({1'b0, cnt} >= last) begin
            cnt   <= 16'd0;
            tick  <= 1'b1;
            acc   <= acc_next[3:0];
            extra <= acc_next[4];    // carry -> stretch the next period
        end else begin
            cnt   <= cnt + 16'd1;
            tick  <= 1'b0;
        end
    end
endmodule

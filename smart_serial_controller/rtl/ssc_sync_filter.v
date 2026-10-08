// =============================================================================
// ssc_sync_filter.v  -  input synchroniser + glitch filter
// -----------------------------------------------------------------------------
// Every input pin of the controller (UART RX, CTS, SPI slave pins, I2C SCL/SDA)
// arrives at a random moment relative to our 100 MHz clock, and may carry
// short spikes picked up on the wires.  This block fixes both problems:
//
//   1. Two flip-flops in a row ("2-FF synchroniser") give a metastable first
//      flip-flop a whole clock period to settle before anyone uses the value.
//   2. A small shift register remembers the last FILTER_LEN samples.  The
//      clean output only changes when ALL of them agree, so a spike shorter
//      than FILTER_LEN clocks never reaches the protocol logic.
//
// rise / fall are one-clock pulses on the edges of the clean signal.
// Delay from pin to dout = 3 + FILTER_LEN clocks (70 ns with the default 4).
// =============================================================================
`timescale 1ns / 1ps
module ssc_sync_filter #(
    parameter FILTER_LEN = 4,      // clocks the input must be stable (>= 2)
    parameter RESET_VAL  = 1'b1    // level of the pin when the line is idle
) (
    input  wire clk,
    input  wire rst_n,
    input  wire din,               // raw, asynchronous pin
    output reg  dout,              // clean, synchronous copy
    output wire rise,              // dout went 0 -> 1 this clock
    output wire fall               // dout went 1 -> 0 this clock
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync;
    reg [FILTER_LEN-1:0] hist;
    reg                  dout_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync   <= {2{RESET_VAL}};
            hist   <= {FILTER_LEN{RESET_VAL}};
            dout   <= RESET_VAL;
            dout_d <= RESET_VAL;
        end else begin
            sync <= {sync[0], din};
            hist <= {hist[FILTER_LEN-2:0], sync[1]};
            if (&hist)       dout <= 1'b1;   // stable high
            else if (~|hist) dout <= 1'b0;   // stable low
            dout_d <= dout;
        end
    end

    assign rise =  dout & ~dout_d;
    assign fall = ~dout &  dout_d;
endmodule

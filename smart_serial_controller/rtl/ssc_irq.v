// =============================================================================
// ssc_irq.v  -  interrupt controller: event flags + enable mask -> one IRQ line
// -----------------------------------------------------------------------------
//   status[i] is set by events[i] and stays set ("sticky") until software
//   writes a 1 to that bit of INT_STATUS (write-1-to-clear).  If the event is a
//   level (e.g. "RX FIFO not empty") the bit sets again straight away while
//   the condition is still true.
//
//   irq = OR of (status AND enable)  ->  goes to the ARM's IRQ_F2P input.
// =============================================================================
`timescale 1ns / 1ps
module ssc_irq #(
    parameter N = 16
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [N-1:0] events,
    input  wire [N-1:0] enable,
    input  wire         clear_we,
    input  wire [N-1:0] clear_mask,
    output reg  [N-1:0] status,
    output reg          irq
);
    wire [N-1:0] clr  = clear_we ? clear_mask : {N{1'b0}};
    wire [N-1:0] next = (status & ~clr) | events;   // a new event beats the clear

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            status <= {N{1'b0}};
            irq    <= 1'b0;
        end else begin
            status <= next;
            irq    <= |(next & enable);              // follows status exactly
        end
    end
endmodule

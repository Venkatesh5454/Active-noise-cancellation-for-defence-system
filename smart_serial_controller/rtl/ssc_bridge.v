// =============================================================================
// ssc_bridge.v  -  bridge engine: copies bytes FIFO -> FIFO without the CPU
// -----------------------------------------------------------------------------
// This keeps the base paper's (MPCU) protocol-conversion idea, but as a mode
// of the controller instead of the whole design.  It has TWO channels so a
// bridge can work in both directions at once (the paper's future work).
//
// BRIDGE_CTRL register:
//     [1:0] ch0 source   [3:2] ch0 destination
//     [5:4] ch1 source   [7:6] ch1 destination
// Protocol codes are the paper's COSE codes:
//     00 = off   01 = SPI   10 = I2C   11 = UART
// A channel runs when both its codes are non-zero.  source == destination is
// the paper's "pass-through" (e.g. UART -> UART = hardware echo).
//
// Every clock, if a channel's source RX FIFO has a byte and its destination
// TX FIFO has room, the byte is popped and pushed in the same clock.  The two
// channels take turns, and the bridge waits while the CPU is accessing the
// registers so the two never touch a FIFO at the same moment.
// =============================================================================
`timescale 1ns / 1ps
module ssc_bridge (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  ctrl,
    input  wire        stall,          // CPU access in progress
    input  wire        cnt_clear,
    // FIFO state, index = protocol code (bit 0 unused)
    input  wire [3:0]  rx_empty,
    input  wire [3:0]  tx_full,
    input  wire [7:0]  rx_data_spi,
    input  wire [7:0]  rx_data_i2c,
    input  wire [7:0]  rx_data_uart,
    // FIFO strobes, index = protocol code
    output reg  [3:0]  rx_pop,
    output reg  [3:0]  tx_push,
    output reg  [7:0]  tx_data,
    // bytes moved by each channel (for the demo / verification)
    output reg  [15:0] cnt0,
    output reg  [15:0] cnt1
);
    wire [1:0] src0 = ctrl[1:0];
    wire [1:0] dst0 = ctrl[3:2];
    wire [1:0] src1 = ctrl[5:4];
    wire [1:0] dst1 = ctrl[7:6];

    function [7:0] pick;
        input [1:0] code;
        case (code)
            2'd1:    pick = rx_data_spi;
            2'd2:    pick = rx_data_i2c;
            2'd3:    pick = rx_data_uart;
            default: pick = 8'd0;
        endcase
    endfunction

    wire ok0 = (src0 != 2'd0) && (dst0 != 2'd0) && !rx_empty[src0] && !tx_full[dst0];
    wire ok1 = (src1 != 2'd0) && (dst1 != 2'd0) && !rx_empty[src1] && !tx_full[dst1];

    reg  turn;                                   // 0: ch0 first, 1: ch1 first
    wire go0 = ~stall & ok0 & (~ok1 | ~turn);
    wire go1 = ~stall & ok1 & (~ok0 |  turn);

    always @* begin
        rx_pop  = 4'b0000;
        tx_push = 4'b0000;
        tx_data = 8'd0;
        if (go0) begin
            rx_pop[src0]  = 1'b1;
            tx_push[dst0] = 1'b1;
            tx_data       = pick(src0);
        end else if (go1) begin
            rx_pop[src1]  = 1'b1;
            tx_push[dst1] = 1'b1;
            tx_data       = pick(src1);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            turn <= 1'b0;
            cnt0 <= 16'd0;
            cnt1 <= 16'd0;
        end else begin
            if (go0 | go1) turn <= ~turn;
            if (cnt_clear) begin
                cnt0 <= 16'd0;
                cnt1 <= 16'd0;
            end else begin
                if (go0) cnt0 <= cnt0 + 16'd1;
                if (go1) cnt1 <= cnt1 + 16'd1;
            end
        end
    end
endmodule

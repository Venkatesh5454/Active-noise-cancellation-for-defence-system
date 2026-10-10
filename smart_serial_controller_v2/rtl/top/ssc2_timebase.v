// =============================================================================
// ssc2_timebase.v  -  microsecond / millisecond time base for the whole core
// -----------------------------------------------------------------------------
//   time_us : microseconds since reset (32 bits, wraps after about 71 minutes)
//   us_tick : one-clock pulse every microsecond
//   ms_tick : one-clock pulse every millisecond (every US_PER_MS microseconds)
//
// Used for record time stamps (sensor hub), time-outs (NIs, hub, DMA writer)
// and the IN time instruction of the serial engine.  Simulations may lower
// US_PER_MS to make "milliseconds" pass faster; the hardware uses 1000.
// =============================================================================
`timescale 1ns / 1ps
module ssc2_timebase #(
    parameter CLK_HZ    = 100_000_000,
    parameter US_PER_MS = 1000
) (
    input  wire        clk,
    input  wire        rst_n,
    output reg  [31:0] time_us,
    output reg         us_tick,
    output reg         ms_tick
);
    /* verilator lint_off WIDTHTRUNC */
    localparam [15:0] DIV_M1 = CLK_HZ / 1_000_000 - 1;   // clocks per us - 1 (99)
    localparam [15:0] MS_M1  = US_PER_MS - 1;
    /* verilator lint_on WIDTHTRUNC */

    reg [15:0] cdiv;
    reg [15:0] ucnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cdiv    <= 16'd0;
            ucnt    <= 16'd0;
            time_us <= 32'd0;
            us_tick <= 1'b0;
            ms_tick <= 1'b0;
        end else begin
            us_tick <= 1'b0;
            ms_tick <= 1'b0;
            if (cdiv == DIV_M1) begin
                cdiv    <= 16'd0;
                us_tick <= 1'b1;
                time_us <= time_us + 32'd1;
                if (ucnt == MS_M1) begin
                    ucnt    <= 16'd0;
                    ms_tick <= 1'b1;
                end else begin
                    ucnt <= ucnt + 16'd1;
                end
            end else begin
                cdiv <= cdiv + 16'd1;
            end
        end
    end
endmodule

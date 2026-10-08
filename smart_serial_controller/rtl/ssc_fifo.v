// =============================================================================
// ssc_fifo.v  -  synchronous first-in first-out buffer (default 16 entries)
// -----------------------------------------------------------------------------
// Used for every TX and RX queue in the controller.  It is a "first-word
// fall-through" (show-ahead) FIFO: rd_data always shows the oldest entry, and
// rd_en simply throws it away.  That lets the register bank and the bridge
// engine read a byte and pop it in the same clock.
//
//   * writes into a full FIFO and reads from an empty FIFO are ignored
//     (the blocks around it raise an "overflow" event instead)
//   * flush empties the FIFO in one clock
//   * count tells software how many entries are waiting (0 .. DEPTH)
//
// The memory has no reset and an asynchronous read, so Vivado builds it from
// distributed RAM (LUTRAM) - 16 x 8 bits costs only a handful of LUTs.
// =============================================================================
`timescale 1ns / 1ps
module ssc_fifo #(
    parameter WIDTH = 8,
    parameter AW    = 4            // address width: DEPTH = 2**AW = 16
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             flush,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    input  wire             rd_en,
    output wire [WIDTH-1:0] rd_data,
    output wire             empty,
    output wire             full,
    output reg  [AW:0]      count
);
    localparam DEPTH = 1 << AW;

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW-1:0]    wptr;
    reg [AW-1:0]    rptr;

    wire do_wr = wr_en & ~full;
    wire do_rd = rd_en & ~empty;

    assign empty   = (count == 0);
    assign full    = (count == DEPTH);
    assign rd_data = mem[rptr];

    always @(posedge clk) begin
        if (do_wr) mem[wptr] <= wr_data;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else if (flush) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else begin
            if (do_wr) wptr <= wptr + 1'b1;
            if (do_rd) rptr <= rptr + 1'b1;
            case ({do_wr, do_rd})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule

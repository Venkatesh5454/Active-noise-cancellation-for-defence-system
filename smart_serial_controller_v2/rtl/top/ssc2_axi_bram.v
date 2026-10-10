// =============================================================================
// ssc2_axi_bram.v  -  small AXI4 write-only memory for the FPGA-only build
// -----------------------------------------------------------------------------
// In "Way 1" there is no ARM and no DDR, so the DMA writer's records go into
// this block RAM instead (4 KB = 256 records of 16 bytes).  It accepts one
// INCR burst at a time:
//     AW handshake -> W beats (one per clock) -> B response (OKAY)
// The address wraps inside the 4 KB.  rd_addr/rd_data is a second, read-only
// port so the rest of the design (or a testbench) can look at the records.
// =============================================================================
`timescale 1ns / 1ps
module ssc2_axi_bram #(
    parameter AW = 10                    // word address bits: 2^10 words = 4 KB
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire [31:0]   s_axi_awaddr,
    input  wire [7:0]    s_axi_awlen,
    input  wire          s_axi_awvalid,
    output wire          s_axi_awready,
    input  wire [31:0]   s_axi_wdata,
    input  wire [3:0]    s_axi_wstrb,
    input  wire          s_axi_wlast,
    input  wire          s_axi_wvalid,
    output wire          s_axi_wready,
    output wire [1:0]    s_axi_bresp,
    output reg           s_axi_bvalid,
    input  wire          s_axi_bready,
    // read-only port
    input  wire [AW-1:0] rd_addr,
    output reg  [31:0]   rd_data
);
    reg [31:0]   mem [0:(1<<AW)-1];
    reg [AW-1:0] waddr;
    reg          in_burst;

    assign s_axi_awready = !in_burst && !s_axi_bvalid;
    assign s_axi_wready  = in_burst;
    assign s_axi_bresp   = 2'b00;

    wire do_w = in_burst && s_axi_wvalid;

    // byte-enable write (inferred as block RAM with byte write enables)
    always @(posedge clk) begin
        if (do_w) begin
            if (s_axi_wstrb[0]) mem[waddr][7:0]   <= s_axi_wdata[7:0];
            if (s_axi_wstrb[1]) mem[waddr][15:8]  <= s_axi_wdata[15:8];
            if (s_axi_wstrb[2]) mem[waddr][23:16] <= s_axi_wdata[23:16];
            if (s_axi_wstrb[3]) mem[waddr][31:24] <= s_axi_wdata[31:24];
        end
        rd_data <= mem[rd_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            waddr        <= {AW{1'b0}};
            in_burst     <= 1'b0;
            s_axi_bvalid <= 1'b0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin
                waddr    <= s_axi_awaddr[AW+1:2];
                in_burst <= 1'b1;
            end
            if (do_w) begin
                waddr <= waddr + 1'b1;
                if (s_axi_wlast) begin
                    in_burst     <= 1'b0;
                    s_axi_bvalid <= 1'b1;
                end
            end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end
endmodule

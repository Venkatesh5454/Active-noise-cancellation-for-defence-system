// =============================================================================
// ssc_axi_apb_bridge.v  -  AXI4-Lite slave -> APB master
// -----------------------------------------------------------------------------
// The ARM in the Zynq talks AXI.  Our register bank talks APB (the simple
// peripheral bus used by the IEEE controller papers).  This bridge does the
// same job as the Xilinx "AXI APB Bridge" IP in the architecture slide, but
// is plain Verilog, so the whole path ARM -> registers can be simulated and
// the controller drops into a block design as one module with its address
// assigned automatically.
//
// One transfer at a time:
//   IDLE   wait for a write (AW + W both valid) or a read (AR valid)
//   SETUP  APB setup phase   (PSEL=1, PENABLE=0)
//   ACCESS APB access phase  (PSEL=1, PENABLE=1) until PREADY
//   RESP   give the AXI write response (B) or read data (R)
// =============================================================================
`timescale 1ns / 1ps
module ssc_axi_apb_bridge #(
    parameter AW = 12
) (
    input  wire          aclk,
    input  wire          aresetn,
    // AXI4-Lite slave
    input  wire [AW-1:0] s_axi_awaddr,
    input  wire          s_axi_awvalid,
    output reg           s_axi_awready,
    input  wire [31:0]   s_axi_wdata,
    input  wire          s_axi_wvalid,
    output reg           s_axi_wready,
    output reg  [1:0]    s_axi_bresp,
    output reg           s_axi_bvalid,
    input  wire          s_axi_bready,
    input  wire [AW-1:0] s_axi_araddr,
    input  wire          s_axi_arvalid,
    output reg           s_axi_arready,
    output reg  [31:0]   s_axi_rdata,
    output reg  [1:0]    s_axi_rresp,
    output reg           s_axi_rvalid,
    input  wire          s_axi_rready,
    // APB master
    output reg  [AW-1:0] paddr,
    output reg           psel,
    output reg           penable,
    output reg           pwrite,
    output reg  [31:0]   pwdata,
    input  wire [31:0]   prdata,
    input  wire          pready,
    input  wire          pslverr
);
    localparam IDLE = 2'd0, SETUP = 2'd1, ACCESS = 2'd2, RESP = 2'd3;
    reg [1:0] state;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state         <= IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            s_axi_bvalid  <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rdata   <= 32'd0;
            s_axi_rresp   <= 2'b00;
            s_axi_rvalid  <= 1'b0;
            paddr         <= {AW{1'b0}};
            psel          <= 1'b0;
            penable       <= 1'b0;
            pwrite        <= 1'b0;
            pwdata        <= 32'd0;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_arready <= 1'b0;

            case (state)
                IDLE: begin
                    if (s_axi_awvalid && s_axi_wvalid) begin      // write first
                        s_axi_awready <= 1'b1;
                        s_axi_wready  <= 1'b1;
                        paddr   <= s_axi_awaddr;
                        pwdata  <= s_axi_wdata;
                        pwrite  <= 1'b1;
                        psel    <= 1'b1;
                        state   <= SETUP;
                    end else if (s_axi_arvalid) begin
                        s_axi_arready <= 1'b1;
                        paddr   <= s_axi_araddr;
                        pwrite  <= 1'b0;
                        psel    <= 1'b1;
                        state   <= SETUP;
                    end
                end
                SETUP: begin
                    penable <= 1'b1;
                    state   <= ACCESS;
                end
                ACCESS: begin
                    if (pready) begin
                        psel    <= 1'b0;
                        penable <= 1'b0;
                        if (pwrite) begin
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= pslverr ? 2'b10 : 2'b00;
                        end else begin
                            s_axi_rvalid <= 1'b1;
                            s_axi_rdata  <= prdata;
                            s_axi_rresp  <= pslverr ? 2'b10 : 2'b00;
                        end
                        state <= RESP;
                    end
                end
                RESP: begin
                    if (s_axi_bvalid && s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        state        <= IDLE;
                    end
                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        state        <= IDLE;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

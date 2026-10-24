// =============================================================================
// study_mesh.v  -  W x H mesh NoC with N nodes for the scalability study
// -----------------------------------------------------------------------------
// A thin wrapper around rtl/noc/noc_mesh.v (which is a grid of noc_router
// blocks).  It gives the mesh the same parameter list as study_bus and
// study_xbar, so the testbench and the synthesis scripts can pick the fabric
// with one switch.  From N it chooses the grid and the dest field width:
//
//      N  =  2    4    6    8   12   16   32
//      W  =  2    2    3    4    4    4    8      (columns, x)
//      H  =  1    2    2    2    3    4    4      (rows, y)
//      IDW=  1    2    3    3    4    4    5      (= ceil(log2 N))
//
// The study uses N = 4 (2x2), 8 (4x2) and 16 (4x4).  N = 6 gives the 3x2
// mesh of the Smart Serial Controller.  Any other N must still equal W*H
// with the W from the table.
//
// Head flit: dest = data[DEST_LSB +: IDW] (DEST_LSB = 24 in the study, so a
// 16-node id fits in data[27:24]); prio = data[PRIO_BIT].  Node n sits at
// x = n % W, y = n / W, and packets use XY routing (docs/SPEC.md 3.5).
//
// Timing: 2 clocks per router, so a packet from node s to node d needs
// 2 + 2*hops clocks for its head flit (hops = |dx| + |dy|).
// =============================================================================
`timescale 1ns / 1ps
module study_mesh #(
    parameter N        = 4,          // number of nodes (see the table above)
    parameter DEST_LSB = 24,         // head flit: dest = data[DEST_LSB +: IDW]
    parameter PRIO_BIT = 22,         // head flit: 1 = urgent
    parameter DEPTH    = 4           // router FIFO depth = credits per link
) (
    input  wire            clk, rst_n,
    // node -> fabric
    input  wire [N-1:0]    tx_valid,
    input  wire [N*34-1:0] tx_flit,
    output wire [N-1:0]    tx_credit,
    // fabric -> node
    output wire [N-1:0]    rx_valid,
    output wire [N*34-1:0] rx_flit,
    input  wire [N-1:0]    rx_credit
);
    // grid shape and dest width chosen from N
    localparam W   = (N <= 2) ? N : (N <= 4) ? 2 : (N == 6) ? 3 :
                     (N <= 16) ? 4 : 8;
    localparam H   = N / W;
    localparam IDW = (N <= 2) ? 1 : (N <= 4) ? 2 : (N <= 8) ? 3 : (N <= 16) ? 4 : 5;

    noc_mesh #(
        .W(W), .H(H), .N(N), .IDW(IDW), .DEST_LSB(DEST_LSB),
        .PRIO_BIT(PRIO_BIT), .DEPTH(DEPTH)
    ) u_mesh (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_valid  (tx_valid),
        .tx_flit   (tx_flit),
        .tx_credit (tx_credit),
        .rx_valid  (rx_valid),
        .rx_flit   (rx_flit),
        .rx_credit (rx_credit)
    );
endmodule

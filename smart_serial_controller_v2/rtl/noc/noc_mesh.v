// =============================================================================
// noc_mesh.v  -  W x H mesh of noc_router blocks (docs/SPEC.md 3.6)
// -----------------------------------------------------------------------------
// Router n sits at x = n % W, y = n / W (node id n = y*W + x).  For the
// Smart Serial Controller W = 3, H = 2:
//
//            x=0           x=1           x=2
//   y=0    node 0  <-->  node 1  <-->  node 2
//            ^              ^             ^
//            v              v             v
//   y=1    node 3  <-->  node 4  <-->  node 5
//
// Every <--> is two links, one in each direction.  A link is valid + flit
// one way and credit the other way.  The wiring between neighbours is:
//
//   router(x,y).EAST  out -> router(x+1,y).WEST  in   (credit comes back)
//   router(x,y).WEST  out -> router(x-1,y).EAST  in
//   router(x,y).SOUTH out -> router(x,y+1).NORTH in
//   router(x,y).NORTH out -> router(x,y-1).SOUTH in
//
// The LOCAL port of router n is the node's connection:
//   tx_* (node -> network) feed router n's LOCAL input,
//   rx_* (network -> node) come from router n's LOCAL output.
//
// Boundary ports (no neighbour): in_valid = 0, in_flit = 0 and
// out_credit = 0, so their starting credits are never refilled.  XY routing
// never sends a flit with a valid destination there.
//
// The mesh has no logic of its own; it is only wires.  It is fully
// parameterised (W, H, IDW, DEST_LSB, PRIO_BIT, DEPTH) for the NoC
// scalability study.  N must equal W*H (it only sets the port widths).
// =============================================================================
`timescale 1ns / 1ps
module noc_mesh #(parameter W = 3, parameter H = 2, parameter N = 6,
                  parameter IDW = 3, parameter DEST_LSB = 26,
                  parameter PRIO_BIT = 22, parameter DEPTH = 4) (
    input  wire            clk, rst_n,
    // node -> network (into router LOCAL input)
    input  wire [N-1:0]    tx_valid,
    input  wire [N*34-1:0] tx_flit,
    output wire [N-1:0]    tx_credit,
    // network -> node (from router LOCAL output)
    output wire [N-1:0]    rx_valid,
    output wire [N*34-1:0] rx_flit,
    input  wire [N-1:0]    rx_credit
);
    localparam FW = 34;
    localparam P_LOCAL = 0, P_NORTH = 1, P_EAST = 2, P_SOUTH = 3, P_WEST = 4;

    // Every router port, flattened: router n, port q is bit 5*n + q
    // (and flit bits FW*(5*n + q) +: FW).
    wire [5*N-1:0]    r_in_valid, r_in_credit, r_out_valid, r_out_credit;
    wire [5*N*FW-1:0] r_in_flit, r_out_flit;

    genvar n;
    generate
        for (n = 0; n < N; n = n + 1) begin : g_node
            localparam XN = n % W;
            localparam YN = n / W;

            noc_router #(
                .X(XN), .Y(YN), .W(W), .H(H), .IDW(IDW),
                .DEST_LSB(DEST_LSB), .PRIO_BIT(PRIO_BIT), .DEPTH(DEPTH)
            ) u_router (
                .clk        (clk),
                .rst_n      (rst_n),
                .in_valid   (r_in_valid  [5*n +: 5]),
                .in_flit    (r_in_flit   [5*FW*n +: 5*FW]),
                .in_credit  (r_in_credit [5*n +: 5]),
                .out_valid  (r_out_valid [5*n +: 5]),
                .out_flit   (r_out_flit  [5*FW*n +: 5*FW]),
                .out_credit (r_out_credit[5*n +: 5])
            );

            // ---- LOCAL port: the node itself ----
            assign r_in_valid[5*n + P_LOCAL]               = tx_valid[n];
            assign r_in_flit[FW*(5*n + P_LOCAL) +: FW]     = tx_flit[FW*n +: FW];
            assign tx_credit[n]                            = r_in_credit[5*n + P_LOCAL];
            assign rx_valid[n]                             = r_out_valid[5*n + P_LOCAL];
            assign rx_flit[FW*n +: FW]                     = r_out_flit[FW*(5*n + P_LOCAL) +: FW];
            assign r_out_credit[5*n + P_LOCAL]             = rx_credit[n];

            // ---- NORTH port <-> SOUTH port of router n-W ----
            if (YN > 0) begin : g_north
                assign r_in_valid[5*n + P_NORTH]           = r_out_valid[5*(n-W) + P_SOUTH];
                assign r_in_flit[FW*(5*n + P_NORTH) +: FW] = r_out_flit[FW*(5*(n-W) + P_SOUTH) +: FW];
                assign r_out_credit[5*n + P_NORTH]         = r_in_credit[5*(n-W) + P_SOUTH];
            end else begin : g_north_edge
                assign r_in_valid[5*n + P_NORTH]           = 1'b0;
                assign r_in_flit[FW*(5*n + P_NORTH) +: FW] = {FW{1'b0}};
                assign r_out_credit[5*n + P_NORTH]         = 1'b0;
            end

            // ---- EAST port <-> WEST port of router n+1 ----
            if (XN < W - 1) begin : g_east
                assign r_in_valid[5*n + P_EAST]            = r_out_valid[5*(n+1) + P_WEST];
                assign r_in_flit[FW*(5*n + P_EAST) +: FW]  = r_out_flit[FW*(5*(n+1) + P_WEST) +: FW];
                assign r_out_credit[5*n + P_EAST]          = r_in_credit[5*(n+1) + P_WEST];
            end else begin : g_east_edge
                assign r_in_valid[5*n + P_EAST]            = 1'b0;
                assign r_in_flit[FW*(5*n + P_EAST) +: FW]  = {FW{1'b0}};
                assign r_out_credit[5*n + P_EAST]          = 1'b0;
            end

            // ---- SOUTH port <-> NORTH port of router n+W ----
            if (YN < H - 1) begin : g_south
                assign r_in_valid[5*n + P_SOUTH]           = r_out_valid[5*(n+W) + P_NORTH];
                assign r_in_flit[FW*(5*n + P_SOUTH) +: FW] = r_out_flit[FW*(5*(n+W) + P_NORTH) +: FW];
                assign r_out_credit[5*n + P_SOUTH]         = r_in_credit[5*(n+W) + P_NORTH];
            end else begin : g_south_edge
                assign r_in_valid[5*n + P_SOUTH]           = 1'b0;
                assign r_in_flit[FW*(5*n + P_SOUTH) +: FW] = {FW{1'b0}};
                assign r_out_credit[5*n + P_SOUTH]         = 1'b0;
            end

            // ---- WEST port <-> EAST port of router n-1 ----
            if (XN > 0) begin : g_west
                assign r_in_valid[5*n + P_WEST]            = r_out_valid[5*(n-1) + P_EAST];
                assign r_in_flit[FW*(5*n + P_WEST) +: FW]  = r_out_flit[FW*(5*(n-1) + P_EAST) +: FW];
                assign r_out_credit[5*n + P_WEST]          = r_in_credit[5*(n-1) + P_EAST];
            end else begin : g_west_edge
                assign r_in_valid[5*n + P_WEST]            = 1'b0;
                assign r_in_flit[FW*(5*n + P_WEST) +: FW]  = {FW{1'b0}};
                assign r_out_credit[5*n + P_WEST]          = 1'b0;
            end
        end
    endgenerate
endmodule

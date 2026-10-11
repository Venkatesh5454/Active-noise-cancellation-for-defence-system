// =============================================================================
// study_bus.v  -  shared bus with N nodes for the NoC scalability study
// -----------------------------------------------------------------------------
// One of the three fabrics compared in noc_study/ (bus, crossbar, mesh).  It
// has exactly the same node interface as noc_mesh.v, so the same traffic
// generator drives all three:
//
//   tx_valid/tx_flit  -> into the fabric, tx_credit <- out of the fabric
//   rx_valid/rx_flit  <- out of the fabric, rx_credit -> into the fabric
//
// Flits are 34 bits {kind[1:0], data[31:0]} (docs/SPEC.md 3.1).  The study
// head flit keeps the destination at data[DEST_LSB +: IDW] with DEST_LSB = 24;
// IDW = ceil(log2 N) is worked out from N.
//
//      node 0 tx -> [FIFO] --\                       /--> rx_valid[0]
//      node 1 tx -> [FIFO] ---+--> arbiter + mux --> bus register ---> ...
//        ...                  |    (one flit/clock)   (flit, dest)  \--> rx_valid[N-1]
//      node N-1 tx -> [FIFO] -/                       rx_flit = the same bus for all
//
// How it works:
//   * Every node has a DEPTH-flit input FIFO (first-word fall-through), the
//     same buffer as a router input, so the tx link has normal credits.
//   * ONE shared medium: a single bus register {valid, flit, dest}.  At most
//     one flit crosses per clock for the whole system.  rx_flit is the same
//     wire for every node; rx_valid is decoded from the bus dest.
//   * The arbiter grants one node at a time.  The granted node keeps the
//     bus until its TAIL has passed (wormhole), a SINGLE flit does not lock.
//   * A flit needs 2 clocks: clock 1 it is written into the input FIFO,
//     clock 2 it wins the bus and is copied into the bus register, so
//     rx_valid goes high one clock later.
//   * One credit counter per destination node (its rx buffer has DEPTH
//     slots).  A flit is only put on the bus while its destination has a
//     credit.
//
// Arbitration: the nodes whose front HEAD/SINGLE flit has a destination with
// a free slot compete.  prio = flit[PRIO_BIT] = 1 wins first, then it is
// round-robin (the search starts after the node that won last time).
//
// Design choices:
//   * Heads whose destination is full do not take part, so one blocked
//     receiver does not stop the whole bus between packets.  Inside a
//     packet the bus does wait for the receiver (that is wormhole).
//   * A dest of N or more (only possible when N is not a power of two) goes
//     back to the sender, like the LOCAL port rule of noc_router.
//   * A stray BODY/TAIL at the front of an idle input is thrown away (and its
//     credit returned), the same rule as noc_router and noc_pkt_rx.
//   * The round-robin pick uses the usual "mask" trick: first look for the
//     lowest candidate above the last winner, else take the lowest one.
//   * The FIFO fronts are kept in a wire array, so f_front[sel] is a plain
//     N:1 multiplexer (yosys maps a variable part-select [34*sel +: 34]
//     to a much bigger barrel shifter).
// =============================================================================
`timescale 1ns / 1ps
module study_bus #(
    parameter N        = 4,          // number of nodes (2 .. 32)
    parameter DEST_LSB = 24,         // head flit: dest = data[DEST_LSB +: IDW]
    parameter PRIO_BIT = 22,         // head flit: 1 = urgent
    parameter DEPTH    = 4           // input FIFO depth = credits per link
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
    localparam FW  = 34;
    // bits needed to number a node (also the dest field width)
    localparam IW  = (N <= 2) ? 1 : (N <= 4) ? 2 : (N <= 8) ? 3 : (N <= 16) ? 4 : 5;
    localparam IDW = IW;
    // FIFO pointer width (0 .. DEPTH-1) and counter width (0 .. DEPTH)
    localparam AW  = (DEPTH <= 2) ? 1 : (DEPTH <= 4) ? 2 : (DEPTH <= 8) ? 3 :
                     (DEPTH <= 16) ? 4 : 5;
    localparam CW  = AW + 1;
    localparam [1:0] K_HEAD = 2'b00, K_TAIL = 2'b10, K_SINGLE = 2'b11;
    /* verilator lint_off WIDTHTRUNC */
    localparam [CW-1:0] FULL = DEPTH;    // FIFO level when full = start credits
    localparam [AW-1:0] LAST = DEPTH - 1;
    localparam [IW:0]   NN   = N;        // first dest value that does not exist
    localparam [IW-1:0] TOP  = N - 1;    // highest node number
    /* verilator lint_on WIDTHTRUNC */

    genvar p;

    // ------------------------------------------------------------------
    // Round-robin pick: lowest set bit of cand above "last", else the
    // lowest set bit of cand
    // ------------------------------------------------------------------
    function [IW-1:0] rr_pick;
        input [N-1:0]  cand;
        input [IW-1:0] last;
        integer k;
        begin
            rr_pick = {IW{1'b0}};
            for (k = N - 1; k >= 0; k = k - 1)
                if (cand[k]) rr_pick = k[IW-1:0];
            for (k = N - 1; k >= 0; k = k - 1)
                if (cand[k] && (k > {{(32-IW){1'b0}}, last})) rr_pick = k[IW-1:0];
        end
    endfunction


    // ------------------------------------------------------------------
    // Bus state
    // ------------------------------------------------------------------
    reg           locked;            // a packet owns the bus
    reg  [IW-1:0] owner;             // ... coming from this node
    reg  [IW-1:0] ldest;             // ... going to this node
    reg  [IW-1:0] last;              // last node granted (round-robin)
    reg           bvalid;            // the bus register
    reg  [FW-1:0] bflit;
    reg  [IW-1:0] bdest;

    wire [N-1:0]    f_ne;            // input FIFO p is not empty
    wire [FW-1:0]   f_front [0:N-1];  // flit at the front of FIFO p
    wire [N-1:0]    f_head;          // that flit is a HEAD or a SINGLE
    wire [N-1:0]    f_prio;          // its prio bit
    wire [IW-1:0]   f_route [0:N-1];  // the node its dest wants
    wire [N-1:0]    cred_ok;         // receiver d has a free buffer slot
    wire [N-1:0]    req;             // node p asks for the bus

    // ------------------------------------------------------------------
    // Arbiter and bus multiplexer (one flit per clock in total)
    // ------------------------------------------------------------------
    wire [N-1:0]  req_hi = req & f_prio;
    wire [N-1:0]  cand   = (|req_hi) ? req_hi : req;
    wire [IW-1:0] sel    = locked ? owner : rr_pick(cand, last);
    wire [IW-1:0] dsel   = locked ? ldest : f_route[sel];
    wire          go     = locked ? (f_ne[owner] && cred_ok[ldest]) : (|cand);
    wire [FW-1:0] flit   = f_front[sel];
    wire [1:0]    kind   = flit[FW-1:FW-2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            locked <= 1'b0;
            owner  <= {IW{1'b0}};
            ldest  <= {IW{1'b0}};
            last   <= TOP;                   // so node 0 is looked at first
            bvalid <= 1'b0;
            bflit  <= {FW{1'b0}};
            bdest  <= {IW{1'b0}};
        end else begin
            bvalid <= go;
            if (go) begin
                bflit <= flit;
                bdest <= dsel;
            end
            // wormhole lock: the bus stays with this node until the TAIL
            if (go) begin
                if (!locked) begin
                    last <= sel;
                    if (kind == K_HEAD) begin
                        locked <= 1'b1;
                        owner  <= sel;
                        ldest  <= dsel;
                    end
                end else if (kind == K_TAIL || kind == K_SINGLE) begin
                    locked <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Per node: input FIFO, receiver credit counter, rx decode
    // ------------------------------------------------------------------
    generate
        for (p = 0; p < N; p = p + 1) begin : g_node
            /* verilator lint_off WIDTHTRUNC */
            localparam [IW-1:0] PI = p;
            /* verilator lint_on WIDTHTRUNC */

            // ---------------- input FIFO ----------------
            reg  [FW-1:0]  mem [0:DEPTH-1];
            reg  [AW-1:0]  wptr, rptr;
            reg  [CW-1:0]  cnt;          // flits in the FIFO (0 .. DEPTH)
            reg            cr;           // tx_credit register
            wire [FW-1:0]  front = mem[rptr];
            wire [IDW-1:0] dest  = front[DEST_LSB +: IDW];
            wire [1:0]     fkind = front[FW-1:FW-2];
            wire           wr    = tx_valid[p] && (cnt != FULL);
            wire           mine  = locked && (owner == PI);
            wire           drop;
            wire           pop;

            assign f_ne[p]             = (cnt != {CW{1'b0}});
            assign f_front[p]          = front;
            assign f_head[p]           = (fkind == K_HEAD) || (fkind == K_SINGLE);
            assign f_prio[p]           = front[PRIO_BIT];
            assign f_route[p]          = ({1'b0, dest} >= NN) ? PI : dest;
            assign req[p]              = f_ne[p] && f_head[p] &&
                                         cred_ok[f_route[p]];

            // stray BODY/TAIL at an idle input: throw it away
            assign drop = f_ne[p] && !f_head[p] && !mine;
            assign pop  = drop || (go && (sel == PI));

            // FIFO memory (no reset, so it can be distributed RAM)
            always @(posedge clk) begin
                if (wr) mem[wptr] <= tx_flit[FW*p +: FW];
            end

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    wptr <= {AW{1'b0}};
                    rptr <= {AW{1'b0}};
                    cnt  <= {CW{1'b0}};
                    cr   <= 1'b0;
                end else begin
                    if (wr)  wptr <= (wptr == LAST) ? {AW{1'b0}} : wptr + 1'b1;
                    if (pop) rptr <= (rptr == LAST) ? {AW{1'b0}} : rptr + 1'b1;
                    case ({wr, pop})
                        2'b10:   cnt <= cnt + 1'b1;
                        2'b01:   cnt <= cnt - 1'b1;
                        default: cnt <= cnt;
                    endcase
                    cr <= pop;               // one credit per flit that left
                end
            end

            assign tx_credit[p] = cr;

            // ---------------- receiver side ----------------
            reg  [CW-1:0] cred;          // free slots in node p's rx buffer
            wire          sent = go && (dsel == PI);

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cred <= FULL;
                end else begin
                    case ({rx_credit[p], sent})
                        2'b10:   cred <= cred + 1'b1;
                        2'b01:   cred <= cred - 1'b1;
                        default: cred <= cred;
                    endcase
                end
            end

            assign cred_ok[p]          = (cred != {CW{1'b0}});
            assign rx_valid[p]         = bvalid && (bdest == PI);
            assign rx_flit[FW*p +: FW] = bflit;      // one shared bus
        end
    endgenerate
endmodule

// =============================================================================
// noc_router.v  -  5-port wormhole router for the 2-D mesh network-on-chip
// -----------------------------------------------------------------------------
// One router sits at every node (X, Y) of the mesh (docs/SPEC.md 3.4, 3.5).
// It has five ports, and each port has an input side (valid + flit in,
// credit out) and an output side (valid + flit out, credit in):
//
//     0 LOCAL  the node's own network interface
//     1 NORTH  to (x, y-1)        2 EAST  to (x+1, y)
//     3 SOUTH  to (x, y+1)        4 WEST  to (x-1, y)
//
// How a flit moves through the router (2 clocks per hop):
//   clock 1  in_valid: the flit is written into the DEPTH-entry FIFO of its
//            input port (first-word fall-through, so the front is visible
//            straight away)
//   clock 2  the FIFO front is routed and arbitrated; the winner is copied
//            into the output register, so out_valid/out_flit go high on the
//            next clock (= clock 1 of the next router)
//
// Routing (XY, dimension order): dest = flit[DEST_LSB +: IDW],
// dx = dest % W, dy = dest / W.  First move along x (EAST/WEST), then along
// y (SOUTH/NORTH), then LOCAL.  A dest of W*H or more goes LOCAL.  The whole
// decision is a small constant table (one entry per possible dest) that is
// built at elaboration time, so no divider is synthesised.
//
// Wormhole switching: a HEAD flit that wins an output locks that output to
// its input.  BODY flits follow on the locked path and the TAIL unlocks it.
// A SINGLE flit (header-only packet) passes without locking.  So the flits
// of two packets never interleave on one output.
//
// Arbitration (per free output): the inputs whose front HEAD/SINGLE flit
// wants this output compete.  If any of them has prio = flit[PRIO_BIT] = 1,
// only those take part.  Among the rest it is round-robin: the search starts
// at the input after the one that won this output last time.
//
// Flow control (credits): each output keeps a counter of the free slots in
// the buffer downstream.  It starts at DEPTH, goes -1 for each flit sent and
// +1 for each out_credit pulse; a flit is only sent while it is above 0.
// Every flit that leaves input FIFO p makes in_credit[p] pulse (one clock
// later, it is a register).
//
// Design choices:
//   * in_credit, out_valid and out_flit are registers: there is no
//     combinational path from one router to the next.  The credit loop is
//     then 4 clocks (send, write, pop, credit), so DEPTH = 4 is exactly
//     enough for a stream of 1 flit per clock on every output.
//   * At most one flit leaves each input and each output per clock; the
//     five outputs work in parallel.
//   * A BODY/TAIL flit at the front of an input that owns no output can only
//     come from a broken sender.  It is thrown away (and its credit is
//     returned) so that input never blocks - the same rule as noc_pkt_rx.
//   * A flit that arrives at a full FIFO (the sender ignored the credits) is
//     ignored.  A correct sender never does that.
//   * One round-robin pointer per output, shared by both priority classes.
//   * The dbg_* wires are not used inside; testbenches read them to check
//     that every credit counter is back at DEPTH at the end of a test.
// =============================================================================
`timescale 1ns / 1ps
module noc_router #(
    parameter X = 0, parameter Y = 0, parameter W = 3, parameter H = 2,
    parameter IDW = 3, parameter DEST_LSB = 26, parameter PRIO_BIT = 22,
    parameter DEPTH = 4
) (
    input  wire            clk, rst_n,
    input  wire [4:0]      in_valid,     // ports: 0 LOCAL 1 NORTH 2 EAST 3 SOUTH 4 WEST
    input  wire [5*34-1:0] in_flit,      // port p at [34*p +: 34]
    output wire [4:0]      in_credit,    // pulse: a slot of input buffer p was freed
    output wire [4:0]      out_valid,    // registered
    output wire [5*34-1:0] out_flit,     // registered
    input  wire [4:0]      out_credit    // pulse: the downstream buffer freed a slot
);
    localparam NP = 5;                   // number of ports
    localparam FW = 34;                  // flit width {kind[1:0], data[31:0]}
    localparam [2:0] P_LOCAL = 3'd0, P_NORTH = 3'd1, P_EAST = 3'd2,
                     P_SOUTH = 3'd3, P_WEST  = 3'd4;
    localparam [1:0] K_HEAD = 2'b00, K_TAIL = 2'b10, K_SINGLE = 2'b11;

    // FIFO pointer width (enough for 0 .. DEPTH-1) and counter width (0 .. DEPTH)
    localparam AW = (DEPTH <= 2) ? 1 : (DEPTH <= 4) ? 2 : (DEPTH <= 8) ? 3 :
                    (DEPTH <= 16) ? 4 : 5;
    localparam CW = AW + 1;
    /* verilator lint_off WIDTHTRUNC */
    localparam [CW-1:0] FULL = DEPTH;    // FIFO level when full = start credits
    localparam [AW-1:0] LAST = DEPTH - 1;
    /* verilator lint_on WIDTHTRUNC */
    localparam NDEST = 1 << IDW;         // number of possible dest values

    genvar d, p, q;

    // ------------------------------------------------------------------
    // Route table: rtab[3*dest +: 3] = output port for that dest
    // ------------------------------------------------------------------
    wire [3*NDEST-1:0] rtab;
    generate
        for (d = 0; d < NDEST; d = d + 1) begin : g_rt
            localparam integer DX = d % W;
            localparam integer DY = d / W;
            localparam [2:0] RP = (d >= W * H) ? P_LOCAL :
                                  (DX > X)     ? P_EAST  :
                                  (DX < X)     ? P_WEST  :
                                  (DY > Y)     ? P_SOUTH :
                                  (DY < Y)     ? P_NORTH : P_LOCAL;
            assign rtab[3*d +: 3] = RP;
        end
    endgenerate

    // ------------------------------------------------------------------
    // Round-robin pick: the first set bit of cand, starting after "last"
    // ------------------------------------------------------------------
    function [2:0] rr_pick;
        input [4:0] cand;
        input [2:0] last;
        integer   k;
        reg [2:0] idx;
        reg       found;
        begin
            rr_pick = 3'd0;
            found   = 1'b0;
            idx     = (last >= 3'd4) ? 3'd0 : last + 3'd1;
            for (k = 0; k < NP; k = k + 1) begin
                if (!found && cand[idx]) begin
                    rr_pick = idx;
                    found   = 1'b1;
                end
                idx = (idx >= 3'd4) ? 3'd0 : idx + 3'd1;
            end
        end
    endfunction

    // ------------------------------------------------------------------
    // Signals shared between the input side and the output side
    // ------------------------------------------------------------------
    wire [NP-1:0]    f_ne;       // input FIFO p is not empty
    wire [NP*FW-1:0] f_front;    // flit at the front of FIFO p
    wire [NP-1:0]    f_head;     // that flit is a HEAD or a SINGLE
    wire [NP-1:0]    f_prio;     // its prio bit
    wire [3*NP-1:0]  f_route;    // the output its dest wants
    wire [NP-1:0]    f_busy;     // input p owns a (locked) output
    wire [NP-1:0]    o_locked;   // output o is locked to an input
    wire [3*NP-1:0]  o_owner;    // ... to this input
    wire [NP-1:0]    o_go;       // output o sends a flit this clock
    wire [3*NP-1:0]  o_sel;      // ... taken from this input

    wire [8*NP-1:0]  dbg_level;  // FIFO level per input   (testbench only)
    wire [8*NP-1:0]  dbg_cred;   // credits per output     (testbench only)

    // ------------------------------------------------------------------
    // Input side: one FIFO per port
    // ------------------------------------------------------------------
    generate
        for (p = 0; p < NP; p = p + 1) begin : g_in
            localparam [2:0] PI = p;

            reg  [FW-1:0]  mem [0:DEPTH-1];
            reg  [AW-1:0]  wptr, rptr;
            reg  [CW-1:0]  cnt;          // flits in the FIFO (0 .. DEPTH)
            reg            cr;           // in_credit register
            wire [FW-1:0]  front = mem[rptr];
            wire [IDW-1:0] dest  = front[DEST_LSB +: IDW];
            wire [1:0]     kind  = front[FW-1:FW-2];
            wire           wr    = in_valid[p] && (cnt != FULL);
            wire [NP-1:0]  own;          // output q is locked to this input
            wire [NP-1:0]  take;         // output q takes our front flit
            wire           drop;
            wire           pop;

            for (q = 0; q < NP; q = q + 1) begin : g_m
                assign own[q]  = o_locked[q] && (o_owner[3*q +: 3] == PI);
                assign take[q] = o_go[q] && (o_sel[3*q +: 3] == PI);
            end

            assign f_ne[p]            = (cnt != {CW{1'b0}});
            assign f_front[FW*p +: FW] = front;
            assign f_head[p]          = (kind == K_HEAD) || (kind == K_SINGLE);
            assign f_prio[p]          = front[PRIO_BIT];
            assign f_route[3*p +: 3]  = rtab[3*dest +: 3];
            assign f_busy[p]          = |own;

            // stray BODY/TAIL at an idle input: throw it away
            assign drop = f_ne[p] && !f_head[p] && !f_busy[p];
            assign pop  = drop || (|take);

            // FIFO memory (no reset, so it can be distributed RAM)
            always @(posedge clk) begin
                if (wr) mem[wptr] <= in_flit[FW*p +: FW];
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

            assign in_credit[p]        = cr;
            assign dbg_level[8*p +: 8] = {{(8-CW){1'b0}}, cnt};
        end
    endgenerate

    // ------------------------------------------------------------------
    // Output side: lock, arbiter, credit counter and output register
    // ------------------------------------------------------------------
    generate
        for (p = 0; p < NP; p = p + 1) begin : g_out
            localparam [2:0] OI = p;

            reg           locked;        // in the middle of a packet
            reg  [2:0]    owner;         // input that owns this output
            reg  [2:0]    last;          // last input granted (round-robin)
            reg  [CW-1:0] cred;          // free slots downstream
            reg           ovalid;
            reg  [FW-1:0] oflit;
            reg  [FW-1:0] flit;          // flit chosen this clock
            wire [NP-1:0] req;           // free inputs whose head wants us

            for (q = 0; q < NP; q = q + 1) begin : g_r
                assign req[q] = f_ne[q] && f_head[q] && !f_busy[q] &&
                                (f_route[3*q +: 3] == OI);
            end

            // urgent requests first, then round-robin among equals
            wire [NP-1:0] req_hi = req & f_prio;
            wire [NP-1:0] cand   = (|req_hi) ? req_hi : req;
            wire [2:0]    sel    = locked ? owner : rr_pick(cand, last);
            wire          have   = locked ? f_ne[owner] : (|cand);
            wire          go     = have && (cred != {CW{1'b0}});
            wire [1:0]    kind   = flit[FW-1:FW-2];

            always @(*) begin
                case (sel)
                    3'd0:    flit = f_front[0*FW +: FW];
                    3'd1:    flit = f_front[1*FW +: FW];
                    3'd2:    flit = f_front[2*FW +: FW];
                    3'd3:    flit = f_front[3*FW +: FW];
                    default: flit = f_front[4*FW +: FW];
                endcase
            end

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    locked <= 1'b0;
                    owner  <= 3'd0;
                    last   <= 3'd4;          // so input 0 is looked at first
                    cred   <= FULL;
                    ovalid <= 1'b0;
                    oflit  <= {FW{1'b0}};
                end else begin
                    ovalid <= go;
                    if (go) oflit <= flit;

                    // credit counter: +1 per returned credit, -1 per flit sent
                    case ({out_credit[p], go})
                        2'b10:   cred <= cred + 1'b1;
                        2'b01:   cred <= cred - 1'b1;
                        default: cred <= cred;
                    endcase

                    // wormhole lock
                    if (go) begin
                        if (!locked) begin
                            last <= sel;
                            if (kind == K_HEAD) begin
                                locked <= 1'b1;
                                owner  <= sel;
                            end
                        end else if (kind == K_TAIL || kind == K_SINGLE) begin
                            locked <= 1'b0;
                        end
                    end
                end
            end

            assign o_locked[p]          = locked;
            assign o_owner[3*p +: 3]    = owner;
            assign o_go[p]              = go;
            assign o_sel[3*p +: 3]      = sel;
            assign out_valid[p]         = ovalid;
            assign out_flit[FW*p +: FW] = oflit;
            assign dbg_cred[8*p +: 8]   = {{(8-CW){1'b0}}, cred};
        end
    endgenerate
endmodule

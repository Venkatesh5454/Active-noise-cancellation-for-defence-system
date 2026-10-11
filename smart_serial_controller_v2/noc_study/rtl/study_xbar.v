// =============================================================================
// study_xbar.v  -  N x N crossbar switch for the NoC scalability study
// -----------------------------------------------------------------------------
// One of the three fabrics compared in noc_study/ (bus, crossbar, mesh).  It
// has exactly the same node interface as noc_mesh.v, so the same traffic
// generator drives all three:
//
//   tx_valid/tx_flit  -> into the fabric, tx_credit <- out of the fabric
//   rx_valid/rx_flit  <- out of the fabric, rx_credit -> into the fabric
//
// Flits are 34 bits {kind[1:0], data[31:0]} (docs/SPEC.md 3.1).  The study
// head flit keeps the destination at data[DEST_LSB +: IDW] with DEST_LSB = 24,
// so up to 16 nodes fit; IDW = ceil(log2 N) is worked out from N.
//
// How it works (it is one big router with N ports and no neighbours):
//   * Every input has a DEPTH-flit FIFO (first-word fall-through), exactly
//     like a router input.  Each flit that leaves it makes tx_credit pulse.
//   * Every output has its own arbiter, wormhole lock, credit counter and
//     output register, exactly like a router output.
//   * A flit needs 2 clocks to cross:  clock 1 it is written into the input
//     FIFO,  clock 2 it wins the arbiter and is copied into the output
//     register, so rx_valid goes high one clock later.
//   * All N outputs work in parallel: up to N flits move per clock, one per
//     output (and at most one per input).
//
// Arbitration per free output: the inputs whose front HEAD/SINGLE flit wants
// this output compete.  prio = flit[PRIO_BIT] = 1 wins first, then it is
// round-robin (the search starts after the input that won last time).  A
// HEAD locks the output to its input until the TAIL has passed (wormhole),
// a SINGLE passes without locking.
//
// Design choices:
//   * A dest of N or more (only possible when N is not a power of two) goes
//     back to the sender, like the LOCAL port rule of noc_router.
//   * "Input p is in the middle of a packet" is kept in one register per
//     input (set when a HEAD leaves, cleared when the TAIL leaves).  The
//     router works it out from the output owners instead; the result is the
//     same but the register saves N*N comparators at N = 16.
//   * The round-robin pick uses the usual "mask" trick: first look for the
//     lowest candidate above the last winner, else take the lowest one.
//   * The FIFO fronts are kept in a wire array, so f_front[sel] is a plain
//     N:1 multiplexer (yosys maps a variable part-select [34*sel +: 34]
//     to a much bigger barrel shifter).
//   * A stray BODY/TAIL at the front of an idle input is thrown away (and its
//     credit returned), the same rule as noc_router and noc_pkt_rx.
//   * The FIFO memory has no reset, so it maps to distributed RAM.
// =============================================================================
`timescale 1ns / 1ps
module study_xbar #(
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

    genvar p, q;

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
    // Signals shared between the input side and the output side
    // ------------------------------------------------------------------
    wire [N-1:0]    f_ne;        // input FIFO p is not empty
    wire [FW-1:0]   f_front [0:N-1];  // flit at the front of FIFO p
    wire [N-1:0]    f_head;      // that flit is a HEAD or a SINGLE
    wire [N-1:0]    f_prio;      // its prio bit
    wire [IW-1:0]   f_route [0:N-1];  // the output its dest wants
    wire [N-1:0]    f_busy;      // input p is in the middle of a packet
    wire [N-1:0]    o_go;        // output o sends a flit this clock
    wire [N*IW-1:0] o_sel;       // ... taken from this input

    // ------------------------------------------------------------------
    // Input side: one DEPTH-flit FIFO per node
    // ------------------------------------------------------------------
    generate
        for (p = 0; p < N; p = p + 1) begin : g_in
            /* verilator lint_off WIDTHTRUNC */
            localparam [IW-1:0] PI = p;
            /* verilator lint_on WIDTHTRUNC */

            reg  [FW-1:0]  mem [0:DEPTH-1];
            reg  [AW-1:0]  wptr, rptr;
            reg  [CW-1:0]  cnt;          // flits in the FIFO (0 .. DEPTH)
            reg            cr;           // tx_credit register
            reg            busy;         // a HEAD has left, its TAIL not yet
            wire [FW-1:0]  front = mem[rptr];
            wire [IDW-1:0] dest  = front[DEST_LSB +: IDW];
            wire [1:0]     kind  = front[FW-1:FW-2];
            wire           wr    = tx_valid[p] && (cnt != FULL);
            wire [N-1:0]   take;         // output q takes our front flit
            wire           taken = |take;
            wire           drop;
            wire           pop;

            for (q = 0; q < N; q = q + 1) begin : g_t
                assign take[q] = o_go[q] && (o_sel[IW*q +: IW] == PI);
            end

            assign f_ne[p]             = (cnt != {CW{1'b0}});
            assign f_front[p]          = front;
            assign f_head[p]           = (kind == K_HEAD) || (kind == K_SINGLE);
            assign f_prio[p]           = front[PRIO_BIT];
            assign f_route[p]          = ({1'b0, dest} >= NN) ? PI : dest;
            assign f_busy[p]           = busy;

            // stray BODY/TAIL at an idle input: throw it away
            assign drop = f_ne[p] && !f_head[p] && !busy;
            assign pop  = drop || taken;

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
                    busy <= 1'b0;
                end else begin
                    if (wr)  wptr <= (wptr == LAST) ? {AW{1'b0}} : wptr + 1'b1;
                    if (pop) rptr <= (rptr == LAST) ? {AW{1'b0}} : rptr + 1'b1;
                    case ({wr, pop})
                        2'b10:   cnt <= cnt + 1'b1;
                        2'b01:   cnt <= cnt - 1'b1;
                        default: cnt <= cnt;
                    endcase
                    cr <= pop;               // one credit per flit that left
                    // packet state: busy from HEAD to TAIL
                    if (taken) begin
                        if (kind == K_HEAD)
                            busy <= 1'b1;
                        else if (kind == K_TAIL || kind == K_SINGLE)
                            busy <= 1'b0;
                    end
                end
            end

            assign tx_credit[p] = cr;
        end
    endgenerate

    // ------------------------------------------------------------------
    // Output side: lock, arbiter, credit counter and output register
    // ------------------------------------------------------------------
    generate
        for (p = 0; p < N; p = p + 1) begin : g_out
            /* verilator lint_off WIDTHTRUNC */
            localparam [IW-1:0] OI = p;
            /* verilator lint_on WIDTHTRUNC */

            reg           locked;        // in the middle of a packet
            reg  [IW-1:0] owner;         // input that owns this output
            reg  [IW-1:0] last;          // last input granted (round-robin)
            reg  [CW-1:0] cred;          // free slots in the node's buffer
            reg           ovalid;
            reg  [FW-1:0] oflit;
            wire [N-1:0]  req;           // free inputs whose head wants us

            for (q = 0; q < N; q = q + 1) begin : g_r
                assign req[q] = f_ne[q] && f_head[q] && !f_busy[q] &&
                                (f_route[q] == OI);
            end

            // urgent requests first, then round-robin among equals
            wire [N-1:0]  req_hi = req & f_prio;
            wire [N-1:0]  cand   = (|req_hi) ? req_hi : req;
            wire [IW-1:0] sel    = locked ? owner : rr_pick(cand, last);
            wire          have   = locked ? f_ne[owner] : (|cand);
            wire          go     = have && (cred != {CW{1'b0}});
            wire [FW-1:0] flit   = f_front[sel];
            wire [1:0]    kind   = flit[FW-1:FW-2];

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    locked <= 1'b0;
                    owner  <= {IW{1'b0}};
                    last   <= TOP;           // so input 0 is looked at first
                    cred   <= FULL;
                    ovalid <= 1'b0;
                    oflit  <= {FW{1'b0}};
                end else begin
                    ovalid <= go;
                    if (go) oflit <= flit;

                    // credit counter: +1 per returned credit, -1 per flit sent
                    case ({rx_credit[p], go})
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

            assign o_go[p]             = go;
            assign o_sel[IW*p +: IW]   = sel;
            assign rx_valid[p]         = ovalid;
            assign rx_flit[FW*p +: FW] = oflit;
        end
    endgenerate
endmodule

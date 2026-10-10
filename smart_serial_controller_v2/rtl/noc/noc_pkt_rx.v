// =============================================================================
// noc_pkt_rx.v  -  turns NoC flits back into "header + bytes" (packet receiver)
// -----------------------------------------------------------------------------
// The other half of noc_pkt_tx.v.  Flits from the router go into a small
// DEPTH-entry buffer (this is the buffer the router's credits count).  The
// node then sees:
//
//   1. hdr_valid with the header fields of the packet at the front.
//      It takes the header with hdr_ready.
//   2. len payload bytes on b_data/b_valid, each taken with b_ready.
//      b_last marks the final byte.
//
// Every time a flit leaves the buffer, in_credit pulses once so the router
// knows a slot is free again.
//
// Self-repair: a BODY/TAIL flit that shows up where a header is expected
// (which a correct sender never produces) is thrown away, so the receiver
// always gets back in step at the next HEAD/SINGLE flit.
// =============================================================================
`timescale 1ns / 1ps
module noc_pkt_rx #(
    parameter DEPTH = 4
) (
    input  wire        clk,
    input  wire        rst_n,
    // from the network
    input  wire        in_valid,
    input  wire [33:0] in_flit,
    output reg         in_credit,
    // header of the packet at the front
    output wire        hdr_valid,
    output wire [2:0]  ptype,
    output wire [2:0]  dest,
    output wire [2:0]  src,
    output wire        prio,
    output wire [5:0]  len,
    output wire [7:0]  tag,
    output wire [7:0]  arg,
    input  wire        hdr_ready,
    // payload bytes
    output wire        b_valid,
    output wire [7:0]  b_data,
    output wire        b_last,
    input  wire        b_ready
);
    localparam K_HEAD = 2'b00, K_SINGLE = 2'b11;
    // pointer width: enough bits to count 0 .. DEPTH-1 (DEPTH up to 16)
    localparam AW = (DEPTH <= 2) ? 1 : (DEPTH <= 4) ? 2 : (DEPTH <= 8) ? 3 : 4;
    /* verilator lint_off WIDTHTRUNC */
    localparam [AW-1:0] LAST = DEPTH - 1;
    /* verilator lint_on WIDTHTRUNC */

    // ---------------- flit buffer (first-word fall-through) ----------------
    reg [33:0] mem [0:DEPTH-1];
    reg [AW-1:0] wptr, rptr;
    reg [4:0]  count;
    wire       empty = (count == 5'd0);
    wire [33:0] front = mem[rptr];
    wire [1:0]  kind  = front[33:32];
    wire        is_head = (kind == K_HEAD) || (kind == K_SINGLE);
    wire        pop;

    always @(posedge clk) begin
        if (in_valid) mem[wptr] <= in_flit;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr      <= {AW{1'b0}};
            rptr      <= {AW{1'b0}};
            count     <= 5'd0;
            in_credit <= 1'b0;
        end else begin
            // the sender only sends with a credit, so in_valid never overflows
            if (in_valid) wptr <= (wptr == LAST) ? {AW{1'b0}} : wptr + 1'b1;
            if (pop)      rptr <= (rptr == LAST) ? {AW{1'b0}} : rptr + 1'b1;
            case ({in_valid, pop})
                2'b10:   count <= count + 5'd1;
                2'b01:   count <= count - 5'd1;
                default: count <= count;
            endcase
            in_credit <= pop;            // one credit per freed slot
        end
    end

    // ---------------- packet state ----------------
    reg        in_body;       // 0: expecting a header, 1: inside the payload
    reg [5:0]  left;          // payload bytes still to give out
    reg [1:0]  idx;           // byte position inside the front flit

    assign hdr_valid = !in_body && !empty && is_head;
    assign ptype = front[31:29];
    assign dest  = front[28:26];
    assign src   = front[25:23];
    assign prio  = front[22];
    assign len   = front[21:16];
    assign tag   = front[15:8];
    assign arg   = front[7:0];

    assign b_valid = in_body && !empty;
    assign b_data  = front[8*idx +: 8];
    assign b_last  = (left == 6'd1);

    wire take_hdr  = hdr_valid && hdr_ready;
    wire take_byte = b_valid && b_ready;
    wire drop      = !in_body && !empty && !is_head;    // stray payload flit

    // a HEAD with len 0 is treated as header-only; its payload flit is dropped
    wire hdr_has_body = (kind == K_HEAD) && (len != 6'd0);

    assign pop = take_hdr || drop ||
                 (take_byte && (idx == 2'd3 || left == 6'd1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_body <= 1'b0;
            left    <= 6'd0;
            idx     <= 2'd0;
        end else if (take_hdr) begin
            in_body <= hdr_has_body;
            left    <= len;
            idx     <= 2'd0;
        end else if (take_byte) begin
            left <= left - 6'd1;
            idx  <= (idx == 2'd3 || left == 6'd1) ? 2'd0 : idx + 2'd1;
            if (left == 6'd1) in_body <= 1'b0;
        end
    end
endmodule

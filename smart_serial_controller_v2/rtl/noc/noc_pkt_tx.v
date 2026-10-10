// =============================================================================
// noc_pkt_tx.v  -  turns "header + bytes" into NoC flits (packet sender)
// -----------------------------------------------------------------------------
// Every node that sends packets (the NIs, the hub) uses this helper, so the
// flit format lives in one place (docs/SPEC.md section 3.1):
//
//     flit = {kind[1:0], data[31:0]}
//       kind 00 HEAD   01 BODY   10 TAIL   11 SINGLE (header only, len = 0)
//
//     head data = {type[2:0], dest[2:0], src[2:0], prio, len[5:0], tag, arg}
//
// How to use it:
//   1. wait for hdr_ready, put the header fields on the inputs, pulse start
//   2. give exactly len bytes on b_data/b_valid (each taken when b_ready = 1)
//
// Bytes are packed little-endian, 4 per flit (byte 0 in data[7:0]).  The last
// flit is a TAIL and its unused bytes are 0.
//
// Credit flow control: the router input behind us has DEPTH buffer slots.
// We start with DEPTH credits, spend one per flit and get one back every time
// out_credit pulses.  With no credit left we simply wait - nothing is lost.
// =============================================================================
`timescale 1ns / 1ps
module noc_pkt_tx #(
    parameter DEPTH = 4
) (
    input  wire        clk,
    input  wire        rst_n,
    // header (taken on start)
    input  wire        start,
    output wire        hdr_ready,
    input  wire [2:0]  ptype,
    input  wire [2:0]  dest,
    input  wire [2:0]  src,
    input  wire        prio,
    input  wire [5:0]  len,
    input  wire [7:0]  tag,
    input  wire [7:0]  arg,
    // payload bytes
    input  wire        b_valid,
    input  wire [7:0]  b_data,
    output wire        b_ready,
    // to the network
    output reg         out_valid,
    output reg  [33:0] out_flit,
    input  wire        out_credit
);
    localparam K_HEAD = 2'b00, K_BODY = 2'b01, K_TAIL = 2'b10, K_SINGLE = 2'b11;
    localparam [4:0] CRED0 = DEPTH;
    localparam S_IDLE = 2'd0, S_HEAD = 2'd1, S_FILL = 2'd2, S_SEND = 2'd3;

    reg [1:0]  state;
    reg [31:0] head;          // head flit data, built on start
    reg [5:0]  left;          // payload bytes still to take
    reg [1:0]  idx;           // next byte position inside the word
    reg [31:0] word;          // payload word being filled
    reg [4:0]  cred;          // credits (free slots downstream)

    assign hdr_ready = (state == S_IDLE);
    assign b_ready   = (state == S_FILL);

    wire can_send = (cred != 5'd0);
    wire send     = ((state == S_HEAD) || (state == S_SEND)) && can_send;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            head      <= 32'd0;
            left      <= 6'd0;
            idx       <= 2'd0;
            word      <= 32'd0;
            cred      <= CRED0;
            out_valid <= 1'b0;
            out_flit  <= 34'd0;
        end else begin
            out_valid <= 1'b0;
            // credit counter: +1 per returned credit, -1 per flit sent
            case ({out_credit, send})
                2'b10:   cred <= cred + 5'd1;
                2'b01:   cred <= cred - 5'd1;
                default: cred <= cred;
            endcase

            case (state)
                S_IDLE: if (start) begin
                    head  <= {ptype, dest, src, prio, len, tag, arg};
                    left  <= len;
                    idx   <= 2'd0;
                    word  <= 32'd0;
                    state <= S_HEAD;
                end

                S_HEAD: if (can_send) begin
                    out_valid <= 1'b1;
                    out_flit  <= {(left == 6'd0) ? K_SINGLE : K_HEAD, head};
                    state     <= (left == 6'd0) ? S_IDLE : S_FILL;
                end

                // take bytes until the word is full or the packet ends
                S_FILL: if (b_valid) begin
                    word[8*idx +: 8] <= b_data;
                    left <= left - 6'd1;
                    idx  <= idx + 2'd1;
                    if (idx == 2'd3 || left == 6'd1) state <= S_SEND;
                end

                S_SEND: if (can_send) begin
                    out_valid <= 1'b1;
                    out_flit  <= {(left == 6'd0) ? K_TAIL : K_BODY, word};
                    word      <= 32'd0;
                    idx       <= 2'd0;
                    state     <= (left == 6'd0) ? S_IDLE : S_FILL;
                end
            endcase
        end
    end
endmodule

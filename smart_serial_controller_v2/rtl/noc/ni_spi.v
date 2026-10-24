// =============================================================================
// ni_spi.v  -  network interface of the SPI engine (NoC node 4)
// -----------------------------------------------------------------------------
// Lets any node of the network-on-chip use the v1 SPI master without the CPU.
// Packets arrive through noc_pkt_rx, the replies leave through noc_pkt_tx
// (docs/SPEC.md sections 3 and 4.2).
//
//   XFER_REQ  payload [wlen][rlen][w0 .. w(wlen-1)], head arg[1:0] = chip select
//       1. raise "active" (the core then forces the SPI engine to: enabled,
//          master, 8-bit words, manual CS, our cs_level / cs_sel) and pull CS
//       2. clock out the wlen write bytes, then rlen 0x00 bytes
//       3. the engine returns one byte for every byte sent; the last rlen of
//          them are kept in a small buffer (rbuf)
//       4. wait until the engine is not busy, release CS
//       5. reply XFER_RESP [0][r0 .. r(rlen-1)] to the requester (head src),
//          with the request's tag
//   DATA      every payload byte goes out in ONE CS frame (arg[1:0] = CS), the
//             bytes that come back are thrown away, no reply
//   bad XFER_REQ (payload shorter than 2, wlen + 2 != payload length, or
//             rlen > 60): the engine is not touched, reply XFER_RESP [4]
//   other types (and everything while en = 0): consumed and thrown away
//
// Flow control with the engine FIFOs (16 entries each):
//   ns = bytes pushed into the TX FIFO, nr = bytes popped from the RX FIFO.
//   We only push while ns - nr < 16, so the TX FIFO, the shifter and the RX
//   FIFO together never hold more than 16 bytes: nothing can overflow, and a
//   transfer of any length (up to 61 + 60 bytes) streams through.
//
// Sharing the engine with the CPU (done in the core): the CPU and NI strobes
// are OR'ed.  So tx_push, rx_pop are combinational and are never 1 while
// stall (= APB psel) is 1.
//
// Design choices (not fixed by the SPEC):
//   * before a transfer the NI waits until the engine is idle and throws away
//     old bytes left in the RX FIFO, so they cannot be mistaken for answers
//   * after CS is released, active stays 1 (CS high) for 8 more clocks so two
//     NI frames in a row always have a CS-high gap of at least 80 ns
//   * the reply copies the request's prio and arg; it comes from my_id
//   * en is looked at when a packet header arrives: a transfer that has
//     already started is finished even if en goes to 0
//   * a DATA packet with no payload does nothing; an XFER_REQ with
//     wlen = rlen = 0 gives a short CS pulse and the reply [0]
// =============================================================================
`timescale 1ns / 1ps
module ni_spi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        stall,        // 1 = CPU is using the bus: do not touch the engine
    input  wire        en,           // 0 = consume and drop every packet
    input  wire [2:0]  my_id,        // our node number (4)
    // from the network
    input  wire        in_valid,
    input  wire [33:0] in_flit,
    output wire        in_credit,
    // to the network
    output wire        out_valid,
    output wire [33:0] out_flit,
    input  wire        out_credit,
    // packet counters
    output reg  [15:0] pkts_in,
    output reg  [15:0] pkts_out,
    // SPI engine FIFOs (first-word fall-through RX FIFO)
    output wire        tx_push,
    output wire [7:0]  tx_data,
    input  wire        tx_full,
    output wire        rx_pop,
    input  wire [7:0]  rx_data,
    input  wire        rx_empty,
    input  wire        busy,         // SPI master busy
    output reg         active,       // the NI owns the engine
    output reg         cs_level,     // 1 = assert the chip select
    output reg  [1:0]  cs_sel
);
    // packet types
    localparam [2:0] T_DATA = 3'd0, T_XFER = 3'd1, T_RESP = 3'd2;

    // states
    localparam [3:0] S_IDLE  = 4'd0,   // waiting for a packet header
                     S_WLEN  = 4'd1,   // take payload byte 0 (wlen)
                     S_RLEN  = 4'd2,   // take payload byte 1 (rlen) and check
                     S_DROP  = 4'd3,   // throw away the rest of the payload
                     S_PREP  = 4'd4,   // wait for an idle engine, empty RX FIFO
                     S_XFER  = 4'd5,   // CS low, bytes stream through
                     S_GAP   = 4'd6,   // CS high again, short gap
                     S_REPLY = 4'd7,   // start the XFER_RESP packet
                     S_RBYTE = 4'd8;   // give the reply bytes

    // ------------------------------------------------------------------
    // packet receiver / sender helpers
    // ------------------------------------------------------------------
    wire       h_valid;
    wire [2:0] h_type, h_src;
    wire       h_prio;
    wire [5:0] h_len;
    wire [7:0] h_tag, h_arg;
    wire       hdr_take;
    wire       b_valid;
    wire [7:0] b_data;
    wire       b_ready;

    noc_pkt_rx #(.DEPTH(4)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_flit(in_flit), .in_credit(in_credit),
        .hdr_valid(h_valid), .ptype(h_type), .dest(), .src(h_src),
        .prio(h_prio), .len(h_len), .tag(h_tag), .arg(h_arg),
        .hdr_ready(hdr_take),
        .b_valid(b_valid), .b_data(b_data), .b_last(), .b_ready(b_ready));

    wire       t_start, t_hdr_ready;
    wire       t_bvalid, t_bready;
    wire [7:0] t_bdata;
    wire [5:0] rep_len;

    reg  [2:0] r_src;
    reg  [7:0] r_tag, r_arg;
    reg        r_prio;

    noc_pkt_tx #(.DEPTH(4)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(t_start), .hdr_ready(t_hdr_ready),
        .ptype(T_RESP), .dest(r_src), .src(my_id), .prio(r_prio),
        .len(rep_len), .tag(r_tag), .arg(r_arg),
        .b_valid(t_bvalid), .b_data(t_bdata), .b_ready(t_bready),
        .out_valid(out_valid), .out_flit(out_flit), .out_credit(out_credit));

    // ------------------------------------------------------------------
    // registers
    // ------------------------------------------------------------------
    reg [3:0] st;
    reg [5:0] r_len;          // payload length of the request
    reg [5:0] left;           // payload bytes not yet taken from u_rx
    reg       r_xfer;         // 1 = XFER_REQ: a reply is due
    reg [2:0] status;         // reply status (0 OK, 4 bad request)
    reg       wlen_ok;        // wlen + 2 == payload length
    reg [5:0] wlen;           // bytes to write (DATA: the whole payload)
    reg [5:0] rlen;           // bytes to read (DATA: 0)
    reg [6:0] ns;             // bytes pushed into the TX FIFO
    reg [6:0] nr;             // bytes popped from the RX FIFO
    reg [2:0] gap;            // CS-high gap counter
    reg [5:0] idx;            // reply byte index

    reg [7:0] rbuf [0:63];    // read bytes (distributed RAM, no reset)

    wire [6:0] total    = {1'b0, wlen} + {1'b0, rlen};
    wire [6:0] inflight = ns - nr;
    wire       w_phase  = (ns < {1'b0, wlen});      // still sending write bytes

    // ------------------------------------------------------------------
    // engine strobes (combinational, never while stall = 1)
    // ------------------------------------------------------------------
    // push the next byte: a payload byte (write part) or 0x00 (read part)
    wire push_x = (st == S_XFER) && (ns != total) && !tx_full && !stall &&
                  (inflight[6:4] == 3'd0) && (!w_phase || b_valid);
    // pop a returned byte while transferring, or an old byte before it
    wire pop_x  = (st == S_XFER) && !rx_empty && !stall && (nr != total);
    wire pop_p  = (st == S_PREP) && !rx_empty && !stall;

    assign tx_push = push_x;
    assign tx_data = w_phase ? b_data : 8'h00;
    assign rx_pop  = pop_x | pop_p;

    // ------------------------------------------------------------------
    // network side handshakes
    // ------------------------------------------------------------------
    assign hdr_take = (st == S_IDLE) && h_valid;
    assign b_ready  = (st == S_WLEN) || (st == S_RLEN) ||
                      ((st == S_DROP) && (left != 6'd0)) ||
                      (push_x && w_phase);
    wire   take_b   = b_valid && b_ready;

    wire [5:0] ridx = idx - 6'd1;
    assign rep_len  = (status == 3'd0) ? rlen + 6'd1 : 6'd1;
    assign t_start  = (st == S_REPLY) && t_hdr_ready;
    assign t_bvalid = (st == S_RBYTE);
    assign t_bdata  = (idx == 6'd0) ? {5'd0, status} : rbuf[ridx];

    // checks on the rlen byte (payload byte 1)
    wire rlen_bad = !wlen_ok || (b_data > 8'd60);

    // ------------------------------------------------------------------
    // read buffer: keep the last rlen returned bytes
    // ------------------------------------------------------------------
    wire [6:0] widx = nr - {1'b0, wlen};
    always @(posedge clk) begin
        if (pop_x && r_xfer && !(nr < {1'b0, wlen}))
            rbuf[widx[5:0]] <= rx_data;
    end

    // ------------------------------------------------------------------
    // main state machine
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            r_src    <= 3'd0;
            r_tag    <= 8'd0;
            r_arg    <= 8'd0;
            r_prio   <= 1'b0;
            r_len    <= 6'd0;
            left     <= 6'd0;
            r_xfer   <= 1'b0;
            status   <= 3'd0;
            wlen_ok  <= 1'b0;
            wlen     <= 6'd0;
            rlen     <= 6'd0;
            ns       <= 7'd0;
            nr       <= 7'd0;
            gap      <= 3'd0;
            idx      <= 6'd0;
            active   <= 1'b0;
            cs_level <= 1'b0;
            cs_sel   <= 2'd0;
            pkts_in  <= 16'd0;
            pkts_out <= 16'd0;
        end else begin
            if (take_b)  left     <= left - 6'd1;
            if (t_start) pkts_out <= pkts_out + 16'd1;

            case (st)
            // ---------------------------------------------------------
            S_IDLE: if (h_valid) begin
                pkts_in <= pkts_in + 16'd1;
                r_src   <= h_src;
                r_tag   <= h_tag;
                r_arg   <= h_arg;
                r_prio  <= h_prio;
                r_len   <= h_len;
                left    <= h_len;
                cs_sel  <= h_arg[1:0];
                wlen    <= h_len;          // DATA: the whole payload is written
                rlen    <= 6'd0;
                status  <= 3'd0;
                r_xfer  <= 1'b0;
                if (!en)
                    st <= (h_len == 6'd0) ? S_IDLE : S_DROP;
                else if (h_type == T_DATA)
                    st <= (h_len == 6'd0) ? S_IDLE : S_PREP;
                else if (h_type == T_XFER) begin
                    r_xfer <= 1'b1;
                    if (h_len < 6'd2) begin               // too short
                        status <= 3'd4;
                        st     <= S_DROP;
                    end else
                        st <= S_WLEN;
                end else
                    st <= (h_len == 6'd0) ? S_IDLE : S_DROP;
            end
            // ---------------------------------------------------------
            S_WLEN: if (b_valid) begin
                wlen    <= b_data[5:0];
                wlen_ok <= (b_data == {2'b00, r_len} - 8'd2);
                st      <= S_RLEN;
            end
            S_RLEN: if (b_valid) begin
                rlen <= b_data[5:0];
                if (rlen_bad) begin
                    status <= 3'd4;
                    st     <= S_DROP;
                end else
                    st <= S_PREP;
            end
            // ---------------------------------------------------------
            S_DROP: if (left == 6'd0)
                st <= r_xfer ? S_REPLY : S_IDLE;
            // ---------------------------------------------------------
            S_PREP: if (!busy && rx_empty) begin
                active   <= 1'b1;
                cs_level <= 1'b1;
                ns       <= 7'd0;
                nr       <= 7'd0;
                st       <= S_XFER;
            end
            // ---------------------------------------------------------
            S_XFER: begin
                if (push_x) ns <= ns + 7'd1;
                if (pop_x)  nr <= nr + 7'd1;
                // every byte is back and the engine has finished the frame
                if ((nr == total) && !busy) begin
                    cs_level <= 1'b0;
                    gap      <= 3'd0;
                    st       <= S_GAP;
                end
            end
            S_GAP: begin
                gap <= gap + 3'd1;
                if (gap == 3'd7) begin
                    active <= 1'b0;
                    st     <= r_xfer ? S_REPLY : S_IDLE;
                end
            end
            // ---------------------------------------------------------
            S_REPLY: if (t_hdr_ready) begin
                idx <= 6'd0;
                st  <= S_RBYTE;
            end
            S_RBYTE: if (t_bready) begin
                if (idx == rep_len - 6'd1) st <= S_IDLE;
                else                       idx <= idx + 6'd1;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule

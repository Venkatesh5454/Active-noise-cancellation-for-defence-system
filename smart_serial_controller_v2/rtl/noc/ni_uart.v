// =============================================================================
// ni_uart.v  -  network interface of the UART (NoC node 3)
// -----------------------------------------------------------------------------
// Connects the v1 UART engine (its TX and RX FIFOs) to the network-on-chip,
// so a PC terminal can talk to the SPI / I2C / other nodes (docs/SPEC.md 4.1).
//
//      PC --> UART RX FIFO --> [ parser + 64-byte buffer ] --> noc_pkt_tx --> network
//      PC <-- UART TX FIFO <-- [ byte filter ]            <-- noc_pkt_rx <-- network
//
// PC -> network (bytes popped from the UART RX FIFO), chosen by `mode`:
//   0 RAW       bytes are collected in the buffer and sent as one DATA packet
//               to `dest` (with `arg`, `prio`) when 16 bytes are there, or when
//               no byte has arrived for `timeout_us` microseconds.
//   1 FRAME     the PC sends [wlen][w0 .. w(wlen-1)][rlen].  The NI sends an
//               XFER_REQ to `dest` with `arg` and payload [wlen][rlen][w...].
//               Slide 17: PC "01 9F 03" -> payload "01 03 9F".
//   2 ADDRESSED the PC sends [dest][arg][wlen][w...][rlen]; dest and arg come
//               from the frame instead of the configuration.
//   3           (not defined by the SPEC) behaves like RAW.
//   Every packet goes from my_id with `prio`.  The tag is an 8-bit counter of
//   the packets this NI has sent (0, 1, 2, ... after reset), so consecutive
//   XFER_REQs carry incrementing tags.
//
// The packet buffer: the header needs the length, so a packet is only started
// when it is complete.  In FRAME modes the bytes are stored in payload order:
// wlen at [0], rlen at [1] and w bytes at [2 ..].  While a packet is being sent
// the NI does not pop the RX FIFO (the UART keeps up to 16 bytes meanwhile).
//
// Network -> PC: every payload byte of an incoming packet is pushed into the
// UART TX FIFO, waiting while tx_full (no byte is ever lost).  The exception
// is XFER_RESP: its first byte (the status) is skipped unless resp_status = 1.
// Reserved types 5-7 are consumed and dropped; header-only packets push
// nothing.
//
// Sharing the engine with the CPU: tx_push and rx_pop are never 1 while
// stall = 1 (the core ORs them with the CPU strobes).  With en = 0 the NI
// does not touch the UART FIFOs at all, but still consumes and drops every
// incoming packet so the network never blocks.
//
// Design choices (places where the SPEC leaves room):
//   * Silence timer: counts us_tick pulses while the RX FIFO is empty (a byte
//     waiting in the FIFO has "arrived").  The time-out fires on the first
//     us_tick after timeout_us whole microseconds of silence, i.e. between
//     timeout_us and timeout_us + 1 us after the last byte.  timeout_us = 0
//     is not "off": it fires at the next us_tick.
//   * Bad frame: wlen > 60, rlen > 60 or (ADDRESSED) a dest byte > 7.  The
//     frame is thrown away, and so is every byte that follows it without a
//     pause; after timeout_us of silence the parser waits for a new frame.
//     This is the same re-synchronisation as for an incomplete frame.
//   * en = 0 (or a change of mode) throws away a half-collected RAW packet
//     or frame.  A packet that is already being sent is always finished,
//     because its bytes come from the NI's own buffer.
//   * pkts_in / pkts_out count HEAD and SINGLE flits on the input and output
//     link (one per packet), exactly like the core's NI_STAT counters.
// =============================================================================
`timescale 1ns / 1ps
module ni_uart (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        stall,          // 1: the CPU uses the engine, keep off
    input  wire        en,
    input  wire [2:0]  my_id,
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
    // configuration
    input  wire [1:0]  mode,           // 0 RAW, 1 FRAME, 2 ADDRESSED FRAME
    input  wire [2:0]  dest,
    input  wire        prio,
    input  wire [7:0]  arg,
    input  wire        resp_status,    // 1: also send the XFER_RESP status byte
    input  wire [15:0] timeout_us,
    input  wire        us_tick,
    // UART engine FIFOs
    output wire        tx_push,
    output wire [7:0]  tx_data,
    input  wire        tx_full,
    output wire        rx_pop,
    input  wire [7:0]  rx_data,        // first-word fall-through
    input  wire        rx_empty
);
    localparam [2:0] T_DATA = 3'd0, T_XREQ = 3'd1, T_XRESP = 3'd2, T_ALARM = 3'd4;

    // =========================================================================
    // Network -> PC
    // =========================================================================
    wire       h_valid;
    wire [2:0] h_type;
    wire       nb_valid;
    wire [7:0] nb_data;
    wire       nb_ready;

    noc_pkt_rx #(.DEPTH(4)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_flit(in_flit), .in_credit(in_credit),
        .hdr_valid(h_valid), .ptype(h_type), .dest(), .src(), .prio(),
        .len(), .tag(), .arg(),
        .hdr_ready(1'b1),                       // a header is always welcome
        .b_valid(nb_valid), .b_data(nb_data), .b_last(), .b_ready(nb_ready));

    reg n_keep;      // this packet's bytes go to the PC (type 0..4)
    reg n_skip;      // the next byte is an XFER_RESP status byte: drop it

    wire n_drop    = !en || !n_keep || n_skip;      // throw this byte away
    wire n_push_ok = !tx_full && !stall;            // may push this cycle

    assign tx_push  = nb_valid && !n_drop && n_push_ok;
    assign tx_data  = nb_data;
    assign nb_ready = n_drop || n_push_ok;          // dropped bytes never wait

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            n_keep <= 1'b0;
            n_skip <= 1'b0;
        end else if (h_valid) begin                 // header taken (hdr_ready = 1)
            n_keep <= (h_type <= T_ALARM);
            n_skip <= (h_type == T_XRESP) && !resp_status;
        end else if (nb_valid && nb_ready) begin
            n_skip <= 1'b0;                         // only the first byte
        end
    end

    // =========================================================================
    // PC -> network
    // =========================================================================
    localparam [3:0] S_RAW   = 4'd0,   // RAW: collecting bytes
                     S_DEST  = 4'd1,   // ADDRESSED: waiting for the dest byte
                     S_ARG   = 4'd2,   // ADDRESSED: waiting for the arg byte
                     S_WLEN  = 4'd3,   // waiting for wlen
                     S_WDATA = 4'd4,   // receiving the wlen write bytes
                     S_RLEN  = 4'd5,   // waiting for rlen
                     S_SKIP  = 4'd6,   // bad frame: drop bytes until silence
                     S_HDR   = 4'd7,   // packet complete: start it
                     S_BYTES = 4'd8;   // feeding the payload to noc_pkt_tx

    reg [3:0]  state;
    reg [1:0]  cur_mode;      // the mode the parser is working in
    reg [5:0]  cnt;           // RAW bytes collected / w bytes received
    reg [5:0]  wlen;
    reg [2:0]  f_dest;        // from an ADDRESSED frame
    reg [7:0]  f_arg;
    reg [15:0] quiet;         // microseconds without a new byte
    reg [7:0]  tag_cnt;       // packets sent so far (used as the tag)
    // header of the packet in the buffer
    reg [2:0]  p_type;
    reg [2:0]  p_dest;
    reg        p_prio;
    reg [7:0]  p_arg;
    reg [5:0]  p_len;
    reg [5:0]  rd_idx;        // next payload byte to send

    // packet buffer: no reset, one write port, asynchronous read (LUTRAM)
    reg [7:0]  pbuf [0:63];

    // the state the parser starts in for a mode
    wire [3:0] home = (mode == 2'd1) ? S_WLEN :
                      (mode == 2'd2) ? S_DEST : S_RAW;

    wire collecting = (state != S_HDR) && (state != S_BYTES);
    // disabled or mode changed: drop what was collected and start again
    wire rehome     = collecting && (!en || (mode != cur_mode));
    // take one byte from the UART RX FIFO this cycle
    wire take       = collecting && !rehome && !stall && !rx_empty;

    assign rx_pop = take;

    // silence timer: cleared while a byte is waiting (or taken), while
    // disabled and while a packet is being sent
    wire quiet_clr = !en || !rx_empty || !collecting;
    wire tmo       = us_tick && !quiet_clr && (quiet == timeout_us);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            quiet <= 16'd0;
        else if (quiet_clr)
            quiet <= 16'd0;
        else if (us_tick && (quiet != 16'hFFFF))
            quiet <= quiet + 16'd1;
    end

    // where a received byte is stored
    wire [5:0] wr_addr = (state == S_RAW)  ? cnt   :
                         (state == S_WLEN) ? 6'd0  :
                         (state == S_RLEN) ? 6'd1  : cnt + 6'd2;
    wire buf_we = take && ((state == S_RAW) || (state == S_WLEN) ||
                           (state == S_WDATA) || (state == S_RLEN));

    always @(posedge clk) begin
        if (buf_we) pbuf[wr_addr] <= rx_data;
    end

    // ---------------- packet sender ----------------
    wire tx_hdr_ready;
    wire tx_b_ready;
    wire tx_start = (state == S_HDR);
    wire tx_b_valid = (state == S_BYTES);

    noc_pkt_tx #(.DEPTH(4)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(tx_start), .hdr_ready(tx_hdr_ready),
        .ptype(p_type), .dest(p_dest), .src(my_id), .prio(p_prio),
        .len(p_len), .tag(tag_cnt), .arg(p_arg),
        .b_valid(tx_b_valid), .b_data(pbuf[rd_idx]), .b_ready(tx_b_ready),
        .out_valid(out_valid), .out_flit(out_flit), .out_credit(out_credit));

    // ---------------- parser / sender state machine ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_RAW;
            cur_mode <= 2'd0;
            cnt      <= 6'd0;
            wlen     <= 6'd0;
            f_dest   <= 3'd0;
            f_arg    <= 8'd0;
            tag_cnt  <= 8'd0;
            p_type   <= T_DATA;
            p_dest   <= 3'd0;
            p_prio   <= 1'b0;
            p_arg    <= 8'd0;
            p_len    <= 6'd0;
            rd_idx   <= 6'd0;
        end else if (rehome) begin
            state    <= home;
            cur_mode <= mode;
            cnt      <= 6'd0;
        end else begin
            case (state)
                // RAW: 16 bytes -> send now; silence -> send what we have
                S_RAW: begin
                    if (take) begin
                        cnt <= cnt + 6'd1;
                        if (cnt == 6'd15) begin
                            p_type <= T_DATA;  p_dest <= dest;
                            p_arg  <= arg;     p_prio <= prio;
                            p_len  <= 6'd16;
                            state  <= S_HDR;
                        end
                    end else if (tmo && (cnt != 6'd0)) begin
                        p_type <= T_DATA;  p_dest <= dest;
                        p_arg  <= arg;     p_prio <= prio;
                        p_len  <= cnt;
                        state  <= S_HDR;
                    end
                end

                S_DEST: begin
                    if (take) begin
                        if (rx_data > 8'd7) begin
                            state <= S_SKIP;            // not a node id
                        end else begin
                            f_dest <= rx_data[2:0];
                            state  <= S_ARG;
                        end
                    end
                    // nothing collected yet, so a time-out has nothing to drop
                end

                S_ARG: begin
                    if (take) begin
                        f_arg <= rx_data;
                        state <= S_WLEN;
                    end else if (tmo) begin
                        state <= home;                  // incomplete frame
                    end
                end

                S_WLEN: begin
                    if (take) begin
                        if (rx_data > 8'd60) begin
                            state <= S_SKIP;
                        end else begin
                            wlen  <= rx_data[5:0];
                            cnt   <= 6'd0;
                            state <= (rx_data == 8'd0) ? S_RLEN : S_WDATA;
                        end
                    end else if (tmo) begin
                        state <= home;                  // (ADDRESSED) incomplete
                    end
                end

                S_WDATA: begin
                    if (take) begin
                        cnt <= cnt + 6'd1;
                        if (cnt + 6'd1 == wlen) state <= S_RLEN;
                    end else if (tmo) begin
                        state <= home;
                    end
                end

                S_RLEN: begin
                    if (take) begin
                        if (rx_data > 8'd60) begin
                            state <= S_SKIP;
                        end else begin
                            p_type <= T_XREQ;
                            p_dest <= (cur_mode == 2'd2) ? f_dest : dest;
                            p_arg  <= (cur_mode == 2'd2) ? f_arg  : arg;
                            p_prio <= prio;
                            p_len  <= wlen + 6'd2;
                            state  <= S_HDR;
                        end
                    end else if (tmo) begin
                        state <= home;
                    end
                end

                // bad frame: take and drop bytes until the line is quiet
                S_SKIP: begin
                    if (tmo) state <= home;
                end

                // wait until the sender is free, then hand it the header
                S_HDR: begin
                    if (tx_hdr_ready) begin
                        rd_idx  <= 6'd0;
                        tag_cnt <= tag_cnt + 8'd1;
                        state   <= S_BYTES;
                    end
                end

                // one buffer byte per accepted b_ready
                S_BYTES: begin
                    if (tx_b_ready) begin
                        rd_idx <= rd_idx + 6'd1;
                        if (rd_idx + 6'd1 == p_len) begin
                            state    <= home;
                            cur_mode <= mode;
                            cnt      <= 6'd0;
                        end
                    end
                end

                default: state <= home;
            endcase
        end
    end

    // =========================================================================
    // Packet counters: one HEAD or SINGLE flit per packet
    // =========================================================================
    wire in_head  = in_valid  && ((in_flit[33:32]  == 2'b00) || (in_flit[33:32]  == 2'b11));
    wire out_head = out_valid && ((out_flit[33:32] == 2'b00) || (out_flit[33:32] == 2'b11));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pkts_in  <= 16'd0;
            pkts_out <= 16'd0;
        end else begin
            if (in_head)  pkts_in  <= pkts_in  + 16'd1;
            if (out_head) pkts_out <= pkts_out + 16'd1;
        end
    end
endmodule

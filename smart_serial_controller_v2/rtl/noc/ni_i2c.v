// =============================================================================
// ni_i2c.v  -  network interface of the I2C engine (NoC node 5)
// -----------------------------------------------------------------------------
// Lets any node of the network-on-chip run I2C transactions on the v1 I2C
// master without the CPU (docs/SPEC.md sections 3 and 4.3).  The sensor hub
// uses it to read the PmodTMP2: XFER_REQ arg 0x4B [01][02][00] -> [00][0C 80].
//
//   XFER_REQ  payload [wlen][rlen][w0 .. w(wlen-1)], head arg[6:0] = address
//     write phase (wlen > 0):
//       push the wlen bytes into the engine TX FIFO, then issue
//       cmd(len = wlen, read = 0, stop = (rlen == 0)) and wait for ev_done.
//       With rlen > 0 the engine keeps the bus (holding = 1, no STOP), so the
//       read below starts with a REPEATED START - the usual "set the register
//       pointer, then read" sequence.
//     read phase (rlen > 0):
//       issue cmd(len = rlen, read = 1, stop = 1) and pop every byte as soon
//       as it arrives (the RX FIFO is only 16 deep, rlen may be up to 60).
//     reply XFER_RESP [status][r0 .. r(rlen-1)] to the requester (head src)
//       with the request's tag.  status: 0 OK, 1 NACK, 2 arbitration lost,
//       3 time-out, 4 bad request (wlen > 16, rlen > 60, wrong length).
//   DATA      one write transaction with all the payload bytes and a STOP
//             (address = arg[6:0]), no reply
//   other types (and everything while en = 0): consumed and thrown away
//
// What the v1 engine does (rtl/v1/ssc_i2c.v), and how we use it:
//   * cmd_valid is only accepted while the engine is idle or holding the bus
//     and has no pending command, which is exactly busy = 0.  So a command
//     is only issued while busy = 0 (and stall = 0); busy rises next clock.
//   * ev_done pulses once per command: after the STOP, after the last byte
//     when the bus is kept (stop = 0), or at once when arbitration is lost.
//     nack_flag / arb_flag are valid in the same clock as ev_done.
//   * after a NACK the engine has already sent STOP; after a lost
//     arbitration it has let go of the bus.
//   * abort puts the engine back to idle, lets go of SCL/SDA and flushes the
//     TX FIFO.
//
// Time-out: every command has its own timer.  It starts when the NI begins
// the command (waiting for the engine, pushing bytes, the command itself)
// and counts us_tick pulses.  When timeout_us ticks have passed without
// ev_done, the NI pulses abort and replies status 3.  So the real time-out is
// between timeout_us - 1 and timeout_us microseconds; timeout_us = 0 times out
// at once.
//
// Design choices (not fixed by the SPEC):
//   * every failed transaction (NACK, arbitration, time-out) ends with ONE
//     abort pulse: it flushes write bytes the engine did not send, so they
//     cannot leak into the next transaction, and leaves the engine idle
//   * before a read the NI throws away old bytes left in the RX FIFO
//   * a DATA packet with more than 16 bytes is still ONE write transaction:
//     the command is issued once 16 bytes are queued, and the rest are pushed
//     while the engine sends (the engine holds SCL low if it runs out of
//     bytes, which is a legal I2C pause).  A DATA packet with no payload
//     does nothing.
//   * an XFER_REQ with wlen = rlen = 0 is an address probe: START, address,
//     STOP.  Status 0 = a device answered, 1 = NACK.
//   * the reply copies the request's prio and arg; it comes from my_id
//   * en is looked at when a packet header arrives: a transaction that has
//     already started is finished even if en goes to 0
// =============================================================================
`timescale 1ns / 1ps
module ni_i2c (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        stall,        // 1 = CPU is using the bus: do not touch the engine
    input  wire        en,           // 0 = consume and drop every packet
    input  wire [2:0]  my_id,        // our node number (5)
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
    // I2C engine FIFOs (first-word fall-through RX FIFO)
    output wire        tx_push,
    output wire [7:0]  tx_data,
    input  wire        tx_full,
    output wire        rx_pop,
    input  wire [7:0]  rx_data,
    input  wire        rx_empty,
    // I2C engine command
    output wire        cmd_valid,
    output wire [7:0]  cmd_len,
    output wire        cmd_read,
    output wire        cmd_stop,
    /* verilator lint_off SYMRSVDWORD */   // "abort" is the SPEC port name
    output wire        abort,
    /* verilator lint_on SYMRSVDWORD */
    input  wire        busy,
    input  wire        holding,
    input  wire        nack_flag,
    input  wire        arb_flag,
    input  wire        ev_done,
    input  wire [15:0] timeout_us,   // per command
    input  wire        us_tick,
    output reg         active,       // the NI owns the engine
    output reg  [6:0]  addr7
);
    // packet types
    localparam [2:0] T_DATA = 3'd0, T_XFER = 3'd1, T_RESP = 3'd2;

    // states
    localparam [3:0] S_IDLE  = 4'd0,   // waiting for a packet header
                     S_WLEN  = 4'd1,   // take payload byte 0 (wlen)
                     S_RLEN  = 4'd2,   // take payload byte 1 (rlen) and check
                     S_DROP  = 4'd3,   // throw away the rest of the payload
                     S_WRITE = 4'd4,   // push bytes, write command, wait ev_done
                     S_READ  = 4'd5,   // read command, pop bytes, wait ev_done
                     S_ABORT = 4'd6,   // pulse abort after an error
                     S_FIN   = 4'd7,   // transaction over: give the engine back
                     S_REPLY = 4'd8,   // start the XFER_RESP packet
                     S_RBYTE = 4'd9;   // give the reply bytes

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
    reg [3:0]  st;
    reg [5:0]  r_len;         // payload length of the request
    reg [5:0]  left;          // payload bytes not yet taken from u_rx
    reg        r_xfer;        // 1 = XFER_REQ: a reply is due
    reg [2:0]  status;        // reply status
    reg        wlen_ok;       // wlen <= 16 and wlen + 2 == payload length
    reg [5:0]  wlen;          // bytes to write (DATA: the whole payload)
    reg [5:0]  rlen;          // bytes to read
    reg [5:0]  np;            // bytes pushed into the TX FIFO
    reg [5:0]  nr;            // bytes popped from the RX FIFO
    reg        issued;        // the command of this phase has been issued
    reg        done;          // read phase: ev_done seen (status 0)
    reg [15:0] tmo;           // us ticks since the phase started
    reg [5:0]  idx;           // reply byte index

    reg [7:0]  rbuf [0:63];   // read bytes (distributed RAM, no reset)

    wire tmo_hit = (tmo >= timeout_us);
    wire my_done = issued && ev_done;          // ev_done of OUR command
    wire err_now = nack_flag || arb_flag;
    wire [2:0] err_code = nack_flag ? 3'd1 : 3'd2;

    // ------------------------------------------------------------------
    // engine strobes (combinational, never while stall = 1)
    // ------------------------------------------------------------------
    // write phase: push payload bytes; before the command only while the
    // engine is idle (so a CPU command cannot take them)
    wire push_w = (st == S_WRITE) && (np != wlen) && b_valid && !tx_full &&
                  !stall && (issued || !busy);
    // issue the write command when every byte is queued (or 16 are, for a
    // long DATA packet); never in the clock the timer runs out
    wire cmd_w  = (st == S_WRITE) && !issued && !busy && !stall && !tmo_hit &&
                  ((np == wlen) || (np == 6'd16));
    // read phase: first empty the RX FIFO, then issue the read command,
    // then pop the bytes as they arrive
    wire pop_r  = (st == S_READ) && !rx_empty && !stall && (!issued || (nr != rlen));
    wire cmd_r  = (st == S_READ) && !issued && !busy && !stall && !tmo_hit && rx_empty;

    assign tx_push   = push_w;
    assign tx_data   = b_data;
    assign rx_pop    = pop_r;
    assign cmd_valid = cmd_w | cmd_r;
    assign cmd_len   = {2'b00, (st == S_READ) ? rlen : wlen};
    assign cmd_read  = (st == S_READ);
    assign cmd_stop  = (st == S_READ) || !r_xfer || (rlen == 6'd0);
    // abort after an error, or if the bus is somehow still held at the end
    assign abort     = !stall && ((st == S_ABORT) || ((st == S_FIN) && holding));

    // ------------------------------------------------------------------
    // network side handshakes
    // ------------------------------------------------------------------
    assign hdr_take = (st == S_IDLE) && h_valid;
    assign b_ready  = (st == S_WLEN) || (st == S_RLEN) ||
                      ((st == S_DROP) && (left != 6'd0)) || push_w;
    wire   take_b   = b_valid && b_ready;

    wire [5:0] ridx = idx - 6'd1;
    assign rep_len  = (status == 3'd0) ? rlen + 6'd1 : 6'd1;
    assign t_start  = (st == S_REPLY) && t_hdr_ready;
    assign t_bvalid = (st == S_RBYTE);
    assign t_bdata  = (idx == 6'd0) ? {5'd0, status} : rbuf[ridx];

    // checks on the rlen byte (payload byte 1)
    wire rlen_bad = !wlen_ok || (b_data > 8'd60);

    // ------------------------------------------------------------------
    // read buffer
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (pop_r && issued) rbuf[nr] <= rx_data;
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
            np       <= 6'd0;
            nr       <= 6'd0;
            issued   <= 1'b0;
            done     <= 1'b0;
            tmo      <= 16'd0;
            idx      <= 6'd0;
            active   <= 1'b0;
            addr7    <= 7'd0;
            pkts_in  <= 16'd0;
            pkts_out <= 16'd0;
        end else begin
            if (take_b)  left     <= left - 6'd1;
            if (t_start) pkts_out <= pkts_out + 16'd1;
            // per-command timer (saturates)
            if (us_tick && (tmo != 16'hFFFF)) tmo <= tmo + 16'd1;

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
                addr7   <= h_arg[6:0];
                wlen    <= h_len;          // DATA: the whole payload is written
                rlen    <= 6'd0;
                status  <= 3'd0;
                r_xfer  <= 1'b0;
                np      <= 6'd0;
                issued  <= 1'b0;
                tmo     <= 16'd0;
                if (!en)
                    st <= (h_len == 6'd0) ? S_IDLE : S_DROP;
                else if (h_type == T_DATA) begin
                    if (h_len == 6'd0) st <= S_IDLE;
                    else begin
                        active <= 1'b1;
                        st     <= S_WRITE;
                    end
                end else if (h_type == T_XFER) begin
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
                wlen_ok <= (b_data <= 8'd16) && (b_data == {2'b00, r_len} - 8'd2);
                st      <= S_RLEN;
            end
            S_RLEN: if (b_valid) begin
                rlen <= b_data[5:0];
                if (rlen_bad) begin
                    status <= 3'd4;
                    st     <= S_DROP;
                end else begin
                    active <= 1'b1;
                    issued <= 1'b0;
                    tmo    <= 16'd0;
                    // no write bytes but a read: go straight to the read
                    // (wlen = rlen = 0 is an address probe: a 0-byte write)
                    if (wlen == 6'd0 && b_data != 8'd0) begin
                        nr   <= 6'd0;
                        done <= 1'b0;
                        st   <= S_READ;
                    end else begin
                        np <= 6'd0;
                        st <= S_WRITE;
                    end
                end
            end
            // ---------------------------------------------------------
            S_DROP: if (left == 6'd0)
                st <= r_xfer ? S_REPLY : S_IDLE;
            // ---------------------------------------------------------
            S_WRITE: begin
                if (push_w) np <= np + 6'd1;
                if (cmd_w) begin
                    issued <= 1'b1;
                end
                if (my_done) begin
                    issued <= 1'b0;
                    if (err_now) begin                    // NACK / arbitration
                        status <= err_code;
                        st     <= S_ABORT;
                    end else if (r_xfer && rlen != 6'd0) begin
                        nr   <= 6'd0;                     // repeated START next
                        done <= 1'b0;
                        tmo  <= 16'd0;
                        st   <= S_READ;
                    end else
                        st <= S_FIN;
                end else if (tmo_hit) begin
                    status <= 3'd3;
                    st     <= S_ABORT;
                end
            end
            // ---------------------------------------------------------
            S_READ: begin
                if (pop_r && issued) nr <= nr + 6'd1;
                if (cmd_r)   issued <= 1'b1;
                if (my_done) done   <= 1'b1;
                if (my_done && err_now) begin             // NACK / arbitration
                    status <= err_code;
                    st     <= S_ABORT;
                end else if ((done || my_done) && (nr == rlen))
                    st <= S_FIN;                          // all bytes are in
                else if (tmo_hit) begin
                    status <= 3'd3;
                    st     <= S_ABORT;
                end
            end
            // ---------------------------------------------------------
            S_ABORT: if (!stall)
                st <= S_DROP;              // abort pulses in this clock
            S_FIN: if (!stall) begin       // (abort pulses here if still holding)
                active <= 1'b0;
                st     <= r_xfer ? S_REPLY : S_IDLE;
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

            // the engine is given back after an error (abort done)
            if ((st == S_ABORT) && !stall) active <= 1'b0;
        end
    end
endmodule

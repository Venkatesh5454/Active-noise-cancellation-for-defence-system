// =============================================================================
// ni_se.v  -  network interface of the serial engine (node 0)
// -----------------------------------------------------------------------------
// Connects the two state machines' FIFOs to the network-on-chip.
//
//   network -> SM : every payload byte of an incoming packet (any type)
//                   becomes one word in the TX FIFO of SM arg[0].  The byte
//                   goes in bits [7:0], or in [31:24] if that SM's OUT
//                   shifts left (so an MSB-first program sends it first).
//
//   SM -> network : words from an SM's RX FIFO are collected into a DATA
//                   packet for destX / prioX, with arg = SM index.  Each word
//                   gives one byte (ISR[7:0] if IN shifts left, ISR[31:24]
//                   if IN shifts right), or 4 bytes little-endian if wordX
//                   is set.  A packet is sent as soon as the RX FIFO is
//                   empty or 16 bytes are collected (the header needs the
//                   length, so the bytes are buffered first).  SM0 and SM1
//                   take turns.
//
// Sharing with the CPU: no push or pop in a clock where stall = 1 (an APB
// transfer is running).  With en = 0 the NI never touches the FIFOs, but it
// still consumes and drops every packet that arrives, so the network never
// blocks.
// =============================================================================
`timescale 1ns / 1ps
module ni_se (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        stall,
    input  wire        en,
    input  wire [2:0]  my_id,
    input  wire [2:0]  dest0,
    input  wire [2:0]  dest1,
    input  wire        prio0,
    input  wire        prio1,
    input  wire        word4_0,
    input  wire        word4_1,
    input  wire [1:0]  out_shift_left,
    input  wire [1:0]  in_shift_left,
    // serial engine FIFOs
    output wire [1:0]  tx_push,
    output wire [31:0] tx_data0,
    output wire [31:0] tx_data1,
    input  wire [1:0]  tx_full,
    output wire [1:0]  rx_pop,
    input  wire [31:0] rx_data0,
    input  wire [31:0] rx_data1,
    input  wire [1:0]  rx_empty,
    // network
    input  wire        in_valid,
    input  wire [33:0] in_flit,
    output wire        in_credit,
    output wire        out_valid,
    output wire [33:0] out_flit,
    input  wire        out_credit,
    output reg  [15:0] pkts_in,
    output reg  [15:0] pkts_out
);
    localparam [2:0] T_DATA = 3'd0;

    // =========================================================================
    // network -> state machines
    // =========================================================================
    wire       r_hv, r_bv, r_blast;
    wire [7:0] r_arg, r_byte;
    reg        r_sm;                      // target SM of the current packet

    // the target SM's TX FIFO has room and the CPU is not on the bus
    wire can_push = en && !stall && !tx_full[r_sm];
    wire r_bready = en ? can_push : 1'b1; // en = 0: drop the bytes

    noc_pkt_rx #(.DEPTH(4)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_flit(in_flit), .in_credit(in_credit),
        .hdr_valid(r_hv), .ptype(), .dest(), .src(), .prio(), .len(),
        .tag(), .arg(r_arg), .hdr_ready(1'b1),
        .b_valid(r_bv), .b_data(r_byte), .b_last(r_blast), .b_ready(r_bready));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_sm    <= 1'b0;
            pkts_in <= 16'd0;
        end else if (r_hv) begin              // header taken (hdr_ready = 1)
            r_sm    <= r_arg[0];
            pkts_in <= pkts_in + 16'd1;
        end
    end

    wire push = r_bv && en && can_push;
    assign tx_push  = {push & r_sm, push & ~r_sm};
    assign tx_data0 = out_shift_left[0] ? {r_byte, 24'd0} : {24'd0, r_byte};
    assign tx_data1 = out_shift_left[1] ? {r_byte, 24'd0} : {24'd0, r_byte};

    // =========================================================================
    // state machines -> network
    // =========================================================================
    localparam S_IDLE = 2'd0, S_COLLECT = 2'd1, S_HDR = 2'd2, S_SEND = 2'd3;

    reg  [1:0]  st;
    reg         cur;                      // SM whose bytes are being collected
    reg         turn;                     // SM to look at first next time
    reg  [7:0]  buf_mem [0:15];
    reg  [4:0]  cnt;                      // bytes in the buffer (0..16)
    reg  [3:0]  idx;                      // next byte to send
    reg  [7:0]  tag;

    wire        w4      = cur ? word4_1 : word4_0;
    wire [31:0] word    = cur ? rx_data1 : rx_data0;
    wire        r_empty = cur ? rx_empty[1] : rx_empty[0];
    wire [4:0]  bpw     = w4 ? 5'd4 : 5'd1;          // bytes per RX word
    wire        room    = (cnt + bpw) <= 5'd16;
    wire [7:0]  one     = (cur ? in_shift_left[1] : in_shift_left[0]) ? word[7:0] : word[31:24];

    wire do_pop = (st == S_COLLECT) && en && !stall && !r_empty && room;
    assign rx_pop = {do_pop & cur, do_pop & ~cur};

    wire        t_hdr_ready, t_bready;
    wire        t_start = (st == S_HDR) && t_hdr_ready;
    wire        t_bvalid = (st == S_SEND);

    noc_pkt_tx #(.DEPTH(4)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(t_start), .hdr_ready(t_hdr_ready),
        .ptype(T_DATA), .dest(cur ? dest1 : dest0), .src(my_id),
        .prio(cur ? prio1 : prio0), .len({1'b0, cnt}), .tag(tag), .arg({7'd0, cur}),
        .b_valid(t_bvalid), .b_data(buf_mem[idx]), .b_ready(t_bready),
        .out_valid(out_valid), .out_flit(out_flit), .out_credit(out_credit));

    always @(posedge clk) begin
        if (do_pop) begin
            if (w4) begin
                buf_mem[cnt[3:0]]        <= word[7:0];
                buf_mem[cnt[3:0] + 4'd1] <= word[15:8];
                buf_mem[cnt[3:0] + 4'd2] <= word[23:16];
                buf_mem[cnt[3:0] + 4'd3] <= word[31:24];
            end else begin
                buf_mem[cnt[3:0]] <= one;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            cur      <= 1'b0;
            turn     <= 1'b0;
            cnt      <= 5'd0;
            idx      <= 4'd0;
            tag      <= 8'd0;
            pkts_out <= 16'd0;
        end else begin
            case (st)
                // pick an SM with data, taking turns
                S_IDLE: begin
                    cnt <= 5'd0;
                    if (en && !rx_empty[turn])       begin cur <= turn;  st <= S_COLLECT; end
                    else if (en && !rx_empty[~turn]) begin cur <= ~turn; st <= S_COLLECT; end
                end
                // pop words until the FIFO is empty or the packet is full
                S_COLLECT: begin
                    if (do_pop) cnt <= cnt + bpw;
                    else if (!en)
                        st <= (cnt != 5'd0) ? S_HDR : S_IDLE;  // send what we have
                    else if (r_empty || !room)
                        st <= (cnt != 5'd0) ? S_HDR : S_IDLE;
                end
                // header goes to noc_pkt_tx, then the bytes
                S_HDR: if (t_hdr_ready) begin
                    idx <= 4'd0;
                    st  <= S_SEND;
                end
                S_SEND: if (t_bready) begin
                    if ({1'b0, idx} == cnt - 5'd1) begin
                        st       <= S_IDLE;
                        turn     <= ~cur;
                        tag      <= tag + 8'd1;
                        pkts_out <= pkts_out + 16'd1;
                    end
                    idx <= idx + 4'd1;
                end
            endcase
        end
    end
endmodule

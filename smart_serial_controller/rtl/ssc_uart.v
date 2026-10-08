// =============================================================================
// ssc_uart.v  -  UART block: baud generator + TX/RX FIFOs + TX and RX engines
// -----------------------------------------------------------------------------
// Frame on the wire (idle level is 1, data goes lowest bit first):
//
//     idle | start 0 | d0 d1 ... d(n-1) | [parity] | stop 1 | [stop 1] | idle
//
// Settings (all from the register bank):
//     data_bits  0 = 5 bits, 1 = 6, 2 = 7, 3 = 8
//     parity_en  add a parity bit;  parity_odd  0 = even, 1 = odd
//     stop2      two stop bits instead of one
//     flow_en    RTS/CTS hardware flow control
//     loopback   connect TX to RX inside the chip (for self-tests)
//
// TRANSMIT: when the TX FIFO has a byte (and CTS allows it) the TX engine
// loads it into a shift register and sends one bit every 16 baud ticks.
//
// RECEIVE: the RX pin is cleaned by ssc_sync_filter.  The RX engine waits
// for a falling edge (start bit), then for every bit takes three samples at
// ticks 7, 8 and 9 of 16 - the middle of the bit - and keeps the majority.
// A noise spike can therefore corrupt at most one of the three samples.
//
// Errors (each is a one-clock event for the interrupt controller):
//     parity  - parity bit wrong (the byte is still stored)
//     frame   - stop bit was 0 (byte dropped)
//     break   - the line was held low for a whole frame (byte dropped)
//     rx_ovf  - a byte arrived while the RX FIFO was full (byte dropped)
//     tx_ovf  - software wrote while the TX FIFO was full (byte dropped)
//
// Flow control: rts_n tells the other side "stop sending" (goes high) when
// our RX FIFO holds 14 or more bytes.  We only start a new frame while the
// other side holds cts_n low.
// =============================================================================
`timescale 1ns / 1ps
module ssc_uart (
    input  wire        clk,
    input  wire        rst_n,
    // settings
    input  wire        tx_en,
    input  wire        rx_en,
    input  wire [1:0]  data_bits,
    input  wire        parity_en,
    input  wire        parity_odd,
    input  wire        stop2,
    input  wire        flow_en,
    input  wire        loopback,
    input  wire [15:0] baud_int,
    input  wire [3:0]  baud_frac,
    input  wire        tx_flush,
    input  wire        rx_flush,
    // FIFO access (register bank or bridge engine)
    input  wire        tx_push,
    input  wire [7:0]  tx_data,
    input  wire        rx_pop,
    output wire [7:0]  rx_data,
    // status
    output wire        tx_empty,
    output wire        tx_full,
    output wire        rx_empty,
    output wire        rx_full,
    output wire [4:0]  tx_count,
    output wire [4:0]  rx_count,
    output wire        tx_busy,
    output wire        rx_busy,
    output wire        cts_ok,
    // events (one clock wide)
    output reg         ev_parity_err,
    output reg         ev_frame_err,
    output reg         ev_break,
    output reg         ev_rx_ovf,
    output wire        ev_tx_ovf,
    // pins
    input  wire        rxd,
    output wire        txd,
    input  wire        cts_n,
    output reg         rts_n
);
    // ------------------------------------------------------------------
    // clock generator: 16 ticks per bit
    // ------------------------------------------------------------------
    wire tick;
    ssc_baud_gen u_baud (
        .clk(clk), .rst_n(rst_n), .en(tx_en | rx_en),
        .div_int(baud_int), .div_frac(baud_frac), .tick(tick));

    wire [3:0] nbits = 4'd5 + {2'b00, data_bits};      // 5..8
    wire [7:0] mask  = 8'hFF >> (4'd8 - nbits);        // keeps the data bits

    // ------------------------------------------------------------------
    // FIFOs
    // ------------------------------------------------------------------
    wire       txf_pop;
    wire [7:0] txf_data;
    reg        rxf_push;
    reg  [7:0] rxf_wdata;

    ssc_fifo #(.WIDTH(8), .AW(4)) u_txf (
        .clk(clk), .rst_n(rst_n), .flush(tx_flush),
        .wr_en(tx_push), .wr_data(tx_data),
        .rd_en(txf_pop), .rd_data(txf_data),
        .empty(tx_empty), .full(tx_full), .count(tx_count));

    ssc_fifo #(.WIDTH(8), .AW(4)) u_rxf (
        .clk(clk), .rst_n(rst_n), .flush(rx_flush),
        .wr_en(rxf_push), .wr_data(rxf_wdata),
        .rd_en(rx_pop), .rd_data(rx_data),
        .empty(rx_empty), .full(rx_full), .count(rx_count));

    assign ev_tx_ovf = tx_push & tx_full;

    // ------------------------------------------------------------------
    // input pins: synchroniser + glitch filter
    // ------------------------------------------------------------------
    reg  txd_r;                                    // TX engine output
    wire rx_raw = loopback ? txd_r : rxd;
    wire rx_s;                                     // clean RX line
    wire cts_n_s;

    ssc_sync_filter #(.FILTER_LEN(4), .RESET_VAL(1'b1)) u_rx_filt (
        .clk(clk), .rst_n(rst_n), .din(rx_raw), .dout(rx_s), .rise(), .fall());
    ssc_sync_filter #(.FILTER_LEN(4), .RESET_VAL(1'b1)) u_cts_filt (
        .clk(clk), .rst_n(rst_n), .din(cts_n), .dout(cts_n_s), .rise(), .fall());

    assign txd    = loopback ? 1'b1 : txd_r;       // pin stays idle in loopback
    assign cts_ok = loopback | ~flow_en | ~cts_n_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rts_n <= 1'b1;
        else        rts_n <= flow_en ? (rx_count >= 5'd14) : 1'b0;
    end

    // ------------------------------------------------------------------
    // TX engine
    // ------------------------------------------------------------------
    localparam TX_IDLE  = 3'd0,
               TX_START = 3'd1,
               TX_DATA  = 3'd2,
               TX_PAR   = 3'd3,
               TX_STOP  = 3'd4;

    reg [2:0] tx_state;
    reg [3:0] tx_tick;          // 0..15 inside the current bit
    reg [2:0] tx_bit;           // data bit being sent
    reg [7:0] tx_shift;
    reg       tx_par;
    reg       tx_stop_2nd;      // sending the second stop bit

    // the last tick of the (final) stop bit: a new frame may follow at once
    wire tx_stop_end = (tx_state == TX_STOP) && (tx_tick == 4'd15) &&
                       !(stop2 && !tx_stop_2nd);
    wire tx_load     = tick & tx_en & ~tx_empty & cts_ok &
                       ((tx_state == TX_IDLE) | tx_stop_end);

    assign txf_pop = tx_load;
    assign tx_busy = (tx_state != TX_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            tx_tick     <= 4'd0;
            tx_bit      <= 3'd0;
            tx_shift    <= 8'd0;
            tx_par      <= 1'b0;
            tx_stop_2nd <= 1'b0;
            txd_r       <= 1'b1;
        end else if (!tx_en) begin
            tx_state <= TX_IDLE;
            txd_r    <= 1'b1;
        end else if (tick) begin
            if (tx_load) begin                       // start bit of a new frame
                tx_shift <= txf_data & mask;
                tx_par   <= (^(txf_data & mask)) ^ parity_odd;
                tx_tick  <= 4'd0;
                tx_state <= TX_START;
                txd_r    <= 1'b0;
            end else begin
                tx_tick <= tx_tick + 4'd1;           // wraps 15 -> 0
                case (tx_state)
                    TX_IDLE: txd_r <= 1'b1;
                    TX_START:
                        if (tx_tick == 4'd15) begin
                            tx_bit   <= 3'd0;
                            tx_state <= TX_DATA;
                            txd_r    <= tx_shift[0];
                        end
                    TX_DATA:
                        if (tx_tick == 4'd15) begin
                            if ({1'b0, tx_bit} == nbits - 4'd1) begin
                                if (parity_en) begin
                                    tx_state <= TX_PAR;
                                    txd_r    <= tx_par;
                                end else begin
                                    tx_state    <= TX_STOP;
                                    tx_stop_2nd <= 1'b0;
                                    txd_r       <= 1'b1;
                                end
                            end else begin
                                tx_bit   <= tx_bit + 3'd1;
                                tx_shift <= {1'b0, tx_shift[7:1]};
                                txd_r    <= tx_shift[1];
                            end
                        end
                    TX_PAR:
                        if (tx_tick == 4'd15) begin
                            tx_state    <= TX_STOP;
                            tx_stop_2nd <= 1'b0;
                            txd_r       <= 1'b1;
                        end
                    TX_STOP:
                        if (tx_tick == 4'd15) begin
                            if (stop2 && !tx_stop_2nd) tx_stop_2nd <= 1'b1;
                            else                       tx_state    <= TX_IDLE;
                        end
                    default: tx_state <= TX_IDLE;
                endcase
            end
        end
    end

    // ------------------------------------------------------------------
    // RX engine
    // ------------------------------------------------------------------
    localparam RX_IDLE  = 3'd0,
               RX_START = 3'd1,
               RX_DATA  = 3'd2,
               RX_PAR   = 3'd3,
               RX_STOP  = 3'd4,
               RX_WAIT  = 3'd5;   // after a framing error / break: wait for idle

    reg [2:0] rx_state;
    reg [3:0] rx_tick;            // sample number inside the current bit
    reg [2:0] rx_bit;
    reg [7:0] rx_shift;
    reg [1:0] rx_votes;           // samples 7 and 8
    reg       rx_par_bit;

    // majority of samples 7, 8 and 9 (sample 9 is the current one)
    wire       vote    = (rx_votes[0] & rx_votes[1]) |
                         (rx_votes[0] & rx_s) | (rx_votes[1] & rx_s);
    // bits came in lowest first from the top, so line them up at bit 0
    wire [7:0] rx_word = rx_shift >> (4'd8 - nbits);

    assign rx_busy = (rx_state != RX_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state      <= RX_IDLE;
            rx_tick       <= 4'd0;
            rx_bit        <= 3'd0;
            rx_shift      <= 8'd0;
            rx_votes      <= 2'b11;
            rx_par_bit    <= 1'b0;
            rxf_push      <= 1'b0;
            rxf_wdata     <= 8'd0;
            ev_parity_err <= 1'b0;
            ev_frame_err  <= 1'b0;
            ev_break      <= 1'b0;
            ev_rx_ovf     <= 1'b0;
        end else begin
            rxf_push      <= 1'b0;
            ev_parity_err <= 1'b0;
            ev_frame_err  <= 1'b0;
            ev_break      <= 1'b0;
            ev_rx_ovf     <= 1'b0;

            if (!rx_en) begin
                rx_state <= RX_IDLE;
            end else if (tick) begin
                if (rx_tick == 4'd7) rx_votes[0] <= rx_s;
                if (rx_tick == 4'd8) rx_votes[1] <= rx_s;

                case (rx_state)
                    RX_IDLE: begin
                        if (!rx_s) begin            // possible start bit
                            rx_tick  <= 4'd1;       // this tick was sample 0
                            rx_state <= RX_START;
                        end else begin
                            rx_tick <= 4'd0;
                        end
                    end
                    RX_START: begin
                        rx_tick <= rx_tick + 4'd1;
                        if (rx_tick == 4'd9 && vote)  // middle of start bit is 1:
                            rx_state <= RX_IDLE;      // it was only a glitch
                        else if (rx_tick == 4'd15) begin
                            rx_bit   <= 3'd0;
                            rx_state <= RX_DATA;
                        end
                    end
                    RX_DATA: begin
                        rx_tick <= rx_tick + 4'd1;
                        if (rx_tick == 4'd9)
                            rx_shift <= {vote, rx_shift[7:1]};
                        if (rx_tick == 4'd15) begin
                            if ({1'b0, rx_bit} == nbits - 4'd1)
                                rx_state <= parity_en ? RX_PAR : RX_STOP;
                            else
                                rx_bit <= rx_bit + 3'd1;
                        end
                    end
                    RX_PAR: begin
                        rx_tick <= rx_tick + 4'd1;
                        if (rx_tick == 4'd9)  rx_par_bit <= vote;
                        if (rx_tick == 4'd15) rx_state   <= RX_STOP;
                    end
                    RX_STOP: begin
                        rx_tick <= rx_tick + 4'd1;
                        if (rx_tick == 4'd9) begin
                            if (vote) begin                       // good stop bit
                                if (parity_en &&
                                    (rx_par_bit != ((^rx_word) ^ parity_odd)))
                                    ev_parity_err <= 1'b1;
                                if (rx_full)
                                    ev_rx_ovf <= 1'b1;
                                else begin
                                    rxf_push  <= 1'b1;
                                    rxf_wdata <= rx_word;
                                end
                                rx_state <= RX_IDLE;              // resync early
                            end else begin
                                if (rx_word == 8'd0 && !(parity_en && rx_par_bit))
                                    ev_break <= 1'b1;
                                else
                                    ev_frame_err <= 1'b1;
                                rx_state <= RX_WAIT;
                            end
                        end
                    end
                    RX_WAIT: if (rx_s) rx_state <= RX_IDLE;
                    default: rx_state <= RX_IDLE;
                endcase
            end
        end
    end
endmodule

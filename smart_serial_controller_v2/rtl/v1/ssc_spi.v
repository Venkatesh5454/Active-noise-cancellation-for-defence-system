// =============================================================================
// ssc_spi.v  -  SPI block: TX/RX FIFOs + master engine + slave engine
// -----------------------------------------------------------------------------
// SPI moves one bit out (MOSI) and one bit in (MISO) on every clock, so every
// word sent also brings one word back.  Settings from the register bank:
//
//     cpol       idle level of SCLK (0 = low, 1 = high)
//     cpha       0 = sample on the first (leading) edge, change on the second
//                1 = change on the leading edge, sample on the trailing edge
//                (CPOL/CPHA together give SPI modes 0..3)
//     len_m1     word length minus one: 3..31 -> 4..32-bit words
//     lsb_first  0 = most significant bit first (normal), 1 = LSB first
//     clk_div    SCLK = f_clk / (2 x (clk_div + 1)); 49 -> 1 MHz.
//                Minimum 3 (12.5 MHz) because MISO goes through a 2-FF
//                synchroniser.
//     cs_sel     which of the 4 chip selects to use
//     cs_manual  0 = automatic: CS goes low while the TX FIFO has words and
//                    goes high when it runs empty
//                1 = software drives CS with cs_level (needed when a command
//                    is longer than what you can queue at once)
//     slave_mode use the slave engine (an outside master drives the clock)
//     loopback   MISO := MOSI inside the chip (self-test)
//     abort      stop at once, release CS, return SCLK to idle, flush TX
//
// MASTER: a half-period counter makes SCLK.  A word takes 2 x N SCLK edges;
// edges 0, 2, 4 .. are "leading", 1, 3, 5 .. are "trailing".  On the sample
// edge we shift MISO into the RX shifter; on the change edge we put the next
// bit on MOSI.  After the last edge the word goes into the RX FIFO.
//
// SLAVE: the outside master's SCLK, MOSI and CS# go through synchronisers;
// edges are detected inside our clock domain, so the outside SCLK must be
// at most about f_clk / 25 (4 MHz at 100 MHz).  Received words go to the RX
// FIFO; the reply comes from the TX FIFO (zeros if it is empty).
// =============================================================================
`timescale 1ns / 1ps
module ssc_spi (
    input  wire        clk,
    input  wire        rst_n,
    // settings
    input  wire        en,
    input  wire        cpol,
    input  wire        cpha,
    input  wire        lsb_first,
    input  wire [4:0]  len_m1,
    input  wire [1:0]  cs_sel,
    input  wire        cs_manual,
    input  wire        cs_level,
    input  wire        slave_mode,
    input  wire        loopback,
    input  wire [15:0] clk_div,
    input  wire        tx_flush,
    input  wire        rx_flush,
    input  wire        abort,
    // FIFO access
    input  wire        tx_push,
    input  wire [31:0] tx_data,
    input  wire        rx_pop,
    output wire [31:0] rx_data,
    // status
    output wire        tx_empty,
    output wire        tx_full,
    output wire        rx_empty,
    output wire        rx_full,
    output wire [4:0]  tx_count,
    output wire [4:0]  rx_count,
    output wire        busy,
    // events (one clock wide)
    output wire        ev_done,
    output wire        ev_rx_ovf,
    output wire        ev_tx_ovf,
    // master pins
    output reg         sclk,
    output reg         mosi,
    input  wire        miso,
    output reg  [3:0]  cs_n,
    // slave pins
    input  wire        s_sclk,
    input  wire        s_mosi,
    input  wire        s_cs_n,
    output wire        s_miso,
    output wire        s_miso_oe
);
    // ------------------------------------------------------------------
    // word length helpers
    // ------------------------------------------------------------------
    wire [4:0]  lm1   = (len_m1 < 5'd3) ? 5'd3 : len_m1;
    wire [5:0]  nbits = {1'b0, lm1} + 6'd1;       // 4..32
    wire [5:0]  pad   = 6'd32 - nbits;            // unused top bits
    wire [6:0]  last_edge = {nbits, 1'b0} - 7'd1; // 2N-1
    wire [15:0] div   = (clk_div < 16'd3) ? 16'd3 : clk_div;

    // place a word in the TX shifter so that the first bit is at the head
    function [31:0] tx_align;
        input [31:0] w;
        input        lsb;
        input [5:0]  p;
        tx_align = lsb ? w : (w << p);
    endfunction
    // the bit currently at the head of the TX shifter
    function tx_head;
        input [31:0] s;
        input        lsb;
        tx_head = lsb ? s[0] : s[31];
    endfunction
    // move the next bit to the head
    function [31:0] tx_shift;
        input [31:0] s;
        input        lsb;
        tx_shift = lsb ? {1'b0, s[31:1]} : {s[30:0], 1'b0};
    endfunction
    // shift a received bit into the RX shifter
    function [31:0] rx_shift;
        input [31:0] s;
        input        b;
        input        lsb;
        rx_shift = lsb ? {b, s[31:1]} : {s[30:0], b};
    endfunction
    // line the finished RX word up at bit 0 (the shifter started at zero)
    function [31:0] rx_final;
        input [31:0] s;
        input        lsb;
        input [5:0]  p;
        rx_final = lsb ? (s >> p) : s;
    endfunction

    // ------------------------------------------------------------------
    // FIFOs (shared by master and slave engine)
    // ------------------------------------------------------------------
    wire        txf_pop_m, txf_pop_s;
    wire [31:0] txf_data;
    reg         rxf_push_m, rxf_push_s;
    reg  [31:0] rxf_wdata_m, rxf_wdata_s;
    wire        rxf_push  = rxf_push_m | rxf_push_s;
    wire [31:0] rxf_wdata = rxf_push_m ? rxf_wdata_m : rxf_wdata_s;

    ssc_fifo #(.WIDTH(32), .AW(4)) u_txf (
        .clk(clk), .rst_n(rst_n), .flush(tx_flush | abort),
        .wr_en(tx_push), .wr_data(tx_data),
        .rd_en(txf_pop_m | txf_pop_s), .rd_data(txf_data),
        .empty(tx_empty), .full(tx_full), .count(tx_count));

    ssc_fifo #(.WIDTH(32), .AW(4)) u_rxf (
        .clk(clk), .rst_n(rst_n), .flush(rx_flush),
        .wr_en(rxf_push), .wr_data(rxf_wdata),
        .rd_en(rx_pop), .rd_data(rx_data),
        .empty(rx_empty), .full(rx_full), .count(rx_count));

    assign ev_tx_ovf = tx_push & tx_full;

    // ==================================================================
    // MASTER ENGINE
    // ==================================================================
    localparam M_IDLE  = 3'd0,   // SCLK idle, waiting for a word
               M_SETUP = 3'd1,   // CS low, half a period before the first edge
               M_XFER  = 3'd2,   // generating the 2N clock edges
               M_HOLD  = 3'd3,   // half a period after the last edge
               M_GAP   = 3'd4;   // CS high for half a period before idle

    reg [2:0]  m_state;
    reg [15:0] m_cnt;           // half-period counter
    reg [5:0]  m_edge;          // edge number inside the word
    reg [31:0] m_tsh;           // TX shifter
    reg [31:0] m_rsh;           // RX shifter
    reg        m_cs;            // automatic chip-select
    reg        m_done;
    reg        m_ovf;

    (* ASYNC_REG = "TRUE" *) reg [1:0] miso_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) miso_sync <= 2'b00;
        else        miso_sync <= {miso_sync[0], miso};
    end
    wire miso_in = loopback ? mosi : miso_sync[1];

    wire m_run  = en & ~slave_mode & ~abort;
    wire m_half = (m_cnt == div);
    // take the next word: from idle, or straight after the previous word
    wire m_load = m_run & ~tx_empty &
                  ((m_state == M_IDLE) | ((m_state == M_HOLD) & m_half));
    assign txf_pop_m = m_load;

    wire [31:0] m_new    = tx_align(txf_data, lsb_first, pad);
    wire [31:0] m_rsh_in = rx_shift(m_rsh, miso_in, lsb_first);
    wire [31:0] m_tsh_nx = tx_shift(m_tsh, lsb_first);
    wire        m_lead   = ~m_edge[0];          // even edge = leading edge

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_state     <= M_IDLE;
            m_cnt       <= 16'd0;
            m_edge      <= 6'd0;
            m_tsh       <= 32'd0;
            m_rsh       <= 32'd0;
            m_cs        <= 1'b0;
            m_done      <= 1'b0;
            m_ovf       <= 1'b0;
            sclk        <= 1'b0;
            mosi        <= 1'b0;
            rxf_push_m  <= 1'b0;
            rxf_wdata_m <= 32'd0;
        end else begin
            rxf_push_m <= 1'b0;
            m_done     <= 1'b0;
            m_ovf      <= 1'b0;

            if (!m_run) begin
                m_state <= M_IDLE;
                m_cnt   <= 16'd0;
                m_cs    <= 1'b0;
                sclk    <= cpol;
            end else if (m_load) begin
                m_tsh   <= m_new;
                mosi    <= tx_head(m_new, lsb_first);
                m_rsh   <= 32'd0;
                m_edge  <= 6'd0;
                m_cnt   <= 16'd0;
                m_cs    <= 1'b1;
                sclk    <= cpol;
                m_state <= M_SETUP;
            end else begin
                if (m_state == M_IDLE) m_cnt <= 16'd0;
                else                   m_cnt <= m_half ? 16'd0 : m_cnt + 16'd1;

                case (m_state)
                    M_IDLE: sclk <= cpol;
                    M_SETUP:
                        if (m_half) m_state <= M_XFER;
                    M_XFER:
                        if (m_half) begin
                            sclk   <= ~sclk;
                            m_edge <= m_edge + 6'd1;
                            if (m_lead) begin
                                if (!cpha)
                                    m_rsh <= m_rsh_in;                 // sample
                                else if (m_edge != 6'd0) begin         // change
                                    m_tsh <= m_tsh_nx;
                                    mosi  <= tx_head(m_tsh_nx, lsb_first);
                                end
                            end else begin
                                if (cpha)
                                    m_rsh <= m_rsh_in;                 // sample
                                else if ({1'b0, m_edge} != last_edge) begin
                                    m_tsh <= m_tsh_nx;                 // change
                                    mosi  <= tx_head(m_tsh_nx, lsb_first);
                                end
                            end
                            if ({1'b0, m_edge} == last_edge) begin     // word done
                                rxf_wdata_m <= rx_final(cpha ? m_rsh_in : m_rsh,
                                                        lsb_first, pad);
                                if (rx_full) m_ovf      <= 1'b1;
                                else         rxf_push_m <= 1'b1;
                                m_state <= M_HOLD;
                            end
                        end
                    M_HOLD:
                        if (m_half) begin            // (m_load handled above)
                            m_cs    <= 1'b0;
                            m_state <= M_GAP;
                        end
                    M_GAP:
                        if (m_half) begin
                            m_state <= M_IDLE;
                            m_done  <= 1'b1;
                        end
                    default: m_state <= M_IDLE;
                endcase
            end
        end
    end

    // chip selects (registered so they never glitch)
    wire cs_on = en & ~slave_mode & (cs_manual ? cs_level : m_cs);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cs_n <= 4'hF;
        else        cs_n <= ~({3'b000, cs_on} << cs_sel);
    end

    // ==================================================================
    // SLAVE ENGINE
    // ==================================================================
    wire ss_sclk, ss_mosi, ss_cs_n;
    wire ss_sclk_rise, ss_sclk_fall, ss_cs_fall, ss_cs_rise;

    ssc_sync_filter #(.FILTER_LEN(2), .RESET_VAL(1'b0)) u_s_sclk (
        .clk(clk), .rst_n(rst_n), .din(s_sclk),
        .dout(ss_sclk), .rise(ss_sclk_rise), .fall(ss_sclk_fall));
    ssc_sync_filter #(.FILTER_LEN(2), .RESET_VAL(1'b0)) u_s_mosi (
        .clk(clk), .rst_n(rst_n), .din(s_mosi),
        .dout(ss_mosi), .rise(), .fall());
    ssc_sync_filter #(.FILTER_LEN(2), .RESET_VAL(1'b1)) u_s_cs (
        .clk(clk), .rst_n(rst_n), .din(s_cs_n),
        .dout(ss_cs_n), .rise(ss_cs_rise), .fall(ss_cs_fall));

    wire s_on    = en & slave_mode;
    wire s_sel   = s_on & ~ss_cs_n;
    wire s_lead  = cpol ? ss_sclk_fall : ss_sclk_rise;
    wire s_trail = cpol ? ss_sclk_rise : ss_sclk_fall;

    reg [31:0] s_tsh;
    reg [31:0] s_rsh;
    reg [5:0]  s_cnt;          // bits sampled in the current word
    reg        s_first;        // CPHA=1: next leading edge drives the head bit
    reg        s_miso_r;
    reg        s_active;       // a CS# frame is in progress
    reg        s_done;
    reg        s_ovf;

    wire [31:0] s_rsh_in = rx_shift(s_rsh, ss_mosi, lsb_first);
    wire [31:0] s_tsh_nx = tx_shift(s_tsh, lsb_first);
    // load a new reply word: at CS# fall, or when a word has just completed
    wire s_word_end = s_trail & (cpha ? (s_cnt == nbits - 6'd1) : (s_cnt == nbits));
    wire s_load     = s_on & ((ss_cs_fall) | (s_sel & s_word_end));
    assign txf_pop_s = s_load & ~tx_empty;
    wire [31:0] s_new = tx_empty ? 32'd0 : tx_align(txf_data, lsb_first, pad);

    assign s_miso    = s_miso_r;
    assign s_miso_oe = s_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_tsh       <= 32'd0;
            s_rsh       <= 32'd0;
            s_cnt       <= 6'd0;
            s_first     <= 1'b1;
            s_miso_r    <= 1'b0;
            s_active    <= 1'b0;
            s_done      <= 1'b0;
            s_ovf       <= 1'b0;
            rxf_push_s  <= 1'b0;
            rxf_wdata_s <= 32'd0;
        end else begin
            rxf_push_s <= 1'b0;
            s_done     <= 1'b0;
            s_ovf      <= 1'b0;

            if (!s_on) begin
                s_cnt    <= 6'd0;
                s_active <= 1'b0;
            end else begin
                if (ss_cs_fall) begin                         // frame starts
                    s_tsh    <= s_new;
                    s_miso_r <= tx_head(s_new, lsb_first);
                    s_rsh    <= 32'd0;
                    s_cnt    <= 6'd0;
                    s_first  <= 1'b1;
                    s_active <= 1'b1;
                end else if (s_sel) begin
                    if (s_lead) begin
                        if (!cpha) begin                      // sample
                            s_rsh <= s_rsh_in;
                            s_cnt <= s_cnt + 6'd1;
                            if (s_cnt == nbits - 6'd1) begin
                                rxf_wdata_s <= rx_final(s_rsh_in, lsb_first, pad);
                                if (rx_full) s_ovf      <= 1'b1;
                                else         rxf_push_s <= 1'b1;
                            end
                        end else begin                        // change
                            if (s_first) begin
                                s_miso_r <= tx_head(s_tsh, lsb_first);
                                s_first  <= 1'b0;
                            end else begin
                                s_tsh    <= s_tsh_nx;
                                s_miso_r <= tx_head(s_tsh_nx, lsb_first);
                            end
                        end
                    end
                    if (s_trail) begin
                        if (!cpha) begin                      // change
                            if (s_word_end) begin             // next word
                                s_tsh    <= s_new;
                                s_miso_r <= tx_head(s_new, lsb_first);
                                s_rsh    <= 32'd0;
                                s_cnt    <= 6'd0;
                            end else begin
                                s_tsh    <= s_tsh_nx;
                                s_miso_r <= tx_head(s_tsh_nx, lsb_first);
                            end
                        end else begin                        // sample
                            if (s_word_end) begin
                                rxf_wdata_s <= rx_final(s_rsh_in, lsb_first, pad);
                                if (rx_full) s_ovf      <= 1'b1;
                                else         rxf_push_s <= 1'b1;
                                s_tsh   <= s_new;
                                s_rsh   <= 32'd0;
                                s_cnt   <= 6'd0;
                                s_first <= 1'b1;
                            end else begin
                                s_rsh <= s_rsh_in;
                                s_cnt <= s_cnt + 6'd1;
                            end
                        end
                    end
                end
                if (ss_cs_rise && s_active) begin             // frame ends
                    s_active <= 1'b0;
                    s_done   <= 1'b1;
                end
            end
        end
    end

    assign busy      = (m_state != M_IDLE) | s_sel;
    assign ev_done   = m_done | s_done;
    assign ev_rx_ovf = m_ovf | s_ovf;
endmodule

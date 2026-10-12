// =============================================================================
// ssc_i2c.v  -  I2C master block: TX/RX FIFOs + bus monitor + master engine
// -----------------------------------------------------------------------------
// I2C uses two open-drain wires.  Nobody ever drives a 1: a device either
// pulls the line to 0 or lets go and a pull-up resistor makes it 1.  So this
// block has, for each line, an input (what the wire really is) and an "oe"
// output (1 = pull the line low).  The top level turns that into a tri-state
// pad:  assign SDA = sda_oe ? 1'b0 : 1'bz;
//
// One SCL period is split into four quarter-periods T:
//
//            S_BD   S_BA   S_BB   S_BC
//     SCL  ___|______________|``````````````|___
//     SDA  ======X=========== stable ===========    (SDA only changes in S_BA)
//                ^ data changes      ^ sampled when SCL is seen high
//
//     T = prescale + 1 clocks,  f_SCL ~= f_clk / (4 x (prescale + 1))
//     100 kHz: prescale = 249        400 kHz: prescale = 62
//
// A transaction is started by writing the CMD register:
//     len      number of data bytes
//     read     0 = write len bytes from the TX FIFO, 1 = read len bytes
//     stop     1 = finish with STOP; 0 = keep the bus, so the next command
//                  starts with a REPEATED START (write-then-read like the
//                  PmodTMP2 temperature read)
//     stop_only  just send a STOP (release a held bus)
//
// Robustness features (the "real I2C" part):
//   * clock stretching : after releasing SCL we wait until it is REALLY high;
//                        a slow slave may hold it low as long as it likes
//   * clock sync       : if another master pulls SCL low early, we follow
//   * arbitration      : if we send a 1 but the bus shows 0, another master
//                        won - we let go of both lines at once, raise ARB_LOST
//   * bus busy         : a START seen on the bus marks it busy until the STOP;
//                        we never start while someone else owns the bus
//   * NACK             : if the slave does not acknowledge, we send STOP and
//                        raise NACK
//   * FIFO stalls      : TX FIFO empty or RX FIFO full -> we hold SCL low (a
//                        legal pause) until software catches up
//   * 10-bit addresses : 11110 A9 A8 W, A7..A0 [, Sr, 11110 A9 A8 R]
//   * auto_wr          : bridge mode - every byte in the TX FIFO becomes a
//                        1-byte write (+STOP) to ADDR without the CPU
// =============================================================================
`timescale 1ns / 1ps
module ssc_i2c (
    input  wire        clk,
    input  wire        rst_n,
    // settings
    input  wire        en,
    input  wire        auto_wr,
    input  wire [15:0] prescale,
    input  wire [9:0]  addr,
    input  wire        ten_bit,
    // command (one-clock strobe from the register bank)
    input  wire        cmd_valid,
    input  wire [7:0]  cmd_len,
    input  wire        cmd_read,
    input  wire        cmd_stop,
    input  wire        cmd_stop_only,
    input  wire        tx_flush,
    input  wire        rx_flush,
    input  wire        abort,
    // FIFO access
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
    output wire        busy,
    output reg         holding,      // we keep the bus (no STOP yet)
    output reg         bus_busy,     // somebody (maybe us) owns the bus
    output reg         nack_flag,    // last command ended with a NACK
    output reg         arb_flag,     // last command lost arbitration
    output wire        scl_state,
    output wire        sda_state,
    // events (one clock wide)
    output reg         ev_done,
    output reg         ev_nack,
    output reg         ev_arb_lost,
    output wire        ev_tx_ovf,
    // open-drain pins
    input  wire        scl_in,
    output wire        scl_oe,       // 1 = pull SCL low
    input  wire        sda_in,
    output wire        sda_oe        // 1 = pull SDA low
);
    // ------------------------------------------------------------------
    // FIFOs
    // ------------------------------------------------------------------
    reg        txf_pop;
    wire [7:0] txf_data;
    reg        rxf_push;
    reg  [7:0] rxf_wdata;

    ssc_fifo #(.WIDTH(8), .AW(4)) u_txf (
        .clk(clk), .rst_n(rst_n), .flush(tx_flush | abort),
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
    // pins: synchroniser + glitch filter (also gives ~50 ns spike
    // suppression, like the I2C fast-mode spec asks for)
    // ------------------------------------------------------------------
    wire scl_s, sda_s, sda_rise, sda_fall;
    ssc_sync_filter #(.FILTER_LEN(4), .RESET_VAL(1'b1)) u_scl_filt (
        .clk(clk), .rst_n(rst_n), .din(scl_in), .dout(scl_s), .rise(), .fall());
    ssc_sync_filter #(.FILTER_LEN(4), .RESET_VAL(1'b1)) u_sda_filt (
        .clk(clk), .rst_n(rst_n), .din(sda_in), .dout(sda_s),
        .rise(sda_rise), .fall(sda_fall));
    assign scl_state = scl_s;
    assign sda_state = sda_s;

    reg scl_low, sda_low;
    assign scl_oe = scl_low;
    assign sda_oe = sda_low;

    // quarter period must be longer than the input filter delay
    wire [15:0] presc = (prescale < 16'd7) ? 16'd7 : prescale;

    // ------------------------------------------------------------------
    // bus monitor: START = SDA falls while SCL high, STOP = SDA rises
    // while SCL high.  If both lines stay high for 8 quarter periods the
    // bus is treated as free (recovers from a missed STOP).
    // ------------------------------------------------------------------
    reg  [19:0] idle_cnt;
    wire [19:0] idle_lim = {1'b0, presc, 3'b000};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bus_busy <= 1'b0;
            idle_cnt <= 20'd0;
        end else begin
            if (scl_s & sda_s) begin
                if (idle_cnt != 20'hFFFFF) idle_cnt <= idle_cnt + 20'd1;
            end else
                idle_cnt <= 20'd0;

            if (scl_s & sda_fall)          bus_busy <= 1'b1;
            else if (scl_s & sda_rise)     bus_busy <= 1'b0;
            else if (idle_cnt > idle_lim)  bus_busy <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // master engine
    // ------------------------------------------------------------------
    localparam S_IDLE = 4'd0,   // bus released
               S_RS   = 4'd1,   // repeated START: SDA up while SCL low
               S_ST1  = 4'd2,   // SCL up (wait for it), SDA high
               S_ST2  = 4'd3,   // START: SDA down while SCL high
               S_ST3  = 4'd4,   // SCL down
               S_BA   = 4'd5,   // bit: SCL low, drive SDA
               S_BB   = 4'd6,   // bit: release SCL, wait high, sample SDA
               S_BC   = 4'd7,   // bit: SCL high
               S_BD   = 4'd8,   // bit: SCL low again
               S_P0   = 4'd9,   // STOP: SDA low while SCL low
               S_P1   = 4'd10,  // STOP: SCL up (wait for it)
               S_P2   = 4'd11,  // STOP: SDA up while SCL high
               S_P3   = 4'd12,  // bus free time
               S_HOLD = 4'd13,  // own the bus, SCL low, wait for next command
               S_WTX  = 4'd14,  // wait for a byte in the TX FIFO (SCL low)
               S_WRX  = 4'd15;  // wait for room in the RX FIFO (SCL low)

    reg [3:0]  st;
    reg [15:0] qcnt;            // quarter-period down counter
    reg        high_seen;       // SCL has been seen high in this phase
    // command waiting to run
    reg        pending, p_read, p_stop, p_stop_only;
    reg [7:0]  p_len;
    // running transaction
    reg        t_read, t_stop;
    reg [7:0]  remaining;       // data bytes still to transfer
    reg [1:0]  stage;           // 0: addr byte 1, 1: addr byte 2 (10-bit),
                                // 2: addr byte after Sr (10-bit read), 3: data
    reg [7:0]  sh;              // byte shift register
    reg [3:0]  bitn;            // 0..7 data bits, 8 = ACK bit
    reg        rx_byte;         // this byte comes from the slave
    reg        sample;          // SDA sampled at the SCL rising edge

    wire qdone = (qcnt == 16'd0);

    // the level we put on SDA for the current bit
    //   data bit we send   : the bit itself
    //   data bit we receive: 1 (let go)
    //   ACK bit after a byte we sent    : 1 (let go, the slave answers)
    //   ACK bit after a byte we received: 0 = ACK, 1 = NACK on the last byte
    wire my_bit  = (bitn == 4'd8) ? (rx_byte ? (remaining == 8'd1) : 1'b1)
                                  : (rx_byte ? 1'b1 : sh[7]);
    wire sending = ~rx_byte & (bitn != 4'd8);   // bits that take part in arbitration

    assign busy = pending | ~((st == S_IDLE) | (st == S_HOLD));

    // start the next byte of the transaction (stg = which byte)
    task load_byte;
        input [1:0] stg;
        begin
            bitn <= 4'd0;
            qcnt <= presc;
            case (stg)
                2'd0: begin
                    sh      <= ten_bit ? {5'b11110, addr[9:8], 1'b0} : {addr[6:0], t_read};
                    rx_byte <= 1'b0;
                    st      <= S_BA;
                end
                2'd1: begin
                    sh      <= addr[7:0];
                    rx_byte <= 1'b0;
                    st      <= S_BA;
                end
                2'd2: begin
                    sh      <= {5'b11110, addr[9:8], 1'b1};
                    rx_byte <= 1'b0;
                    st      <= S_BA;
                end
                default: begin
                    if (t_read) begin
                        sh      <= 8'hFF;
                        rx_byte <= 1'b1;
                        st      <= S_BA;
                    end else if (!tx_empty) begin
                        sh      <= txf_data;
                        txf_pop <= 1'b1;
                        rx_byte <= 1'b0;
                        st      <= S_BA;
                    end else
                        st <= S_WTX;
                end
            endcase
        end
    endtask

    // all bytes done: STOP, or keep the bus for a repeated START
    task finish;
        begin
            qcnt <= presc;
            if (t_stop) st <= S_P0;
            else begin
                st      <= S_HOLD;
                holding <= 1'b1;
                ev_done <= 1'b1;
            end
        end
    endtask

    // arbitration lost: let go of both lines immediately
    task lose;
        begin
            scl_low     <= 1'b0;
            sda_low     <= 1'b0;
            arb_flag    <= 1'b1;
            ev_arb_lost <= 1'b1;
            ev_done     <= 1'b1;
            holding     <= 1'b0;
            st          <= S_IDLE;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            qcnt        <= 16'd0;
            high_seen   <= 1'b0;
            pending     <= 1'b0;
            p_read      <= 1'b0;
            p_stop      <= 1'b0;
            p_stop_only <= 1'b0;
            p_len       <= 8'd0;
            t_read      <= 1'b0;
            t_stop      <= 1'b0;
            remaining   <= 8'd0;
            stage       <= 2'd0;
            sh          <= 8'd0;
            bitn        <= 4'd0;
            rx_byte     <= 1'b0;
            sample      <= 1'b1;
            scl_low     <= 1'b0;
            sda_low     <= 1'b0;
            holding     <= 1'b0;
            nack_flag   <= 1'b0;
            arb_flag    <= 1'b0;
            ev_done     <= 1'b0;
            ev_nack     <= 1'b0;
            ev_arb_lost <= 1'b0;
            txf_pop     <= 1'b0;
            rxf_push    <= 1'b0;
            rxf_wdata   <= 8'd0;
        end else begin
            ev_done     <= 1'b0;
            ev_nack     <= 1'b0;
            ev_arb_lost <= 1'b0;
            txf_pop     <= 1'b0;
            rxf_push    <= 1'b0;

            // accept a command while idle or while holding the bus
            if (cmd_valid && en && !pending && (st == S_IDLE || st == S_HOLD)) begin
                pending     <= 1'b1;
                p_len       <= cmd_len;
                p_read      <= cmd_read;
                p_stop      <= cmd_stop;
                p_stop_only <= cmd_stop_only;
                nack_flag   <= 1'b0;
                arb_flag    <= 1'b0;
            end

            if (abort || !en) begin
                st      <= S_IDLE;
                scl_low <= 1'b0;
                sda_low <= 1'b0;
                holding <= 1'b0;
                pending <= 1'b0;
            end else begin
                case (st)
                // ---------------------------------------------------------
                S_IDLE: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    holding <= 1'b0;
                    if (pending && !bus_busy) begin
                        pending <= 1'b0;
                        if (p_stop_only)
                            ev_done <= 1'b1;               // nothing to stop
                        else begin
                            t_read    <= p_read;
                            t_stop    <= p_stop;
                            remaining <= p_len;
                            stage     <= 2'd0;
                            high_seen <= 1'b0;
                            qcnt      <= presc;
                            st        <= S_ST1;
                        end
                    end else if (!pending && auto_wr && !tx_empty && !bus_busy) begin
                        // bridge mode: one byte -> one write transaction
                        t_read    <= 1'b0;
                        t_stop    <= 1'b1;
                        remaining <= 8'd1;
                        stage     <= 2'd0;
                        nack_flag <= 1'b0;
                        arb_flag  <= 1'b0;
                        high_seen <= 1'b0;
                        qcnt      <= presc;
                        st        <= S_ST1;
                    end
                end
                // ---------------------------------------------------------
                S_HOLD: begin
                    scl_low <= 1'b1;
                    sda_low <= 1'b0;
                    holding <= 1'b1;
                    if (pending) begin
                        pending <= 1'b0;
                        qcnt    <= presc;
                        if (p_stop_only) begin
                            t_stop <= 1'b1;
                            st     <= S_P0;
                        end else begin
                            t_read    <= p_read;
                            t_stop    <= p_stop;
                            remaining <= p_len;
                            stage     <= 2'd0;
                            st        <= S_RS;
                        end
                    end
                end
                // ---------------------------------------------------------
                // START / repeated START
                // ---------------------------------------------------------
                S_RS: begin
                    scl_low <= 1'b1;
                    sda_low <= 1'b0;
                    holding <= 1'b0;
                    if (qdone) begin
                        high_seen <= 1'b0;
                        qcnt      <= presc;
                        st        <= S_ST1;
                    end else qcnt <= qcnt - 16'd1;
                end
                S_ST1: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    if (!high_seen) begin
                        if (scl_s) begin               // (a slave may stretch)
                            high_seen <= 1'b1;
                            qcnt      <= presc;
                        end
                    end else if (qdone) begin
                        if (!sda_s) lose;              // someone else holds SDA
                        else begin
                            qcnt <= presc;
                            st   <= S_ST2;
                        end
                    end else qcnt <= qcnt - 16'd1;
                end
                S_ST2: begin
                    sda_low <= 1'b1;                   // the START condition
                    if (qdone) begin
                        qcnt <= presc;
                        st   <= S_ST3;
                    end else qcnt <= qcnt - 16'd1;
                end
                S_ST3: begin
                    scl_low <= 1'b1;
                    if (qdone) load_byte(stage);
                    else       qcnt <= qcnt - 16'd1;
                end
                // ---------------------------------------------------------
                // one bit (data or ACK)
                // ---------------------------------------------------------
                S_BA: begin
                    scl_low <= 1'b1;
                    sda_low <= ~my_bit;
                    if (qdone) begin
                        high_seen <= 1'b0;
                        qcnt      <= presc;
                        st        <= S_BB;
                    end else qcnt <= qcnt - 16'd1;
                end
                S_BB: begin
                    scl_low <= 1'b0;
                    if (!high_seen) begin
                        if (scl_s) begin                // SCL really is high now
                            high_seen <= 1'b1;
                            qcnt      <= presc;
                            sample    <= sda_s;
                            if (sending && my_bit && !sda_s)
                                lose;                   // we sent 1, bus says 0
                        end
                    end else if (!scl_s) begin          // another master pulled
                        qcnt <= presc;                  // SCL low early: follow
                        st   <= S_BD;
                    end else if (qdone) begin
                        qcnt <= presc;
                        st   <= S_BC;
                    end else qcnt <= qcnt - 16'd1;
                end
                S_BC: begin
                    if (!scl_s || qdone) begin
                        qcnt <= presc;
                        st   <= S_BD;
                    end else qcnt <= qcnt - 16'd1;
                end
                S_BD: begin
                    scl_low <= 1'b1;
                    if (!qdone)
                        qcnt <= qcnt - 16'd1;
                    else if (bitn != 4'd8) begin
                        // next bit of the byte (or the ACK bit)
                        sh   <= {sh[6:0], sample};
                        bitn <= bitn + 4'd1;
                        qcnt <= presc;
                        if (bitn == 4'd7 && rx_byte) begin
                            if (rx_full) st <= S_WRX;
                            else begin
                                rxf_push  <= 1'b1;
                                rxf_wdata <= {sh[6:0], sample};
                                st        <= S_BA;
                            end
                        end else
                            st <= S_BA;
                    end else if (!rx_byte) begin
                        // ACK bit of a byte we sent has finished
                        if (sample) begin                       // NACK
                            nack_flag <= 1'b1;
                            ev_nack   <= 1'b1;
                            t_stop    <= 1'b1;
                            qcnt      <= presc;
                            st        <= S_P0;
                        end else begin
                            case (stage)
                                2'd0:
                                    if (ten_bit) begin
                                        stage <= 2'd1;
                                        load_byte(2'd1);
                                    end else begin
                                        stage <= 2'd3;
                                        if (remaining == 8'd0) finish;
                                        else                   load_byte(2'd3);
                                    end
                                2'd1:
                                    if (t_read) begin           // 10-bit read: Sr
                                        stage <= 2'd2;
                                        qcnt  <= presc;
                                        st    <= S_RS;
                                    end else begin
                                        stage <= 2'd3;
                                        if (remaining == 8'd0) finish;
                                        else                   load_byte(2'd3);
                                    end
                                2'd2: begin
                                    stage <= 2'd3;
                                    if (remaining == 8'd0) finish;
                                    else                   load_byte(2'd3);
                                end
                                default: begin
                                    remaining <= remaining - 8'd1;
                                    if (remaining == 8'd1) finish;
                                    else                   load_byte(2'd3);
                                end
                            endcase
                        end
                    end else begin
                        // ACK/NACK we sent after a received byte has finished
                        remaining <= remaining - 8'd1;
                        if (remaining == 8'd1) finish;
                        else                   load_byte(2'd3);
                    end
                end
                S_WRX: begin
                    scl_low <= 1'b1;
                    if (!rx_full) begin
                        rxf_push  <= 1'b1;
                        rxf_wdata <= sh;
                        qcnt      <= presc;
                        st        <= S_BA;
                    end
                end
                S_WTX: begin
                    scl_low <= 1'b1;
                    if (!tx_empty) load_byte(2'd3);
                end
                // ---------------------------------------------------------
                // STOP
                // ---------------------------------------------------------
                S_P0: begin
                    scl_low <= 1'b1;
                    sda_low <= 1'b1;
                    if (qdone) begin
                        high_seen <= 1'b0;
                        qcnt      <= presc;
                        st        <= S_P1;
                    end else qcnt <= qcnt - 16'd1;
                end
                S_P1: begin
                    scl_low <= 1'b0;
                    if (!high_seen) begin
                        if (scl_s) begin
                            high_seen <= 1'b1;
                            qcnt      <= presc;
                        end
                    end else if (qdone) begin
                        qcnt <= presc;
                        st   <= S_P2;
                    end else qcnt <= qcnt - 16'd1;
                end
                S_P2: begin
                    sda_low <= 1'b0;                    // the STOP condition
                    if (qdone) begin
                        if (!sda_s) lose;
                        else begin
                            qcnt <= {presc[14:0], 1'b1}; // 2 quarters of bus-free time
                            st   <= S_P3;
                        end
                    end else qcnt <= qcnt - 16'd1;
                end
                S_P3: begin
                    if (qdone) begin
                        holding <= 1'b0;
                        ev_done <= 1'b1;
                        st      <= S_IDLE;
                    end else qcnt <= qcnt - 16'd1;
                end
                default: st <= S_IDLE;
                endcase
            end
        end
    end
endmodule

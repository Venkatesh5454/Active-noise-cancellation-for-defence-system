// =============================================================================
// ssc_apb_top.v  -  Smart Serial Controller, top of the controller IP (APB)
// -----------------------------------------------------------------------------
//                 +--------------+     +------------+---- UART pins (JA)
//   APB  -------->| register bank|---->| UART block |
//                 |  (ssc_regbank)|    +------------+
//                 |              |---->| SPI block  |---- SPI pins  (JB)
//                 |              |     +------------+
//                 |              |---->| I2C block  |---- I2C pins  (JC)
//                 +--------------+     +------------+
//                        |   bridge engine moves bytes RX FIFO -> TX FIFO
//                        |   interrupt controller -> irq
//
// This file only wires the blocks together.  The FIFO ports of each protocol
// block can be driven by the CPU (through the register bank) or by the bridge
// engine; the bridge waits whenever the CPU is on the bus.
//
// INT_STATUS / INT_ENABLE bits
//   0 UART_RX        RX FIFO not empty (level)
//   1 UART_TX_IDLE   TX FIFO empty and transmitter idle (level)
//   2 UART_PARITY    parity error
//   3 UART_FRAME     framing error
//   4 UART_BREAK     break received
//   5 UART_RX_OVF    byte lost, RX FIFO full
//   6 SPI_DONE       transfer finished (master: queue empty; slave: CS# high)
//   7 SPI_RX         RX FIFO not empty (level)
//   8 SPI_RX_OVF     word lost, RX FIFO full
//   9 I2C_DONE       command finished (also after NACK or arbitration loss)
//  10 I2C_NACK       slave did not acknowledge
//  11 I2C_ARB_LOST   another master won the bus
//  12 I2C_RX         RX FIFO not empty (level)
//  13 UART_TX_OVF    write to a full TX FIFO
//  14 SPI_TX_OVF
//  15 I2C_TX_OVF
// =============================================================================
`timescale 1ns / 1ps
module ssc_apb_top (
    input  wire        pclk,
    input  wire        presetn,
    // APB slave
    input  wire [11:0] paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,
    // interrupt to the CPU
    output wire        irq,
    // UART pins
    input  wire        uart_rxd,
    output wire        uart_txd,
    input  wire        uart_cts_n,
    output wire        uart_rts_n,
    // SPI master pins
    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire [3:0]  spi_cs_n,
    // SPI slave pins
    input  wire        spis_sclk,
    input  wire        spis_mosi,
    input  wire        spis_cs_n,
    output wire        spis_miso,
    output wire        spis_miso_oe,
    output wire        spi_slave_mode,   // tells the pads which way SPI pins go
    // I2C pins (open drain)
    input  wire        i2c_scl_in,
    output wire        i2c_scl_oe,
    input  wire        i2c_sda_in,
    output wire        i2c_sda_oe
);
    wire clk   = pclk;
    wire rst_n = presetn;

    // ------------------------------------------------------------------
    // register bank outputs
    // ------------------------------------------------------------------
    wire        cpu_access;
    wire [15:0] int_status, int_enable;
    wire        int_clear;
    wire [7:0]  bridge_ctrl;
    wire        bridge_cnt_clear;
    wire [15:0] bridge_cnt0, bridge_cnt1;

    wire        u_tx_en, u_rx_en, u_par_en, u_par_odd, u_stop2, u_flow, u_loop;
    wire [1:0]  u_bits;
    wire [15:0] u_baud_int;
    wire [3:0]  u_baud_frac;
    wire        u_tx_flush, u_rx_flush, rb_u_tx_push, rb_u_rx_pop;

    wire        s_en, s_cpol, s_cpha, s_lsb, s_cs_manual, s_cs_level, s_slave, s_loop;
    wire [4:0]  s_len_m1;
    wire [1:0]  s_cs_sel;
    wire [15:0] s_clk_div;
    wire        s_tx_flush, s_rx_flush, s_abort, rb_s_tx_push, rb_s_rx_pop;

    wire        i_en, i_auto, i_ten, i_cmd_valid;
    wire [15:0] i_prescale;
    wire [9:0]  i_addr;
    wire        i_tx_flush, i_rx_flush, i_abort, rb_i_tx_push, rb_i_rx_pop;

    // block status
    wire [31:0] uart_status, spi_status, i2c_status;
    wire [7:0]  uart_rx_data, i2c_rx_data;
    wire [31:0] spi_rx_data;

    ssc_regbank u_regs (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .cpu_access(cpu_access),
        .int_status(int_status), .int_enable(int_enable), .int_clear(int_clear),
        .bridge_ctrl(bridge_ctrl), .bridge_cnt_clear(bridge_cnt_clear),
        .bridge_cnt0(bridge_cnt0), .bridge_cnt1(bridge_cnt1),
        .uart_tx_en(u_tx_en), .uart_rx_en(u_rx_en), .uart_bits(u_bits),
        .uart_par_en(u_par_en), .uart_par_odd(u_par_odd), .uart_stop2(u_stop2),
        .uart_flow(u_flow), .uart_loop(u_loop),
        .uart_baud_int(u_baud_int), .uart_baud_frac(u_baud_frac),
        .uart_tx_flush(u_tx_flush), .uart_rx_flush(u_rx_flush),
        .uart_tx_push(rb_u_tx_push), .uart_rx_pop(rb_u_rx_pop),
        .uart_status(uart_status), .uart_rx_data(uart_rx_data),
        .spi_en(s_en), .spi_cpol(s_cpol), .spi_cpha(s_cpha), .spi_lsb(s_lsb),
        .spi_len_m1(s_len_m1), .spi_cs_sel(s_cs_sel), .spi_cs_manual(s_cs_manual),
        .spi_cs_level(s_cs_level), .spi_slave(s_slave), .spi_loop(s_loop),
        .spi_clk_div(s_clk_div),
        .spi_tx_flush(s_tx_flush), .spi_rx_flush(s_rx_flush), .spi_abort(s_abort),
        .spi_tx_push(rb_s_tx_push), .spi_rx_pop(rb_s_rx_pop),
        .spi_status(spi_status), .spi_rx_data(spi_rx_data),
        .i2c_en(i_en), .i2c_auto(i_auto), .i2c_prescale(i_prescale),
        .i2c_addr(i_addr), .i2c_ten(i_ten), .i2c_cmd_valid(i_cmd_valid),
        .i2c_tx_flush(i_tx_flush), .i2c_rx_flush(i_rx_flush), .i2c_abort(i_abort),
        .i2c_tx_push(rb_i_tx_push), .i2c_rx_pop(rb_i_rx_pop),
        .i2c_status(i2c_status), .i2c_rx_data(i2c_rx_data)
    );

    // ------------------------------------------------------------------
    // bridge engine (index: 1 = SPI, 2 = I2C, 3 = UART)
    // ------------------------------------------------------------------
    wire [3:0] br_rx_pop, br_tx_push;
    wire [7:0] br_tx_data;
    wire [3:0] rx_empty_v = {uart_status[2], i2c_status[2], spi_status[2], 1'b1};
    wire [3:0] tx_full_v  = {uart_status[1], i2c_status[1], spi_status[1], 1'b1};

    ssc_bridge u_bridge (
        .clk(clk), .rst_n(rst_n),
        .ctrl(bridge_ctrl), .stall(cpu_access), .cnt_clear(bridge_cnt_clear),
        .rx_empty(rx_empty_v), .tx_full(tx_full_v),
        .rx_data_spi(spi_rx_data[7:0]), .rx_data_i2c(i2c_rx_data),
        .rx_data_uart(uart_rx_data),
        .rx_pop(br_rx_pop), .tx_push(br_tx_push), .tx_data(br_tx_data),
        .cnt0(bridge_cnt0), .cnt1(bridge_cnt1)
    );

    // FIFO port sharing: CPU or bridge (never both in the same clock)
    wire        u_tx_push = rb_u_tx_push | br_tx_push[3];
    wire [7:0]  u_tx_data = rb_u_tx_push ? pwdata[7:0] : br_tx_data;
    wire        u_rx_pop  = rb_u_rx_pop  | br_rx_pop[3];
    wire        s_tx_push = rb_s_tx_push | br_tx_push[1];
    wire [31:0] s_tx_data = rb_s_tx_push ? pwdata : {24'd0, br_tx_data};
    wire        s_rx_pop  = rb_s_rx_pop  | br_rx_pop[1];
    wire        i_tx_push = rb_i_tx_push | br_tx_push[2];
    wire [7:0]  i_tx_data = rb_i_tx_push ? pwdata[7:0] : br_tx_data;
    wire        i_rx_pop  = rb_i_rx_pop  | br_rx_pop[2];

    // ------------------------------------------------------------------
    // UART block
    // ------------------------------------------------------------------
    wire       u_tx_empty, u_tx_full, u_rx_empty, u_rx_full, u_tx_busy, u_rx_busy, u_cts_ok;
    wire [4:0] u_tx_count, u_rx_count;
    wire       u_ev_par, u_ev_frame, u_ev_break, u_ev_rx_ovf, u_ev_tx_ovf;

    ssc_uart u_uart (
        .clk(clk), .rst_n(rst_n),
        .tx_en(u_tx_en), .rx_en(u_rx_en), .data_bits(u_bits),
        .parity_en(u_par_en), .parity_odd(u_par_odd), .stop2(u_stop2),
        .flow_en(u_flow), .loopback(u_loop),
        .baud_int(u_baud_int), .baud_frac(u_baud_frac),
        .tx_flush(u_tx_flush), .rx_flush(u_rx_flush),
        .tx_push(u_tx_push), .tx_data(u_tx_data),
        .rx_pop(u_rx_pop), .rx_data(uart_rx_data),
        .tx_empty(u_tx_empty), .tx_full(u_tx_full),
        .rx_empty(u_rx_empty), .rx_full(u_rx_full),
        .tx_count(u_tx_count), .rx_count(u_rx_count),
        .tx_busy(u_tx_busy), .rx_busy(u_rx_busy), .cts_ok(u_cts_ok),
        .ev_parity_err(u_ev_par), .ev_frame_err(u_ev_frame), .ev_break(u_ev_break),
        .ev_rx_ovf(u_ev_rx_ovf), .ev_tx_ovf(u_ev_tx_ovf),
        .rxd(uart_rxd), .txd(uart_txd), .cts_n(uart_cts_n), .rts_n(uart_rts_n)
    );

    assign uart_status = {11'd0, u_rx_count, 3'd0, u_tx_count,
                          1'b0, u_cts_ok, u_rx_busy, u_tx_busy,
                          u_rx_full, u_rx_empty, u_tx_full, u_tx_empty};

    // ------------------------------------------------------------------
    // SPI block
    // ------------------------------------------------------------------
    wire       s_tx_empty, s_tx_full, s_rx_empty, s_rx_full, s_busy;
    wire [4:0] s_tx_count, s_rx_count;
    wire       s_ev_done, s_ev_rx_ovf, s_ev_tx_ovf;

    ssc_spi u_spi (
        .clk(clk), .rst_n(rst_n),
        .en(s_en), .cpol(s_cpol), .cpha(s_cpha), .lsb_first(s_lsb),
        .len_m1(s_len_m1), .cs_sel(s_cs_sel), .cs_manual(s_cs_manual),
        .cs_level(s_cs_level), .slave_mode(s_slave), .loopback(s_loop),
        .clk_div(s_clk_div),
        .tx_flush(s_tx_flush), .rx_flush(s_rx_flush), .abort(s_abort),
        .tx_push(s_tx_push), .tx_data(s_tx_data),
        .rx_pop(s_rx_pop), .rx_data(spi_rx_data),
        .tx_empty(s_tx_empty), .tx_full(s_tx_full),
        .rx_empty(s_rx_empty), .rx_full(s_rx_full),
        .tx_count(s_tx_count), .rx_count(s_rx_count), .busy(s_busy),
        .ev_done(s_ev_done), .ev_rx_ovf(s_ev_rx_ovf), .ev_tx_ovf(s_ev_tx_ovf),
        .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso), .cs_n(spi_cs_n),
        .s_sclk(spis_sclk), .s_mosi(spis_mosi), .s_cs_n(spis_cs_n),
        .s_miso(spis_miso), .s_miso_oe(spis_miso_oe)
    );

    assign spi_slave_mode = s_en & s_slave;
    assign spi_status = {11'd0, s_rx_count, 3'd0, s_tx_count,
                         3'd0, s_busy, s_rx_full, s_rx_empty, s_tx_full, s_tx_empty};

    // ------------------------------------------------------------------
    // I2C block
    // ------------------------------------------------------------------
    wire       i_tx_empty, i_tx_full, i_rx_empty, i_rx_full, i_busy;
    wire       i_holding, i_bus_busy, i_nack, i_arb, i_scl, i_sda;
    wire [4:0] i_tx_count, i_rx_count;
    wire       i_ev_done, i_ev_nack, i_ev_arb, i_ev_tx_ovf;

    ssc_i2c u_i2c (
        .clk(clk), .rst_n(rst_n),
        .en(i_en), .auto_wr(i_auto), .prescale(i_prescale),
        .addr(i_addr), .ten_bit(i_ten),
        .cmd_valid(i_cmd_valid), .cmd_len(pwdata[7:0]), .cmd_read(pwdata[8]),
        .cmd_stop(pwdata[9]), .cmd_stop_only(pwdata[10]),
        .tx_flush(i_tx_flush), .rx_flush(i_rx_flush), .abort(i_abort),
        .tx_push(i_tx_push), .tx_data(i_tx_data),
        .rx_pop(i_rx_pop), .rx_data(i2c_rx_data),
        .tx_empty(i_tx_empty), .tx_full(i_tx_full),
        .rx_empty(i_rx_empty), .rx_full(i_rx_full),
        .tx_count(i_tx_count), .rx_count(i_rx_count),
        .busy(i_busy), .holding(i_holding), .bus_busy(i_bus_busy),
        .nack_flag(i_nack), .arb_flag(i_arb),
        .scl_state(i_scl), .sda_state(i_sda),
        .ev_done(i_ev_done), .ev_nack(i_ev_nack), .ev_arb_lost(i_ev_arb),
        .ev_tx_ovf(i_ev_tx_ovf),
        .scl_in(i2c_scl_in), .scl_oe(i2c_scl_oe),
        .sda_in(i2c_sda_in), .sda_oe(i2c_sda_oe)
    );

    assign i2c_status = {5'd0, i_sda, i_scl, i_bus_busy,
                         3'd0, i_rx_count, 3'd0, i_tx_count,
                         i_arb, i_nack, i_holding, i_busy,
                         i_rx_full, i_rx_empty, i_tx_full, i_tx_empty};

    // ------------------------------------------------------------------
    // interrupt controller
    // ------------------------------------------------------------------
    wire [15:0] events = {
        i_ev_tx_ovf,                 // 15
        s_ev_tx_ovf,                 // 14
        u_ev_tx_ovf,                 // 13
        ~i_rx_empty,                 // 12
        i_ev_arb,                    // 11
        i_ev_nack,                   // 10
        i_ev_done,                   // 9
        s_ev_rx_ovf,                 // 8
        ~s_rx_empty,                 // 7
        s_ev_done,                   // 6
        u_ev_rx_ovf,                 // 5
        u_ev_break,                  // 4
        u_ev_frame,                  // 3
        u_ev_par,                    // 2
        u_tx_empty & ~u_tx_busy,     // 1
        ~u_rx_empty                  // 0
    };

    ssc_irq #(.N(16)) u_irq (
        .clk(clk), .rst_n(rst_n),
        .events(events), .enable(int_enable),
        .clear_we(int_clear), .clear_mask(pwdata[15:0]),
        .status(int_status), .irq(irq)
    );
endmodule

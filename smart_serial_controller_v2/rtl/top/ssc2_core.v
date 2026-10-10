// =============================================================================
// ssc2_core.v  -  Smart Serial Controller v2: the whole controller (APB slave)
// -----------------------------------------------------------------------------
//                 control plane (APB, set-up only)
//     APB ------> v1 regs | xbar | NI cfg | hub | DMA | serial engine | misc
//
//                 data plane: 2 x 3 network-on-chip
//        (0,0) node 0 SERIAL ENGINE --- (1,0) node 1 HUB --- (2,0) node 2 DMA ---> AXI4 (HP0)
//              |                          |                    |
//        (0,1) node 3 UART NI ------- (1,1) node 4 SPI NI - (2,1) node 5 I2C NI
//              |                          |                    |
//           ssc_uart                   ssc_spi              ssc_i2c     (v1 engines)
//              \______________ pin crossbar (xbar) ___________/  + SM0, SM1, GPIO
//                                  |
//                     6 ports x 4 pins: JA JB JC JD OLED LED
//
// This file only wires the blocks together and holds the small "core"
// registers (NI configuration, misc, second interrupt controller).
//
// Register map (12-bit byte address, see docs/REGISTER_MAP.md):
//   0x000-0x3FF  v1 registers (ssc_regbank, ID = 0x53534302)
//   0x400-0x4FF  pin crossbar          0x500-0x5FF  NI configuration (here)
//   0x600-0x6FF  sensor hub            0x700-0x7FF  DMA writer
//   0x800-0x9FF  serial engine         0xA00-0xAFF  misc + INT2 (here)
//
// Sharing an engine between the CPU and its network interface (NI):
//   * the CPU's FIFO strobes and the NI's strobes are OR'ed, the data muxed;
//     the NIs never touch the engine while stall (= psel) is 1, so the two
//     can never collide;
//   * while the SPI or I2C NI runs a transaction ("active") it overrides the
//     engine settings it needs (8-bit, manual CS, master / 7-bit address).
//
// Pin roles on a crossbar port (pin 0..3 = Pmod pin 1..4):
//   UART : CTS# in | TXD out | RXD in  | RTS# out
//   SPI  : CS0#    | MOSI    | MISO    | SCK        (slave mode: directions flip)
//   I2C  : -       | -       | SCL od  | SDA od
//   SM0/SM1: the state machine's own pins 0..3
// =============================================================================
`timescale 1ns / 1ps
module ssc2_core #(
    parameter CLK_HZ    = 100_000_000,
    parameter US_PER_MS = 1000          // simulations may shorten a millisecond
) (
    input  wire        clk,
    input  wire        rst_n,
    // APB slave
    input  wire [11:0] paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,
    output wire        pslverr,
    output reg         irq,
    // AXI4 write master (DMA writer -> DDR through S_AXI_HP0)
    output wire [31:0] m_axi_awaddr,
    output wire [7:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output wire [3:0]  m_axi_awcache,
    output wire [2:0]  m_axi_awprot,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [31:0] m_axi_wdata,
    output wire [3:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    // crossbar pads: port p, pin i at bit 4p+i (0 JA, 1 JB, 2 JC, 3 JD, 4 OLED, 5 LED)
    output wire [23:0] pad_out,
    output wire [23:0] pad_oe,
    input  wire [23:0] pad_in,
    // fixed pins
    output wire [2:0]  spi_cs_hi_n,     // SPI chip selects CS1#..CS3# (CS1# on JB7)
    output wire [3:0]  mon,             // probe copies for JD7..10: TXD, RXD, SCL, SDA
    input  wire [7:0]  board_sw,
    input  wire [4:0]  board_btn,
    output wire [1:0]  oled_pwr,        // [0] VDD pin, [1] VBAT pin (1 = off)
    output wire [3:0]  status_led       // heartbeat, DMA batch toggle, hub error, irq
);
    // ------------------------------------------------------------------
    // node ids (docs/SPEC.md section 1)
    // ------------------------------------------------------------------
    localparam [2:0] N_SE = 3'd0, N_HUB = 3'd1, N_DMA = 3'd2,
                     N_UART = 3'd3, N_SPI = 3'd4, N_I2C = 3'd5;

    // ------------------------------------------------------------------
    // time base
    // ------------------------------------------------------------------
    wire [31:0] time_us;
    wire        us_tick, ms_tick;

    ssc2_timebase #(.CLK_HZ(CLK_HZ), .US_PER_MS(US_PER_MS)) u_time (
        .clk(clk), .rst_n(rst_n),
        .time_us(time_us), .us_tick(us_tick), .ms_tick(ms_tick));

    // ------------------------------------------------------------------
    // APB address decode: one strobe pair per region
    // ------------------------------------------------------------------
    wire        wr    = psel & penable &  pwrite;
    wire        rd    = psel & penable & ~pwrite;
    wire [3:0]  page  = paddr[11:8];
    wire [11:0] a     = {paddr[11:2], 2'b00};
    wire        stall = psel;                       // the NIs keep off the engines

    wire sel_v1   = (page <= 4'h3);
    wire sel_xbar = (page == 4'h4);
    wire sel_ni   = (page == 4'h5);
    wire sel_hub  = (page == 4'h6);
    wire sel_dma  = (page == 4'h7);
    wire sel_se   = (page == 4'h8) || (page == 4'h9);
    wire sel_misc = (page == 4'hA);

    assign pready  = 1'b1;
    assign pslverr = 1'b0;

    // ------------------------------------------------------------------
    // v1 register bank (UART / SPI / I2C settings and FIFOs, v1 interrupts)
    // ------------------------------------------------------------------
    wire [31:0] rb_rdata;
    wire        rb_cpu_access;
    wire [15:0] int_status, int_enable;
    wire        int_clear;
    wire [7:0]  bridge_ctrl;                       // kept for v1 software, no effect
    wire        bridge_cnt_clear;

    wire        u_tx_en, u_rx_en, u_par_en, u_par_odd, u_stop2, u_flow, u_loop;
    wire [1:0]  u_bits;
    wire [15:0] u_baud_int;
    wire [3:0]  u_baud_frac;
    wire        u_tx_flush, u_rx_flush, rb_u_tx_push, rb_u_rx_pop;

    wire        s_en_r, s_cpol, s_cpha, s_lsb, s_cs_manual_r, s_cs_level_r, s_slave_r, s_loop;
    wire [4:0]  s_len_m1_r;
    wire [1:0]  s_cs_sel_r;
    wire [15:0] s_clk_div;
    wire        s_tx_flush, s_rx_flush, rb_s_abort, rb_s_tx_push, rb_s_rx_pop;

    wire        i_en_r, i_auto_r, i_ten_r, rb_i_cmd_valid;
    wire [15:0] i_prescale;
    wire [9:0]  i_addr_r;
    wire        i_tx_flush, i_rx_flush, rb_i_abort, rb_i_tx_push, rb_i_rx_pop;

    wire [31:0] uart_status, spi_status, i2c_status;
    wire [7:0]  uart_rx_data, i2c_rx_data;
    wire [31:0] spi_rx_data;

    ssc_regbank #(.ID_VALUE(32'h5353_4302)) u_regs (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr), .psel(psel & sel_v1), .penable(penable), .pwrite(pwrite),
        .pwdata(pwdata), .prdata(rb_rdata), .pready(), .pslverr(),
        .cpu_access(rb_cpu_access),
        .int_status(int_status), .int_enable(int_enable), .int_clear(int_clear),
        .bridge_ctrl(bridge_ctrl), .bridge_cnt_clear(bridge_cnt_clear),
        .bridge_cnt0(16'd0), .bridge_cnt1(16'd0),
        .uart_tx_en(u_tx_en), .uart_rx_en(u_rx_en), .uart_bits(u_bits),
        .uart_par_en(u_par_en), .uart_par_odd(u_par_odd), .uart_stop2(u_stop2),
        .uart_flow(u_flow), .uart_loop(u_loop),
        .uart_baud_int(u_baud_int), .uart_baud_frac(u_baud_frac),
        .uart_tx_flush(u_tx_flush), .uart_rx_flush(u_rx_flush),
        .uart_tx_push(rb_u_tx_push), .uart_rx_pop(rb_u_rx_pop),
        .uart_status(uart_status), .uart_rx_data(uart_rx_data),
        .spi_en(s_en_r), .spi_cpol(s_cpol), .spi_cpha(s_cpha), .spi_lsb(s_lsb),
        .spi_len_m1(s_len_m1_r), .spi_cs_sel(s_cs_sel_r), .spi_cs_manual(s_cs_manual_r),
        .spi_cs_level(s_cs_level_r), .spi_slave(s_slave_r), .spi_loop(s_loop),
        .spi_clk_div(s_clk_div),
        .spi_tx_flush(s_tx_flush), .spi_rx_flush(s_rx_flush), .spi_abort(rb_s_abort),
        .spi_tx_push(rb_s_tx_push), .spi_rx_pop(rb_s_rx_pop),
        .spi_status(spi_status), .spi_rx_data(spi_rx_data),
        .i2c_en(i_en_r), .i2c_auto(i_auto_r), .i2c_prescale(i_prescale),
        .i2c_addr(i_addr_r), .i2c_ten(i_ten_r), .i2c_cmd_valid(rb_i_cmd_valid),
        .i2c_tx_flush(i_tx_flush), .i2c_rx_flush(i_rx_flush), .i2c_abort(rb_i_abort),
        .i2c_tx_push(rb_i_tx_push), .i2c_rx_pop(rb_i_rx_pop),
        .i2c_status(i2c_status), .i2c_rx_data(i2c_rx_data)
    );

    // ------------------------------------------------------------------
    // core registers: NI configuration (0x500) and misc (0xA00)
    // ------------------------------------------------------------------
    reg  [23:0] ni_cfg0, ni_cfg3, ni_cfg4, ni_cfg5;  // nodes 1 and 2 have their own CTRL
    reg  [15:0] uart_tmo_us;
    reg  [15:0] i2c_tmo_us;
    reg  [7:0]  int2_enable;
    reg  [31:0] apb_count;
    reg  [31:0] irq_count;
    reg  [1:0]  oled_pwr_r;
    reg         se_loop;
    wire [7:0]  int2_status;
    wire [15:0] pk_in [0:5];
    wire [15:0] pk_out [0:5];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ni_cfg0     <= 24'd0;
            ni_cfg3     <= 24'd0;
            ni_cfg4     <= 24'd0;
            ni_cfg5     <= 24'd0;
            uart_tmo_us <= 16'd2000;
            i2c_tmo_us  <= 16'd50000;
            int2_enable <= 8'd0;
            oled_pwr_r  <= 2'b11;                  // OLED supplies off
            se_loop     <= 1'b0;
        end else if (wr) begin
            case (a)
                12'h500: ni_cfg0     <= pwdata[23:0];
                12'h50C: ni_cfg3     <= pwdata[23:0];
                12'h510: ni_cfg4     <= pwdata[23:0];
                12'h514: ni_cfg5     <= pwdata[23:0];
                12'h540: uart_tmo_us <= pwdata[15:0];
                12'h544: i2c_tmo_us  <= pwdata[15:0];
                12'hA0C: int2_enable <= pwdata[7:0];
                12'hA18: oled_pwr_r  <= pwdata[1:0];
                12'hA20: se_loop     <= pwdata[0];
                default: ;
            endcase
        end
    end

    // APB_COUNT counts bus transfers, IRQ_COUNT counts rising edges of irq
    // (slide 24, measurement 4: CPU load).  Writing either register clears it.
    reg irq_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apb_count <= 32'd0;
            irq_count <= 32'd0;
            irq_q     <= 1'b0;
        end else begin
            irq_q <= irq;
            if (wr && a == 12'hA10)        apb_count <= 32'd0;
            else if (psel && penable)      apb_count <= apb_count + 32'd1;
            if (wr && a == 12'hA14)        irq_count <= 32'd0;
            else if (irq && !irq_q)        irq_count <= irq_count + 32'd1;
        end
    end

    assign oled_pwr = oled_pwr_r;

    // ------------------------------------------------------------------
    // engine sharing: CPU strobes OR NI strobes (never in the same clock)
    // ------------------------------------------------------------------
    wire       niu_tx_push, niu_rx_pop;
    wire [7:0] niu_tx_data;
    wire       nis_tx_push, nis_rx_pop, nis_active, nis_cs_level;
    wire [7:0] nis_tx_data;
    wire [1:0] nis_cs_sel;
    wire       nii_tx_push, nii_rx_pop, nii_cmd_valid, nii_cmd_read, nii_cmd_stop;
    wire       nii_abort, nii_active;
    wire [7:0] nii_tx_data, nii_cmd_len;
    wire [6:0] nii_addr7;

    wire        u_tx_push = rb_u_tx_push | niu_tx_push;
    wire [7:0]  u_tx_data = rb_u_tx_push ? pwdata[7:0] : niu_tx_data;
    wire        u_rx_pop  = rb_u_rx_pop  | niu_rx_pop;

    wire        s_tx_push = rb_s_tx_push | nis_tx_push;
    wire [31:0] s_tx_data = rb_s_tx_push ? pwdata : {24'd0, nis_tx_data};
    wire        s_rx_pop  = rb_s_rx_pop  | nis_rx_pop;
    // while the SPI NI is active it needs: enabled master, 8-bit words, manual CS
    wire        s_en        = nis_active | s_en_r;
    wire        s_slave     = nis_active ? 1'b0    : s_slave_r;
    wire [4:0]  s_len_m1    = nis_active ? 5'd7    : s_len_m1_r;
    wire        s_cs_manual = nis_active | s_cs_manual_r;
    wire        s_cs_level  = nis_active ? nis_cs_level : s_cs_level_r;
    wire [1:0]  s_cs_sel    = nis_active ? nis_cs_sel   : s_cs_sel_r;

    wire        i_tx_push   = rb_i_tx_push | nii_tx_push;
    wire [7:0]  i_tx_data   = rb_i_tx_push ? pwdata[7:0] : nii_tx_data;
    wire        i_rx_pop    = rb_i_rx_pop  | nii_rx_pop;
    wire        i_cmd_valid = rb_i_cmd_valid | nii_cmd_valid;
    wire [7:0]  i_cmd_len   = rb_i_cmd_valid ? pwdata[7:0] : nii_cmd_len;
    wire        i_cmd_read  = rb_i_cmd_valid ? pwdata[8]   : nii_cmd_read;
    wire        i_cmd_stop  = rb_i_cmd_valid ? pwdata[9]   : nii_cmd_stop;
    wire        i_cmd_sonly = rb_i_cmd_valid & pwdata[10];
    wire        i_abort     = rb_i_abort | nii_abort;
    // while the I2C NI is active it needs: enabled, no auto-write, 7-bit address
    wire        i_en        = nii_active | i_en_r;
    wire        i_auto      = nii_active ? 1'b0 : i_auto_r;
    wire [9:0]  i_addr      = nii_active ? {3'd0, nii_addr7} : i_addr_r;
    wire        i_ten       = nii_active ? 1'b0 : i_ten_r;

    // ------------------------------------------------------------------
    // crossbar source signals (source s at [4*(s-1) +: 4])
    // ------------------------------------------------------------------
    wire [19:0] src_out, src_oe, src_in, src_in_idle;
    wire [4:0]  src_idle;

    // ------------------------------------------------------------------
    // UART engine (v1)
    // ------------------------------------------------------------------
    wire       u_tx_empty, u_tx_full, u_rx_empty, u_rx_full, u_tx_busy, u_rx_busy, u_cts_ok;
    wire [4:0] u_tx_count, u_rx_count;
    wire       u_ev_par, u_ev_frame, u_ev_break, u_ev_rx_ovf, u_ev_tx_ovf;
    wire       u_txd, u_rts_n;
    wire       u_rxd   = src_in[2];
    wire       u_cts_n = src_in[0];

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
        .rxd(u_rxd), .txd(u_txd), .cts_n(u_cts_n), .rts_n(u_rts_n)
    );

    assign uart_status = {11'd0, u_rx_count, 3'd0, u_tx_count,
                          1'b0, u_cts_ok, u_rx_busy, u_tx_busy,
                          u_rx_full, u_rx_empty, u_tx_full, u_tx_empty};

    //                     pin3     pin2  pin1   pin0
    assign src_out[3:0]  = {u_rts_n, 1'b0, u_txd, 1'b0};
    assign src_oe[3:0]   = {1'b1,    1'b0, 1'b1,  1'b0};
    assign src_in_idle[3:0] = 4'b0100;              // RXD idles high, CTS# = 0 (clear)
    assign src_idle[0]   = u_tx_empty & ~u_tx_busy & ~u_rx_busy;

    // ------------------------------------------------------------------
    // SPI engine (v1)
    // ------------------------------------------------------------------
    wire       s_tx_empty, s_tx_full, s_rx_empty, s_rx_full, s_busy;
    wire [4:0] s_tx_count, s_rx_count;
    wire       s_ev_done, s_ev_rx_ovf, s_ev_tx_ovf;
    wire       s_sclk, s_mosi, s_miso_out, s_miso_oe;
    wire [3:0] s_cs_n;
    wire       spi_slave_mode = s_en & s_slave;

    ssc_spi u_spi (
        .clk(clk), .rst_n(rst_n),
        .en(s_en), .cpol(s_cpol), .cpha(s_cpha), .lsb_first(s_lsb),
        .len_m1(s_len_m1), .cs_sel(s_cs_sel), .cs_manual(s_cs_manual),
        .cs_level(s_cs_level), .slave_mode(s_slave), .loopback(s_loop),
        .clk_div(s_clk_div),
        .tx_flush(s_tx_flush), .rx_flush(s_rx_flush), .abort(rb_s_abort),
        .tx_push(s_tx_push), .tx_data(s_tx_data),
        .rx_pop(s_rx_pop), .rx_data(spi_rx_data),
        .tx_empty(s_tx_empty), .tx_full(s_tx_full),
        .rx_empty(s_rx_empty), .rx_full(s_rx_full),
        .tx_count(s_tx_count), .rx_count(s_rx_count), .busy(s_busy),
        .ev_done(s_ev_done), .ev_rx_ovf(s_ev_rx_ovf), .ev_tx_ovf(s_ev_tx_ovf),
        .sclk(s_sclk), .mosi(s_mosi), .miso(src_in[6]), .cs_n(s_cs_n),
        .s_sclk(src_in[7]), .s_mosi(src_in[5]), .s_cs_n(src_in[4]),
        .s_miso(s_miso_out), .s_miso_oe(s_miso_oe)
    );

    assign spi_status = {11'd0, s_rx_count, 3'd0, s_tx_count,
                         3'd0, s_busy, s_rx_full, s_rx_empty, s_tx_full, s_tx_empty};

    //                                       pin3    pin2        pin1    pin0
    assign src_out[7:4]  = spi_slave_mode ? {1'b0,   s_miso_out, 1'b0,   1'b0}
                                          : {s_sclk, 1'b0,       s_mosi, s_cs_n[0]};
    assign src_oe[7:4]   = spi_slave_mode ? {1'b0,   s_miso_oe,  1'b0,   1'b0}
                                          : {1'b1,   1'b0,       1'b1,   1'b1};
    assign src_in_idle[7:4] = {s_cpol, 1'b1, 1'b0, 1'b1};   // SCK, MISO, MOSI, CS#
    assign src_idle[1]   = ~s_busy & s_tx_empty & (spi_slave_mode ? src_in[4] : s_cs_n[0]);
    assign spi_cs_hi_n   = s_cs_n[3:1];

    // ------------------------------------------------------------------
    // I2C engine (v1)
    // ------------------------------------------------------------------
    wire       i_tx_empty, i_tx_full, i_rx_empty, i_rx_full, i_busy;
    wire       i_holding, i_bus_busy, i_nack, i_arb, i_scl, i_sda;
    wire [4:0] i_tx_count, i_rx_count;
    wire       i_ev_done, i_ev_nack, i_ev_arb, i_ev_tx_ovf;
    wire       i_scl_oe, i_sda_oe;

    ssc_i2c u_i2c (
        .clk(clk), .rst_n(rst_n),
        .en(i_en), .auto_wr(i_auto), .prescale(i_prescale),
        .addr(i_addr), .ten_bit(i_ten),
        .cmd_valid(i_cmd_valid), .cmd_len(i_cmd_len), .cmd_read(i_cmd_read),
        .cmd_stop(i_cmd_stop), .cmd_stop_only(i_cmd_sonly),
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
        .scl_in(src_in[10]), .scl_oe(i_scl_oe),
        .sda_in(src_in[11]), .sda_oe(i_sda_oe)
    );

    assign i2c_status = {5'd0, i_sda, i_scl, i_bus_busy,
                         3'd0, i_rx_count, 3'd0, i_tx_count,
                         i_arb, i_nack, i_holding, i_busy,
                         i_rx_full, i_rx_empty, i_tx_full, i_tx_empty};

    // open drain: the pin is driven low (oe = 1, out = 0) or released
    assign src_out[11:8] = 4'b0000;
    assign src_oe[11:8]  = {i_sda_oe, i_scl_oe, 2'b00};
    assign src_in_idle[11:8] = 4'b1100;              // SCL and SDA idle high
    assign src_idle[2]   = ~i_bus_busy & ~i_busy;

    // ------------------------------------------------------------------
    // v1 interrupt controller (same 16 bits as v1)
    // ------------------------------------------------------------------
    wire v1_irq;
    wire [15:0] events = {
        i_ev_tx_ovf, s_ev_tx_ovf, u_ev_tx_ovf, ~i_rx_empty,
        i_ev_arb, i_ev_nack, i_ev_done, s_ev_rx_ovf,
        ~s_rx_empty, s_ev_done, u_ev_rx_ovf, u_ev_break,
        u_ev_frame, u_ev_par, u_tx_empty & ~u_tx_busy, ~u_rx_empty
    };

    ssc_irq #(.N(16)) u_irq1 (
        .clk(clk), .rst_n(rst_n),
        .events(events), .enable(int_enable),
        .clear_we(int_clear), .clear_mask(pwdata[15:0]),
        .status(int_status), .irq(v1_irq)
    );

    // ------------------------------------------------------------------
    // serial engine (node 0 is its NI)
    // ------------------------------------------------------------------
    wire [31:0] se_rdata;
    wire [1:0]  se_tx_push, se_tx_full, se_rx_pop, se_rx_empty;
    wire [31:0] se_tx_data0, se_tx_data1, se_rx_data0, se_rx_data1;
    wire [1:0]  se_out_left, se_in_left, sm_idle, se_rx_not_empty;
    wire [3:0]  sm0_out, sm0_oe, sm1_out, sm1_oe;
    // SE_LOOP (0xA20 bit 0): SM1 pin 2 listens to SM0 pin 1 (Manchester demo, no wire)
    wire [3:0]  sm1_in = se_loop ? {src_in[19], sm0_out[1], src_in[17:16]} : src_in[19:16];

    se_engine u_se (
        .clk(clk), .rst_n(rst_n), .time_us(time_us),
        .reg_we(wr & sel_se), .reg_re(rd & sel_se), .reg_addr(paddr[8:0]),
        .reg_wdata(pwdata), .reg_rdata(se_rdata),
        .ni_tx_push(se_tx_push), .ni_tx_data0(se_tx_data0), .ni_tx_data1(se_tx_data1),
        .ni_tx_full(se_tx_full),
        .ni_rx_pop(se_rx_pop), .ni_rx_data0(se_rx_data0), .ni_rx_data1(se_rx_data1),
        .ni_rx_empty(se_rx_empty),
        .out_shift_left(se_out_left), .in_shift_left(se_in_left),
        .sm0_out(sm0_out), .sm0_oe(sm0_oe), .sm0_in(src_in[15:12]),
        .sm1_out(sm1_out), .sm1_oe(sm1_oe), .sm1_in(sm1_in),
        .sm_idle(sm_idle), .rx_not_empty(se_rx_not_empty)
    );

    assign src_out[15:12] = sm0_out;
    assign src_oe[15:12]  = sm0_oe;
    assign src_out[19:16] = sm1_out;
    assign src_oe[19:16]  = sm1_oe;
    assign src_in_idle[19:12] = 8'hFF;              // released lines read high
    assign src_idle[4:3]  = sm_idle;

    // ------------------------------------------------------------------
    // pin crossbar
    // ------------------------------------------------------------------
    wire [31:0] xbar_rdata;
    wire        xbar_switch;

    xbar #(.NP(6)) u_xbar (
        .clk(clk), .rst_n(rst_n),
        .reg_we(wr & sel_xbar), .reg_re(rd & sel_xbar), .reg_addr(paddr[7:0]),
        .reg_wdata(pwdata), .reg_rdata(xbar_rdata),
        .src_out(src_out), .src_oe(src_oe), .src_in(src_in),
        .src_idle(src_idle), .src_in_idle(src_in_idle),
        .pad_out(pad_out), .pad_oe(pad_oe), .pad_in(pad_in),
        .ev_switch(xbar_switch)
    );

    // probe copies for JD7..10: what the UART and I2C engines send and see
    assign mon = {src_in[11], src_in[10], u_rxd, u_txd};

    // ------------------------------------------------------------------
    // network-on-chip: 2 x 3 mesh, node n at [34*n +: 34]
    // ------------------------------------------------------------------
    wire [5:0]    tx_valid, tx_credit, rx_valid, rx_credit;
    wire [6*34-1:0] tx_flit, rx_flit;

    noc_mesh #(.W(3), .H(2), .N(6), .IDW(3), .DEST_LSB(26), .PRIO_BIT(22), .DEPTH(4)) u_noc (
        .clk(clk), .rst_n(rst_n),
        .tx_valid(tx_valid), .tx_flit(tx_flit), .tx_credit(tx_credit),
        .rx_valid(rx_valid), .rx_flit(rx_flit), .rx_credit(rx_credit)
    );

    // node 0: serial engine NI
    ni_se u_ni_se (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(ni_cfg0[0]), .my_id(N_SE),
        .dest0(ni_cfg0[6:4]), .dest1(ni_cfg0[22:20]),
        .prio0(ni_cfg0[7]), .prio1(ni_cfg0[23]),
        .word4_0(ni_cfg0[17]), .word4_1(ni_cfg0[18]),
        .out_shift_left(se_out_left), .in_shift_left(se_in_left),
        .tx_push(se_tx_push), .tx_data0(se_tx_data0), .tx_data1(se_tx_data1),
        .tx_full(se_tx_full),
        .rx_pop(se_rx_pop), .rx_data0(se_rx_data0), .rx_data1(se_rx_data1),
        .rx_empty(se_rx_empty),
        .in_valid(rx_valid[0]), .in_flit(rx_flit[0*34 +: 34]), .in_credit(rx_credit[0]),
        .out_valid(tx_valid[0]), .out_flit(tx_flit[0*34 +: 34]), .out_credit(tx_credit[0]),
        .pkts_in(), .pkts_out()
    );

    // node 1: sensor hub
    wire [31:0] hub_rdata;
    wire        hub_error, hub_record;

    hub u_hub (
        .clk(clk), .rst_n(rst_n), .time_us(time_us), .ms_tick(ms_tick), .my_id(N_HUB),
        .reg_we(wr & sel_hub), .reg_re(rd & sel_hub), .reg_addr(paddr[7:0]),
        .reg_wdata(pwdata), .reg_rdata(hub_rdata),
        .in_valid(rx_valid[1]), .in_flit(rx_flit[1*34 +: 34]), .in_credit(rx_credit[1]),
        .out_valid(tx_valid[1]), .out_flit(tx_flit[1*34 +: 34]), .out_credit(tx_credit[1]),
        .ev_error(hub_error), .ev_record(hub_record)
    );

    // node 2: DMA writer
    wire [31:0] dma_rdata;
    wire        dma_batch, dma_overflow, dma_axi_err;

    dma_writer u_dma (
        .clk(clk), .rst_n(rst_n), .ms_tick(ms_tick), .my_id(N_DMA),
        .reg_we(wr & sel_dma), .reg_re(rd & sel_dma), .reg_addr(paddr[7:0]),
        .reg_wdata(pwdata), .reg_rdata(dma_rdata),
        .in_valid(rx_valid[2]), .in_flit(rx_flit[2*34 +: 34]), .in_credit(rx_credit[2]),
        .out_valid(tx_valid[2]), .out_flit(tx_flit[2*34 +: 34]), .out_credit(tx_credit[2]),
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awcache(m_axi_awcache), .m_axi_awprot(m_axi_awprot),
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .ev_batch(dma_batch), .ev_overflow(dma_overflow), .ev_axi_err(dma_axi_err)
    );

    // node 3: UART NI
    ni_uart u_ni_uart (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(ni_cfg3[0]), .my_id(N_UART),
        .in_valid(rx_valid[3]), .in_flit(rx_flit[3*34 +: 34]), .in_credit(rx_credit[3]),
        .out_valid(tx_valid[3]), .out_flit(tx_flit[3*34 +: 34]), .out_credit(tx_credit[3]),
        .pkts_in(), .pkts_out(),
        .mode(ni_cfg3[2:1]), .dest(ni_cfg3[6:4]), .prio(ni_cfg3[7]), .arg(ni_cfg3[15:8]),
        .resp_status(ni_cfg3[16]),
        .timeout_us(uart_tmo_us), .us_tick(us_tick),
        .tx_push(niu_tx_push), .tx_data(niu_tx_data), .tx_full(u_tx_full),
        .rx_pop(niu_rx_pop), .rx_data(uart_rx_data), .rx_empty(u_rx_empty)
    );

    // node 4: SPI NI
    ni_spi u_ni_spi (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(ni_cfg4[0]), .my_id(N_SPI),
        .in_valid(rx_valid[4]), .in_flit(rx_flit[4*34 +: 34]), .in_credit(rx_credit[4]),
        .out_valid(tx_valid[4]), .out_flit(tx_flit[4*34 +: 34]), .out_credit(tx_credit[4]),
        .pkts_in(), .pkts_out(),
        .tx_push(nis_tx_push), .tx_data(nis_tx_data), .tx_full(s_tx_full),
        .rx_pop(nis_rx_pop), .rx_data(spi_rx_data[7:0]), .rx_empty(s_rx_empty),
        .busy(s_busy), .active(nis_active), .cs_level(nis_cs_level), .cs_sel(nis_cs_sel)
    );

    // node 5: I2C NI
    ni_i2c u_ni_i2c (
        .clk(clk), .rst_n(rst_n), .stall(stall), .en(ni_cfg5[0]), .my_id(N_I2C),
        .in_valid(rx_valid[5]), .in_flit(rx_flit[5*34 +: 34]), .in_credit(rx_credit[5]),
        .out_valid(tx_valid[5]), .out_flit(tx_flit[5*34 +: 34]), .out_credit(tx_credit[5]),
        .pkts_in(), .pkts_out(),
        .tx_push(nii_tx_push), .tx_data(nii_tx_data), .tx_full(i_tx_full),
        .rx_pop(nii_rx_pop), .rx_data(i2c_rx_data), .rx_empty(i_rx_empty),
        .cmd_valid(nii_cmd_valid), .cmd_len(nii_cmd_len), .cmd_read(nii_cmd_read),
        .cmd_stop(nii_cmd_stop), .abort(nii_abort),
        .busy(i_busy), .holding(i_holding), .nack_flag(i_nack), .arb_flag(i_arb),
        .ev_done(i_ev_done),
        .timeout_us(i2c_tmo_us), .us_tick(us_tick),
        .active(nii_active), .addr7(nii_addr7)
    );

    // packet counters for NI_STAT: count head flits entering / leaving each node
    genvar n;
    generate
        for (n = 0; n < 6; n = n + 1) begin : g_cnt
            reg [15:0] cin, cout;
            wire head_in  = rx_valid[n] & (rx_flit[34*n+32 +: 2] != 2'b01) &
                                          (rx_flit[34*n+32 +: 2] != 2'b10);
            wire head_out = tx_valid[n] & (tx_flit[34*n+32 +: 2] != 2'b01) &
                                          (tx_flit[34*n+32 +: 2] != 2'b10);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cin  <= 16'd0;
                    cout <= 16'd0;
                end else begin
                    if (head_in)  cin  <= cin + 16'd1;
                    if (head_out) cout <= cout + 16'd1;
                end
            end
            assign pk_in[n]  = cin;
            assign pk_out[n] = cout;
        end
    endgenerate

    // ------------------------------------------------------------------
    // second interrupt controller (v2 events)
    // ------------------------------------------------------------------
    wire int2_irq;
    wire [7:0] events2 = {se_rx_not_empty[1], se_rx_not_empty[0], xbar_switch,
                          hub_record, hub_error, dma_axi_err, dma_overflow, dma_batch};

    ssc_irq #(.N(8)) u_irq2 (
        .clk(clk), .rst_n(rst_n),
        .events(events2), .enable(int2_enable),
        .clear_we(wr && a == 12'hA08), .clear_mask(pwdata[7:0]),
        .status(int2_status), .irq(int2_irq)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) irq <= 1'b0;
        else        irq <= v1_irq | int2_irq;
    end

    // ------------------------------------------------------------------
    // status LEDs (LD4..7 on the board)
    // ------------------------------------------------------------------
    reg [8:0] hb_ms;
    reg       hb, batch_t, hub_err_seen;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_ms        <= 9'd0;
            hb           <= 1'b0;
            batch_t      <= 1'b0;
            hub_err_seen <= 1'b0;
        end else begin
            if (ms_tick) begin
                if (hb_ms == 9'd499) begin hb_ms <= 9'd0; hb <= ~hb; end
                else                 hb_ms <= hb_ms + 9'd1;
            end
            if (dma_batch) batch_t      <= ~batch_t;
            if (hub_error) hub_err_seen <= 1'b1;
        end
    end
    assign status_led = {irq, hub_err_seen, batch_t, hb};

    // ------------------------------------------------------------------
    // read multiplexer
    // ------------------------------------------------------------------
    reg [31:0] core_rdata;
    always @* begin
        case (a)
            12'h500: core_rdata = {8'd0, ni_cfg0};
            12'h50C: core_rdata = {8'd0, ni_cfg3};
            12'h510: core_rdata = {8'd0, ni_cfg4};
            12'h514: core_rdata = {8'd0, ni_cfg5};
            12'h520: core_rdata = {pk_out[0], pk_in[0]};
            12'h524: core_rdata = {pk_out[1], pk_in[1]};
            12'h528: core_rdata = {pk_out[2], pk_in[2]};
            12'h52C: core_rdata = {pk_out[3], pk_in[3]};
            12'h530: core_rdata = {pk_out[4], pk_in[4]};
            12'h534: core_rdata = {pk_out[5], pk_in[5]};
            12'h540: core_rdata = {16'd0, uart_tmo_us};
            12'h544: core_rdata = {16'd0, i2c_tmo_us};
            12'hA00: core_rdata = 32'h0002_0000;            // VERSION 2.0
            12'hA04: core_rdata = time_us;
            12'hA08: core_rdata = {24'd0, int2_status};
            12'hA0C: core_rdata = {24'd0, int2_enable};
            12'hA10: core_rdata = apb_count;
            12'hA14: core_rdata = irq_count;
            12'hA18: core_rdata = {30'd0, oled_pwr_r};
            12'hA1C: core_rdata = {19'd0, board_btn, board_sw};
            12'hA20: core_rdata = {31'd0, se_loop};
            default: core_rdata = 32'd0;
        endcase
    end

    always @* begin
        if (sel_v1)        prdata = rb_rdata;
        else if (sel_xbar) prdata = xbar_rdata;
        else if (sel_hub)  prdata = hub_rdata;
        else if (sel_dma)  prdata = dma_rdata;
        else if (sel_se)   prdata = se_rdata;
        else               prdata = core_rdata;
    end
endmodule

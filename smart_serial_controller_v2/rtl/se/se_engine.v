// =============================================================================
// se_engine.v  -  programmable serial engine: 2 state machines + registers
// -----------------------------------------------------------------------------
//                       +---------------- se_engine ----------------+
//   CPU registers ----->| SE_CTRL  CRC_CFG  CLKDIV  PINCTRL  ...    |
//   (reg_* port)        |                                           |
//                       |   program memory 32 x 16 (PROG[0..31])   |
//                       |        |  (two read ports)  |             |
//   ni_tx_push/data --->| TX FIFO 0 -> se_sm SM0 -> RX FIFO 0 ----->| ni_rx_data0
//   (and SMs_TXF)       | TX FIFO 1 -> se_sm SM1 -> RX FIFO 1 ----->| ni_rx_data1
//                       |              |  ^                         |
//                       +--------------|--|-------------------------+
//                                 out/oe  in (2-flop synchroniser)
//
// Each SM has a 4-word TX FIFO (CPU or ni_se -> SM) and a 4-word RX FIFO
// (SM -> CPU or ni_se).  The FIFOs are v1 ssc_fifo instances (32 bit wide,
// first-word fall-through), so ni_rx_dataS always shows the oldest word.
//
// Register map (byte offset in the 0x800-0x9FF region, reg_addr[8:0]):
//   0x000 SE_CTRL      [0] SM0_EN [1] SM1_EN; write 1: [8] SM0_RESTART [9] SM1_RESTART
//   0x004 SE_FSTAT     FIFO levels / full / empty, stalled, RX overflow (RO,
//                      the overflow bits clear when this register is read)
//   0x008 SE_CRC_CFG   [15:0] POLY (0x1021), [31:16] INIT (0xFFFF)
//   0x00C SE_TIME      time_us (RO)
//   0x010 + 0x40*s     SMs_CLKDIV    [31:16] INT, [15:8] FRAC (reset 1.0)
//   0x014 + 0x40*s     SMs_PINCTRL   pin numbers, SIDE_EN, OD_MASK, INIT_OUT/OE
//   0x018 + 0x40*s     SMs_SHIFTCTRL [0] OUT left [1] IN left [12:8] START_PC
//   0x01C + 0x40*s     SMs_TXF  (W)  push a word, dropped if the FIFO is full
//   0x020 + 0x40*s     SMs_RXF  (R)  pop a word, reads 0 when empty
//   0x024 + 0x40*s     SMs_EXEC (W)  run this instruction at the next tick
//   0x028 + 0x40*s     SMs_STATE     [4:0] PC [8] stalled [9] enabled [10] exec pending
//   0x02C/0x030/0x034  SMs_X / SMs_Y / SMs_CRC (RO)
//   0x038 + 0x40*s     SMs_PINS      [3:0] in [7:4] out [11:8] oe (RO)
//   0x100 + 4*i        PROG[i]       [15:0] instruction i, read / write
// Every other address reads 0.  Write-only registers read 0.
//
// CPU and ni_se share the FIFOs: the push / pop strobes are OR'ed.  If both
// push in the same clock the CPU data is taken (the NI never does this,
// because it waits while stall = 1).
//
// The smX_in pins pass through a 2-flop synchroniser (reset value 1 = idle
// line), so an SM sees a pin change 2 clocks after it happens at the input.
//
// The program memory has no reset (so it can be built from LUT RAM).  Load
// it before enabling an SM.  RESTART does not flush the FIFOs.
// =============================================================================
`timescale 1ns / 1ps
module se_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] time_us,
    // register port
    input  wire        reg_we,
    input  wire        reg_re,
    input  wire [8:0]  reg_addr,
    input  wire [31:0] reg_wdata,
    output reg  [31:0] reg_rdata,
    // FIFO ports for ni_se (CPU register access has priority)
    input  wire [1:0]  ni_tx_push,
    input  wire [31:0] ni_tx_data0,
    input  wire [31:0] ni_tx_data1,
    output wire [1:0]  ni_tx_full,
    input  wire [1:0]  ni_rx_pop,
    output wire [31:0] ni_rx_data0,
    output wire [31:0] ni_rx_data1,
    output wire [1:0]  ni_rx_empty,
    output wire [1:0]  out_shift_left,
    output wire [1:0]  in_shift_left,
    // pins
    output wire [3:0]  sm0_out,
    output wire [3:0]  sm0_oe,
    input  wire [3:0]  sm0_in,
    output wire [3:0]  sm1_out,
    output wire [3:0]  sm1_oe,
    input  wire [3:0]  sm1_in,
    output wire [1:0]  sm_idle,        // !enabled, or stalled on PULL block with TX empty
    output wire [1:0]  rx_not_empty
);
    // ---------------- address decode ----------------
    wire [3:0] k       = reg_addr[5:2];             // word inside a 0x40 block
    wire       low     = (reg_addr[8:7] == 2'b00);  // 0x000 - 0x07F
    wire       glob    = low && !reg_addr[6] && (k < 4'd4);       // 0x000 - 0x00C
    wire       is_prog = (reg_addr[8:7] == 2'b10);  // 0x100 - 0x17C
    wire       smsel   = reg_addr[6];               // which SM block
    wire       sm_reg  = low && (k >= 4'd4);        // 0x010 - 0x03C (+0x40)

    // ---------------- global registers ----------------
    reg  [1:0]  sm_en;
    reg  [15:0] crc_poly;
    reg  [15:0] crc_init;
    reg  [1:0]  ovf;                                // sticky RX overflow flags

    wire        we_ctrl   = reg_we && glob && (k == 4'd0);
    wire        re_fstat  = reg_re && glob && (k == 4'd1);
    wire        we_crccfg = reg_we && glob && (k == 4'd2);
    wire [1:0]  restart   = we_ctrl ? reg_wdata[9:8] : 2'b00;

    wire [1:0]  sm_ovf;                             // pulses from the SMs

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sm_en    <= 2'b00;
            crc_poly <= 16'h1021;
            crc_init <= 16'hFFFF;
            ovf      <= 2'b00;
        end else begin
            if (we_ctrl)
                sm_en <= reg_wdata[1:0];
            if (we_crccfg) begin
                crc_poly <= reg_wdata[15:0];
                crc_init <= reg_wdata[31:16];
            end
            // clear on read, but a new overflow in the same clock wins
            ovf <= (re_fstat ? 2'b00 : ovf) | sm_ovf;
        end
    end

    // ---------------- program memory ----------------
    reg [15:0] prog [0:31];

    always @(posedge clk) begin
        if (reg_we && is_prog)
            prog[reg_addr[6:2]] <= reg_wdata[15:0];
    end

    wire [15:0] prog_rd = prog[reg_addr[6:2]];      // CPU read port

    // ---------------- per-SM blocks ----------------
    // Flat vectors, SM s at [W*s +: W], so the read mux can index them.
    wire [63:0] rd_clkdiv, rd_pinctrl, rd_shift, rd_rxf, rd_state;
    wire [63:0] rd_x, rd_y, rd_crc, rd_pins;
    wire [5:0]  tx_level, rx_level;
    wire [7:0]  pout_w, poe_w;
    wire [1:0]  tx_full_w, rx_empty_w, stalled_w, outl_w, inl_w;

    genvar s;
    generate
        for (s = 0; s < 2; s = s + 1) begin : g_sm
            // this SM's register block is selected
            wire mine = sm_reg && (smsel == (s == 1));

            reg  [15:0] clk_int;
            reg  [7:0]  clk_frac;
            reg  [27:0] pinctrl;
            reg         out_left, in_left;
            reg  [4:0]  start_pc;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    clk_int  <= 16'd1;
                    clk_frac <= 8'd0;
                    pinctrl  <= 28'd0;
                    out_left <= 1'b0;
                    in_left  <= 1'b0;
                    start_pc <= 5'd0;
                end else if (reg_we && mine) begin
                    case (k)
                        4'd4: begin
                            clk_int  <= reg_wdata[31:16];
                            clk_frac <= reg_wdata[15:8];
                        end
                        4'd5: pinctrl <= reg_wdata[27:0] & 28'hFFF7F7F;   // bits 7, 15 unused
                        4'd6: begin
                            out_left <= reg_wdata[0];
                            in_left  <= reg_wdata[1];
                            start_pc <= reg_wdata[12:8];
                        end
                        default: ;
                    endcase
                end
            end

            // ---- input synchroniser: 2 flip-flops per pin ----
            (* ASYNC_REG = "TRUE" *) reg [3:0] sync0, sync1;
            wire [3:0] pins_raw = (s == 0) ? sm0_in : sm1_in;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    sync0 <= 4'hF;
                    sync1 <= 4'hF;
                end else begin
                    sync0 <= pins_raw;
                    sync1 <= sync0;
                end
            end

            // ---- FIFOs ----
            wire        cpu_push = reg_we && mine && (k == 4'd7);
            wire        cpu_pop  = reg_re && mine && (k == 4'd8);
            wire        cpu_exec = reg_we && mine && (k == 4'd9);
            wire [31:0] ni_data  = (s == 0) ? ni_tx_data0 : ni_tx_data1;

            wire        tx_empty, tx_full, rx_empty, rx_full;
            wire [31:0] tx_front, rx_front;
            wire        sm_pop, sm_push;
            wire [31:0] sm_rxd;

            ssc_fifo #(.WIDTH(32), .AW(2)) u_txf (
                .clk(clk), .rst_n(rst_n), .flush(1'b0),
                .wr_en(cpu_push | ni_tx_push[s]),
                .wr_data(cpu_push ? reg_wdata : ni_data),      // CPU first
                .rd_en(sm_pop), .rd_data(tx_front),
                .empty(tx_empty), .full(tx_full), .count(tx_level[3*s +: 3])
            );

            ssc_fifo #(.WIDTH(32), .AW(2)) u_rxf (
                .clk(clk), .rst_n(rst_n), .flush(1'b0),
                .wr_en(sm_push), .wr_data(sm_rxd),
                .rd_en(cpu_pop | ni_rx_pop[s]), .rd_data(rx_front),
                .empty(rx_empty), .full(rx_full), .count(rx_level[3*s +: 3])
            );

            // ---- the state machine ----
            wire [4:0]  pc;
            wire [3:0]  p_out, p_oe;
            wire        stl, idl, xpend;
            wire [31:0] x, y;
            wire [15:0] crc;

            se_sm u_sm (
                .clk(clk), .rst_n(rst_n),
                .enable(sm_en[s]), .restart(restart[s]),
                .clk_int(clk_int), .clk_frac(clk_frac),
                .pinctrl(pinctrl), .out_left(out_left), .in_left(in_left),
                .start_pc(start_pc),
                .crc_poly(crc_poly), .crc_init(crc_init), .time_us(time_us),
                .exec_wr(cpu_exec), .exec_instr(reg_wdata[15:0]),
                .pc_out(pc), .prog_data(prog[pc]),
                .tx_empty(tx_empty), .tx_data(tx_front), .tx_pop(sm_pop),
                .rx_full(rx_full), .rx_push(sm_push), .rx_data(sm_rxd),
                .rx_overflow(sm_ovf[s]),
                .pin_in(sync1), .pin_out(p_out), .pin_oe(p_oe),
                .stalled(stl), .idle(idl), .exec_pending(xpend),
                .x(x), .y(y), .crc(crc)
            );

            assign tx_full_w[s]  = tx_full;
            assign rx_empty_w[s] = rx_empty;
            assign stalled_w[s]  = stl;
            assign sm_idle[s]    = idl;
            assign outl_w[s]     = out_left;
            assign inl_w[s]      = in_left;
            assign pout_w[4*s +: 4] = p_out;
            assign poe_w[4*s +: 4]  = p_oe;

            // read-back values
            assign rd_clkdiv[32*s +: 32]  = {clk_int, clk_frac, 8'd0};
            assign rd_pinctrl[32*s +: 32] = {4'd0, pinctrl};
            assign rd_shift[32*s +: 32]   = {19'd0, start_pc, 6'd0, in_left, out_left};
            assign rd_rxf[32*s +: 32]     = rx_empty ? 32'd0 : rx_front;
            assign rd_state[32*s +: 32]   = {21'd0, xpend, sm_en[s], stl, 3'd0, pc};
            assign rd_x[32*s +: 32]       = x;
            assign rd_y[32*s +: 32]       = y;
            assign rd_crc[32*s +: 32]     = {16'd0, crc};
            assign rd_pins[32*s +: 32]    = {20'd0, p_oe, p_out, sync1};
        end
    endgenerate

    // ---------------- outputs ----------------
    assign sm0_out        = pout_w[3:0];
    assign sm0_oe         = poe_w[3:0];
    assign sm1_out        = pout_w[7:4];
    assign sm1_oe         = poe_w[7:4];
    assign ni_tx_full     = tx_full_w;
    assign ni_rx_empty    = rx_empty_w;
    assign ni_rx_data0    = rd_rxf[31:0];       // front of the RX FIFO, 0 when empty
    assign ni_rx_data1    = rd_rxf[63:32];
    assign rx_not_empty   = ~rx_empty_w;
    assign out_shift_left = outl_w;
    assign in_shift_left  = inl_w;

    wire [31:0] fstat = {2'b00, ovf, 2'b00, stalled_w,
                         4'd0, rx_empty_w[1], tx_full_w[1], rx_empty_w[0], tx_full_w[0],
                         1'b0, rx_level[5:3], 1'b0, tx_level[5:3],
                         1'b0, rx_level[2:0], 1'b0, tx_level[2:0]};

    // ---------------- register read (combinational) ----------------
    always @(*) begin
        reg_rdata = 32'd0;
        if (is_prog) begin
            reg_rdata = {16'd0, prog_rd};
        end else if (glob) begin
            case (k)
                4'd0:    reg_rdata = {30'd0, sm_en};
                4'd1:    reg_rdata = fstat;
                4'd2:    reg_rdata = {crc_init, crc_poly};
                default: reg_rdata = time_us;
            endcase
        end else if (sm_reg) begin
            case (k)
                4'd4:    reg_rdata = smsel ? rd_clkdiv[63:32] : rd_clkdiv[31:0];
                4'd5:    reg_rdata = smsel ? rd_pinctrl[63:32] : rd_pinctrl[31:0];
                4'd6:    reg_rdata = smsel ? rd_shift[63:32] : rd_shift[31:0];
                4'd8:    reg_rdata = smsel ? rd_rxf[63:32] : rd_rxf[31:0];
                4'd10:   reg_rdata = smsel ? rd_state[63:32] : rd_state[31:0];
                4'd11:   reg_rdata = smsel ? rd_x[63:32] : rd_x[31:0];
                4'd12:   reg_rdata = smsel ? rd_y[63:32] : rd_y[31:0];
                4'd13:   reg_rdata = smsel ? rd_crc[63:32] : rd_crc[31:0];
                4'd14:   reg_rdata = smsel ? rd_pins[63:32] : rd_pins[31:0];
                default: reg_rdata = 32'd0;          // TXF, EXEC (write only), 0x03C
            endcase
        end
    end
endmodule

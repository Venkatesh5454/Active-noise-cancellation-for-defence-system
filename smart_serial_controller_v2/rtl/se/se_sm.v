// =============================================================================
// se_sm.v  -  one state machine (SM) of the programmable serial engine
// -----------------------------------------------------------------------------
// A tiny processor with 8 instructions that is built to wiggle and sample
// pins with exact timing (like the RP2040 "PIO").  Its program lives in the
// 32 x 16-bit program memory of se_engine; this module only reads it.
//
//   tick      the fractional clock divider makes a one-clock "tick" every
//             INT + FRAC/256 system clocks (INT = 0 counts as 1).  One
//             instruction (or one delay step) runs per tick.
//   PC        5-bit program counter, wraps from 31 to 0.
//   X, Y      32-bit scratch registers (loop counters, data).
//   OSR       output shift register: PULL fills it from the TX FIFO, OUT
//             shifts bits out of it.  out count = bits already shifted out.
//   ISR       input shift register: IN shifts bits into it, PUSH sends it to
//             the RX FIFO.  in count = bits shifted in.
//   CRC       16-bit CRC register, fed one bit per system clock.
//   pins      4 pins, each with out (level) and oe (output enable).
//
// Instruction word: [15:13] opcode | [12:8] delay / side-set | [7:0] args
//   0 JMP  cond, addr    1 WAIT level, pin    2 IN src, n     3 OUT dst, n
//   4 PUSH/PULL          5 SET dst, value     6 OD pin, 0|Z   7 CRC n | reset
// (full table in docs/SPEC.md 5.1 and docs/blocks/serial_engine.md)
//
// How one tick is used (in this order of priority):
//   1. a CRC is still running (CRC n / OUT crc, n) -> wait, the SM is
//      stalled for n system clocks while the CRC unit eats one bit per clock
//   2. an EXEC instruction is pending              -> run it (see below)
//   3. the delay counter is not 0                  -> count it down
//   4. otherwise                                   -> run the instruction at PC
// "Run" means: drive the side-set pin (if SIDE_EN), then check whether the
// instruction can complete.  WAIT, PULL block (TX FIFO empty) and
// PUSH block (RX FIFO full) cannot: they "stall" and are tried again at the
// next tick.  When an instruction completes, its effects happen, the PC
// moves on and the delay counter is loaded with the delay field, so the
// delay is counted in ticks AFTER the instruction completes.
//
// Pin writes (SET pins, OUT pins, side-set): a pin in OD_MASK behaves as an
// open-drain pin (value 0 -> oe = 1, out = 0; value 1 -> oe = 0, out kept);
// any other pin gets out = value with oe unchanged.  The OD instruction
// always uses the open-drain rule.  Pin numbers wrap mod 4.  If side-set and
// the instruction write the same pin, side-set wins.
//
// Design choices (where the SPEC leaves room; see the block doc):
//   * While the SM is disabled it is frozen (pins hold) and its divider is
//     parked so that it ticks on every clock: an EXEC on a stopped SM runs on
//     the next clock, and the first instruction after SM_EN goes high runs
//     one clock after the write.  Two SMs enabled by the same write run in
//     lock-step.  RESTART also restarts the divider.
//   * EXEC runs at the next tick even if a delay is being counted (the old
//     delay is dropped).  A running CRC is finished first.  If the EXEC'd
//     instruction stalls it stays pending and is retried every tick.  A new
//     EXEC write replaces a pending one.  The delay of an EXEC'd instruction
//     is loaded as usual; on a disabled SM it is only counted once the SM is
//     enabled.  RESTART also drops a pending EXEC.
//   * OUT pins / OUT pindirs write at most 4 pins (bits 0..3 of the data);
//     IN pins with n > 4 reads the pins round and round (bit j = pin
//     (IN_BASE + j) mod 4).  SET_COUNT 0 and 5..7 mean 4.
//   * OUT crc, n feeds the n bits into the CRC in wire order (OUT right:
//     LSB first; OUT left: MSB first) and, like CRC n, takes n clocks.
//   * PUSH noblock with a full RX FIFO still clears ISR / in count.
//   * RESTART does not change the CRC register (SPEC lists what it resets);
//     it does abort a CRC that is still running.
// =============================================================================
`timescale 1ns / 1ps
module se_sm (
    input  wire        clk,
    input  wire        rst_n,
    // configuration (from the se_engine registers)
    input  wire        enable,          // SM_EN
    input  wire        restart,         // 1-clock pulse (SE_CTRL W1 bit)
    input  wire [15:0] clk_int,         // CLKDIV.INT
    input  wire [7:0]  clk_frac,        // CLKDIV.FRAC
    input  wire [27:0] pinctrl,         // SMs_PINCTRL[27:0]
    input  wire        out_left,        // SHIFTCTRL.OUT_SHIFT_LEFT
    input  wire        in_left,         // SHIFTCTRL.IN_SHIFT_LEFT
    input  wire [4:0]  start_pc,        // SHIFTCTRL.START_PC
    input  wire [15:0] crc_poly,
    input  wire [15:0] crc_init,
    input  wire [31:0] time_us,
    // EXEC: write an instruction to run at the next tick
    input  wire        exec_wr,
    input  wire [15:0] exec_instr,
    // program memory (asynchronous read, shared with the other SM)
    output wire [4:0]  pc_out,
    input  wire [15:0] prog_data,       // = program[pc_out]
    // TX FIFO (first-word fall-through)
    input  wire        tx_empty,
    input  wire [31:0] tx_data,
    output wire        tx_pop,
    // RX FIFO
    input  wire        rx_full,
    output wire        rx_push,
    output wire [31:0] rx_data,
    output wire        rx_overflow,     // 1-clock pulse: PUSH noblock dropped a word
    // pins
    input  wire [3:0]  pin_in,          // already synchronised by se_engine
    output reg  [3:0]  pin_out,
    output reg  [3:0]  pin_oe,
    // status
    output wire        stalled,
    output wire        idle,            // !enable, or stalled on PULL block with TX empty
    output wire        exec_pending,
    output reg  [31:0] x,
    output reg  [31:0] y,
    output reg  [15:0] crc
);
    // ---------------- opcodes ----------------
    localparam OP_JMP  = 3'd0;
    localparam OP_WAIT = 3'd1;
    localparam OP_IN   = 3'd2;
    localparam OP_OUT  = 3'd3;
    localparam OP_PP   = 3'd4;     // PUSH / PULL
    localparam OP_SET  = 3'd5;
    localparam OP_OD   = 3'd6;
    localparam OP_CRC  = 3'd7;

    // ---------------- state ----------------
    reg [4:0]  pc;
    reg [31:0] isr, osr;
    reg [5:0]  in_cnt, out_cnt;    // 0..32
    reg [4:0]  delay_cnt;
    reg        stall_r;            // the last try of the current instruction stalled
    reg        stall_pull;         // ... and it was a PULL block
    reg        exec_pend;
    reg [15:0] exec_ins;
    // CRC unit: crc_src[crc_idx] is the next bit, crc_left bits to go
    reg [31:0] crc_src;
    reg [4:0]  crc_idx;
    reg        crc_dn;             // 1: index counts down, 0: up
    reg [5:0]  crc_left;
    // fractional clock divider
    reg [16:0] div_cnt;
    reg [7:0]  div_acc;

    // ---------------- configuration fields ----------------
    wire [1:0] out_base  = pinctrl[1:0];
    wire [1:0] set_base  = pinctrl[3:2];
    wire [2:0] set_cnt_f = pinctrl[6:4];
    wire [1:0] in_base   = pinctrl[9:8];
    wire [1:0] side_pin  = pinctrl[11:10];
    wire       side_en   = pinctrl[12];
    wire [1:0] jmp_pin   = pinctrl[14:13];
    wire [3:0] od_mask   = pinctrl[19:16];
    wire [3:0] init_out  = pinctrl[23:20];
    wire [3:0] init_oe   = pinctrl[27:24];

    // ---------------- clock divider ----------------
    // div_cnt counts the clocks down to the next tick.  At a tick it is
    // reloaded with INT, plus 1 whenever the 8-bit FRAC accumulator
    // overflows, so on average a tick comes every INT + FRAC/256 clocks.
    wire        tick     = (div_cnt[16:1] == 16'd0);           // div_cnt <= 1
    wire [16:0] int_eff  = (clk_int == 16'd0) ? 17'd1 : {1'b0, clk_int};
    wire [8:0]  frac_sum = {1'b0, div_acc} + {1'b0, clk_frac};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 17'd1;
            div_acc <= 8'd0;
        end else if (restart || !enable) begin
            div_cnt <= 17'd1;               // parked: tick on every clock
            div_acc <= 8'd0;
        end else if (tick) begin
            div_cnt <= int_eff + {16'd0, frac_sum[8]};
            div_acc <= frac_sum[7:0];
        end else begin
            div_cnt <= div_cnt - 17'd1;
        end
    end

    // ---------------- current instruction ----------------
    wire [15:0] ins    = exec_pend ? exec_ins : prog_data;
    wire [2:0]  op     = ins[15:13];
    wire [4:0]  dly    = side_en ? {1'b0, ins[11:8]} : ins[12:8];
    wire        side_v = ins[12];
    wire [2:0]  sel    = ins[7:5];            // JMP cond / IN src / OUT dst / SET dst
    wire [4:0]  val    = ins[4:0];            // address / bit count / SET value
    wire [5:0]  nbits  = (val == 5'd0) ? 6'd32 : {1'b0, val};
    wire        is_pull = ins[7];
    wire        blk     = ins[6];

    wire crc_busy = (crc_left != 6'd0);
    // run an instruction at this clock?
    wire go  = tick && !crc_busy && (exec_pend || (enable && delay_cnt == 5'd0));
    // count one delay tick at this clock?
    wire dec = tick && !crc_busy && !exec_pend && enable && (delay_cnt != 5'd0);

    // ---------------- can the instruction complete? ----------------
    reg complete;
    always @(*) begin
        case (op)
            OP_WAIT: complete = (pin_in[ins[1:0]] == ins[7]);
            OP_PP:   complete = is_pull ? (!tx_empty || !blk) : (!rx_full || !blk);
            default: complete = 1'b1;
        endcase
    end

    // ---------------- JMP condition ----------------
    reg cond_ok;
    always @(*) begin
        case (sel)
            3'd0:    cond_ok = 1'b1;                   // always
            3'd1:    cond_ok = (x == 32'd0);           // !x
            3'd2:    cond_ok = (x != 32'd0);           // x--
            3'd3:    cond_ok = (y == 32'd0);           // !y
            3'd4:    cond_ok = (y != 32'd0);           // y--
            3'd5:    cond_ok = (x != y);               // x!=y
            3'd6:    cond_ok = pin_in[jmp_pin];        // pin
            default: cond_ok = (out_cnt < 6'd32);      // !osre
        endcase
    end

    // ---------------- shifting ----------------
    // mask with the low n bits set (n = 1..32)
    wire [31:0] mask_n = nbits[5] ? 32'hFFFF_FFFF : ((32'd1 << nbits[4:0]) - 32'd1);
    wire [5:0]  n_inv  = 6'd32 - nbits;                 // 32 - n (0..31)

    // IN: rotate the pins so bit j = pin (IN_BASE + j) mod 4, repeat 8 times
    wire [3:0]  in_rot = (in_base == 2'd0) ? pin_in :
                         (in_base == 2'd1) ? {pin_in[0],   pin_in[3:1]} :
                         (in_base == 2'd2) ? {pin_in[1:0], pin_in[3:2]} :
                                             {pin_in[2:0], pin_in[3]};
    reg [31:0] din;
    always @(*) begin
        case (sel)
            3'd0:    din = {8{in_rot}};                  // pins
            3'd1:    din = x;
            3'd2:    din = y;
            3'd3:    din = 32'd0;                        // null
            3'd4:    din = time_us;                      // time
            3'd5:    din = {16'd0, crc};                 // crc
            3'd6:    din = isr;
            default: din = osr;
        endcase
    end
    // IN right: new bits enter at the top; IN left: at the bottom
    wire [31:0] isr_in  = in_left ? ((isr << nbits) | (din & mask_n))
                                  : ((isr >> nbits) | (din << n_inv));
    // OUT right: LSB first; OUT left: MSB first
    wire [31:0] out_dat = out_left ? (osr >> n_inv) : (osr & mask_n);
    wire [31:0] osr_out = out_left ? (osr << nbits) : (osr >> nbits);

    // saturating counters (stop at 32)
    wire [6:0]  in_sum  = {1'b0, in_cnt}  + {1'b0, nbits};
    wire [6:0]  out_sum = {1'b0, out_cnt} + {1'b0, nbits};
    wire [5:0]  in_sat  = (in_sum  > 7'd32) ? 6'd32 : in_sum[5:0];
    wire [5:0]  out_sat = (out_sum > 7'd32) ? 6'd32 : out_sum[5:0];

    // ---------------- next PC ----------------
    // an EXEC'd instruction does not advance the PC unless it jumps
    wire [4:0] pc_inc = exec_pend ? pc : pc + 5'd1;
    wire [4:0] pc_nxt = (op == OP_JMP && cond_ok)  ? val :
                        (op == OP_OUT && sel == 3'd5) ? out_dat[4:0] : pc_inc;

    // ---------------- pins ----------------
    // rotate a 4-bit value left by s: bit j moves to pin (j + s) mod 4
    function [3:0] rotl4;
        input [3:0] v;
        input [1:0] s;
        begin
            case (s)
                2'd0:    rotl4 = v;
                2'd1:    rotl4 = {v[2:0], v[3]};
                2'd2:    rotl4 = {v[1:0], v[3:2]};
                default: rotl4 = {v[0],   v[3:1]};
            endcase
        end
    endfunction

    // the low n bits set, n = 1..32 but at most 4 pins
    function [3:0] lowmask4;
        input [5:0] n;
        begin
            case (n)
                6'd1:    lowmask4 = 4'b0001;
                6'd2:    lowmask4 = 4'b0011;
                6'd3:    lowmask4 = 4'b0111;
                default: lowmask4 = 4'b1111;
            endcase
        end
    endfunction

    // write value v to the pins in wm; pins in od use the open-drain rule.
    // Returns {oe, out}.
    function [7:0] pin_wr;
        input [3:0] o_in, e_in, v, wm, od;
        reg   [3:0] pp, odw, o, e;
        begin
            pp     = wm & ~od;                         // push-pull pins
            odw    = wm &  od;                         // open-drain pins
            o      = (o_in & ~pp) | (v & pp);          // out = value
            o      = o & ~(odw & ~v);                  // OD 0 -> out = 0
            e      = (e_in & ~odw) | (~v & odw);       // OD: oe = !value
            pin_wr = {e, o};
        end
    endfunction

    wire [3:0] set_wm  = rotl4(lowmask4((set_cnt_f == 3'd0 || set_cnt_f > 3'd4) ? 6'd4
                                                                                : {3'd0, set_cnt_f}),
                               set_base);
    wire [3:0] set_v   = rotl4(val[3:0], set_base);
    wire [3:0] out_wm  = rotl4(lowmask4(nbits), out_base);
    wire [3:0] out_v   = rotl4(out_dat[3:0], out_base);

    reg [3:0] n_out, n_oe;
    reg [7:0] pw;
    always @(*) begin
        n_out = pin_out;
        n_oe  = pin_oe;
        pw    = {pin_oe, pin_out};
        if (complete) begin
            case (op)
                OP_SET: begin
                    if (sel == 3'd0) begin                       // SET pins
                        pw = pin_wr(pin_out, pin_oe, set_v, set_wm, od_mask);
                        n_out = pw[3:0];
                        n_oe  = pw[7:4];
                    end else if (sel == 3'd4) begin              // SET pindirs
                        n_oe = (pin_oe & ~set_wm) | (set_v & set_wm);
                    end
                end
                OP_OUT: begin
                    if (sel == 3'd0) begin                       // OUT pins
                        pw = pin_wr(pin_out, pin_oe, out_v, out_wm, od_mask);
                        n_out = pw[3:0];
                        n_oe  = pw[7:4];
                    end else if (sel == 3'd4) begin              // OUT pindirs
                        n_oe = (pin_oe & ~out_wm) | (out_v & out_wm);
                    end
                end
                OP_OD: begin                                     // always open drain
                    pw = pin_wr(pin_out, pin_oe, {4{ins[7]}}, 4'b0001 << ins[1:0], 4'b1111);
                    n_out = pw[3:0];
                    n_oe  = pw[7:4];
                end
                default: ;
            endcase
        end
        // side-set: driven at every try, even if the instruction stalls
        if (side_en) begin
            pw = pin_wr(n_out, n_oe, {4{side_v}}, 4'b0001 << side_pin, od_mask);
            n_out = pw[3:0];
            n_oe  = pw[7:4];
        end
    end

    // ---------------- main sequential logic ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc         <= 5'd0;
            x          <= 32'd0;
            y          <= 32'd0;
            isr        <= 32'd0;
            osr        <= 32'd0;
            in_cnt     <= 6'd0;
            out_cnt    <= 6'd32;
            delay_cnt  <= 5'd0;
            stall_r    <= 1'b0;
            stall_pull <= 1'b0;
            exec_pend  <= 1'b0;
            exec_ins   <= 16'd0;
            pin_out    <= 4'd0;
            pin_oe     <= 4'd0;
            crc        <= 16'hFFFF;
            crc_src    <= 32'd0;
            crc_idx    <= 5'd0;
            crc_dn     <= 1'b0;
            crc_left   <= 6'd0;
        end else if (restart) begin
            pc         <= start_pc;
            x          <= 32'd0;
            y          <= 32'd0;
            isr        <= 32'd0;
            osr        <= 32'd0;
            in_cnt     <= 6'd0;
            out_cnt    <= 6'd32;            // OSR empty
            delay_cnt  <= 5'd0;
            stall_r    <= 1'b0;
            stall_pull <= 1'b0;
            exec_pend  <= 1'b0;
            pin_out    <= init_out;
            pin_oe     <= init_oe;
            crc_left   <= 6'd0;             // abort a running CRC
        end else begin
            // ---- CRC unit: one bit per system clock ----
            if (crc_busy) begin
                crc      <= {crc[14:0], 1'b0} ^ ((crc[15] ^ crc_src[crc_idx]) ? crc_poly : 16'h0000);
                crc_idx  <= crc_dn ? crc_idx - 5'd1 : crc_idx + 5'd1;
                crc_left <= crc_left - 6'd1;
            end

            // ---- delay ----
            if (dec)
                delay_cnt <= delay_cnt - 5'd1;

            // ---- run one instruction ----
            if (go) begin
                pin_out <= n_out;               // side-set (and pin writes)
                pin_oe  <= n_oe;
                if (!complete) begin
                    stall_r    <= 1'b1;
                    stall_pull <= (op == OP_PP) && is_pull;
                    delay_cnt  <= 5'd0;
                end else begin
                    stall_r    <= 1'b0;
                    stall_pull <= 1'b0;
                    delay_cnt  <= dly;
                    exec_pend  <= 1'b0;
                    pc         <= pc_nxt;
                    case (op)
                        OP_JMP: begin
                            if (sel == 3'd2) x <= x - 32'd1;    // x-- always decrements
                            if (sel == 3'd4) y <= y - 32'd1;    // y-- always decrements
                        end
                        OP_IN: begin
                            isr    <= isr_in;
                            in_cnt <= in_sat;
                        end
                        OP_OUT: begin
                            osr     <= osr_out;
                            out_cnt <= out_sat;
                            case (sel)
                                3'd1: x <= out_dat;
                                3'd2: y <= out_dat;
                                3'd6: begin
                                    isr    <= out_dat;
                                    in_cnt <= nbits;
                                end
                                3'd7: begin                     // feed the CRC, wire order
                                    crc_src  <= osr;
                                    crc_idx  <= out_left ? 5'd31 : 5'd0;
                                    crc_dn   <= out_left;
                                    crc_left <= nbits;
                                end
                                default: ;
                            endcase
                        end
                        OP_PP: begin
                            if (is_pull) begin
                                osr     <= tx_empty ? x : tx_data;   // noblock + empty: copy X
                                out_cnt <= 6'd0;
                            end else begin
                                isr    <= 32'd0;                     // pushed (or dropped)
                                in_cnt <= 6'd0;
                            end
                        end
                        OP_SET: begin
                            if (sel == 3'd1) x <= {27'd0, val};
                            if (sel == 3'd2) y <= {27'd0, val};
                        end
                        OP_CRC: begin
                            if (ins[7]) begin
                                crc <= crc_init;                    // CRC reset
                            end else begin                          // newest n ISR bits, oldest first
                                crc_src  <= isr;
                                crc_idx  <= in_left ? nbits[4:0] - 5'd1 : n_inv[4:0];
                                crc_dn   <= in_left;
                                crc_left <= nbits;
                            end
                        end
                        default: ;                                  // WAIT, OD: nothing else
                    endcase
                end
            end

            // a new EXEC write wins over the one that just finished
            if (exec_wr) begin
                exec_pend <= 1'b1;
                exec_ins  <= exec_instr;
            end
        end
    end

    // ---------------- outputs ----------------
    assign pc_out       = pc;
    assign tx_pop       = go && (op == OP_PP) &&  is_pull && !tx_empty;
    assign rx_push      = go && (op == OP_PP) && !is_pull && !rx_full;
    assign rx_overflow  = go && (op == OP_PP) && !is_pull &&  rx_full && !blk;
    assign rx_data      = isr;
    assign stalled      = stall_r || crc_busy;
    assign idle         = !enable || (stall_r && stall_pull && tx_empty);
    assign exec_pending = exec_pend;
endmodule

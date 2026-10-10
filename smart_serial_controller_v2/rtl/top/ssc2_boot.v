// =============================================================================
// ssc2_boot.v  -  "set-up CPU" for the FPGA-only build (Way 1)
// -----------------------------------------------------------------------------
// In Way 2 the ARM writes the set-up registers once and then sleeps.  In
// Way 1 there is no ARM, so this small sequencer plays its part: it is an APB
// master that works through a fixed list of steps (the ROM below).
//
//   WR   addr, data          write one register
//   CP   src -> dst          read src, then write dst = ((src >> SH) & MASK) << DSH | OR
//   WAIT n                   wait n milliseconds
//   JMP  i                   continue at step i (used for the endless loop)
//   END                      stop
//
// What the list does (all values are explained in docs/GETTING_STARTED.md):
//   1. UART 115200 8N1 on, SPI master 8-bit 1 MHz on, I2C 100 kHz on
//   2. UART NI in FRAME mode to the SPI node: the PC sends 01 9F 03 and gets
//      the flash ID 20 BA 19 back (slide 17)
//   3. SPI NI and I2C NI on, serial-engine NI on
//   4. SM0 runs the slide-13 UART TX program (8 ticks per bit, 115200 baud)
//   5. hub task 0: every 100 ms read 2 bytes from the TMP2 (0x4B, register 0)
//      and make a record for the DMA writer (slide 18)
//      hub task 1: every 100 ms send the last value to SM0 (shows on JD2
//      as two UART bytes when JD is switched to SM0)
//   6. DMA writer on: records go into the 4 KB block RAM
//   7. loop every 10 ms:  SW2..0 -> JD source (crossbar demo, slide 14)
//                         SW7    -> UART NI mode (0 FRAME, 1 ADDRESSED)
//                         LD0..3 <- number of records written / 4
// =============================================================================
`timescale 1ns / 1ps
module ssc2_boot #(
    parameter CLK_HZ = 100_000_000
) (
    input  wire        clk,
    input  wire        rst_n,
    // APB master
    output reg  [11:0] paddr,
    output reg         psel,
    output reg         penable,
    output reg         pwrite,
    output reg  [31:0] pwdata,
    input  wire [31:0] prdata,
    input  wire        pready,
    output reg         running        // the set-up list is done, the loop runs
);
    localparam OP_END = 3'd0, OP_WR = 3'd1, OP_WAIT = 3'd2, OP_CP = 3'd3, OP_JMP = 3'd4;
    localparam [16:0] MS_M1 = CLK_HZ / 1000 - 1;

    // ---------------- the step list ----------------
    // {op[2:0], addr[11:0], addr2[11:0], sh[4:0], mask[15:0], dsh[4:0], data[31:0]}
    localparam EW = 3 + 12 + 12 + 5 + 16 + 5 + 32;
    localparam LOOP = 6'd36;

    function [EW-1:0] wr(input [11:0] a, input [31:0] d);
        wr = {OP_WR, a, 12'd0, 5'd0, 16'd0, 5'd0, d};
    endfunction
    function [EW-1:0] cp(input [11:0] src, input [11:0] dst, input [4:0] sh,
                         input [15:0] mask, input [4:0] dsh, input [31:0] orv);
        cp = {OP_CP, src, dst, sh, mask, dsh, orv};
    endfunction

    function [EW-1:0] rom(input [5:0] i);
        case (i)
            // 1. engines (v1 registers)
            6'd0:  rom = wr(12'h100, 32'h0000_000F);     // UART_CTRL: TX, RX, 8 bits
            6'd1:  rom = wr(12'h200, 32'h0000_0071);     // SPI_CTRL: EN, mode 0, 8 bits
            6'd2:  rom = wr(12'h300, 32'h0000_0001);     // I2C_CTRL: EN (100 kHz)
            // 2./3. network interfaces
            6'd3:  rom = wr(12'h50C, 32'h0000_0043);     // NI_CFG3 UART: EN, FRAME, dest 4
            6'd4:  rom = wr(12'h510, 32'h0000_0001);     // NI_CFG4 SPI: EN
            6'd5:  rom = wr(12'h514, 32'h0000_0001);     // NI_CFG5 I2C: EN
            6'd6:  rom = wr(12'h500, 32'h0000_0031);     // NI_CFG0 SE: EN, SM0 RX -> node 3
            // 4. slide-13 UART TX program into PROG[0..6], SM0 set-up
            6'd7:  rom = wr(12'h900, 32'h0000_80C0);     // loop: PULL block
            6'd8:  rom = wr(12'h904, 32'h0000_A027);     //       SET  x, 7
            6'd9:  rom = wr(12'h908, 32'h0000_A700);     //       SET  pins, 0 [7]
            6'd10: rom = wr(12'h90C, 32'h0000_6601);     // bit:  OUT  pins, 1 [6]
            6'd11: rom = wr(12'h910, 32'h0000_0043);     //       JMP  x--, bit
            6'd12: rom = wr(12'h914, 32'h0000_A701);     //       SET  pins, 1 [7]
            6'd13: rom = wr(12'h918, 32'h0000_0000);     //       JMP  loop
            6'd14: rom = wr(12'h810, 32'h006C_8000);     // SM0_CLKDIV 108.5 -> 921.6 kHz
            6'd15: rom = wr(12'h814, 32'h0220_0015);     // SM0_PINCTRL TX = pin 1, idles high
            6'd16: rom = wr(12'h818, 32'h0000_0000);     // SM0_SHIFTCTRL: right, START_PC 0
            6'd17: rom = wr(12'h800, 32'h0000_0101);     // SE_CTRL: SM0 restart + enable
            // 5. sensor hub tasks
            6'd18: rom = wr(12'h640, 32'h0121_4B1B);     // T0: EN, I2C, XFER_REQ, 0x4B, w1 r2, RECORD
            6'd19: rom = wr(12'h644, 32'h0000_0000);     //     write byte 0 = 0x00 (temperature reg)
            6'd20: rom = wr(12'h64C, 32'h0000_0064);     //     every 100 ms
            6'd21: rom = wr(12'h650, 32'h0200_0001);     // T1: EN, SE node, DATA, SEND_LAST
            6'd22: rom = wr(12'h65C, 32'h0032_0064);     //     every 100 ms, 50 ms after T0
            // 6. DMA writer into the block RAM at 0, then the hub
            6'd23: rom = wr(12'h704, 32'h0000_0000);     // DMA_BASE
            6'd24: rom = wr(12'h700, 32'h0000_0001);     // DMA_CTRL: EN
            6'd25: rom = wr(12'h484, 32'h00F0_F000);     // GPIO_OE: LD0..3 and JD (if JD = GPIO)
            6'd26: rom = wr(12'h600, 32'h0000_0001);     // HUB_CTRL: EN
            // 7. the endless loop (starts at LOOP)
            6'd36: rom = cp(12'hA1C, 12'h40C, 5'd0, 16'h0007, 5'd0, 32'd0);      // SW2..0 -> PIN_SEL[JD]
            6'd37: rom = cp(12'hA1C, 12'h50C, 5'd7, 16'h0001, 5'd2, 32'h41);     // SW7 -> UART NI mode
            6'd38: rom = cp(12'h71C, 12'h480, 5'd2, 16'h000F, 5'd20, 32'd0);     // COUNT/4 -> LD0..3
            6'd39: rom = {OP_WAIT, 12'd0, 12'd0, 5'd0, 16'd0, 5'd0, 32'd10};       // 10 ms
            6'd40: rom = {OP_JMP,  12'd0, 12'd0, 5'd0, 16'd0, 5'd0, {26'd0, LOOP}};
            // steps 27..35 jump to the loop
            default: rom = (i < LOOP) ? {OP_JMP, 12'd0, 12'd0, 5'd0, 16'd0, 5'd0, {26'd0, LOOP}}
                                      : {EW{1'b0}};
        endcase
    endfunction

    // ---------------- the sequencer ----------------
    localparam S_FETCH = 3'd0, S_WSETUP = 3'd1, S_WACCESS = 3'd2,
               S_RSETUP = 3'd3, S_RACCESS = 3'd4, S_WAIT = 3'd5, S_STOP = 3'd6;

    reg  [5:0]    pc;
    reg  [2:0]    st;
    reg  [16:0]   div;
    reg  [31:0]   ms_left;
    wire [EW-1:0] e     = rom(pc);
    wire [2:0]    op    = e[EW-1 -: 3];
    wire [11:0]   a1    = e[EW-4 -: 12];
    wire [11:0]   a2    = e[EW-16 -: 12];
    wire [4:0]    sh    = e[EW-28 -: 5];
    wire [15:0]   mask  = e[EW-33 -: 16];
    wire [4:0]    dsh   = e[EW-49 -: 5];
    wire [31:0]   data  = e[31:0];
    wire [31:0]   moved = (((prdata >> sh) & {16'd0, mask}) << dsh) | data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc      <= 6'd0;
            st      <= S_FETCH;
            paddr   <= 12'd0;
            psel    <= 1'b0;
            penable <= 1'b0;
            pwrite  <= 1'b0;
            pwdata  <= 32'd0;
            div     <= 17'd0;
            ms_left <= 32'd0;
            running <= 1'b0;
        end else begin
            case (st)
                S_FETCH: begin
                    if (pc == LOOP) running <= 1'b1;
                    case (op)
                        OP_WR: begin
                            paddr <= a1; pwdata <= data; pwrite <= 1'b1; psel <= 1'b1;
                            st <= S_WSETUP;
                        end
                        OP_CP: begin
                            paddr <= a1; pwrite <= 1'b0; psel <= 1'b1;
                            st <= S_RSETUP;
                        end
                        OP_WAIT: begin
                            ms_left <= data; div <= 17'd0;
                            st <= S_WAIT;
                        end
                        OP_JMP:  pc <= data[5:0];
                        default: st <= S_STOP;
                    endcase
                end
                // APB write: setup clock, then access until PREADY
                S_WSETUP:  begin penable <= 1'b1; st <= S_WACCESS; end
                S_WACCESS: if (pready) begin
                    psel <= 1'b0; penable <= 1'b0; pwrite <= 1'b0;
                    pc   <= pc + 6'd1;
                    st   <= S_FETCH;
                end
                // APB read for CP, then the write of the moved value
                S_RSETUP:  begin penable <= 1'b1; st <= S_RACCESS; end
                S_RACCESS: if (pready) begin
                    penable <= 1'b0;                 // psel stays: next setup clock
                    paddr   <= a2;
                    pwdata  <= moved;
                    pwrite  <= 1'b1;
                    st      <= S_WSETUP;
                end
                S_WAIT: begin
                    if (ms_left == 32'd0) begin
                        pc <= pc + 6'd1;
                        st <= S_FETCH;
                    end else if (div == MS_M1) begin
                        div     <= 17'd0;
                        ms_left <= ms_left - 32'd1;
                    end else begin
                        div <= div + 17'd1;
                    end
                end
                default: ;                           // S_STOP
            endcase
        end
    end
endmodule

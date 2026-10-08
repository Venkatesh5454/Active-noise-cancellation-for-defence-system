// =============================================================================
// ssc_cmd_fsm.v  -  "Way 1" quick bring-up: a tiny APB master that acts as CPU
// -----------------------------------------------------------------------------
// No ARM, no software.  This state machine sits where the processor would sit
// and drives the controller's registers over APB, exactly like the C program
// does in Way 2.  It reads the keys you type on the PC terminal (through the
// UART block and the PmodUSBUART on JA) and reacts:
//
//     t   read the PmodTMP2 temperature over I2C (the worked example):
//           S 0x4B+W A 0x00 A Sr 0x4B+R A 0x0C A 0x80 N P
//         and print e.g. "Temp = +25.0000 C"
//     f   read the PmodSF3 JEDEC ID over SPI (command 0x9F) and print
//           e.g. "Flash ID = 20 BA 19"
//     h   print the help text
//     any other key is echoed back
//
// Building blocks of this FSM:
//   apb_call  - one APB transfer (setup + access), then jump to a return state
//   print     - send a message character by character to UART_TXDATA,
//               waiting whenever the TX FIFO is full.  Messages are stored as
//               right-aligned 128-byte strings; zero bytes are skipped, which
//               also hides leading zeros of numbers.
//   S_CONV    - binary -> decimal (double dabble, one bit per clock)
// =============================================================================
`timescale 1ns / 1ps
module ssc_cmd_fsm #(
    parameter [15:0] UART_DIV_INT  = 16'd54,  // 115200 baud at 100 MHz
    parameter [3:0]  UART_DIV_FRAC = 4'd4,
    parameter [6:0]  TMP2_ADDR     = 7'h4B    // PmodTMP2, both jumpers open
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
    // status for an LED
    output reg         err_flag
);
    // ---- register addresses (same map as ssc_regbank) ----
    localparam [11:0] R_INT_STATUS  = 12'h004,
                      R_UART_CTRL   = 12'h100,
                      R_UART_BAUD   = 12'h104,
                      R_UART_STATUS = 12'h108,
                      R_UART_TX     = 12'h10C,
                      R_UART_RX     = 12'h110,
                      R_SPI_CTRL    = 12'h200,
                      R_SPI_CLKDIV  = 12'h204,
                      R_SPI_STATUS  = 12'h208,
                      R_SPI_TX      = 12'h20C,
                      R_SPI_RX      = 12'h210,
                      R_I2C_CTRL    = 12'h300,
                      R_I2C_PRESC   = 12'h304,
                      R_I2C_ADDR    = 12'h308,
                      R_I2C_CMD     = 12'h30C,
                      R_I2C_RX      = 12'h318,
                      R_I2C_TX      = 12'h314;

    // ---- register values used here ----
    localparam [31:0] UART_ON   = 32'h0003_000F;  // TX+RX on, 8N1, flush both
    localparam [31:0] SPI_IDLE  = 32'h0000_0871;  // EN, mode 0, 8 bit, manual CS high
    localparam [31:0] SPI_SEL   = 32'h0000_1871;  // ... CS0 low
    localparam [31:0] SPI_FLUSH = 32'h0003_0000;
    localparam [31:0] I2C_ON    = 32'h0000_0001;
    localparam [31:0] I2C_FLUSH = 32'h0003_0000;
    localparam [31:0] I2C_ABORT = 32'h0004_0001;
    localparam [31:0] SPI_ABORT = 32'h0004_0871;
    localparam [31:0] CMD_W1    = 32'h0000_0001;  // write 1 byte, keep the bus
    localparam [31:0] CMD_R2P   = 32'h0000_0302;  // read 2 bytes, then STOP
    localparam        INT_I2C_DONE = 9,
                      INT_I2C_NACK = 10;

    // ---- states ----
    localparam [5:0]
        S_I0     = 6'd0,  S_I1   = 6'd1,  S_I2   = 6'd2,  S_I3   = 6'd3,
        S_I4     = 6'd4,  S_I5   = 6'd5,  S_I6   = 6'd6,
        S_MAIN   = 6'd7,  S_MAIN2 = 6'd8,
        S_T0     = 6'd9,  S_T1   = 6'd10, S_T2   = 6'd11, S_T3   = 6'd12,
        S_T4     = 6'd13, S_T5   = 6'd14, S_T6   = 6'd15, S_T7   = 6'd16,
        S_T8     = 6'd17, S_T9   = 6'd18, S_T10  = 6'd19, S_T11  = 6'd20,
        S_T12    = 6'd21, S_T13  = 6'd22,
        S_CONV0  = 6'd23, S_CONV1 = 6'd24,
        S_F0     = 6'd25, S_F1   = 6'd26, S_F2   = 6'd27, S_F3   = 6'd28,
        S_F4     = 6'd29, S_F5   = 6'd30, S_F6   = 6'd31, S_F7   = 6'd32,
        S_F8     = 6'd33, S_F9   = 6'd34, S_F10  = 6'd35, S_F11  = 6'd36,
        S_F12    = 6'd37, S_F13  = 6'd38,
        S_PRINT  = 6'd39, S_PR_CHK = 6'd40, S_PR_NEXT = 6'd41,
        S_APB    = 6'd42, S_APB_W = 6'd43,
        S_TMO0   = 6'd44, S_TMO1 = 6'd45, S_TMO2 = 6'd46;

    // ---- messages ----
    localparam [2:0] M_BANNER = 3'd0, M_TEMP = 3'd1, M_ID = 3'd2, M_NACK = 3'd3,
                     M_PROMPT = 3'd4, M_ECHO = 3'd5, M_TMO = 3'd6;

    reg [5:0]  st, ret_st, pret;
    reg [31:0] rdata;
    reg [2:0]  msg;
    reg [6:0]  pidx;                 // byte index, counts 127 -> 0
    reg [7:0]  cmd_ch;               // the key that was pressed
    reg [7:0]  t_msb, t_lsb;         // temperature register bytes
    reg [7:0]  id0, id1, id2;        // JEDEC ID bytes
    reg [20:0] dd;                   // double dabble: [20:9] BCD, [8:0] binary
    reg [3:0]  dd_n;
    reg [21:0] tmo;                  // poll counter for time-outs
    reg        t_neg_r;              // sign and fraction digits, captured once
    reg [15:0] fbcd;                 //   per reading (keeps the printer fast)

    // ------------------------------------------------------------------
    // temperature maths (ADT7420 13-bit mode, 0.0625 C per LSB)
    //   raw = {MSB, LSB};  t13 = raw >> 3 (two's complement)
    //   e.g. 0x0C80 >> 3 = 400 -> 400 x 0.0625 = 25.0 C
    // ------------------------------------------------------------------
    wire [12:0] t13   = {t_msb, t_lsb[7:3]};
    wire        t_neg = t13[12];
    wire [12:0] t_mag = t_neg ? (~t13 + 13'd1) : t13;
    wire [8:0]  t_int = t_mag[12:4];         // whole degrees
    wire [3:0]  t_frc = t_mag[3:0];          // sixteenths of a degree

    // sixteenths -> four decimal digits (exact: n x 0.0625)
    function [15:0] frac_bcd;
        input [3:0] f;
        case (f)
            4'd0:  frac_bcd = 16'h0000;  4'd1:  frac_bcd = 16'h0625;
            4'd2:  frac_bcd = 16'h1250;  4'd3:  frac_bcd = 16'h1875;
            4'd4:  frac_bcd = 16'h2500;  4'd5:  frac_bcd = 16'h3125;
            4'd6:  frac_bcd = 16'h3750;  4'd7:  frac_bcd = 16'h4375;
            4'd8:  frac_bcd = 16'h5000;  4'd9:  frac_bcd = 16'h5625;
            4'd10: frac_bcd = 16'h6250;  4'd11: frac_bcd = 16'h6875;
            4'd12: frac_bcd = 16'h7500;  4'd13: frac_bcd = 16'h8125;
            4'd14: frac_bcd = 16'h8750;  default: frac_bcd = 16'h9375;
        endcase
    endfunction

    // one step of double dabble: add 3 to every BCD digit >= 5, then shift
    /* verilator lint_off BLKSEQ */
    function [20:0] dd_step;
        input [20:0] x;
        reg   [20:0] y;
        begin
            y = x;
            if (y[12:9]  >= 4'd5) y[12:9]  = y[12:9]  + 4'd3;
            if (y[16:13] >= 4'd5) y[16:13] = y[16:13] + 4'd3;
            if (y[20:17] >= 4'd5) y[20:17] = y[20:17] + 4'd3;
            dd_step = y << 1;
        end
    endfunction
    /* verilator lint_on BLKSEQ */

    function [7:0] hex_ch;
        input [3:0] n;
        hex_ch = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n});
    endfunction

    function [7:0] dig_ch;                   // BCD digit -> ASCII
        input [3:0] n;
        dig_ch = 8'h30 + {4'd0, n};
    endfunction

    wire [11:0] ibcd = dd[20:9];
    // hide leading zeros (0 bytes are skipped by the printer)
    wire [7:0]  c_hun = (ibcd[11:8] == 4'd0) ? 8'd0 : dig_ch(ibcd[11:8]);
    wire [7:0]  c_ten = (ibcd[11:4] == 8'd0) ? 8'd0 : dig_ch(ibcd[7:4]);
    wire [7:0]  c_one = dig_ch(ibcd[3:0]);

    // ------------------------------------------------------------------
    // message text (right aligned in 128 bytes, "\015\n" = CR LF)
    // ------------------------------------------------------------------
    // (strings shorter than 128 bytes are zero-padded on the left on purpose)
    reg [8*128-1:0] mtext;
    /* verilator lint_off WIDTHEXPAND */
    always @* begin
        case (msg)
            M_BANNER: mtext = {"\015\nSmart Serial Controller - Way 1 bring-up\015\n",
                               "  t = temperature (PmodTMP2 on JC)\015\n",
                               "  f = flash JEDEC ID (PmodSF3 on JB)\015\n> "};
            M_TEMP:   mtext = {cmd_ch, "\015\nTemp = ", (t_neg_r ? "-" : "+"),
                               c_hun, c_ten, c_one, ".",
                               dig_ch(fbcd[15:12]), dig_ch(fbcd[11:8]),
                               dig_ch(fbcd[7:4]),   dig_ch(fbcd[3:0]),
                               " C\015\n> "};
            M_ID:     mtext = {cmd_ch, "\015\nFlash ID = ",
                               hex_ch(id0[7:4]), hex_ch(id0[3:0]), " ",
                               hex_ch(id1[7:4]), hex_ch(id1[3:0]), " ",
                               hex_ch(id2[7:4]), hex_ch(id2[3:0]), "\015\n> "};
            M_NACK:   mtext = {cmd_ch, "\015\nNo ACK from PmodTMP2 - check JC\015\n> "};
            M_PROMPT: mtext = "\015\n> ";
            M_ECHO:   mtext = {1016'd0, cmd_ch};
            default:  mtext = {cmd_ch, "\015\nTimeout - is the Pmod plugged in?\015\n> "};
        endcase
    end
    /* verilator lint_on WIDTHEXPAND */
    wire [7:0] pch = mtext[8*pidx +: 8];

    // ------------------------------------------------------------------
    // helpers (called inside the state machine)
    // ------------------------------------------------------------------
    task apb_call;
        input        wr;
        input [11:0] a;
        input [31:0] d;
        input [5:0]  ret;
        begin
            pwrite  <= wr;
            paddr   <= a;
            pwdata  <= d;
            psel    <= 1'b1;            // setup phase starts next clock
            penable <= 1'b0;
            ret_st  <= ret;
            st      <= S_APB;
        end
    endtask

    task print;
        input [2:0] id;
        input [5:0] ret;
        begin
            msg  <= id;
            pidx <= 7'd127;
            pret <= ret;
            st   <= S_PRINT;
        end
    endtask

    // ------------------------------------------------------------------
    // the state machine
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_I0;
            ret_st   <= S_I0;
            pret     <= S_MAIN;
            paddr    <= 12'd0;
            psel     <= 1'b0;
            penable  <= 1'b0;
            pwrite   <= 1'b0;
            pwdata   <= 32'd0;
            rdata    <= 32'd0;
            msg      <= M_BANNER;
            pidx     <= 7'd0;
            cmd_ch   <= 8'd0;
            t_msb    <= 8'd0;
            t_lsb    <= 8'd0;
            id0      <= 8'd0;
            id1      <= 8'd0;
            id2      <= 8'd0;
            dd       <= 21'd0;
            dd_n     <= 4'd0;
            tmo      <= 22'd0;
            t_neg_r  <= 1'b0;
            fbcd     <= 16'd0;
            err_flag <= 1'b0;
        end else begin
            case (st)
            // ---------------- APB transfer ----------------
            S_APB: begin
                penable <= 1'b1;                    // access phase
                st      <= S_APB_W;
            end
            S_APB_W: if (pready) begin
                rdata   <= prdata;
                psel    <= 1'b0;
                penable <= 1'b0;
                st      <= ret_st;
            end

            // ---------------- set-up after reset ----------------
            S_I0: apb_call(1'b1, R_UART_BAUD, {12'd0, UART_DIV_FRAC, UART_DIV_INT}, S_I1);
            S_I1: apb_call(1'b1, R_UART_CTRL,  UART_ON,           S_I2);
            S_I2: apb_call(1'b1, R_SPI_CLKDIV, 32'd49,            S_I3); // 1 MHz
            S_I3: apb_call(1'b1, R_SPI_CTRL,   SPI_IDLE,          S_I4);
            S_I4: apb_call(1'b1, R_I2C_PRESC,  32'd249,           S_I5); // 100 kHz
            S_I5: apb_call(1'b1, R_I2C_CTRL,   I2C_ON,            S_I6);
            S_I6: print(M_BANNER, S_MAIN);

            // ---------------- wait for a key ----------------
            S_MAIN: apb_call(1'b0, R_UART_RX, 32'd0, S_MAIN2);
            S_MAIN2:
                if (rdata[31]) st <= S_MAIN;        // RX FIFO empty
                else begin
                    cmd_ch <= rdata[7:0];
                    case (rdata[7:0])
                        "t", "T":      st <= S_T0;
                        "f", "F":      st <= S_F0;
                        "h", "H", "?": print(M_BANNER, S_MAIN);
                        8'h0D:         print(M_PROMPT, S_MAIN);
                        default:       print(M_ECHO,   S_MAIN);
                    endcase
                end

            // ---------------- 't': temperature over I2C ----------------
            S_T0: apb_call(1'b1, R_I2C_CTRL,   I2C_ON | I2C_FLUSH, S_T1);
            S_T1: apb_call(1'b1, R_INT_STATUS, 32'h0000_FFFF,      S_T2);
            S_T2: apb_call(1'b1, R_I2C_ADDR,   {25'd0, TMP2_ADDR}, S_T3);
            S_T3: apb_call(1'b1, R_I2C_TX,     32'h0000_0000,      S_T4); // pointer = temp
            S_T4: begin
                tmo <= 22'd0;
                apb_call(1'b1, R_I2C_CMD, CMD_W1, S_T5);   // S, 0x4B+W, 0x00, (keep bus)
            end
            S_T5: apb_call(1'b0, R_INT_STATUS, 32'd0, S_T6);
            S_T6:
                if (rdata[INT_I2C_DONE]) begin
                    if (rdata[INT_I2C_NACK]) begin
                        err_flag <= 1'b1;
                        print(M_NACK, S_MAIN);
                    end else st <= S_T7;
                end else if (&tmo) st <= S_TMO0;
                else begin
                    tmo <= tmo + 22'd1;
                    st  <= S_T5;
                end
            S_T7: apb_call(1'b1, R_INT_STATUS, 32'h0000_FFFF, S_T8);
            S_T8: begin
                tmo <= 22'd0;
                apb_call(1'b1, R_I2C_CMD, CMD_R2P, S_T9);  // Sr, 0x4B+R, 2 bytes, P
            end
            S_T9: apb_call(1'b0, R_INT_STATUS, 32'd0, S_T10);
            S_T10:
                if (rdata[INT_I2C_DONE]) begin
                    if (rdata[INT_I2C_NACK]) begin
                        err_flag <= 1'b1;
                        print(M_NACK, S_MAIN);
                    end else st <= S_T11;
                end else if (&tmo) st <= S_TMO0;
                else begin
                    tmo <= tmo + 22'd1;
                    st  <= S_T9;
                end
            S_T11: apb_call(1'b0, R_I2C_RX, 32'd0, S_T12);
            S_T12: begin
                t_msb <= rdata[7:0];
                apb_call(1'b0, R_I2C_RX, 32'd0, S_T13);
            end
            S_T13: begin
                t_lsb <= rdata[7:0];
                st    <= S_CONV0;
            end
            S_CONV0: begin
                t_neg_r <= t_neg;
                fbcd    <= frac_bcd(t_frc);
                dd   <= {12'd0, t_int};
                dd_n <= 4'd0;
                st   <= S_CONV1;
            end
            S_CONV1: begin
                dd   <= dd_step(dd);
                dd_n <= dd_n + 4'd1;
                if (dd_n == 4'd8) begin              // 9 shifts for 9 bits
                    err_flag <= 1'b0;
                    print(M_TEMP, S_MAIN);
                end
            end

            // ---------------- 'f': flash JEDEC ID over SPI ----------------
            S_F0: apb_call(1'b1, R_SPI_CTRL, SPI_IDLE | SPI_FLUSH, S_F1);
            S_F1: apb_call(1'b1, R_SPI_CTRL, SPI_SEL,  S_F2);   // CS0 low
            S_F2: apb_call(1'b1, R_SPI_TX,   32'h9F,   S_F3);   // READ ID command
            S_F3: apb_call(1'b1, R_SPI_TX,   32'h00,   S_F4);   // 3 dummy bytes
            S_F4: apb_call(1'b1, R_SPI_TX,   32'h00,   S_F5);
            S_F5: begin
                tmo <= 22'd0;
                apb_call(1'b1, R_SPI_TX, 32'h00, S_F6);
            end
            S_F6: apb_call(1'b0, R_SPI_STATUS, 32'd0, S_F7);
            S_F7:
                if (rdata[20:16] >= 5'd4 && !rdata[4]) st <= S_F8;   // 4 words, idle
                else if (&tmo) st <= S_TMO0;
                else begin
                    tmo <= tmo + 22'd1;
                    st  <= S_F6;
                end
            S_F8:  apb_call(1'b1, R_SPI_CTRL, SPI_IDLE, S_F9);      // CS0 high
            S_F9:  apb_call(1'b0, R_SPI_RX,   32'd0,    S_F10);     // byte during 0x9F
            S_F10: apb_call(1'b0, R_SPI_RX,   32'd0,    S_F11);
            S_F11: begin
                id0 <= rdata[7:0];
                apb_call(1'b0, R_SPI_RX, 32'd0, S_F12);
            end
            S_F12: begin
                id1 <= rdata[7:0];
                apb_call(1'b0, R_SPI_RX, 32'd0, S_F13);
            end
            S_F13: begin
                id2 <= rdata[7:0];
                print(M_ID, S_MAIN);
            end

            // ---------------- time-out: reset both engines ----------------
            S_TMO0: apb_call(1'b1, R_I2C_CTRL, I2C_ABORT, S_TMO1);
            S_TMO1: apb_call(1'b1, R_SPI_CTRL, SPI_ABORT, S_TMO2);
            S_TMO2: begin
                err_flag <= 1'b1;
                print(M_TMO, S_MAIN);
            end

            // ---------------- printer ----------------
            S_PRINT:
                if (pch == 8'd0) begin              // skip padding / hidden digit
                    if (pidx == 7'd0) st <= pret;
                    else              pidx <= pidx - 7'd1;
                end else
                    apb_call(1'b0, R_UART_STATUS, 32'd0, S_PR_CHK);
            S_PR_CHK:
                if (rdata[1])                       // TX FIFO full: ask again
                    apb_call(1'b0, R_UART_STATUS, 32'd0, S_PR_CHK);
                else
                    apb_call(1'b1, R_UART_TX, {24'd0, pch}, S_PR_NEXT);
            S_PR_NEXT:
                if (pidx == 7'd0) st <= pret;
                else begin
                    pidx <= pidx - 7'd1;
                    st   <= S_PRINT;
                end

            default: st <= S_I0;
            endcase
        end
    end
endmodule

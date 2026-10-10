// =============================================================================
// xbar.v  -  pin crossbar: any source on any port, switched safely (slide 14)
// -----------------------------------------------------------------------------
// The board has NP ports of 4 pins (0 JA, 1 JB, 2 JC, 3 JD, 4 OLED, 5 LED).
// Each port has ONE select register PIN_SEL[p] that names its source:
//     0 OFF (Z)  1 UART  2 SPI  3 I2C  4 SM0  5 SM1  6 GPIO  7 OFF
// Because every pin has exactly one select, two sources can never drive the
// same pin.  A source may be chosen by several ports at once.
//
// Data path (combinational, nothing is registered):
//   * pad_out/pad_oe of port p = the out/oe nibble of its source
//     (GPIO = nibble p of GPIO_OUT/GPIO_OE, OFF = oe 0, FLOAT = oe 0).
//   * src_in of source s = pad_in of the LOWEST-numbered port that selects s
//     and is connected (not FLOAT, not SETTLE); otherwise src_in_idle of s.
//   * pad_in is NOT synchronised here: every engine already has its own
//     2-FF synchroniser / glitch filter, so adding flops would only add delay.
//
// Switch state machine, one per port (only the control is registered):
//
//   IDLE --write PIN_SEL[p] != current--> WAIT_IDLE --old source idle--> FLOAT
//     ^       (old = OFF/GPIO or FORCE: straight to FLOAT)                |
//     |                                                       (1 clock, oe=0)
//     +---- 8 clocks ---- SETTLE <---- HAND-OVER: current = new ----------+
//                                      SW_CYCLES = counter, ev_switch pulse
//
//   * The cycle counter is 0 in the first clock after the write and counts
//     up by one every clock (it saturates at 0xFFFFFFFF).
//   * WAIT_IDLE looks at src_idle of the old source.  If it is 1 in clock c,
//     clock c+1 is the FLOAT clock, so the last clock in which the old source
//     drove the pins was an idle clock: a frame is never cut on the pins.
//     (A frame that the engine starts in the very clock the port floats never
//     reaches this port at all - software must stop feeding an engine before
//     it moves it away.)
//   * SW_CYCLES = counter value in the first clock in which the new source
//     drives the pins.  Old source idle: 2 (WAIT 1 clock + FLOAT 1 clock).
//     Old source OFF/GPIO or FORCE: 1 (FLOAT only).
//   * SETTLE lasts 8 clocks starting with the hand-over clock.  The new
//     source already drives the pins, but sees src_in_idle on this port so
//     its input filters restart from the idle level.
//   * SW_EDGE: the effective output of a pin is {oe, oe & out}.  From the
//     clock after the hand-over on, the first clock whose effective output
//     differs from the clock before latches SW_EDGE = counter.  It is 0 until
//     then and is cleared by every new request (so 0 means "no edge yet").
//   * SW_CYCLES keeps its old value until the next hand-over writes it.
//   * A write to PIN_SEL[p] is ignored while the port is switching (WAIT_IDLE,
//     FLOAT or SETTLE), and writing the current value does nothing.  The value
//     7 is stored as 7 (it behaves exactly like OFF).
//   * ev_switch is the OR of all ports: two ports that hand over in the same
//     clock give one pulse.
//
// Registers (byte offset inside the 0x400 region, reg_addr[1:0] ignored):
//   0x00+4p PIN_SEL[p]   W: [2:0] new source [8] FORCE
//                        R: [2:0] current [6:4] requested [8] switching
//   0x20    XBAR_STATUS  [NP-1:0] switching per port (RO)
//   0x40+4p SW_CYCLES[p] (RO)        0x60+4p SW_EDGE[p] (RO)
//   0x80    GPIO_OUT     4 bits per port   0x84 GPIO_OE   (reset 0)
//   0x88    GPIO_IN      pad_in through a 2-FF synchroniser (2 clocks late)
//   Unused addresses (and ports >= NP) read 0.  NP may be 1..8.
// Reset: JA = 1 (UART), JB = 2 (SPI), JC = 3 (I2C), JD = 0, OLED = 0, LED = 6.
// =============================================================================
`timescale 1ns / 1ps
module xbar #(
    parameter NP = 6                       // number of ports (1..8)
) (
    input  wire              clk,
    input  wire              rst_n,
    // register port
    input  wire              reg_we,
    input  wire              reg_re,       // unused: no pop-on-read registers
    input  wire [7:0]        reg_addr,
    input  wire [31:0]       reg_wdata,
    output wire [31:0]       reg_rdata,
    // sources 1..5 (UART, SPI, I2C, SM0, SM1): source s at [4*(s-1) +: 4]
    input  wire [19:0]       src_out,
    input  wire [19:0]       src_oe,
    output wire [19:0]       src_in,       // what each source sees on its inputs
    input  wire [4:0]        src_idle,     // bit s-1: source s is between frames
    input  wire [19:0]       src_in_idle,  // input level while not connected
    // pads: port p at [4*p +: 4]
    output wire [4*NP-1:0]   pad_out,
    output wire [4*NP-1:0]   pad_oe,
    input  wire [4*NP-1:0]   pad_in,
    output wire              ev_switch     // 1-clock pulse at every hand-over
);
    // switch states
    localparam [1:0] S_IDLE   = 2'd0,
                     S_WAIT   = 2'd1,      // WAIT_IDLE: old source still drives
                     S_FLOAT  = 2'd2,      // all 4 pins oe = 0 for one clock
                     S_SETTLE = 2'd3;      // new source drives, sees idle inputs

    localparam [31:0] CNT_MAX = 32'hFFFF_FFFF;

    // ------------------------------------------------------------------
    // GPIO registers and the GPIO_IN synchroniser
    // ------------------------------------------------------------------
    reg [4*NP-1:0] gpio_out;
    reg [4*NP-1:0] gpio_oe;
    (* ASYNC_REG = "TRUE" *) reg [4*NP-1:0] gin_s1;
    (* ASYNC_REG = "TRUE" *) reg [4*NP-1:0] gin_s2;

    wire wr_gpio_out = reg_we && (reg_addr[7:2] == 6'h20);   // 0x80
    wire wr_gpio_oe  = reg_we && (reg_addr[7:2] == 6'h21);   // 0x84

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpio_out <= {(4*NP){1'b0}};
            gpio_oe  <= {(4*NP){1'b0}};
            gin_s1   <= {(4*NP){1'b0}};
            gin_s2   <= {(4*NP){1'b0}};
        end else begin
            if (wr_gpio_out) gpio_out <= reg_wdata[4*NP-1:0];
            if (wr_gpio_oe)  gpio_oe  <= reg_wdata[4*NP-1:0];
            gin_s1 <= pad_in;
            gin_s2 <= gin_s1;
        end
    end

    // "idle" flag of every source code 0..7.  OFF and GPIO are always idle
    // (they are skipped anyway: the port goes straight to FLOAT).
    wire [7:0] idle8 = {1'b1, 1'b1, src_idle, 1'b1};

    // ------------------------------------------------------------------
    // per-port state, collected into flat vectors sized for 8 ports so the
    // read mux can index them directly (ports >= NP read 0)
    // ------------------------------------------------------------------
    wire [8*9-1:0]  pinsel_all;    // PIN_SEL read value [8:0] per port
    wire [8*32-1:0] swc_all;       // SW_CYCLES per port
    wire [8*32-1:0] swe_all;       // SW_EDGE per port
    wire [7:0]      switching;     // port is switching (status bits)
    wire [8*NP-1:0] in_hot;        // port p feeds source code c: bit 8*p + c
    wire [NP-1:0]   ho;            // port p is in its hand-over clock

    genvar gp;
    generate
        for (gp = 0; gp < NP; gp = gp + 1) begin : g_port
            /* verilator lint_off WIDTHTRUNC */
            localparam [2:0] PIDX    = gp;
            localparam [2:0] RST_SRC = (gp == 0) ? 3'd1 :      // JA  UART
                                       (gp == 1) ? 3'd2 :      // JB  SPI
                                       (gp == 2) ? 3'd3 :      // JC  I2C
                                       (gp == 5) ? 3'd6 :      // LED GPIO
                                                   3'd0;       // JD, OLED: OFF
            /* verilator lint_on WIDTHTRUNC */

            reg [2:0]  cur;        // source on the pins now
            reg [2:0]  req;        // source asked for (== cur when idle)
            reg [1:0]  st;         // switch state
            reg [2:0]  sc;         // settle clock counter 0..7
            reg [31:0] cnt;        // clocks since the request
            reg [31:0] swc;        // SW_CYCLES
            reg [31:0] swe;        // SW_EDGE
            reg        armed;      // waiting for the first output edge
            reg [7:0]  prev_eff;   // effective output one clock ago

            // the 8 possible output nibbles, indexed by the source code
            wire [31:0] out8 = {4'b0000, gpio_out[4*gp +: 4], src_out, 4'b0000};
            wire [31:0] oe8  = {4'b0000, gpio_oe[4*gp +: 4],  src_oe,  4'b0000};

            wire       flt    = (st == S_FLOAT);
            wire [3:0] p_out  = flt ? 4'b0000 : out8[{cur, 2'b00} +: 4];
            wire [3:0] p_oe   = flt ? 4'b0000 : oe8[{cur, 2'b00} +: 4];
            wire [7:0] eff    = {p_oe, p_oe & p_out};   // what the pins really show

            // a write to this port's PIN_SEL that starts a switch
            wire       wr_me  = reg_we && (reg_addr[7:5] == 3'b000) &&
                                (reg_addr[4:2] == PIDX);
            wire       start  = wr_me && (st == S_IDLE) && (reg_wdata[2:0] != cur);
            // no need to wait: old source is OFF (0/7) or GPIO (6), or FORCE
            wire       skip   = reg_wdata[8] || (cur == 3'd0) || (cur[2:1] == 2'b11);

            wire       ho_now   = (st == S_SETTLE) && (sc == 3'd0);
            wire       edge_now = armed && !ho_now && (eff != prev_eff);

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cur      <= RST_SRC;
                    req      <= RST_SRC;
                    st       <= S_IDLE;
                    sc       <= 3'd0;
                    cnt      <= 32'd0;
                    swc      <= 32'd0;
                    swe      <= 32'd0;
                    armed    <= 1'b0;
                    prev_eff <= 8'd0;
                end else begin
                    prev_eff <= eff;
                    if (cnt != CNT_MAX) cnt <= cnt + 32'd1;

                    // first output edge after the hand-over
                    if (edge_now) begin
                        swe   <= cnt;
                        armed <= 1'b0;
                    end

                    case (st)
                        S_IDLE: begin
                            if (start) begin
                                req   <= reg_wdata[2:0];
                                cnt   <= 32'd0;
                                swe   <= 32'd0;
                                armed <= 1'b0;
                                st    <= skip ? S_FLOAT : S_WAIT;
                            end
                        end
                        S_WAIT: begin
                            if (idle8[cur]) st <= S_FLOAT;
                        end
                        S_FLOAT: begin                   // hand-over
                            cur   <= req;
                            swc   <= (cnt == CNT_MAX) ? CNT_MAX : cnt + 32'd1;
                            armed <= 1'b1;
                            sc    <= 3'd0;
                            st    <= S_SETTLE;
                        end
                        default: begin                   // S_SETTLE
                            if (sc == 3'd7) st <= S_IDLE;
                            else            sc <= sc + 3'd1;
                        end
                    endcase
                end
            end

            // connected = the port's pins feed its source's inputs
            wire conn = (st == S_IDLE) || (st == S_WAIT);

            assign pad_out[4*gp +: 4]   = p_out;
            assign pad_oe[4*gp +: 4]    = p_oe;
            assign in_hot[8*gp +: 8]    = conn ? (8'd1 << cur) : 8'd0;
            assign ho[gp]               = ho_now;
            assign switching[gp]        = (st != S_IDLE);
            assign pinsel_all[9*gp +: 9] = {(st != S_IDLE), 1'b0, req, 1'b0, cur};
            assign swc_all[32*gp +: 32] = swc;
            assign swe_all[32*gp +: 32] = swe;
        end

        // ports that do not exist read 0
        for (gp = NP; gp < 8; gp = gp + 1) begin : g_none
            assign switching[gp]         = 1'b0;
            assign pinsel_all[9*gp +: 9] = 9'd0;
            assign swc_all[32*gp +: 32]  = 32'd0;
            assign swe_all[32*gp +: 32]  = 32'd0;
        end
    endgenerate

    assign ev_switch = |ho;

    // ------------------------------------------------------------------
    // source inputs: the lowest-numbered connected port wins
    // ------------------------------------------------------------------
    reg [19:0] src_in_r;
    integer s, p;
    always @(*) begin
        src_in_r = src_in_idle;
        for (s = 0; s < 5; s = s + 1)
            for (p = NP - 1; p >= 0; p = p - 1)        // last write (lowest p) wins
                if (in_hot[8*p + s + 1])
                    src_in_r[4*s +: 4] = pad_in[4*p +: 4];
    end
    assign src_in = src_in_r;

    // ------------------------------------------------------------------
    // register read (combinational)
    // ------------------------------------------------------------------
    wire [31:0] gpio_out32, gpio_oe32, gpio_in32;
    generate
        if (NP < 8) begin : g_pad32
            assign gpio_out32 = {{(32-4*NP){1'b0}}, gpio_out};
            assign gpio_oe32  = {{(32-4*NP){1'b0}}, gpio_oe};
            assign gpio_in32  = {{(32-4*NP){1'b0}}, gin_s2};
        end else begin : g_full32
            assign gpio_out32 = gpio_out;
            assign gpio_oe32  = gpio_oe;
            assign gpio_in32  = gin_s2;
        end
    endgenerate

    wire [2:0] idx = reg_addr[4:2];
    reg [31:0] rdata;
    always @(*) begin
        rdata = 32'd0;
        case (reg_addr[7:5])
            3'b000: rdata = {23'd0, pinsel_all[9*idx +: 9]};          // PIN_SEL
            3'b001: if (idx == 3'd0) rdata = {24'd0, switching};      // XBAR_STATUS
            3'b010: rdata = swc_all[{idx, 5'b00000} +: 32];           // SW_CYCLES
            3'b011: rdata = swe_all[{idx, 5'b00000} +: 32];           // SW_EDGE
            3'b100: begin
                case (idx)
                    3'd0:    rdata = gpio_out32;
                    3'd1:    rdata = gpio_oe32;
                    3'd2:    rdata = gpio_in32;
                    default: rdata = 32'd0;
                endcase
            end
            default: rdata = 32'd0;
        endcase
    end
    assign reg_rdata = rdata;

endmodule

// =============================================================================
// ssc_regbank.v  -  register bank = the controller's "control panel" (APB slave)
// -----------------------------------------------------------------------------
// APB (AMBA Advanced Peripheral Bus) in two clocks:
//     setup  : PSEL=1, PENABLE=0, address/data/direction are valid
//     access : PSEL=1, PENABLE=1  -> the write or read happens on this clock
// PREADY is always 1 (no wait states), PSLVERR always 0.
//
// Register map (byte offsets; every register is 32 bits)
// ---------------------------------------------------------------------------
// 0x000 ID           RO  0x53534301 ("SSC" v1)
// 0x004 INT_STATUS   R/W1C  event flags (see ssc_apb_top for the bit list)
// 0x008 INT_ENABLE   RW  which flags drive the IRQ line
// 0x00C BRIDGE_CTRL  RW  [1:0] ch0 src [3:2] ch0 dst [5:4] ch1 src [7:6] ch1 dst
//                        codes: 0 off, 1 SPI, 2 I2C, 3 UART
// 0x010 BRIDGE_CNT   RO  [15:0] bytes moved by ch0, [31:16] by ch1 (write clears)
// 0x014 SCRATCH      RW  free register for bus tests
//
// 0x100 UART_CTRL    RW  [0] TX_EN [1] RX_EN [3:2] BITS (0=5..3=8) [4] PAR_EN
//                        [5] PAR_ODD [6] STOP2 [7] FLOW (RTS/CTS) [8] LOOPBACK
//                        W1: [16] TX_FLUSH [17] RX_FLUSH
// 0x104 UART_BAUD    RW  [15:0] divisor integer part, [19:16] fraction (1/16)
//                        divisor = f_clk / (16 x baud); reset = 115200 @100MHz
// 0x108 UART_STATUS  RO  [0] TX_EMPTY [1] TX_FULL [2] RX_EMPTY [3] RX_FULL
//                        [4] TX_BUSY [5] RX_BUSY [6] CTS_OK
//                        [12:8] TX_COUNT [20:16] RX_COUNT
// 0x10C UART_TXDATA  WO  [7:0] byte -> TX FIFO
// 0x110 UART_RXDATA  RO  [7:0] byte from RX FIFO, [31] EMPTY (nothing popped)
//
// 0x200 SPI_CTRL     RW  [0] EN [1] CPOL [2] CPHA [3] LSB_FIRST [8:4] LEN-1
//                        [10:9] CS_SEL [11] CS_MANUAL [12] CS_LEVEL [13] SLAVE
//                        [14] LOOPBACK   W1: [16] TX_FLUSH [17] RX_FLUSH [18] ABORT
// 0x204 SPI_CLKDIV   RW  [15:0] SCLK = f_clk / (2 x (DIV+1)); reset 49 = 1 MHz
// 0x208 SPI_STATUS   RO  [0] TX_EMPTY [1] TX_FULL [2] RX_EMPTY [3] RX_FULL
//                        [4] BUSY [12:8] TX_COUNT [20:16] RX_COUNT
// 0x20C SPI_TXDATA   WO  word -> TX FIFO
// 0x210 SPI_RXDATA   RO  word from RX FIFO (0 if empty - check RX_EMPTY first)
//
// 0x300 I2C_CTRL     RW  [0] EN [1] AUTO_WR   W1: [16] TX_FLUSH [17] RX_FLUSH
//                        [18] ABORT
// 0x304 I2C_PRESCALE RW  [15:0] f_SCL ~= f_clk / (4 x (P+1)); reset 249=100 kHz
// 0x308 I2C_ADDR     RW  [9:0] slave address, [15] TEN_BIT
// 0x30C I2C_CMD      WO  [7:0] LEN [8] READ [9] STOP [10] STOP_ONLY -> starts
// 0x310 I2C_STATUS   RO  [0] TX_EMPTY [1] TX_FULL [2] RX_EMPTY [3] RX_FULL
//                        [4] BUSY [5] HOLDING [6] NACK [7] ARB_LOST
//                        [12:8] TX_COUNT [20:16] RX_COUNT
//                        [24] BUS_BUSY [25] SCL [26] SDA
// 0x314 I2C_TXDATA   WO  [7:0] byte -> TX FIFO
// 0x318 I2C_RXDATA   RO  [7:0] byte from RX FIFO, [31] EMPTY
// =============================================================================
`timescale 1ns / 1ps
module ssc_regbank (
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
    output wire        cpu_access,       // tells the bridge engine to wait
    // interrupt controller
    input  wire [15:0] int_status,
    output reg  [15:0] int_enable,
    output wire        int_clear,        // W1C strobe (mask = pwdata)
    // bridge engine
    output reg  [7:0]  bridge_ctrl,
    output wire        bridge_cnt_clear,
    input  wire [15:0] bridge_cnt0,
    input  wire [15:0] bridge_cnt1,
    // UART
    output reg         uart_tx_en,
    output reg         uart_rx_en,
    output reg  [1:0]  uart_bits,
    output reg         uart_par_en,
    output reg         uart_par_odd,
    output reg         uart_stop2,
    output reg         uart_flow,
    output reg         uart_loop,
    output reg  [15:0] uart_baud_int,
    output reg  [3:0]  uart_baud_frac,
    output wire        uart_tx_flush,
    output wire        uart_rx_flush,
    output wire        uart_tx_push,
    output wire        uart_rx_pop,
    input  wire [31:0] uart_status,
    input  wire [7:0]  uart_rx_data,
    // SPI
    output reg         spi_en,
    output reg         spi_cpol,
    output reg         spi_cpha,
    output reg         spi_lsb,
    output reg  [4:0]  spi_len_m1,
    output reg  [1:0]  spi_cs_sel,
    output reg         spi_cs_manual,
    output reg         spi_cs_level,
    output reg         spi_slave,
    output reg         spi_loop,
    output reg  [15:0] spi_clk_div,
    output wire        spi_tx_flush,
    output wire        spi_rx_flush,
    output wire        spi_abort,
    output wire        spi_tx_push,
    output wire        spi_rx_pop,
    input  wire [31:0] spi_status,
    input  wire [31:0] spi_rx_data,
    // I2C
    output reg         i2c_en,
    output reg         i2c_auto,
    output reg  [15:0] i2c_prescale,
    output reg  [9:0]  i2c_addr,
    output reg         i2c_ten,
    output wire        i2c_cmd_valid,
    output wire        i2c_tx_flush,
    output wire        i2c_rx_flush,
    output wire        i2c_abort,
    output wire        i2c_tx_push,
    output wire        i2c_rx_pop,
    input  wire [31:0] i2c_status,
    input  wire [7:0]  i2c_rx_data
);
    localparam A_ID          = 12'h000,
               A_INT_STATUS  = 12'h004,
               A_INT_ENABLE  = 12'h008,
               A_BRIDGE_CTRL = 12'h00C,
               A_BRIDGE_CNT  = 12'h010,
               A_SCRATCH     = 12'h014,
               A_UART_CTRL   = 12'h100,
               A_UART_BAUD   = 12'h104,
               A_UART_STATUS = 12'h108,
               A_UART_TX     = 12'h10C,
               A_UART_RX     = 12'h110,
               A_SPI_CTRL    = 12'h200,
               A_SPI_CLKDIV  = 12'h204,
               A_SPI_STATUS  = 12'h208,
               A_SPI_TX      = 12'h20C,
               A_SPI_RX      = 12'h210,
               A_I2C_CTRL    = 12'h300,
               A_I2C_PRESC   = 12'h304,
               A_I2C_ADDR    = 12'h308,
               A_I2C_CMD     = 12'h30C,
               A_I2C_STATUS  = 12'h310,
               A_I2C_TX      = 12'h314,
               A_I2C_RX      = 12'h318;

    localparam [31:0] ID_VALUE = 32'h5353_4301;

    wire [11:0] a  = {paddr[11:2], 2'b00};
    wire        wr = psel & penable &  pwrite;
    wire        rd = psel & penable & ~pwrite;

    assign pready     = 1'b1;
    assign pslverr    = 1'b0;
    assign cpu_access = psel;

    reg [31:0] scratch;

    // ---- one-clock strobes ----
    assign int_clear        = wr & (a == A_INT_STATUS);
    assign bridge_cnt_clear = wr & (a == A_BRIDGE_CNT);

    assign uart_tx_flush = wr & (a == A_UART_CTRL) & pwdata[16];
    assign uart_rx_flush = wr & (a == A_UART_CTRL) & pwdata[17];
    assign uart_tx_push  = wr & (a == A_UART_TX);
    assign uart_rx_pop   = rd & (a == A_UART_RX) & ~uart_status[2];

    assign spi_tx_flush  = wr & (a == A_SPI_CTRL) & pwdata[16];
    assign spi_rx_flush  = wr & (a == A_SPI_CTRL) & pwdata[17];
    assign spi_abort     = wr & (a == A_SPI_CTRL) & pwdata[18];
    assign spi_tx_push   = wr & (a == A_SPI_TX);
    assign spi_rx_pop    = rd & (a == A_SPI_RX) & ~spi_status[2];

    assign i2c_tx_flush  = wr & (a == A_I2C_CTRL) & pwdata[16];
    assign i2c_rx_flush  = wr & (a == A_I2C_CTRL) & pwdata[17];
    assign i2c_abort     = wr & (a == A_I2C_CTRL) & pwdata[18];
    assign i2c_cmd_valid = wr & (a == A_I2C_CMD);
    assign i2c_tx_push   = wr & (a == A_I2C_TX);
    assign i2c_rx_pop    = rd & (a == A_I2C_RX) & ~i2c_status[2];

    // ---- settings registers ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_enable     <= 16'd0;
            bridge_ctrl    <= 8'd0;
            scratch        <= 32'd0;
            uart_tx_en     <= 1'b0;
            uart_rx_en     <= 1'b0;
            uart_bits      <= 2'd3;          // 8 data bits
            uart_par_en    <= 1'b0;
            uart_par_odd   <= 1'b0;
            uart_stop2     <= 1'b0;
            uart_flow      <= 1'b0;
            uart_loop      <= 1'b0;
            uart_baud_int  <= 16'd54;        // 115200 baud at 100 MHz
            uart_baud_frac <= 4'd4;
            spi_en         <= 1'b0;
            spi_cpol       <= 1'b0;
            spi_cpha       <= 1'b0;
            spi_lsb        <= 1'b0;
            spi_len_m1     <= 5'd7;          // 8-bit words
            spi_cs_sel     <= 2'd0;
            spi_cs_manual  <= 1'b0;
            spi_cs_level   <= 1'b0;
            spi_slave      <= 1'b0;
            spi_loop       <= 1'b0;
            spi_clk_div    <= 16'd49;        // 1 MHz
            i2c_en         <= 1'b0;
            i2c_auto       <= 1'b0;
            i2c_prescale   <= 16'd249;       // 100 kHz
            i2c_addr       <= 10'd0;
            i2c_ten        <= 1'b0;
        end else if (wr) begin
            case (a)
                A_INT_ENABLE:  int_enable  <= pwdata[15:0];
                A_BRIDGE_CTRL: bridge_ctrl <= pwdata[7:0];
                A_SCRATCH:     scratch     <= pwdata;
                A_UART_CTRL: begin
                    uart_tx_en   <= pwdata[0];
                    uart_rx_en   <= pwdata[1];
                    uart_bits    <= pwdata[3:2];
                    uart_par_en  <= pwdata[4];
                    uart_par_odd <= pwdata[5];
                    uart_stop2   <= pwdata[6];
                    uart_flow    <= pwdata[7];
                    uart_loop    <= pwdata[8];
                end
                A_UART_BAUD: begin
                    uart_baud_int  <= pwdata[15:0];
                    uart_baud_frac <= pwdata[19:16];
                end
                A_SPI_CTRL: begin
                    spi_en        <= pwdata[0];
                    spi_cpol      <= pwdata[1];
                    spi_cpha      <= pwdata[2];
                    spi_lsb       <= pwdata[3];
                    spi_len_m1    <= pwdata[8:4];
                    spi_cs_sel    <= pwdata[10:9];
                    spi_cs_manual <= pwdata[11];
                    spi_cs_level  <= pwdata[12];
                    spi_slave     <= pwdata[13];
                    spi_loop      <= pwdata[14];
                end
                A_SPI_CLKDIV: spi_clk_div <= pwdata[15:0];
                A_I2C_CTRL: begin
                    i2c_en   <= pwdata[0];
                    i2c_auto <= pwdata[1];
                end
                A_I2C_PRESC: i2c_prescale <= pwdata[15:0];
                A_I2C_ADDR: begin
                    i2c_addr <= pwdata[9:0];
                    i2c_ten  <= pwdata[15];
                end
                default: ;
            endcase
        end
    end

    // ---- read multiplexer ----
    always @* begin
        case (a)
            A_ID:          prdata = ID_VALUE;
            A_INT_STATUS:  prdata = {16'd0, int_status};
            A_INT_ENABLE:  prdata = {16'd0, int_enable};
            A_BRIDGE_CTRL: prdata = {24'd0, bridge_ctrl};
            A_BRIDGE_CNT:  prdata = {bridge_cnt1, bridge_cnt0};
            A_SCRATCH:     prdata = scratch;
            A_UART_CTRL:   prdata = {23'd0, uart_loop, uart_flow, uart_stop2,
                                     uart_par_odd, uart_par_en, uart_bits,
                                     uart_rx_en, uart_tx_en};
            A_UART_BAUD:   prdata = {12'd0, uart_baud_frac, uart_baud_int};
            A_UART_STATUS: prdata = uart_status;
            A_UART_RX:     prdata = uart_status[2] ? 32'h8000_0000
                                                   : {24'd0, uart_rx_data};
            A_SPI_CTRL:    prdata = {17'd0, spi_loop, spi_slave, spi_cs_level,
                                     spi_cs_manual, spi_cs_sel, spi_len_m1,
                                     spi_lsb, spi_cpha, spi_cpol, spi_en};
            A_SPI_CLKDIV:  prdata = {16'd0, spi_clk_div};
            A_SPI_STATUS:  prdata = spi_status;
            A_SPI_RX:      prdata = spi_status[2] ? 32'd0 : spi_rx_data;
            A_I2C_CTRL:    prdata = {30'd0, i2c_auto, i2c_en};
            A_I2C_PRESC:   prdata = {16'd0, i2c_prescale};
            A_I2C_ADDR:    prdata = {16'd0, i2c_ten, 5'd0, i2c_addr};
            A_I2C_STATUS:  prdata = i2c_status;
            A_I2C_RX:      prdata = i2c_status[2] ? 32'h8000_0000
                                                  : {24'd0, i2c_rx_data};
            default:       prdata = 32'd0;
        endcase
    end
endmodule

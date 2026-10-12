/* =============================================================================
 * se_demos.c - serial-engine demos: protocols as programs (slides 12-14, 24)
 * -----------------------------------------------------------------------------
 * Every demo loads one or two programs from se_programs.h (made by
 * tools/build_programs.sh from the programs/ folder), sets up the state machine
 * with the values the assembler put in the .se file, routes the SM to some
 * pins with the crossbar, and then feeds the TX FIFO / reads the RX FIFO.
 * The word formats are described at the top of each .se file in programs/.
 *
 *   se uart    uart_tx on SM0 -> uart_rx on SM1 (internal loop, SE_LOOP),
 *              TX also visible on JD2
 *   se spi     spi_master on SM0 drives the PmodSF3 on JB: flash READ ID
 *   se i2c     i2c_master on SM0 drives the PmodTMP2 on JC: temperature
 *   se man     the "new protocol": Manchester TX on SM0 -> RX on SM1
 *              (internal loop), with the CRC-16 instruction; TX on JD2
 *   se oled T  oled_spi on SM1 drives the on-board OLED: shows text T and
 *              the temperature
 *   switch     slide 14: JD switches from the SPI engine to the uart_tx
 *              program; prints the switch time (measurement 3)
 * ===========================================================================*/
#include <string.h>
#include "xil_printf.h"
#include "ssc2_driver.h"
#include "se_programs.h"
#include "oled_font.h"
#include "se_demos.h"

#define WAIT_US 20000u                 /* time-out for one FIFO word */

/* Load a program and set up its state machine with the assembler's values. */
#define SE_SETUP(sm, NAME, prog)                                              \
    do {                                                                      \
        se_load(prog, SE_PROG_##NAME##_LEN, SE_PROG_##NAME##_ORIGIN);         \
        se_sm_config(sm, SE_PROG_##NAME##_CLKDIV, SE_PROG_##NAME##_PINCTRL,   \
                     SE_PROG_##NAME##_SHIFTCTRL);                             \
    } while (0)

/* the CPU uses the SM FIFOs itself, so the serial-engine NI must be off */
static void se_cpu_mode(void)
{
    se_stop(3u);
    ni_config(NODE_SE, 0u);
    ssc_wr(SSC2_SE_LOOP, 0u);
}

static void se_help(void)
{
    xil_printf("serial-engine demos:\r\n"
               "  se uart      uart_tx (SM0) -> uart_rx (SM1), internal loop; TX on JD2\r\n"
               "  se spi       spi_master (SM0) on JB: PmodSF3 READ ID\r\n"
               "  se i2c       i2c_master (SM0) on JC: PmodTMP2 temperature\r\n"
               "  se man       Manchester TX (SM0) -> RX (SM1) + CRC-16; TX on JD2\r\n"
               "  se oled TEXT oled_spi (SM1) on the OLED: TEXT + temperature\r\n"
               "  se oled off  switch the OLED supplies off\r\n"
               "  switch       slide 14: JD from the SPI engine to SM0, switch time\r\n");
}

/* ------------------------------------------------------------------------- */
/* UART: uart_tx on SM0, uart_rx on SM1, SM0 pin 1 -> SM1 pin 2 inside       */
/* ------------------------------------------------------------------------- */
static void demo_uart(void)
{
    static const char msg[] = "HELLO FROM THE SERIAL ENGINE";
    uint32_t i, w, errs = 0u, n = (uint32_t)strlen(msg);

    se_cpu_mode();
    SE_SETUP(0u, UART_TX, se_prog_uart_tx);
    SE_SETUP(1u, UART_RX, se_prog_uart_rx);
    ssc_wr(SSC2_SE_LOOP, 1u);                      /* SM1 RXD <- SM0 TXD */
    se_start(3u);
    (void)xbar_select(PORT_JD, SRC_SM0, 0, 10000u); /* see TXD on JD2 */

    xil_printf("sending \"%s\" with uart_tx, receiving with uart_rx:\r\n  ", msg);
    for (i = 0u; i < n; i++) {
        se_put(0u, (uint8_t)msg[i], WAIT_US);       /* TX word: byte in [7:0] */
        if (se_get(1u, &w, WAIT_US) != SSC_OK) { errs += n - i; break; }
        xil_printf("%c", (char)(w >> 24));          /* RX word: byte in [31:24] */
        if ((w >> 24) != (uint8_t)msg[i]) errs++;
    }
    xil_printf("\r\n%d bytes, %d errors  %s  (115200 baud, 8 ticks per bit)\r\n",
               (int)n, (int)errs, errs ? "FAIL" : "PASS");
}

/* ------------------------------------------------------------------------- */
/* SPI: spi_master on SM0, routed to JB instead of the fixed SPI engine        */
/* ------------------------------------------------------------------------- */
static void demo_spi(void)
{
    static const uint32_t tx[5] = { 4u << 24, 0x9Fu << 24, 0u, 0u, 0u };  /* N = 4, 9F 00 00 00 */
    uint32_t rx[4], i;

    ni_config(NODE_SPI, 0u);
    se_cpu_mode();
    SE_SETUP(0u, SPI_MASTER, se_prog_spi_master);
    se_start(1u);
    if (xbar_select(PORT_JB, SRC_SM0, 0, 100000u) != SSC_OK) {
        xil_printf("JB did not switch (SPI engine busy?)\r\n");
        return;
    }
    for (i = 0u; i < 5u; i++) se_put(0u, tx[i], WAIT_US);
    for (i = 0u; i < 4u; i++) {
        if (se_get(0u, &rx[i], WAIT_US) != SSC_OK) {
            xil_printf("no answer from spi_master\r\n");
            break;
        }
    }
    if (i == 4u)
        xil_printf("spi_master on SM0: READ ID -> %02x %02x %02x %s\r\n",
                   (int)(rx[1] & 0xFFu), (int)(rx[2] & 0xFFu), (int)(rx[3] & 0xFFu),
                   ((rx[1] & 0xFFu) == 0x20u) ? "(Micron - same as the SPI engine)" : "");
    (void)xbar_select(PORT_JB, SRC_SPI, 0, 100000u);  /* give JB back */
    se_stop(1u);
}

/* ------------------------------------------------------------------------- */
/* I2C: i2c_master on SM0, routed to JC instead of the fixed I2C engine        */
/* ------------------------------------------------------------------------- */
#define I2C_W(start, stop, byte, ninth) \
    (((uint32_t)(start) << 31) | ((uint32_t)(stop) << 30) | ((uint32_t)(byte) << 22) | \
     ((uint32_t)(ninth) << 21))

static void demo_i2c(void)
{
    const uint32_t tx[5] = {
        I2C_W(1, 0, 0x96, 1),        /* START, 0x4B + write          */
        I2C_W(0, 0, 0x00, 1),        /* register pointer 0           */
        I2C_W(1, 0, 0x97, 1),        /* repeated START, 0x4B + read  */
        I2C_W(0, 0, 0xFF, 0),        /* read byte 1, ACK             */
        I2C_W(0, 1, 0xFF, 1)         /* read byte 2, NACK, STOP      */
    };
    uint32_t rx[5], i;
    uint8_t b0, b1;

    ni_config(NODE_I2C, 0u);
    hub_stop();
    se_cpu_mode();
    SE_SETUP(0u, I2C_MASTER, se_prog_i2c_master);   /* 32 words: loaded alone */
    se_start(1u);
    if (xbar_select(PORT_JC, SRC_SM0, 0, 100000u) != SSC_OK) {
        xil_printf("JC did not switch (I2C engine busy?)\r\n");
        return;
    }
    for (i = 0u; i < 5u; i++) se_put(0u, tx[i], WAIT_US);
    for (i = 0u; i < 5u; i++) {
        if (se_get(0u, &rx[i], WAIT_US) != SSC_OK) break;
        if (rx[i] & 1u && i < 4u) break;            /* NACK: the SM sent STOP */
    }
    if (i == 0u || (rx[0] & 1u)) {
        xil_printf("no ACK from 0x4B - is the PmodTMP2 on JC (pins 3-6 / 9-12)?\r\n");
    } else if (i < 5u) {
        xil_printf("I2C transaction stopped after %d bytes\r\n", (int)i);
    } else {
        b0 = (uint8_t)(rx[3] >> 1);                 /* RX word: [8:1] byte, [0] ACK bit */
        b1 = (uint8_t)(rx[4] >> 1);
        xil_printf("i2c_master on SM0: TMP2 -> 0x%02x 0x%02x = ", b0, b1);
        print_temp(tmp2_convert(b0, b1));
        xil_printf("  (same bytes as the I2C engine)\r\n");
    }
    (void)xbar_select(PORT_JC, SRC_I2C, 0, 100000u);  /* give JC back */
    se_stop(1u);
}

/* ------------------------------------------------------------------------- */
/* Manchester (the new protocol) with the CRC instruction                     */
/* ------------------------------------------------------------------------- */
/* CRC-16 the way the serial engine computes it: bits in wire order (each
 * byte LSB first), POLY 0x1021, INIT 0xFFFF (SE_CRC_CFG reset values) */
static uint16_t crc16_wire(const uint8_t *d, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i, b, bit;
    for (i = 0u; i < n; i++) {
        for (b = 0u; b < 8u; b++) {
            bit = (d[i] >> b) & 1u;
            crc = (uint16_t)((crc << 1) ^ ((((crc >> 15) ^ bit) & 1u) ? 0x1021u : 0u));
        }
    }
    return crc;
}

static void demo_manchester(void)
{
    uint8_t tx[32], rx[32];
    uint32_t i, w, errs = 0u, n = 32u, got = 0u;
    uint16_t sw_crc, hw_crc;

    for (i = 0u; i < n; i++) tx[i] = (uint8_t)(i * 37u + 0x5Au);
    se_cpu_mode();
    SE_SETUP(0u, MANCHESTER_TX, se_prog_manchester_tx);
    SE_SETUP(1u, MANCHESTER_RX, se_prog_manchester_rx);
    ssc_wr(SSC2_SE_LOOP, 1u);                      /* SM1 pin 2 <- SM0 pin 1 */
    se_start(3u);
    se_exec(1u, 0xE080u);                          /* CRC reset on SM1 */
    (void)xbar_select(PORT_JD, SRC_SM0, 0, 10000u); /* see the line on JD2 */

    for (i = 0u; i < n; i++) {
        se_put(0u, tx[i], WAIT_US);                /* TX word: byte in [7:0] */
        if (se_get(1u, &w, WAIT_US) != SSC_OK) break;
        rx[got++] = (uint8_t)(w >> 24);            /* RX word: byte in [31:24] */
    }
    for (i = 0u; i < n; i++) if (i >= got || rx[i] != tx[i]) errs++;
    sw_crc = crc16_wire(tx, n);
    hw_crc = (uint16_t)ssc_rd(SM_CRC(1));
    xil_printf("Manchester (IEEE 802.3, 100 kbit/s): %d bytes sent, %d received, %d errors\r\n",
               (int)n, (int)got, (int)errs);
    xil_printf("CRC-16 in SM1: 0x%04x, computed by the CPU: 0x%04x  %s\r\n",
               hw_crc, sw_crc, (errs == 0u && hw_crc == sw_crc) ? "PASS" : "FAIL");
}

/* ------------------------------------------------------------------------- */
/* OLED (SSD1306, 128 x 32) through oled_spi on SM1                            */
/* ------------------------------------------------------------------------- */
#define OLED_CMD(b)   (0x20000000u | ((uint32_t)(b) << 20))   /* DC 0, RES# 1 */
#define OLED_DATA(b)  (0xA0000000u | ((uint32_t)(b) << 20))   /* DC 1, RES# 1 */
#define OLED_RESET    0x0E300000u                             /* RES# 0 */

static uint8_t g_fb[4][128];                       /* 4 pages of 8 pixel rows */

static void oled_cmd(uint8_t b) { (void)se_put(1u, OLED_CMD(b), WAIT_US); }

static void oled_text(uint32_t page, const char *s)
{
    uint32_t x = 0u, c, k;
    while (*s && x + 6u <= 128u) {
        c = (uint8_t)*s++;
        if (c >= 'a' && c <= 'z') c -= 32u;         /* the font is upper case */
        if (c < OLED_FONT_FIRST || c > OLED_FONT_LAST) c = ' ';
        for (k = 0u; k < 5u; k++) g_fb[page][x++] = oled_font[c - OLED_FONT_FIRST][k];
        g_fb[page][x++] = 0u;                       /* 1 column gap */
    }
}

static void demo_oled(const char *text)
{
    uint32_t p, x;
    int32_t t;
    uint8_t raw[2];
    char line[24];

    if (strcmp(text, "off") == 0) {
        oled_cmd(0xAE);                             /* display off */
        ssc_wr(SSC2_OLED_PWR, 3u);                  /* VBAT and VDD off */
        xil_printf("OLED off\r\n");
        return;
    }
    se_stop(2u);
    ni_config(NODE_SE, 0u);
    SE_SETUP(1u, OLED_SPI, se_prog_oled_spi);
    se_start(2u);
    (void)xbar_select(PORT_OLED, SRC_SM1, 1, 10000u);

    /* power-up sequence of the ZedBoard OLED (UG-2832HSWEG04 data sheet) */
    ssc_wr(SSC2_OLED_PWR, 2u);                      /* VDD on, VBAT still off */
    ssc_delay_us(1000u);
    oled_cmd(0xAE);                                 /* display off */
    (void)se_put(1u, OLED_RESET, WAIT_US);          /* RES# low ... */
    ssc_delay_us(1000u);
    oled_cmd(0xE3);                                 /* ... and high again (NOP) */
    oled_cmd(0x8D); oled_cmd(0x14);                 /* charge pump on */
    oled_cmd(0xD9); oled_cmd(0xF1);                 /* pre-charge period */
    ssc_wr(SSC2_OLED_PWR, 0u);                      /* VBAT on */
    ssc_delay_us(100000u);
    oled_cmd(0xA1); oled_cmd(0xC8);                 /* flip so the text is upright */
    oled_cmd(0xDA); oled_cmd(0x20);                 /* COM pins for 128 x 32 */
    oled_cmd(0x20); oled_cmd(0x00);                 /* horizontal addressing */
    oled_cmd(0x21); oled_cmd(0x00); oled_cmd(0x7F); /* columns 0..127 */
    oled_cmd(0x22); oled_cmd(0x00); oled_cmd(0x03); /* pages 0..3 */
    oled_cmd(0xAF);                                 /* display on */

    memset(g_fb, 0, sizeof g_fb);
    oled_text(0u, "SSC V2 SERIAL ENGINE");
    oled_text(1u, text[0] ? text : "OLED VIA OLED_SPI");
    ni_config(NODE_I2C, 0u);
    hub_stop();
    if (tmp2_read(&t, raw) == SSC_OK) {
        if (t < 0) t = -t;                          /* show the magnitude */
        line[0] = 'T'; line[1] = ' '; line[2] = '=';  line[3] = ' ';
        line[4] = (char)('0' + (t / 100000) % 10);  line[5] = (char)('0' + (t / 10000) % 10);
        line[6] = '.';
        line[7] = (char)('0' + (t / 1000) % 10);    line[8] = (char)('0' + (t / 100) % 10);
        line[9] = ' '; line[10] = 'C'; line[11] = '\0';
        oled_text(2u, line);
    } else {
        oled_text(2u, "NO TMP2 ON JC");
    }
    oled_text(3u, "SM1 -> OLED PORT");
    for (p = 0u; p < 4u; p++)
        for (x = 0u; x < 128u; x++) (void)se_put(1u, OLED_DATA(g_fb[p][x]), WAIT_US);
    xil_printf("OLED: 512 bytes sent by oled_spi on SM1 (SCLK 5 MHz)\r\n");
}

/* ------------------------------------------------------------------------- */
/* slide 14: JD switches from the SPI engine to the uart_tx program           */
/* ------------------------------------------------------------------------- */
void se_demo_switch(void)
{
    static const char msg[] = "\r\nHELLO FROM SM0 ON JD\r\n";
    uint32_t i, cyc, edge, ctrl;

    /* SM0 runs the slide-13 UART program, waiting for bytes */
    se_cpu_mode();
    SE_SETUP(0u, UART_TX, se_prog_uart_tx);
    se_start(1u);

    /* JD shows a copy of the SPI engine's pins */
    ni_config(NODE_SPI, 0u);
    ssc_spi_init(1000000u, 0u);
    (void)xbar_select(PORT_JD, SRC_SPI, 1, 10000u);

    /* start an SPI frame (CS# low, 16 bytes), then ask for JD -> SM0 */
    ctrl = SPI_CTRL_EN | SPI_CTRL_LEN(8) | SPI_CTRL_CS_MANUAL;
    ssc_wr(SSC_SPI_CTRL, ctrl | SPI_CTRL_CS_LEVEL | SPI_CTRL_RX_FLUSH);
    ssc_wr(SSC_SPI_TXDATA, 0x05u);                  /* flash READ STATUS ... */
    for (i = 0u; i < 15u; i++) ssc_wr(SSC_SPI_TXDATA, 0x00u);
    ssc_wr(XBAR_PIN_SEL(PORT_JD), SRC_SM0);         /* not forced: must wait */
    ssc_delay_us(300u);
    xil_printf("JD while the SPI frame runs: %s, switching=%d\r\n",
               (xbar_current(PORT_JD) == SRC_SPI) ? "still SPI" : "??",
               (int)((ssc_rd(XBAR_PIN_SEL(PORT_JD)) & XBAR_SEL_BUSY) != 0u));
    ssc_wr(SSC_SPI_CTRL, ctrl | SPI_CTRL_RX_FLUSH); /* end of the frame: CS# high */
    while (ssc_rd(XBAR_PIN_SEL(PORT_JD)) & XBAR_SEL_BUSY) { }
    cyc = xbar_switch_cycles(PORT_JD);
    xil_printf("JD now: %s.  SW_CYCLES = %d clocks (%d us, it waited for the SPI frame)\r\n",
               (xbar_current(PORT_JD) == SRC_SM0) ? "SM0" : "??", (int)cyc, (int)(cyc / 100u));

    for (i = 0u; msg[i]; i++) (void)se_put(0u, (uint8_t)msg[i], WAIT_US);
    edge = xbar_switch_edge(PORT_JD);
    xil_printf("SM0 sent \"HELLO FROM SM0 ON JD\" on JD2 (115200 8N1).  SW_EDGE = %d clocks\r\n",
               (int)edge);

    /* a switch away from an idle engine takes only a few clocks */
    ssc_delay_us(2000u);                            /* let SM0 finish its last stop bit */
    (void)xbar_select(PORT_JD, SRC_SPI, 0, 10000u);
    xil_printf("switch from the idle SM0 back to SPI: SW_CYCLES = %d clocks (%d ns)\r\n",
               (int)xbar_switch_cycles(PORT_JD), (int)(xbar_switch_cycles(PORT_JD) * 10u));
    (void)xbar_select(PORT_JD, SRC_SM0, 0, 10000u);
    ssc_spi_init(1000000u, 0u);
}

/* ------------------------------------------------------------------------- */
void se_demo(const char *arg)
{
    if      (strcmp(arg, "uart") == 0)         demo_uart();
    else if (strcmp(arg, "spi") == 0)          demo_spi();
    else if (strcmp(arg, "i2c") == 0)          demo_i2c();
    else if (strcmp(arg, "man") == 0)          demo_manchester();
    else if (strncmp(arg, "oled", 4) == 0)     demo_oled(arg[4] ? arg + 5 : "");
    else                                       se_help();
}

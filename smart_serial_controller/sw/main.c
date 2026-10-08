/* =============================================================================
 * main.c - "Way 2" demo: the ARM drives the controller over AXI -> APB
 * -----------------------------------------------------------------------------
 * Console: the ZedBoard's on-board USB-UART (PS UART1, 115200 8N1).
 * Type a command and press Enter:
 *
 *   selftest      loop-back tests inside the FPGA (no Pmods needed)
 *   temp          PmodTMP2 temperature over I2C   (the slide-13 example)
 *   id            PmodSF3 JEDEC ID over SPI
 *   flash         erase / program / read back 256 bytes of the PmodSF3
 *   send TEXT     send TEXT out of the controller's UART (PmodUSBUART, JA)
 *   rx            show what arrived on the controller's UART
 *   bridge echo   UART -> UART in hardware: the PC's keys come straight back
 *   bridge temp   I2C -> UART in hardware: sensor bytes go to the PmodUSBUART
 *   bridge off    stop the bridge and show how many bytes it moved
 *   bench         CPU time with and without the FIFO
 *   regs          dump the status registers
 * ===========================================================================*/
#include <string.h>
#include "xil_printf.h"
#include "xtime_l.h"
#include "ssc_driver.h"

extern char inbyte(void);              /* standalone BSP: read the console UART */

/* ------------------------------------------------------------------------- */
static void print_temp(int32_t t)      /* t in 1e-4 degC */
{
    const char *sign = (t < 0) ? "-" : "+";
    if (t < 0) t = -t;
    xil_printf("%s%d.%04d C", sign, (int)(t / 10000), (int)(t % 10000));
}

static const char *err_text(int r)
{
    switch (r) {
    case SSC_ENACK:    return "no ACK - is the PmodTMP2 on JC (pins 3-6 / 9-12)?";
    case SSC_EARB:     return "arbitration lost - another master on the bus";
    case SSC_ETIMEOUT: return "time-out - SCL stuck low? check pull-ups / wiring";
    default:           return "ok";
    }
}

static int read_line(char *buf, int max)
{
    int n = 0;
    char c;
    for (;;) {
        c = inbyte();
        if (c == '\r' || c == '\n') {
            xil_printf("\r\n");
            buf[n] = '\0';
            return n;
        }
        if ((c == 8 || c == 127) && n > 0) {            /* backspace */
            n--;
            xil_printf("\b \b");
        } else if (c >= 32 && c < 127 && n < max - 1) {
            buf[n++] = c;
            xil_printf("%c", c);
        }
    }
}

static void help(void)
{
    xil_printf("\r\nCommands:\r\n"
               "  selftest      loop-back tests inside the FPGA (no Pmods needed)\r\n"
               "  temp          PmodTMP2 temperature (I2C on JC)\r\n"
               "  id            PmodSF3 JEDEC ID (SPI on JB)\r\n"
               "  flash         erase / program / read back 256 bytes of the PmodSF3\r\n"
               "  send TEXT     send TEXT from the controller's UART (PmodUSBUART on JA)\r\n"
               "  rx            show bytes received by the controller's UART\r\n"
               "  bridge echo   UART -> UART pass-through in hardware\r\n"
               "  bridge temp   I2C -> UART: sensor bytes go to the PmodUSBUART\r\n"
               "  bridge off    stop the bridge, show its byte counters\r\n"
               "  bench         CPU time with and without the FIFO\r\n"
               "  regs          dump the status registers\r\n");
}

/* ------------------------------------------------------------------------- */
static void cmd_selftest(void)
{
    uint32_t i, v, d, errs;
    uint64_t end;
    int c;

    xil_printf("ID register      : 0x%08x  %s\r\n", ssc_rd(SSC_ID),
               ssc_rd(SSC_ID) == SSC_ID_VALUE ? "PASS" : "FAIL");

    ssc_wr(SSC_SCRATCH, 0xA5A55A5Au);
    v = ssc_rd(SSC_SCRATCH);
    xil_printf("scratch register : 0x%08x  %s\r\n", v, v == 0xA5A55A5Au ? "PASS" : "FAIL");

    /* UART: internal loop-back at 921600 baud, 64 bytes */
    ssc_uart_init(921600, 8, 'N', 1, 0);
    ssc_wr(SSC_UART_CTRL, ssc_rd(SSC_UART_CTRL) | UART_CTRL_LOOP);
    errs = 0;
    for (i = 0; i < 64u; i++) {
        ssc_uart_putc((uint8_t)(i * 37u + 11u));
        end = ssc_time_us() + 1000u;
        while ((c = ssc_uart_getc()) < 0 && ssc_time_us() < end) { }
        if (c != (int)((i * 37u + 11u) & 0xFFu)) errs++;
    }
    xil_printf("UART loop-back   : 64 bytes, %d errors  %s\r\n", (int)errs, errs ? "FAIL" : "PASS");
    ssc_uart_init(115200, 8, 'N', 1, 0);

    /* SPI: internal loop-back, 32-bit words, all four modes, CS3 (nothing attached) */
    ssc_wr(SSC_SPI_CLKDIV, 3u);
    errs = 0;
    for (i = 0; i < 64u; i++) {
        v = 0x9E3779B9u * (i + 1u);
        ssc_wr(SSC_SPI_CTRL, SPI_CTRL_EN | SPI_CTRL_LEN(32) | SPI_CTRL_CS_SEL(3) |
                             SPI_CTRL_LOOP | ((i & 3u) << 1));
        ssc_wr(SSC_SPI_TXDATA, v);
        while (ssc_rd(SSC_SPI_STATUS) & ST_RX_EMPTY) { }
        d = ssc_rd(SSC_SPI_RXDATA);
        if (d != v) errs++;
    }
    xil_printf("SPI loop-back    : 64 words, %d errors  %s\r\n", (int)errs, errs ? "FAIL" : "PASS");
    ssc_spi_init(1000000u, 0u);

    xil_printf("interrupts seen  : %d\r\n", (int)ssc_irq_count());
}

static void cmd_temp(void)
{
    int32_t t;
    uint8_t raw[2];
    int r = tmp2_read(&t, raw);
    if (r != SSC_OK) {
        xil_printf("I2C error: %s\r\n", err_text(r));
        return;
    }
    xil_printf("raw 0x%02x 0x%02x -> (0x%04x >> 3) x 0.0625 = ", raw[0], raw[1],
               (raw[0] << 8) | raw[1]);
    print_temp(t);
    xil_printf("\r\n");
}

static void cmd_id(void)
{
    uint8_t id[3];
    flash_read_id(id);
    xil_printf("JEDEC ID: %02x %02x %02x %s\r\n", id[0], id[1], id[2],
               (id[0] == 0x20u) ? "(Micron - PmodSF3)" :
               (id[0] == 0xFFu || id[0] == 0x00u) ? "(nothing answering - check JB)" : "");
}

static void cmd_flash(void)
{
    static uint8_t wr[256], rd[256];
    const uint32_t addr = 0x00FF0000u;              /* a 4 KB block near the top of 16 MB */
    uint32_t i, errs = 0;
    uint64_t t0;

    for (i = 0; i < 256u; i++) wr[i] = (uint8_t)(i ^ 0x5Au);
    t0 = ssc_time_us();
    if (flash_erase_4k(addr) != SSC_OK) { xil_printf("erase time-out\r\n"); return; }
    xil_printf("erase 4 KB at 0x%06x   : %d us\r\n", addr, (int)(ssc_time_us() - t0));
    t0 = ssc_time_us();
    if (flash_program(addr, wr, 256u) != SSC_OK) { xil_printf("program time-out\r\n"); return; }
    xil_printf("program 256 bytes       : %d us\r\n", (int)(ssc_time_us() - t0));
    flash_read(addr, rd, 256u);
    for (i = 0; i < 256u; i++) if (rd[i] != wr[i]) errs++;
    xil_printf("read back 256 bytes     : %d errors  %s\r\n", (int)errs, errs ? "FAIL" : "PASS");
    xil_printf("first bytes: %02x %02x %02x %02x ...\r\n", rd[0], rd[1], rd[2], rd[3]);
}

static void cmd_rx(void)
{
    int c, n = 0;
    xil_printf("received: ");
    while ((c = ssc_uart_getc()) >= 0) {
        if (c >= 32 && c < 127) xil_printf("%c", c);
        else                    xil_printf("<%02x>", c);
        n++;
    }
    xil_printf("%s\r\n", n ? "" : "(nothing)");
}

static void bridge_counts(void)
{
    uint32_t cnt = ssc_rd(SSC_BRIDGE_CNT);
    xil_printf("bridge moved %d bytes on channel 0, %d on channel 1 - with no CPU work\r\n",
               (int)(cnt & 0xFFFFu), (int)(cnt >> 16));
}

static void cmd_bridge(const char *arg)
{
    uint32_t en = ssc_rd(SSC_INT_ENABLE);
    if (strcmp(arg, "echo") == 0) {
        ssc_wr(SSC_INT_ENABLE, en & ~SSC_INT_UART_RX);    /* the bridge owns UART RX now */
        ssc_wr(SSC_BRIDGE_CNT, 0u);
        ssc_wr(SSC_BRIDGE_CTRL, SSC_BRIDGE(BR_UART, BR_UART, BR_OFF, BR_OFF));
        xil_printf("UART -> UART bridge on: type in the PmodUSBUART terminal, every key\r\n"
                   "comes straight back. 'bridge off' to stop.\r\n");
    } else if (strcmp(arg, "temp") == 0) {
        ssc_wr(SSC_BRIDGE_CNT, 0u);
        ssc_wr(SSC_BRIDGE_CTRL, SSC_BRIDGE(BR_OFF, BR_OFF, BR_I2C, BR_UART));
        /* start the I2C read by hand: the bridge, not the CPU, collects the bytes */
        ssc_wr(SSC_I2C_CTRL, I2C_CTRL_EN | I2C_CTRL_TX_FLUSH | I2C_CTRL_RX_FLUSH);
        ssc_wr(SSC_I2C_ADDR, TMP2_ADDR);
        (void)ssc_take_events(SSC_INT_I2C_DONE);
        ssc_wr(SSC_I2C_TXDATA, 0x00u);
        ssc_wr(SSC_I2C_CMD, I2C_CMD_LEN(1));
        (void)ssc_wait_events(SSC_INT_I2C_DONE, 100000u);
        ssc_wr(SSC_I2C_CMD, I2C_CMD_LEN(2) | I2C_CMD_READ | I2C_CMD_STOP);
        (void)ssc_wait_events(SSC_INT_I2C_DONE, 100000u);
        xil_printf("2 temperature bytes went I2C -> UART by hardware (view the\r\n"
                   "PmodUSBUART terminal in hex mode: 0C 80 at 25 C).\r\n");
        bridge_counts();
        ssc_wr(SSC_BRIDGE_CTRL, 0u);
    } else {
        ssc_wr(SSC_BRIDGE_CTRL, 0u);
        ssc_wr(SSC_INT_ENABLE, en | SSC_INT_UART_RX);
        bridge_counts();
    }
}

static void cmd_bench(void)
{
    XTime t0, t1;
    uint32_t i, ns_fifo, us_poll;

    ssc_uart_init(115200, 8, 'N', 1, 0);

    XTime_GetTime(&t0);                              /* 1) let the FIFO do the work */
    for (i = 0; i < 16u; i++) ssc_wr(SSC_UART_TXDATA, 'A' + i);
    XTime_GetTime(&t1);
    ns_fifo = (uint32_t)((t1 - t0) * 1000000000ull / COUNTS_PER_SECOND);
    while ((ssc_rd(SSC_UART_STATUS) & (ST_TX_EMPTY | ST_BUSY)) != ST_TX_EMPTY) { }

    XTime_GetTime(&t0);                              /* 2) one byte at a time */
    for (i = 0; i < 16u; i++) {
        ssc_wr(SSC_UART_TXDATA, 'a' + i);
        while ((ssc_rd(SSC_UART_STATUS) & (ST_TX_EMPTY | ST_BUSY)) != ST_TX_EMPTY) { }
    }
    XTime_GetTime(&t1);
    us_poll = (uint32_t)((t1 - t0) * 1000000ull / COUNTS_PER_SECOND);
    ssc_uart_puts("\r\n");

    xil_printf("16 bytes at 115200 baud (about 1389 us on the wire):\r\n");
    xil_printf("  CPU time with the 16-entry FIFO : %d ns\r\n", (int)ns_fifo);
    xil_printf("  CPU time byte by byte (no FIFO) : %d us\r\n", (int)us_poll);
    xil_printf("  -> the FIFO frees the CPU for about %d x longer\r\n",
               ns_fifo ? (int)((us_poll * 1000u) / ns_fifo) : 0);
}

static void cmd_regs(void)
{
    xil_printf("ID          0x%08x\r\n", ssc_rd(SSC_ID));
    xil_printf("INT_STATUS  0x%08x   INT_ENABLE 0x%08x   IRQs seen %d\r\n",
               ssc_rd(SSC_INT_STATUS), ssc_rd(SSC_INT_ENABLE), (int)ssc_irq_count());
    xil_printf("BRIDGE_CTRL 0x%08x   BRIDGE_CNT 0x%08x\r\n",
               ssc_rd(SSC_BRIDGE_CTRL), ssc_rd(SSC_BRIDGE_CNT));
    xil_printf("UART_CTRL   0x%08x   UART_STATUS 0x%08x\r\n",
               ssc_rd(SSC_UART_CTRL), ssc_rd(SSC_UART_STATUS));
    xil_printf("SPI_CTRL    0x%08x   SPI_STATUS  0x%08x\r\n",
               ssc_rd(SSC_SPI_CTRL), ssc_rd(SSC_SPI_STATUS));
    xil_printf("I2C_CTRL    0x%08x   I2C_STATUS  0x%08x\r\n",
               ssc_rd(SSC_I2C_CTRL), ssc_rd(SSC_I2C_STATUS));
}

/* ------------------------------------------------------------------------- */
int main(void)
{
    char line[100];

    xil_printf("\r\n\r\n=== Smart Serial Controller - Way 2 (ARM + FPGA) ===\r\n");
    if (ssc_init() != 0) {
        xil_printf("ERROR: ID register reads 0x%08x, expected 0x53534301.\r\n"
                   "Is the bitstream loaded, and is SSC_BASEADDR (0x%08x) right?\r\n",
                   ssc_rd(SSC_ID), SSC_BASEADDR);
        return -1;
    }
    xil_printf("controller found at 0x%08x\r\n", SSC_BASEADDR);
    if (ssc_irq_setup() != 0)
        xil_printf("warning: interrupt set-up failed - using polling\r\n");

    ssc_uart_init(115200u, 8u, 'N', 1u, 0);
    ssc_spi_init(1000000u, 0u);                      /* 1 MHz, mode 0 */
    ssc_i2c_init(100000u);                           /* 100 kHz */
    ssc_uart_puts("\r\nHello from the Smart Serial Controller UART (Pmod JA)\r\n");
    help();

    for (;;) {
        xil_printf("\r\nssc> ");
        read_line(line, (int)sizeof line);
        if      (strcmp(line, "selftest") == 0)      cmd_selftest();
        else if (strcmp(line, "temp") == 0)          cmd_temp();
        else if (strcmp(line, "id") == 0)            cmd_id();
        else if (strcmp(line, "flash") == 0)         cmd_flash();
        else if (strncmp(line, "send ", 5) == 0) {
            ssc_uart_puts(line + 5);
            ssc_uart_puts("\r\n");
            xil_printf("sent %d characters\r\n", (int)strlen(line + 5));
        }
        else if (strcmp(line, "rx") == 0)            cmd_rx();
        else if (strncmp(line, "bridge ", 7) == 0)   cmd_bridge(line + 7);
        else if (strcmp(line, "bench") == 0)         cmd_bench();
        else if (strcmp(line, "regs") == 0)          cmd_regs();
        else if (line[0] != '\0')                    help();
    }
    return 0;
}

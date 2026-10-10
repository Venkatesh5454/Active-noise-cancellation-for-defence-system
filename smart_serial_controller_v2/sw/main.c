/* =============================================================================
 * main.c - "Way 2" demo for the Smart Serial Controller v2 (ARM + FPGA)
 * -----------------------------------------------------------------------------
 * Console: the ZedBoard's on-board USB-UART (PS UART1, 115200 8N1).
 * Type a command and press Enter (help lists them):
 *
 *   info        ID, version, crossbar, network and hub/DMA counters
 *   id          PmodSF3 flash ID, the CPU drives the SPI engine (v1 way)
 *   temp        PmodTMP2 temperature, the CPU drives the I2C engine (v1 way),
 *               with the bus accesses and interrupts it cost (slide 20, left)
 *   flowa       slide 17: the PC reads the flash ID through the network
 *   hub [n]     slides 18-20: the hub reads the TMP2 every 100 ms, the DMA
 *               writer stores records in DDR, the CPU sleeps until a batch of
 *               32 is ready; prints the records and the CPU load (n batches)
 *   switch      slide 14: JD switches from the SPI engine to the slide-13
 *               UART program on SM0; prints the switch time
 *   se ...      serial-engine programs (se help)
 *   regs        dump the main status registers
 * ===========================================================================*/
#include <string.h>
#include <stdlib.h>
#include "xil_printf.h"
#include "ssc2_driver.h"
#include "se_demos.h"

extern char inbyte(void);              /* standalone BSP: read the console UART */

/* DMA ring buffer in DDR: 64 records of 16 bytes, cache-line aligned */
#define RING_LOG2 6u
static ssc2_record_t g_ring_buf[1u << RING_LOG2] __attribute__((aligned(32)));

/* ------------------------------------------------------------------------- */
void print_temp(int32_t t)             /* t in 1e-4 degC */
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
               "  info        ID, version, crossbar, network, hub and DMA counters\r\n"
               "  id          flash ID, CPU drives the SPI engine (v1 way)\r\n"
               "  temp        TMP2 temperature, CPU drives the I2C engine (v1 way) + CPU load\r\n"
               "  flowa       slide 17: PC reads the flash ID through the NoC\r\n"
               "  hub [n]     slides 18-20: hub + DMA, CPU sleeps, n batches of 32 records\r\n"
               "  switch      slide 14: JD from the SPI engine to the SM0 UART program\r\n"
               "  se help     serial-engine programs (UART, SPI, I2C, Manchester, OLED)\r\n"
               "  regs        dump the status registers\r\n");
}

static const char *src_name(uint32_t s)
{
    static const char *n[8] = { "off", "UART", "SPI", "I2C", "SM0", "SM1", "GPIO", "off" };
    return n[s & 7u];
}

/* ------------------------------------------------------------------------- */
static void cmd_info(void)
{
    static const char *port[6] = { "JA", "JB", "JC", "JD", "OLED", "LED" };
    static const char *node[6] = { "serial engine", "sensor hub", "DMA writer",
                                   "UART", "SPI", "I2C" };
    uint32_t p, n;
    xil_printf("ID 0x%08x  VERSION 0x%08x  time %d us\r\n",
               ssc_rd(SSC_ID), ssc_rd(SSC2_VERSION), (int)ssc_rd(SSC2_TIME_US));
    xil_printf("crossbar:");
    for (p = 0; p < 6u; p++) xil_printf("  %s=%s", port[p], src_name(xbar_current(p)));
    xil_printf("\r\nnetwork (packets in / out per node):\r\n");
    for (n = 0; n < 6u; n++)
        xil_printf("  node %d %s: %d in / %d out\r\n", (int)n, node[n],
                   (int)ni_packets_in(n), (int)ni_packets_out(n));
    xil_printf("hub: requests %d, ok %d, errors %d, time-outs %d, records %d, next seq %d\r\n",
               (int)ssc_rd(HUB_CNT_REQ), (int)ssc_rd(HUB_CNT_RESP_OK), (int)ssc_rd(HUB_CNT_ERR),
               (int)ssc_rd(HUB_CNT_TMO), (int)ssc_rd(HUB_CNT_REC), (int)ssc_rd(HUB_SEQ));
    xil_printf("DMA: written %d, overflow %d, AXI errors %d, pending %d, wr %d rd %d\r\n",
               (int)ssc_rd(DMA_COUNT), (int)ssc_rd(DMA_OVERFLOW), (int)ssc_rd(DMA_AXI_ERR),
               (int)ssc_rd(DMA_PENDING), (int)ssc_rd(DMA_WR_IDX), (int)ssc_rd(DMA_RD_IDX));
}

static void cmd_id(void)
{
    uint8_t id[3];
    ni_config(NODE_SPI, 0u);                          /* the CPU owns the SPI engine */
    ssc_spi_init(1000000u, 0u);
    flash_read_id(id);
    xil_printf("JEDEC ID: %02x %02x %02x %s\r\n", id[0], id[1], id[2],
               (id[0] == 0x20u) ? "(Micron - PmodSF3)" :
               (id[0] == 0xFFu || id[0] == 0x00u) ? "(nothing answering - check JB)" : "");
}

/* v1 way: the CPU runs every step of one reading.  APB_COUNT and IRQ_COUNT
 * count what it cost (slide 20, left: "about 20 accesses + 2 interrupts"). */
static void cmd_temp(void)
{
    int32_t t;
    uint8_t raw[2];
    uint32_t apb, irqs;
    int r;

    ni_config(NODE_I2C, 0u);                          /* the CPU owns the I2C engine */
    hub_stop();
    ssc_irq_enable(SSC_INT_I2C_DONE | SSC_INT_I2C_NACK | SSC_INT_I2C_ARB_LOST, 0u);
    ssc_count_clear();
    r = tmp2_read(&t, raw);
    apb  = ssc_apb_count();
    irqs = ssc_hw_irq_count();
    if (r != SSC_OK) {
        xil_printf("I2C error: %s\r\n", err_text(r));
        return;
    }
    xil_printf("raw 0x%02x 0x%02x -> (0x%04x >> 3) x 0.0625 = ", raw[0], raw[1],
               (raw[0] << 8) | raw[1]);
    print_temp(t);
    xil_printf("\r\nCPU load of this one reading: %d APB accesses, %d interrupts\r\n",
               (int)apb, (int)irqs);
}

/* Slide 17, flow A: the UART network interface turns PC frames into packets
 * for the SPI node.  The CPU only sets it up. */
static void cmd_flowa(void)
{
    ssc_uart_init(115200u, 8u, 'N', 1u, 0);
    ssc_irq_enable(SSC_INT_I2C_DONE | SSC_INT_I2C_NACK | SSC_INT_I2C_ARB_LOST, 0u);
    ssc_wr(SSC_SPI_CTRL, SPI_CTRL_EN | SPI_CTRL_LEN(8));     /* mode 0, CPU off it */
    ni_config(NODE_SPI, NI_EN);
    ni_config(NODE_I2C, NI_EN);
    ni_config(NODE_UART, NI_EN | NI_MODE(NI_MODE_FRAME) | NI_DEST(NODE_SPI) | NI_ARG(0));
    xil_printf("UART NI: FRAME mode, destination node 4 (SPI), chip select 0.\r\n"
               "On the PC (PmodUSBUART on JA) run:\r\n"
               "    python tools/pc_frame.py COMx 01 9F 03       -> 20 BA 19\r\n"
               "    python tools/pc_frame.py COMx 04 03 00 00 00 10   -> 16 flash bytes\r\n"
               "Press Enter here when done (the UART stays in network mode until 'temp',\r\n"
               "'id' or a reset).\r\n");
    (void)inbyte();
    xil_printf("UART node: %d packets out, %d in;  SPI node: %d in, %d out\r\n",
               (int)ni_packets_out(NODE_UART), (int)ni_packets_in(NODE_UART),
               (int)ni_packets_in(NODE_SPI), (int)ni_packets_out(NODE_SPI));
}

/* Slides 18-20: the hub and the DMA writer do the work, the CPU sleeps. */
static void cmd_hub(int batches)
{
    uint32_t i, n, apb, irqs;
    int b;
    const ssc2_record_t *rec;

    if (batches < 1) batches = 1;
    ssc_i2c_init(100000u);
    ni_config(NODE_I2C, NI_EN);                        /* the I2C engine belongs to the NoC */
    dma_setup(g_ring_buf, RING_LOG2, 32u, 1000u);      /* batch of 32, 1 s time-out */
    ssc_wr(HUB_REC_DEST, NODE_DMA);
    ssc_wr(HUB_TIMEOUT, 50u);
    /* task 0: every 100 ms ask node 5 (I2C) for 2 bytes from 0x4B, register 0x00 */
    hub_task(0u, TASK_EN | TASK_DEST(NODE_I2C) | TASK_TYPE(PKT_XFER_REQ) | TASK_ARG(TMP2_ADDR) |
                 TASK_WLEN(1) | TASK_RLEN(2) | TASK_RECORD,
             0x00u, 0u, TASK_TIMING(100u, 0u));
    ssc_irq_enable(0u, INT2_DMA_BATCH | INT2_DMA_OVERFLOW | INT2_DMA_AXI_ERR);
    xil_printf("hub task 0: every 100 ms read the TMP2 -> record -> DMA -> DDR at 0x%08x\r\n",
               (unsigned)(UINTPTR)g_ring_buf);
    xil_printf("the CPU now sleeps (WFI) until the batch interrupt (~3.2 s per batch)...\r\n");
    hub_start();

    for (b = 0; b < batches; b++) {
        ssc_count_clear();
        (void)ssc_wait_events2(INT2_DMA_BATCH, 0u);    /* WFI: no bus accesses */
        apb  = ssc_apb_count();
        irqs = ssc_hw_irq_count();
        n = dma_available();
        xil_printf("\r\nbatch %d: %d records ready\r\n", b + 1, (int)n);
        for (i = 0; i < n; i++) {
            rec = dma_record(i);
            if (i < 4u || i + 2u >= n) {
                xil_printf("  seq %5d  t=%10d us  src %d  len %d  data %02x %02x  = ",
                           rec->seq, (int)rec->timestamp_us, rec->source, rec->length,
                           rec->data[0], rec->data[1]);
                print_temp(tmp2_convert(rec->data[0], rec->data[1]));
                xil_printf("\r\n");
            } else if (i == 4u) {
                xil_printf("  ...\r\n");
            }
        }
        dma_consume(n);
        xil_printf("CPU load for %d readings: %d APB accesses (2 of them in the ISR),"
                   " %d interrupt(s)\r\n", (int)n, (int)apb, (int)irqs);
        if (n) xil_printf("  -> %d.%02d accesses and 1/%d interrupt per reading\r\n",
                          (int)(apb / n), (int)((apb * 100u / n) % 100u), (int)n);
    }
    hub_stop();
    xil_printf("hub stopped. DMA overflow %d, AXI errors %d, hub errors %d\r\n",
               (int)ssc_rd(DMA_OVERFLOW), (int)ssc_rd(DMA_AXI_ERR), (int)ssc_rd(HUB_CNT_ERR));
}

static void cmd_regs(void)
{
    xil_printf("ID          0x%08x   VERSION 0x%08x\r\n", ssc_rd(SSC_ID), ssc_rd(SSC2_VERSION));
    xil_printf("INT_STATUS  0x%08x   INT_ENABLE  0x%08x\r\n",
               ssc_rd(SSC_INT_STATUS), ssc_rd(SSC_INT_ENABLE));
    xil_printf("INT2_STATUS 0x%08x   INT2_ENABLE 0x%08x   IRQs seen %d\r\n",
               ssc_rd(SSC2_INT2_STATUS), ssc_rd(SSC2_INT2_ENABLE), (int)ssc_irq_count());
    xil_printf("UART_STATUS 0x%08x   SPI_STATUS  0x%08x   I2C_STATUS 0x%08x\r\n",
               ssc_rd(SSC_UART_STATUS), ssc_rd(SSC_SPI_STATUS), ssc_rd(SSC_I2C_STATUS));
    xil_printf("XBAR_STATUS 0x%08x   HUB_STATUS  0x%08x   DMA_STATUS 0x%08x\r\n",
               ssc_rd(XBAR_STATUS), ssc_rd(HUB_STATUS), ssc_rd(DMA_STATUS));
    xil_printf("SE_FSTAT    0x%08x   SM0_STATE   0x%08x   SM1_STATE  0x%08x\r\n",
               ssc_rd(SE_FSTAT), ssc_rd(SM_STATE(0)), ssc_rd(SM_STATE(1)));
    xil_printf("NI_CFG UART 0x%08x   SPI 0x%08x   I2C 0x%08x   SE 0x%08x\r\n",
               ssc_rd(NI_CFG(NODE_UART)), ssc_rd(NI_CFG(NODE_SPI)),
               ssc_rd(NI_CFG(NODE_I2C)), ssc_rd(NI_CFG(NODE_SE)));
}

/* ------------------------------------------------------------------------- */
int main(void)
{
    char line[100];

    xil_printf("\r\n\r\n=== Smart Serial Controller v2 - Way 2 (ARM + FPGA) ===\r\n");
    xil_printf("reading the controller ID at 0x%08x (a hang here = bitstream not loaded)\r\n",
               SSC_BASEADDR);
    if (ssc_init() != SSC_OK) {
        xil_printf("ERROR: ID 0x%08x / VERSION 0x%08x, expected 0x53534302 / 0x00020000.\r\n"
                   "Is the v2 bitstream loaded, and is SSC_BASEADDR (0x%08x) right?\r\n",
                   ssc_rd(SSC_ID), ssc_rd(SSC2_VERSION), SSC_BASEADDR);
        return -1;
    }
    xil_printf("controller v2 found at 0x%08x\r\n", SSC_BASEADDR);
    if (ssc_irq_setup() != 0)
        xil_printf("warning: interrupt set-up failed - using polling\r\n");

    ssc_uart_init(115200u, 8u, 'N', 1u, 0);
    ssc_spi_init(1000000u, 0u);                      /* 1 MHz, mode 0 */
    ssc_i2c_init(100000u);                           /* 100 kHz */
    ssc_uart_puts("\r\nHello from the Smart Serial Controller v2 UART (Pmod JA)\r\n");
    help();

    for (;;) {
        xil_printf("\r\nssc2> ");
        read_line(line, (int)sizeof line);
        if      (strcmp(line, "info") == 0)          cmd_info();
        else if (strcmp(line, "id") == 0)            cmd_id();
        else if (strcmp(line, "temp") == 0)          cmd_temp();
        else if (strcmp(line, "flowa") == 0)         cmd_flowa();
        else if (strncmp(line, "hub", 3) == 0)       cmd_hub(line[3] ? atoi(line + 3) : 1);
        else if (strcmp(line, "switch") == 0)        se_demo_switch();
        else if (strncmp(line, "se", 2) == 0)        se_demo(line[2] ? line + 3 : "");
        else if (strcmp(line, "regs") == 0)          cmd_regs();
        else if (line[0] != '\0')                    help();
    }
    return 0;
}

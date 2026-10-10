/* =============================================================================
 * ssc2_driver.c - bare-metal driver for the Smart Serial Controller v2
 * -----------------------------------------------------------------------------
 * Runs on the Zynq Cortex-A9 with the Xilinx "standalone" BSP.
 *   - every register access is one 32-bit AXI read/write (Xil_In32/Xil_Out32)
 *   - the controller's IRQ line is interrupt 61 in the GIC; the ISR collects
 *     the v1 (INT_STATUS) and v2 (INT2_STATUS) event flags
 *   - waiting never touches the bus: the CPU spins on a variable set by the
 *     ISR (or sleeps with WFI), and time comes from the CPU's own global
 *     timer, so APB_COUNT measures only the accesses that do real work
 * ===========================================================================*/
#include <string.h>
#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xscugic.h"
#ifdef SDT
#include "xiltimer.h"   /* Vitis 2023.2+ (SDT / Unified IDE): XTime, XTime_GetTime, COUNTS_PER_SECOND */
#else
#include "xtime_l.h"    /* Vitis Classic 2020.2 - 2023.1 */
#endif
#include "ssc2_driver.h"

/* ------------------------------------------------------------------------- */
/* register access, time                                                      */
/* ------------------------------------------------------------------------- */
uint32_t ssc_rd(uint32_t off)             { return Xil_In32(SSC_BASEADDR + off); }
void     ssc_wr(uint32_t off, uint32_t v) { Xil_Out32(SSC_BASEADDR + off, v); }

uint64_t ssc_time_us(void)
{
    XTime t;
    XTime_GetTime(&t);
    return (uint64_t)t / ((COUNTS_PER_SECOND) / 1000000u);
}

void ssc_delay_us(uint32_t us)
{
    uint64_t end = ssc_time_us() + us;
    while (ssc_time_us() < end) { }
}

/* The Cortex-A9 global timer is the XTime time base.  The Vitis 2023.2+ (SDT)
 * BSP only starts it on the first sleep()/usleep(), so start it here if it is
 * stopped.  Harmless in Vitis Classic, where the boot code already started it. */
#define GTIMER_BASE 0xF8F00200u
static void ssc_timer_start(void)
{
    if ((Xil_In32(GTIMER_BASE + 0x08u) & 1u) == 0u) {   /* control: timer enable */
        Xil_Out32(GTIMER_BASE + 0x00u, 0u);             /* counter, low word  */
        Xil_Out32(GTIMER_BASE + 0x04u, 0u);             /* counter, high word */
        Xil_Out32(GTIMER_BASE + 0x08u, 1u);             /* enable             */
    }
}

/* ------------------------------------------------------------------------- */
/* interrupts                                                                 */
/* ------------------------------------------------------------------------- */
static XScuGic           g_gic;
static volatile int      g_irq_on;
static volatile uint32_t g_int_en;          /* copies of INT_ENABLE / INT2_ENABLE */
static volatile uint32_t g_int2_en;
static volatile uint32_t g_events;          /* flags collected by the ISR */
static volatile uint32_t g_events2;
static volatile uint32_t g_irq_count;

#define RXBUF_SIZE 256u                     /* UART receive ring buffer */
static volatile uint8_t  g_rxbuf[RXBUF_SIZE];
static volatile uint32_t g_rx_head, g_rx_tail;

static void ssc_isr(void *ref)
{
    uint32_t st = 0u, st2 = 0u, d;
    (void)ref;

    if (g_int_en) {
        st = ssc_rd(SSC_INT_STATUS) & g_int_en;
        if (st & SSC_INT_UART_RX) {         /* empty the hardware RX FIFO */
            while (!((d = ssc_rd(SSC_UART_RXDATA)) & RXDATA_EMPTY)) {
                uint32_t next = (g_rx_head + 1u) % RXBUF_SIZE;
                if (next != g_rx_tail) {
                    g_rxbuf[g_rx_head] = (uint8_t)d;
                    g_rx_head = next;
                }
            }
        }
        if (st) ssc_wr(SSC_INT_STATUS, st); /* write 1 to clear */
    }
    if (g_int2_en) {
        st2 = ssc_rd(SSC2_INT2_STATUS) & g_int2_en;
        if (st2) ssc_wr(SSC2_INT2_STATUS, st2);
    }
    g_events  |= st;
    g_events2 |= st2;
    g_irq_count++;
}

int ssc_irq_setup(void)
{
    XScuGic_Config *cfg;
#if defined(SDT) || !defined(XPAR_SCUGIC_SINGLE_DEVICE_ID)
    cfg = XScuGic_LookupConfig(XPAR_XSCUGIC_0_BASEADDR);   /* Vitis 2023.2+ (SDT flow) */
#else                                                      /* Vitis 2023.1 and older   */
    cfg = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
#endif
    if (cfg == NULL) return -1;
    if (XScuGic_CfgInitialize(&g_gic, cfg, cfg->CpuBaseAddress) != XST_SUCCESS) return -1;

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler, &g_gic);
    XScuGic_SetPriorityTriggerType(&g_gic, SSC_IRQ_ID, 0xA0, 0x1);   /* level, active high */
    if (XScuGic_Connect(&g_gic, SSC_IRQ_ID, (Xil_InterruptHandler)ssc_isr, NULL) != XST_SUCCESS)
        return -1;
    XScuGic_Enable(&g_gic, SSC_IRQ_ID);
    Xil_ExceptionEnable();
    g_irq_on = 1;

    /* default: "done" and error events of the v1 engines, nothing from v2 */
    ssc_irq_enable(SSC_INT_UART_RX | SSC_INT_UART_PARITY | SSC_INT_UART_FRAME |
                   SSC_INT_UART_BREAK | SSC_INT_UART_RX_OVF | SSC_INT_SPI_DONE |
                   SSC_INT_SPI_RX_OVF | SSC_INT_I2C_DONE | SSC_INT_I2C_NACK |
                   SSC_INT_I2C_ARB_LOST, 0u);
    return 0;
}

/* Choose which events raise the interrupt.  INT2_SE_RX0/1 are level flags:
 * only enable them if the code that waits also empties the SM's RX FIFO. */
void ssc_irq_enable(uint32_t v1_mask, uint32_t v2_mask)
{
    ssc_wr(SSC_INT_STATUS, 0xFFFFu);
    ssc_wr(SSC2_INT2_STATUS, 0xFFu);
    g_int_en  = v1_mask & 0xFFFFu;
    g_int2_en = v2_mask & 0xFFu;
    ssc_wr(SSC_INT_ENABLE, g_int_en);
    ssc_wr(SSC2_INT2_ENABLE, g_int2_en);
}

uint32_t ssc_irq_count(void) { return g_irq_count; }

/* Return the events in mask that have happened since the last call, and
 * forget them.  Events handled by the interrupt come from the ISR's copy
 * (no bus access); events without an interrupt are polled. */
static uint32_t take(volatile uint32_t *store, uint32_t enabled, uint32_t status_reg,
                     uint32_t mask)
{
    uint32_t ev = 0u, poll_mask, polled;
    if (g_irq_on) {
        Xil_ExceptionDisable();
        ev = *store & mask;
        *store &= ~ev;
        Xil_ExceptionEnable();
        poll_mask = mask & ~enabled;
    } else {
        poll_mask = mask;
    }
    if (poll_mask) {
        polled = ssc_rd(status_reg) & poll_mask;
        if (polled) {
            ssc_wr(status_reg, polled);
            ev |= polled;
        }
    }
    return ev;
}

uint32_t ssc_take_events(uint32_t mask)
{
    return take(&g_events, g_irq_on ? g_int_en : 0u, SSC_INT_STATUS, mask);
}

uint32_t ssc_take_events2(uint32_t mask)
{
    return take(&g_events2, g_irq_on ? g_int2_en : 0u, SSC2_INT2_STATUS, mask);
}

uint32_t ssc_wait_events(uint32_t mask, uint32_t timeout_us)
{
    uint64_t end = ssc_time_us() + timeout_us;
    uint32_t ev;
    while ((ev = ssc_take_events(mask)) == 0u) {
        if (ssc_time_us() > end) return 0u;
    }
    return ev;
}

/* Wait for v2 events.  timeout_us = 0 means "sleep until it happens": the CPU
 * executes WFI and only wakes up for an interrupt (slide 19, step 8). */
uint32_t ssc_wait_events2(uint32_t mask, uint32_t timeout_us)
{
    uint64_t end = ssc_time_us() + timeout_us;
    uint32_t ev;
    for (;;) {
        if (timeout_us == 0u && g_irq_on && (mask & ~g_int2_en) == 0u) {
            Xil_ExceptionDisable();               /* no race between test and WFI */
            if ((g_events2 & mask) == 0u)
                __asm__ volatile ("wfi");          /* wakes on the pending IRQ */
            Xil_ExceptionEnable();                 /* ...which the ISR then handles */
        }
        ev = ssc_take_events2(mask);
        if (ev) return ev;
        if (timeout_us != 0u && ssc_time_us() > end) return 0u;
    }
}

void     ssc_count_clear(void)  { ssc_wr(SSC2_APB_COUNT, 0u); ssc_wr(SSC2_IRQ_COUNT, 0u); }
uint32_t ssc_apb_count(void)    { return ssc_rd(SSC2_APB_COUNT); }
uint32_t ssc_hw_irq_count(void) { return ssc_rd(SSC2_IRQ_COUNT); }

int ssc_init(void)
{
    ssc_timer_start();
    if (ssc_rd(SSC_ID) != SSC_ID_VALUE) return SSC_EID;        /* wrong design or address */
    if (ssc_rd(SSC2_VERSION) != SSC2_VERSION_VALUE) return SSC_EID;
    ssc_wr(SSC_INT_ENABLE, 0u);
    ssc_wr(SSC_INT_STATUS, 0xFFFFu);
    ssc_wr(SSC2_INT2_ENABLE, 0u);
    ssc_wr(SSC2_INT2_STATUS, 0xFFu);
    g_int_en = g_int2_en = 0u;
    return SSC_OK;
}

/* ------------------------------------------------------------------------- */
/* v1 engines: UART                                                           */
/* ------------------------------------------------------------------------- */
void ssc_uart_init(uint32_t baud, uint32_t bits, char parity, uint32_t stop_bits, int flow)
{
    /* divisor = f_clk / (16 x baud), kept as a 16.4 fixed-point number */
    uint32_t div16 = (SSC_CLK_HZ + baud / 2u) / baud;
    uint32_t c;

    ssc_wr(SSC_UART_BAUD, ((div16 & 0xFu) << 16) | ((div16 >> 4) & 0xFFFFu));
    c = UART_CTRL_TX_EN | UART_CTRL_RX_EN | UART_CTRL_BITS(bits) |
        UART_CTRL_TX_FLUSH | UART_CTRL_RX_FLUSH;
    if (parity == 'E' || parity == 'e') c |= UART_CTRL_PAR_EN;
    if (parity == 'O' || parity == 'o') c |= UART_CTRL_PAR_EN | UART_CTRL_PAR_ODD;
    if (stop_bits == 2u) c |= UART_CTRL_STOP2;
    if (flow)            c |= UART_CTRL_FLOW;
    ssc_wr(SSC_UART_CTRL, c);
    g_rx_head = g_rx_tail = 0u;
}

void ssc_uart_putc(uint8_t c)
{
    while (ssc_rd(SSC_UART_STATUS) & ST_TX_FULL) { }
    ssc_wr(SSC_UART_TXDATA, c);
}

void ssc_uart_puts(const char *s)
{
    while (*s) ssc_uart_putc((uint8_t)*s++);
}

int ssc_uart_getc(void)
{
    uint32_t d;
    if (g_irq_on && (g_int_en & SSC_INT_UART_RX)) {
        if (g_rx_tail == g_rx_head) return -1;           /* ring buffer empty */
        d = g_rxbuf[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1u) % RXBUF_SIZE;
        return (int)d;
    }
    d = ssc_rd(SSC_UART_RXDATA);
    return (d & RXDATA_EMPTY) ? -1 : (int)(d & 0xFFu);
}

/* ------------------------------------------------------------------------- */
/* v1 engines: SPI + PmodSF3 flash                                            */
/* ------------------------------------------------------------------------- */
static uint32_t g_spi_ctrl;

void ssc_spi_init(uint32_t sclk_hz, uint32_t mode)
{
    uint32_t div = SSC_CLK_HZ / (2u * sclk_hz);
    if (div > 0u) div--;
    if (div < 3u) div = 3u;                       /* 12.5 MHz maximum */
    ssc_wr(SSC_SPI_CLKDIV, div);

    g_spi_ctrl = SPI_CTRL_EN | SPI_CTRL_LEN(8) | SPI_CTRL_CS_MANUAL | SPI_CTRL_CS_SEL(0);
    if (mode & 2u) g_spi_ctrl |= SPI_CTRL_CPOL;
    if (mode & 1u) g_spi_ctrl |= SPI_CTRL_CPHA;
    ssc_wr(SSC_SPI_CTRL, g_spi_ctrl | SPI_CTRL_TX_FLUSH | SPI_CTRL_RX_FLUSH);
}

/* One SPI transaction: CS0 goes low, n bytes go out (tx may be NULL -> 0xFF),
 * n bytes come back (rx may be NULL), CS0 goes high. */
void ssc_spi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t n)
{
    uint32_t done = 0u, chunk, i, d;

    ssc_wr(SSC_SPI_CTRL, g_spi_ctrl | SPI_CTRL_TX_FLUSH | SPI_CTRL_RX_FLUSH);
    ssc_wr(SSC_SPI_CTRL, g_spi_ctrl | SPI_CTRL_CS_LEVEL);          /* CS0 low */
    while (done < n) {
        chunk = n - done;
        if (chunk > 16u) chunk = 16u;
        for (i = 0u; i < chunk; i++)
            ssc_wr(SSC_SPI_TXDATA, tx ? tx[done + i] : 0xFFu);
        while (ST_RX_COUNT(ssc_rd(SSC_SPI_STATUS)) < chunk) { }
        for (i = 0u; i < chunk; i++) {
            d = ssc_rd(SSC_SPI_RXDATA);
            if (rx) rx[done + i] = (uint8_t)d;
        }
        done += chunk;
    }
    while (ssc_rd(SSC_SPI_STATUS) & ST_BUSY) { }
    ssc_wr(SSC_SPI_CTRL, g_spi_ctrl);                               /* CS0 high */
}

static uint8_t g_fbuf[4u + 256u];

void flash_read_id(uint8_t id[3])
{
    uint8_t tx[4] = { 0x9Fu, 0u, 0u, 0u }, rx[4];
    ssc_spi_xfer(tx, rx, 4u);
    id[0] = rx[1]; id[1] = rx[2]; id[2] = rx[3];
}

static uint8_t flash_status(void)
{
    uint8_t tx[2] = { 0x05u, 0u }, rx[2];
    ssc_spi_xfer(tx, rx, 2u);
    return rx[1];
}

static void flash_write_enable(void)
{
    uint8_t c = 0x06u;
    ssc_spi_xfer(&c, NULL, 1u);
}

static int flash_wait_ready(uint32_t timeout_ms)
{
    uint64_t end = ssc_time_us() + (uint64_t)timeout_ms * 1000u;
    while (flash_status() & 0x01u) {                /* WIP bit */
        if (ssc_time_us() > end) return SSC_ETIMEOUT;
    }
    return SSC_OK;
}

static void flash_cmd_addr(uint8_t cmd, uint32_t addr)
{
    g_fbuf[0] = cmd;
    g_fbuf[1] = (uint8_t)(addr >> 16);
    g_fbuf[2] = (uint8_t)(addr >> 8);
    g_fbuf[3] = (uint8_t)addr;
}

int flash_erase_4k(uint32_t addr)
{
    flash_write_enable();
    flash_cmd_addr(0x20u, addr);
    ssc_spi_xfer(g_fbuf, NULL, 4u);
    return flash_wait_ready(1000u);
}

int flash_program(uint32_t addr, const uint8_t *data, uint32_t n)
{
    if (n > 256u) n = 256u;
    flash_write_enable();
    flash_cmd_addr(0x02u, addr);
    memcpy(&g_fbuf[4], data, n);
    ssc_spi_xfer(g_fbuf, NULL, 4u + n);
    return flash_wait_ready(100u);
}

void flash_read(uint32_t addr, uint8_t *data, uint32_t n)
{
    if (n > 256u) n = 256u;
    flash_cmd_addr(0x03u, addr);
    memset(&g_fbuf[4], 0, n);
    ssc_spi_xfer(g_fbuf, g_fbuf, 4u + n);
    memcpy(data, &g_fbuf[4], n);
}

/* ------------------------------------------------------------------------- */
/* v1 engines: I2C + PmodTMP2                                                 */
/* ------------------------------------------------------------------------- */
void ssc_i2c_init(uint32_t scl_hz)
{
    uint32_t p = SSC_CLK_HZ / (4u * scl_hz);
    if (p > 0u) p--;
    ssc_wr(SSC_I2C_PRESCALE, p);
    ssc_wr(SSC_I2C_CTRL, I2C_CTRL_EN | I2C_CTRL_TX_FLUSH | I2C_CTRL_RX_FLUSH);
}

static int i2c_finish(void)
{
    uint32_t st;
    if (!ssc_wait_events(SSC_INT_I2C_DONE, 200000u)) {
        ssc_wr(SSC_I2C_CTRL, I2C_CTRL_EN | I2C_CTRL_ABORT);     /* free the bus */
        return SSC_ETIMEOUT;
    }
    st = ssc_rd(SSC_I2C_STATUS);
    if (st & I2C_ST_ARB_LOST) return SSC_EARB;
    if (st & I2C_ST_NACK)     return SSC_ENACK;
    return SSC_OK;
}

/* Write wlen bytes, then (if rlen > 0) read rlen bytes with a repeated START. */
int ssc_i2c_xfer(uint16_t addr, int ten_bit,
                 const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen)
{
    uint32_t i, d;
    uint64_t end;
    int r;

    ssc_wr(SSC_I2C_CTRL, I2C_CTRL_EN | I2C_CTRL_TX_FLUSH | I2C_CTRL_RX_FLUSH);
    ssc_wr(SSC_I2C_ADDR, (addr & 0x3FFu) | (ten_bit ? I2C_ADDR_TEN_BIT : 0u));

    if (wlen > 0u) {
        (void)ssc_take_events(SSC_INT_I2C_DONE);
        for (i = 0u; i < wlen && i < 16u; i++) ssc_wr(SSC_I2C_TXDATA, wbuf[i]);
        ssc_wr(SSC_I2C_CMD, I2C_CMD_LEN(wlen) | (rlen ? 0u : I2C_CMD_STOP));
        while (i < wlen) {                                   /* longer writes */
            uint32_t st = ssc_rd(SSC_I2C_STATUS);
            if (!(st & ST_BUSY)) break;                      /* stopped early: NACK */
            if (!(st & ST_TX_FULL)) ssc_wr(SSC_I2C_TXDATA, wbuf[i++]);
        }
        r = i2c_finish();
        if (r != SSC_OK) return r;
    }

    if (rlen > 0u) {
        (void)ssc_take_events(SSC_INT_I2C_DONE);
        ssc_wr(SSC_I2C_CMD, I2C_CMD_LEN(rlen) | I2C_CMD_READ | I2C_CMD_STOP);
        if (rlen <= 16u) {
            /* short read: the bytes wait in the 16-entry FIFO until DONE */
            r = i2c_finish();
            for (i = 0u; i < rlen; i++) {
                d = ssc_rd(SSC_I2C_RXDATA);
                if (d & RXDATA_EMPTY) break;
                rbuf[i] = (uint8_t)d;
            }
            if (r != SSC_OK) return r;
            return (i < rlen) ? SSC_ETIMEOUT : SSC_OK;
        }
        end = ssc_time_us() + 200000u;
        for (i = 0u; i < rlen; ) {                           /* long read: empty the FIFO */
            d = ssc_rd(SSC_I2C_RXDATA);
            if (!(d & RXDATA_EMPTY))  rbuf[i++] = (uint8_t)d;
            else if (!(ssc_rd(SSC_I2C_STATUS) & ST_BUSY)) break;   /* NACK / ARB */
            else if (ssc_time_us() > end) break;
        }
        while (i < rlen && !((d = ssc_rd(SSC_I2C_RXDATA)) & RXDATA_EMPTY))
            rbuf[i++] = (uint8_t)d;
        r = i2c_finish();
        if (r != SSC_OK) return r;
        if (i < rlen) return SSC_ETIMEOUT;
    }
    return SSC_OK;
}

/* PmodTMP2 (ADT7420): 13-bit temperature in 0.0625 C steps */
int32_t tmp2_convert(uint8_t msb, uint8_t lsb)
{
    int16_t v = (int16_t)((((uint16_t)msb << 8) | lsb) & 0xFFF8u);   /* drop 3 flag bits */
    return (int32_t)(v / 8) * 625;               /* (raw >> 3) x 0.0625 C, in 1e-4 C */
}

int tmp2_read(int32_t *temp_e4, uint8_t raw[2])
{
    uint8_t ptr = 0x00u, b[2];
    int r = ssc_i2c_xfer(TMP2_ADDR, 0, &ptr, 1u, b, 2u);
    if (r != SSC_OK) return r;
    raw[0] = b[0];
    raw[1] = b[1];
    *temp_e4 = tmp2_convert(b[0], b[1]);
    return SSC_OK;
}

/* ------------------------------------------------------------------------- */
/* v2: pin crossbar                                                           */
/* ------------------------------------------------------------------------- */
/* Ask for a new source on a port and wait until the hand-over has happened.
 * Without force the crossbar waits for the old engine to go idle. */
int xbar_select(uint32_t port, uint32_t src, int force, uint32_t timeout_us)
{
    uint64_t end = ssc_time_us() + timeout_us;
    uint32_t v;
    ssc_wr(XBAR_PIN_SEL(port), (src & 7u) | (force ? XBAR_FORCE : 0u));
    for (;;) {
        v = ssc_rd(XBAR_PIN_SEL(port));
        if (!(v & XBAR_SEL_BUSY) && XBAR_SEL_CUR(v) == (src & 7u)) return SSC_OK;
        if (ssc_time_us() > end) return SSC_ETIMEOUT;
    }
}

uint32_t xbar_current(uint32_t port)       { return XBAR_SEL_CUR(ssc_rd(XBAR_PIN_SEL(port))); }
uint32_t xbar_switch_cycles(uint32_t port) { return ssc_rd(XBAR_SW_CYCLES(port)); }
uint32_t xbar_switch_edge(uint32_t port)   { return ssc_rd(XBAR_SW_EDGE(port)); }

void xbar_gpio(uint32_t port, uint32_t out4, uint32_t oe4)
{
    uint32_t sh = 4u * port, m = 0xFu << sh;
    ssc_wr(XBAR_GPIO_OUT, (ssc_rd(XBAR_GPIO_OUT) & ~m) | ((out4 & 0xFu) << sh));
    ssc_wr(XBAR_GPIO_OE,  (ssc_rd(XBAR_GPIO_OE)  & ~m) | ((oe4  & 0xFu) << sh));
}

/* ------------------------------------------------------------------------- */
/* v2: network interfaces                                                     */
/* ------------------------------------------------------------------------- */
void     ni_config(uint32_t node, uint32_t cfg) { ssc_wr(NI_CFG(node), cfg); }
uint32_t ni_packets_in(uint32_t node)           { return ssc_rd(NI_STAT(node)) & 0xFFFFu; }
uint32_t ni_packets_out(uint32_t node)          { return ssc_rd(NI_STAT(node)) >> 16; }

/* ------------------------------------------------------------------------- */
/* v2: sensor hub                                                             */
/* ------------------------------------------------------------------------- */
/* Write one task.  Word 0 goes first (with EN), the timing word last, because
 * writing the timing word (re)starts the task's countdown. */
void hub_task(uint32_t t, uint32_t w0, uint32_t bytes0_3, uint32_t bytes4_7, uint32_t timing)
{
    ssc_wr(HUB_TASK(t, 0), w0);
    ssc_wr(HUB_TASK(t, 1), bytes0_3);
    ssc_wr(HUB_TASK(t, 2), bytes4_7);
    ssc_wr(HUB_TASK(t, 3), timing);
}

void hub_start(void)                  { ssc_wr(HUB_CTRL, HUB_CTRL_EN); }
void hub_stop(void)                   { ssc_wr(HUB_CTRL, 0u); }
void hub_run_now(uint32_t task_mask)  { ssc_wr(HUB_CTRL, HUB_CTRL_EN | ((task_mask & 0xFFu) << 8)); }

/* ------------------------------------------------------------------------- */
/* v2: DMA writer                                                             */
/* ------------------------------------------------------------------------- */
/* The HP0 port writes DDR behind the CPU's data cache, so every record is
 * invalidated in the cache before the CPU reads it (slide 18).  The ring is
 * only ever written by the hardware, so invalidating never loses CPU data. */
static ssc2_record_t *g_ring;
static uint32_t       g_ring_mask;
static uint32_t       g_rd;

void dma_setup(void *ring, uint32_t size_log2, uint32_t batch, uint32_t timeout_ms)
{
    uint32_t size = 1u << size_log2;
    g_ring = (ssc2_record_t *)ring;
    g_ring_mask = size - 1u;
    g_rd = 0u;
    memset(ring, 0, size * sizeof(ssc2_record_t));
    Xil_DCacheFlushRange((INTPTR)ring, size * sizeof(ssc2_record_t));

    ssc_wr(DMA_CTRL, DMA_CTRL_RESET);
    ssc_wr(DMA_BASE, (uint32_t)(UINTPTR)ring);
    ssc_wr(DMA_SIZE_LOG2, size_log2);
    ssc_wr(DMA_RD_IDX, 0u);
    ssc_wr(DMA_BATCH, batch);
    ssc_wr(DMA_TIMEOUT, timeout_ms);
    ssc_wr(DMA_CTRL, DMA_CTRL_EN);
}

uint32_t dma_available(void)
{
    return (ssc_rd(DMA_WR_IDX) - g_rd) & g_ring_mask;
}

const ssc2_record_t *dma_record(uint32_t k)
{
    ssc2_record_t *p = &g_ring[(g_rd + k) & g_ring_mask];
    Xil_DCacheInvalidateRange((INTPTR)p, sizeof(*p));
    return p;
}

void dma_consume(uint32_t n)
{
    g_rd = (g_rd + n) & g_ring_mask;
    ssc_wr(DMA_RD_IDX, g_rd);
}

/* ------------------------------------------------------------------------- */
/* v2: serial engine                                                          */
/* ------------------------------------------------------------------------- */
static uint32_t g_se_en;

void se_load(const uint16_t *prog, uint32_t len, uint32_t origin)
{
    uint32_t i;
    for (i = 0u; i < len && origin + i < 32u; i++)
        ssc_wr(SE_PROG(origin + i), prog[i]);
}

void se_sm_config(uint32_t sm, uint32_t clkdiv, uint32_t pinctrl, uint32_t shiftctrl)
{
    ssc_wr(SM_CLKDIV(sm), clkdiv);
    ssc_wr(SM_PINCTRL(sm), pinctrl);
    ssc_wr(SM_SHIFTCTRL(sm), shiftctrl);
}

void se_start(uint32_t sm_mask)
{
    g_se_en |= sm_mask & 3u;
    ssc_wr(SE_CTRL, g_se_en | ((sm_mask & 3u) << 8));   /* restart + enable */
}

void se_stop(uint32_t sm_mask)
{
    g_se_en &= ~sm_mask & 3u;
    ssc_wr(SE_CTRL, g_se_en);
}

int se_put(uint32_t sm, uint32_t word, uint32_t timeout_us)
{
    uint64_t end = ssc_time_us() + timeout_us;
    while (ssc_rd(SE_FSTAT) & SE_FSTAT_TX_FULL(sm)) {
        if (ssc_time_us() > end) return SSC_ETIMEOUT;
    }
    ssc_wr(SM_TXF(sm), word);
    return SSC_OK;
}

int se_get(uint32_t sm, uint32_t *word, uint32_t timeout_us)
{
    uint64_t end = ssc_time_us() + timeout_us;
    while (ssc_rd(SE_FSTAT) & SE_FSTAT_RX_EMPTY(sm)) {
        if (ssc_time_us() > end) return SSC_ETIMEOUT;
    }
    *word = ssc_rd(SM_RXF(sm));
    return SSC_OK;
}

void se_exec(uint32_t sm, uint16_t instr) { ssc_wr(SM_EXEC(sm), instr); }

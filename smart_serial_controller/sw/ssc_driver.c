/* =============================================================================
 * ssc_driver.c - bare-metal driver for the Smart Serial Controller
 * -----------------------------------------------------------------------------
 * Runs on the Zynq Cortex-A9 with the Xilinx "standalone" BSP.
 *   - every register access is a 32-bit AXI read/write (Xil_In32 / Xil_Out32)
 *   - the controller's IRQ line is interrupt 61 in the GIC; the ISR collects
 *     the event flags and moves received UART bytes into a ring buffer
 * ===========================================================================*/
#include <string.h>
#include "xparameters.h"
#include "xil_io.h"
#include "xil_exception.h"
#include "xscugic.h"
#ifdef SDT
#include "xiltimer.h"   /* Vitis 2023.2+ (SDT / Unified IDE): XTime, XTime_GetTime, COUNTS_PER_SECOND */
#else
#include "xtime_l.h"    /* Vitis Classic 2020.2 - 2023.1 */
#endif
#include "ssc_driver.h"

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

/* ------------------------------------------------------------------------- */
/* interrupts                                                                 */
/* ------------------------------------------------------------------------- */
static XScuGic           g_gic;
static volatile int      g_irq_on;
static volatile uint32_t g_events;          /* flags collected by the ISR */
static volatile uint32_t g_irq_count;

#define RXBUF_SIZE 256u                     /* UART receive ring buffer */
static volatile uint8_t  g_rxbuf[RXBUF_SIZE];
static volatile uint32_t g_rx_head, g_rx_tail;

static void ssc_isr(void *ref)
{
    uint32_t st, d;
    (void)ref;
    st = ssc_rd(SSC_INT_STATUS) & ssc_rd(SSC_INT_ENABLE);

    if (st & SSC_INT_UART_RX) {             /* empty the hardware RX FIFO */
        while (!((d = ssc_rd(SSC_UART_RXDATA)) & RXDATA_EMPTY)) {
            uint32_t next = (g_rx_head + 1u) % RXBUF_SIZE;
            if (next != g_rx_tail) {
                g_rxbuf[g_rx_head] = (uint8_t)d;
                g_rx_head = next;
            }
        }
    }
    ssc_wr(SSC_INT_STATUS, st);             /* write 1 to clear */
    g_events |= st;
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

    /* events the ISR handles: received UART data, "done" and every error */
    ssc_wr(SSC_INT_STATUS, 0xFFFFu);
    ssc_wr(SSC_INT_ENABLE, SSC_INT_UART_RX | SSC_INT_UART_PARITY | SSC_INT_UART_FRAME |
                           SSC_INT_UART_BREAK | SSC_INT_UART_RX_OVF | SSC_INT_SPI_DONE |
                           SSC_INT_SPI_RX_OVF | SSC_INT_I2C_DONE | SSC_INT_I2C_NACK |
                           SSC_INT_I2C_ARB_LOST);
    g_irq_on = 1;
    return 0;
}

uint32_t ssc_irq_count(void) { return g_irq_count; }

/* Return the events in mask that have happened since the last call, and
 * forget them.  Works with and without interrupts. */
uint32_t ssc_take_events(uint32_t mask)
{
    uint32_t ev, polled;
    if (g_irq_on) {
        Xil_ExceptionDisable();
        ev = g_events & mask;
        g_events &= ~ev;
        Xil_ExceptionEnable();
        polled = ssc_rd(SSC_INT_STATUS) & mask & ~ssc_rd(SSC_INT_ENABLE);
    } else {
        ev = 0;
        polled = ssc_rd(SSC_INT_STATUS) & mask;
    }
    if (polled) {
        ssc_wr(SSC_INT_STATUS, polled);
        ev |= polled;
    }
    return ev;
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

int ssc_init(void)
{
    ssc_timer_start();
    if (ssc_rd(SSC_ID) != SSC_ID_VALUE) return -1;   /* wrong design or address */
    ssc_wr(SSC_BRIDGE_CTRL, 0u);
    ssc_wr(SSC_INT_ENABLE, 0u);
    ssc_wr(SSC_INT_STATUS, 0xFFFFu);
    return 0;
}

/* ------------------------------------------------------------------------- */
/* UART                                                                       */
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
    if (g_irq_on && (ssc_rd(SSC_INT_ENABLE) & SSC_INT_UART_RX)) {
        if (g_rx_tail == g_rx_head) return -1;           /* ring buffer empty */
        d = g_rxbuf[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1u) % RXBUF_SIZE;
        return (int)d;
    }
    d = ssc_rd(SSC_UART_RXDATA);
    return (d & RXDATA_EMPTY) ? -1 : (int)(d & 0xFFu);
}

/* ------------------------------------------------------------------------- */
/* SPI                                                                        */
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
 * n bytes come back (rx may be NULL), CS0 goes high.  Works for any n: the
 * data is pushed through the 16-entry FIFOs in chunks while CS stays low. */
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

/* ------------------------------------------------------------------------- */
/* PmodSF3 flash (Micron N25Q256 / MT25QL256)                                 */
/* ------------------------------------------------------------------------- */
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
/* I2C                                                                        */
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

/* Write wlen bytes, then (if rlen > 0) read rlen bytes with a repeated START,
 * exactly like the worked example on slide 13. */
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
        end = ssc_time_us() + 200000u;
        for (i = 0u; i < rlen; ) {                           /* empty the RX FIFO */
            d = ssc_rd(SSC_I2C_RXDATA);
            if (!(d & RXDATA_EMPTY))  rbuf[i++] = (uint8_t)d;
            else if (!(ssc_rd(SSC_I2C_STATUS) & ST_BUSY)) break;   /* NACK / ARB */
            else if (ssc_time_us() > end) break;
        }
        while (i < rlen && !((d = ssc_rd(SSC_I2C_RXDATA)) & RXDATA_EMPTY))
            rbuf[i++] = (uint8_t)d;                          /* last byte, if any */
        r = i2c_finish();
        if (r != SSC_OK) return r;
        if (i < rlen) return SSC_ETIMEOUT;
    }
    return SSC_OK;
}

/* PmodTMP2 (ADT7420): register 0x00/0x01 = temperature, 13-bit, 0.0625 C */
int tmp2_read(int32_t *temp_e4, uint8_t raw[2])
{
    uint8_t ptr = 0x00u, b[2];
    int16_t v;
    int r = ssc_i2c_xfer(TMP2_ADDR, 0, &ptr, 1u, b, 2u);
    if (r != SSC_OK) return r;
    raw[0] = b[0];
    raw[1] = b[1];
    v = (int16_t)((((uint16_t)b[0] << 8) | b[1]) & 0xFFF8u);   /* drop 3 flag bits */
    *temp_e4 = (int32_t)(v / 8) * 625;          /* (raw >> 3) x 0.0625 C, in 1e-4 C */
    return SSC_OK;
}

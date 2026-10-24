/* =============================================================================
 * ssc2_driver.h - bare-metal driver for the Smart Serial Controller v2
 * -----------------------------------------------------------------------------
 * Runs on the Zynq Cortex-A9 with the Xilinx "standalone" BSP.
 * The first half is the v1 driver (UART, SPI, I2C engines used directly by
 * the CPU).  The second half sets up the v2 blocks: crossbar, network
 * interfaces, sensor hub, DMA writer and serial engine.
 * ===========================================================================*/
#ifndef SSC2_DRIVER_H
#define SSC2_DRIVER_H

#include <stdint.h>
#include "ssc2_regs.h"

/* error codes */
#define SSC_OK        0
#define SSC_ENACK    -1      /* I2C slave did not acknowledge   */
#define SSC_EARB     -2      /* I2C arbitration lost            */
#define SSC_ETIMEOUT -3
#define SSC_EID      -4      /* wrong bitstream / base address  */

/* ---- raw register access, time ---- */
uint32_t ssc_rd(uint32_t off);
void     ssc_wr(uint32_t off, uint32_t val);
uint64_t ssc_time_us(void);                /* CPU global timer: no bus access */
void     ssc_delay_us(uint32_t us);

/* ---- set-up, interrupts, events ---- */
int      ssc_init(void);                   /* checks ID and VERSION, everything off */
int      ssc_irq_setup(void);              /* GIC + ISR (interrupt 61) */
void     ssc_irq_enable(uint32_t v1_mask, uint32_t v2_mask);
uint32_t ssc_take_events(uint32_t mask);   /* v1 INT_STATUS events: return + forget */
uint32_t ssc_wait_events(uint32_t mask, uint32_t timeout_us);
uint32_t ssc_take_events2(uint32_t mask);  /* v2 INT2_STATUS events */
uint32_t ssc_wait_events2(uint32_t mask, uint32_t timeout_us);   /* sleeps with WFI */
uint32_t ssc_irq_count(void);              /* interrupts seen by the ISR */

/* ---- CPU-load counters in the controller (slide 24, measurement 4) ---- */
void     ssc_count_clear(void);            /* clears APB_COUNT and IRQ_COUNT */
uint32_t ssc_apb_count(void);              /* bus transfers since the clear  */
uint32_t ssc_hw_irq_count(void);           /* rising edges of the IRQ line   */

/* ---- v1 engines: UART ---- */
void ssc_uart_init(uint32_t baud, uint32_t bits, char parity, uint32_t stop_bits, int flow);
void ssc_uart_putc(uint8_t c);
void ssc_uart_puts(const char *s);
int  ssc_uart_getc(void);                  /* -1 when nothing has arrived */

/* ---- v1 engines: SPI (master, chip-select 0, 8-bit words) ---- */
void ssc_spi_init(uint32_t sclk_hz, uint32_t mode);
void ssc_spi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t n);   /* CS0 low throughout */
void flash_read_id(uint8_t id[3]);
int  flash_erase_4k(uint32_t addr);
int  flash_program(uint32_t addr, const uint8_t *data, uint32_t n);   /* n <= 256 */
void flash_read(uint32_t addr, uint8_t *data, uint32_t n);            /* n <= 256 */

/* ---- v1 engines: I2C ---- */
void ssc_i2c_init(uint32_t scl_hz);
int  ssc_i2c_xfer(uint16_t addr, int ten_bit,
                  const uint8_t *wbuf, uint32_t wlen,
                  uint8_t *rbuf, uint32_t rlen);   /* write then repeated-START read */
#define TMP2_ADDR 0x4Bu
int  tmp2_read(int32_t *temp_e4, uint8_t raw[2]);  /* temperature x 10000 (1e-4 C) */
int32_t tmp2_convert(uint8_t msb, uint8_t lsb);    /* raw bytes -> 1e-4 C */

/* ---- v2: pin crossbar ---- */
int      xbar_select(uint32_t port, uint32_t src, int force, uint32_t timeout_us);
uint32_t xbar_current(uint32_t port);
uint32_t xbar_switch_cycles(uint32_t port);   /* request -> hand-over (clocks) */
uint32_t xbar_switch_edge(uint32_t port);     /* request -> first new edge (clocks) */
void     xbar_gpio(uint32_t port, uint32_t out4, uint32_t oe4);

/* ---- v2: network interfaces ---- */
void     ni_config(uint32_t node, uint32_t cfg);   /* NI_CFG[node] (nodes 0, 3, 4, 5) */
uint32_t ni_packets_in(uint32_t node);
uint32_t ni_packets_out(uint32_t node);

/* ---- v2: sensor hub ---- */
void hub_task(uint32_t t, uint32_t w0, uint32_t bytes0_3, uint32_t bytes4_7, uint32_t timing);
void hub_start(void);
void hub_stop(void);
void hub_run_now(uint32_t task_mask);

/* ---- v2: DMA writer (records into a DDR ring buffer) ---- */
void     dma_setup(void *ring, uint32_t size_log2, uint32_t batch, uint32_t timeout_ms);
uint32_t dma_available(void);                  /* records written but not yet consumed */
const ssc2_record_t *dma_record(uint32_t k);   /* k-th unread record (cache invalidated) */
void     dma_consume(uint32_t n);              /* give n records back to the hardware */

/* ---- v2: serial engine ---- */
void     se_load(const uint16_t *prog, uint32_t len, uint32_t origin);
void     se_sm_config(uint32_t sm, uint32_t clkdiv, uint32_t pinctrl, uint32_t shiftctrl);
void     se_start(uint32_t sm_mask);           /* restart + enable */
void     se_stop(uint32_t sm_mask);
int      se_put(uint32_t sm, uint32_t word, uint32_t timeout_us);
int      se_get(uint32_t sm, uint32_t *word, uint32_t timeout_us);
void     se_exec(uint32_t sm, uint16_t instr);

#endif /* SSC2_DRIVER_H */

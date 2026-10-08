/* =============================================================================
 * ssc_driver.h - bare-metal driver for the Smart Serial Controller (Zynq ARM)
 * ===========================================================================*/
#ifndef SSC_DRIVER_H
#define SSC_DRIVER_H

#include <stdint.h>
#include "ssc_regs.h"

/* error codes */
#define SSC_OK        0
#define SSC_ENACK    -1      /* I2C slave did not acknowledge   */
#define SSC_EARB     -2      /* I2C arbitration lost            */
#define SSC_ETIMEOUT -3

/* ---- raw register access ---- */
uint32_t ssc_rd(uint32_t off);
void     ssc_wr(uint32_t off, uint32_t val);

/* ---- set-up, interrupts, events ---- */
int      ssc_init(void);                   /* checks the ID register */
int      ssc_irq_setup(void);              /* GIC + ISR (interrupt 61) */
uint32_t ssc_take_events(uint32_t mask);   /* returns and clears events */
uint32_t ssc_wait_events(uint32_t mask, uint32_t timeout_us);
uint32_t ssc_irq_count(void);
uint64_t ssc_time_us(void);

/* ---- UART ---- */
void ssc_uart_init(uint32_t baud, uint32_t bits, char parity, uint32_t stop_bits, int flow);
void ssc_uart_putc(uint8_t c);
void ssc_uart_puts(const char *s);
int  ssc_uart_getc(void);                  /* -1 when nothing has arrived */

/* ---- SPI (master, chip-select 0, 8-bit words) ---- */
void ssc_spi_init(uint32_t sclk_hz, uint32_t mode);
void ssc_spi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t n);   /* CS0 low throughout */

/* ---- PmodSF3 flash on SPI ---- */
void flash_read_id(uint8_t id[3]);
int  flash_erase_4k(uint32_t addr);
int  flash_program(uint32_t addr, const uint8_t *data, uint32_t n);   /* n <= 256, one page */
void flash_read(uint32_t addr, uint8_t *data, uint32_t n);            /* n <= 256 */

/* ---- I2C ---- */
void ssc_i2c_init(uint32_t scl_hz);
int  ssc_i2c_xfer(uint16_t addr, int ten_bit,
                  const uint8_t *wbuf, uint32_t wlen,
                  uint8_t *rbuf, uint32_t rlen);   /* write then repeated-START read */

/* ---- PmodTMP2 on I2C ---- */
#define TMP2_ADDR 0x4Bu
int  tmp2_read(int32_t *temp_e4, uint8_t raw[2]);   /* temperature x 10000 (1e-4 C) */

#endif /* SSC_DRIVER_H */

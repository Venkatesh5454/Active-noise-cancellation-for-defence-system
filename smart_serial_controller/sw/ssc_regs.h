/* =============================================================================
 * ssc_regs.h - register map of the Smart Serial Controller
 * -----------------------------------------------------------------------------
 * Must match rtl/ssc_regbank.v.  All registers are 32 bit, byte offsets below.
 * ===========================================================================*/
#ifndef SSC_REGS_H
#define SSC_REGS_H

#include <stdint.h>

/* Base address given to ssc_0 in the block design (Address Editor).
 * vivado/build_ps_system.tcl sets it to 0x43C0_0000. */
#ifndef SSC_BASEADDR
#define SSC_BASEADDR        0x43C00000u
#endif
#define SSC_IRQ_ID          61u            /* IRQ_F2P[0] = shared peripheral interrupt 61 */
#define SSC_CLK_HZ          100000000u     /* FCLK_CLK0 */

/* ---------------- global ---------------- */
#define SSC_ID              0x000u         /* reads 0x53534301 */
#define SSC_INT_STATUS      0x004u         /* write 1 to clear */
#define SSC_INT_ENABLE      0x008u
#define SSC_BRIDGE_CTRL     0x00Cu
#define SSC_BRIDGE_CNT      0x010u         /* [15:0] ch0, [31:16] ch1, write clears */
#define SSC_SCRATCH         0x014u
#define SSC_ID_VALUE        0x53534301u

/* INT_STATUS / INT_ENABLE bits */
#define SSC_INT_UART_RX       (1u << 0)    /* level: RX FIFO not empty        */
#define SSC_INT_UART_TX_IDLE  (1u << 1)    /* level: TX FIFO empty, line idle */
#define SSC_INT_UART_PARITY   (1u << 2)
#define SSC_INT_UART_FRAME    (1u << 3)
#define SSC_INT_UART_BREAK    (1u << 4)
#define SSC_INT_UART_RX_OVF   (1u << 5)
#define SSC_INT_SPI_DONE      (1u << 6)
#define SSC_INT_SPI_RX        (1u << 7)    /* level */
#define SSC_INT_SPI_RX_OVF    (1u << 8)
#define SSC_INT_I2C_DONE      (1u << 9)
#define SSC_INT_I2C_NACK      (1u << 10)
#define SSC_INT_I2C_ARB_LOST  (1u << 11)
#define SSC_INT_I2C_RX        (1u << 12)   /* level */
#define SSC_INT_UART_TX_OVF   (1u << 13)
#define SSC_INT_SPI_TX_OVF    (1u << 14)
#define SSC_INT_I2C_TX_OVF    (1u << 15)

/* bridge: protocol codes are the base paper's COSE codes */
#define BR_OFF   0u
#define BR_SPI   1u
#define BR_I2C   2u
#define BR_UART  3u
#define SSC_BRIDGE(src0, dst0, src1, dst1) \
    (((src0) & 3u) | (((dst0) & 3u) << 2) | (((src1) & 3u) << 4) | (((dst1) & 3u) << 6))

/* STATUS registers: same layout for UART, SPI and I2C */
#define ST_TX_EMPTY         (1u << 0)
#define ST_TX_FULL          (1u << 1)
#define ST_RX_EMPTY         (1u << 2)
#define ST_RX_FULL          (1u << 3)
#define ST_BUSY             (1u << 4)      /* UART: TX busy */
#define ST_TX_COUNT(s)      (((s) >> 8)  & 0x1Fu)
#define ST_RX_COUNT(s)      (((s) >> 16) & 0x1Fu)
#define RXDATA_EMPTY        (1u << 31)     /* UART_RXDATA / I2C_RXDATA */

/* ---------------- UART ---------------- */
#define SSC_UART_CTRL       0x100u
#define SSC_UART_BAUD       0x104u
#define SSC_UART_STATUS     0x108u
#define SSC_UART_TXDATA     0x10Cu
#define SSC_UART_RXDATA     0x110u

#define UART_CTRL_TX_EN     (1u << 0)
#define UART_CTRL_RX_EN     (1u << 1)
#define UART_CTRL_BITS(n)   ((((n) - 5u) & 3u) << 2)   /* 5..8 data bits */
#define UART_CTRL_PAR_EN    (1u << 4)
#define UART_CTRL_PAR_ODD   (1u << 5)
#define UART_CTRL_STOP2     (1u << 6)
#define UART_CTRL_FLOW      (1u << 7)
#define UART_CTRL_LOOP      (1u << 8)
#define UART_CTRL_TX_FLUSH  (1u << 16)
#define UART_CTRL_RX_FLUSH  (1u << 17)
#define UART_ST_RX_BUSY     (1u << 5)
#define UART_ST_CTS_OK      (1u << 6)

/* ---------------- SPI ---------------- */
#define SSC_SPI_CTRL        0x200u
#define SSC_SPI_CLKDIV      0x204u         /* SCLK = 100 MHz / (2 x (DIV+1)), DIV >= 3 */
#define SSC_SPI_STATUS      0x208u
#define SSC_SPI_TXDATA      0x20Cu
#define SSC_SPI_RXDATA      0x210u

#define SPI_CTRL_EN         (1u << 0)
#define SPI_CTRL_CPOL       (1u << 1)
#define SPI_CTRL_CPHA       (1u << 2)
#define SPI_CTRL_LSB_FIRST  (1u << 3)
#define SPI_CTRL_LEN(n)     ((((n) - 1u) & 31u) << 4)  /* 4..32 bit words */
#define SPI_CTRL_CS_SEL(c)  (((c) & 3u) << 9)
#define SPI_CTRL_CS_MANUAL  (1u << 11)
#define SPI_CTRL_CS_LEVEL   (1u << 12)                 /* 1 = CS# low */
#define SPI_CTRL_SLAVE      (1u << 13)
#define SPI_CTRL_LOOP       (1u << 14)
#define SPI_CTRL_TX_FLUSH   (1u << 16)
#define SPI_CTRL_RX_FLUSH   (1u << 17)
#define SPI_CTRL_ABORT      (1u << 18)

/* ---------------- I2C ---------------- */
#define SSC_I2C_CTRL        0x300u
#define SSC_I2C_PRESCALE    0x304u         /* f_SCL ~= 100 MHz / (4 x (P+1)) */
#define SSC_I2C_ADDR        0x308u
#define SSC_I2C_CMD         0x30Cu
#define SSC_I2C_STATUS      0x310u
#define SSC_I2C_TXDATA      0x314u
#define SSC_I2C_RXDATA      0x318u

#define I2C_CTRL_EN         (1u << 0)
#define I2C_CTRL_AUTO_WR    (1u << 1)
#define I2C_CTRL_TX_FLUSH   (1u << 16)
#define I2C_CTRL_RX_FLUSH   (1u << 17)
#define I2C_CTRL_ABORT      (1u << 18)
#define I2C_ADDR_TEN_BIT    (1u << 15)
#define I2C_CMD_LEN(n)      ((n) & 0xFFu)
#define I2C_CMD_READ        (1u << 8)
#define I2C_CMD_STOP        (1u << 9)
#define I2C_CMD_STOP_ONLY   (1u << 10)
#define I2C_ST_HOLDING      (1u << 5)
#define I2C_ST_NACK         (1u << 6)
#define I2C_ST_ARB_LOST     (1u << 7)
#define I2C_ST_BUS_BUSY     (1u << 24)
#define I2C_ST_SCL          (1u << 25)
#define I2C_ST_SDA          (1u << 26)

#endif /* SSC_REGS_H */

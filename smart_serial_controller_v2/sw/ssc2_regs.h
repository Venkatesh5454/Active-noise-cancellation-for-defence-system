/* =============================================================================
 * ssc2_regs.h - register map of the Smart Serial Controller v2
 * -----------------------------------------------------------------------------
 * Must match rtl/top/ssc2_core.v and docs/SPEC.md.  All registers are 32 bit;
 * the offsets below are bytes from the controller's base address.
 *
 *   0x000-0x3FF  v1 registers (same as v1, ID = 0x53534302)
 *   0x400        pin crossbar          0x500  NI configuration
 *   0x600        sensor hub            0x700  DMA writer
 *   0x800-0x9FF  serial engine         0xA00  misc + second interrupt register
 * ===========================================================================*/
#ifndef SSC2_REGS_H
#define SSC2_REGS_H

#include <stdint.h>

/* Base address given to ssc_0 in the block design (Address Editor).
 * vivado/build_ps_system.tcl sets it to 0x43C0_0000. */
#ifndef SSC_BASEADDR
#define SSC_BASEADDR        0x43C00000u
#endif
#define SSC_IRQ_ID          61u            /* IRQ_F2P[0] = shared peripheral interrupt 61 */
#define SSC_CLK_HZ          100000000u     /* FCLK_CLK0 */

/* ---------------- global ---------------- */
#define SSC_ID              0x000u         /* reads 0x53534302 in v2 */
#define SSC_INT_STATUS      0x004u         /* write 1 to clear */
#define SSC_INT_ENABLE      0x008u
#define SSC_BRIDGE_CTRL     0x00Cu         /* kept for v1 software; no effect in v2 */
#define SSC_BRIDGE_CNT      0x010u         /* [15:0] ch0, [31:16] ch1, write clears */
#define SSC_SCRATCH         0x014u
#define SSC_ID_VALUE        0x53534302u

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


/* ============================ v2 additions ============================ */

/* ---------------- node ids and codes (SPEC section 1) ---------------- */
#define NODE_SE             0u             /* (0,0) serial engine */
#define NODE_HUB            1u             /* (1,0) sensor hub    */
#define NODE_DMA            2u             /* (2,0) DMA writer    */
#define NODE_UART           3u             /* (0,1) UART          */
#define NODE_SPI            4u             /* (1,1) SPI           */
#define NODE_I2C            5u             /* (2,1) I2C           */

#define PKT_DATA            0u
#define PKT_XFER_REQ        1u
#define PKT_XFER_RESP       2u
#define PKT_RECORD          3u
#define PKT_ALARM           4u

/* crossbar sources and ports */
#define SRC_OFF             0u
#define SRC_UART            1u
#define SRC_SPI             2u
#define SRC_I2C             3u
#define SRC_SM0             4u
#define SRC_SM1             5u
#define SRC_GPIO            6u
#define PORT_JA             0u
#define PORT_JB             1u
#define PORT_JC             2u
#define PORT_JD             3u
#define PORT_OLED           4u
#define PORT_LED            5u

/* ---------------- pin crossbar (0x400) ---------------- */
#define XBAR_PIN_SEL(p)     (0x400u + 4u * (p))   /* W: [2:0] source [8] FORCE */
#define XBAR_STATUS         0x420u                /* [5:0] switching per port   */
#define XBAR_SW_CYCLES(p)   (0x440u + 4u * (p))   /* request -> hand-over, clocks */
#define XBAR_SW_EDGE(p)     (0x460u + 4u * (p))   /* request -> first new edge    */
#define XBAR_GPIO_OUT       0x480u                /* 4 bits per port */
#define XBAR_GPIO_OE        0x484u
#define XBAR_GPIO_IN        0x488u
#define XBAR_FORCE          (1u << 8)
#define XBAR_SEL_CUR(v)     ((v) & 7u)
#define XBAR_SEL_REQ(v)     (((v) >> 4) & 7u)
#define XBAR_SEL_BUSY       (1u << 8)

/* ---------------- NI configuration (0x500) ---------------- */
#define NI_CFG(n)           (0x500u + 4u * (n))
#define NI_STAT(n)          (0x520u + 4u * (n))   /* [15:0] packets in, [31:16] out */
#define NI_UART_TIMEOUT     0x540u                /* us, reset 2000  */
#define NI_I2C_TIMEOUT      0x544u                /* us, reset 50000 */
#define NI_EN               (1u << 0)
#define NI_MODE(m)          (((m) & 3u) << 1)     /* UART: 0 RAW 1 FRAME 2 ADDRESSED */
#define NI_DEST(d)          (((d) & 7u) << 4)
#define NI_PRIO             (1u << 7)
#define NI_ARG(a)           (((a) & 0xFFu) << 8)
#define NI_RESP_STATUS      (1u << 16)
#define NI_WORD4_SM0        (1u << 17)
#define NI_WORD4_SM1        (1u << 18)
#define NI_DEST_SM1(d)      (((d) & 7u) << 20)
#define NI_PRIO_SM1         (1u << 23)
#define NI_MODE_RAW         0u
#define NI_MODE_FRAME       1u
#define NI_MODE_ADDRESSED   2u

/* ---------------- sensor hub (0x600) ---------------- */
#define HUB_CTRL            0x600u                /* [0] EN, W1 [15:8] RUN_NOW */
#define HUB_STATUS          0x604u                /* [0] busy [3:1] task [8] waiting */
#define HUB_REC_DEST        0x608u
#define HUB_TIMEOUT         0x60Cu                /* ms */
#define HUB_SEQ             0x610u
#define HUB_LAST            0x614u                /* [3:0] len [15:8] status [23:16] task */
#define HUB_LAST_LO         0x618u
#define HUB_LAST_HI         0x61Cu
#define HUB_CNT_REQ         0x620u
#define HUB_CNT_RESP_OK     0x624u
#define HUB_CNT_ERR         0x628u
#define HUB_CNT_TMO         0x62Cu
#define HUB_CNT_REC         0x630u
#define HUB_TASK(t, w)      (0x640u + 16u * (t) + 4u * (w))
#define HUB_CTRL_EN         (1u << 0)
#define HUB_RUN_NOW(t)      (1u << (8u + (t)))

/* task word 0 */
#define TASK_EN             (1u << 0)
#define TASK_DEST(d)        (((d) & 7u) << 1)
#define TASK_TYPE(t)        (((t) & 7u) << 4)
#define TASK_PRIO           (1u << 7)
#define TASK_ARG(a)         (((a) & 0xFFu) << 8)
#define TASK_WLEN(n)        (((n) & 0xFu) << 16)
#define TASK_RLEN(n)        (((n) & 0xFu) << 20)
#define TASK_RECORD         (1u << 24)
#define TASK_SEND_LAST      (1u << 25)
#define TASK_ADDR_INC       (1u << 26)
#define TASK_TRIG_RECORDS   (1u << 27)
#define TASK_APPEND_REC     (1u << 28)
/* task word 3 */
#define TASK_TIMING(period, phase) (((period) & 0xFFFFu) | (((phase) & 0xFFFFu) << 16))

/* ---------------- DMA writer (0x700) ---------------- */
#define DMA_CTRL            0x700u                /* [0] EN, W1 [8] RESET */
#define DMA_BASE            0x704u
#define DMA_SIZE_LOG2       0x708u                /* ring = 2^n records of 16 bytes */
#define DMA_WR_IDX          0x70Cu
#define DMA_RD_IDX          0x710u
#define DMA_BATCH           0x714u
#define DMA_TIMEOUT         0x718u                /* ms */
#define DMA_COUNT           0x71Cu
#define DMA_OVERFLOW        0x720u
#define DMA_AXI_ERR         0x724u
#define DMA_PENDING         0x728u
#define DMA_STATUS          0x72Cu
#define DMA_IGNORED         0x730u
#define DMA_CTRL_EN         (1u << 0)
#define DMA_CTRL_RESET      (1u << 8)

/* one record as the DMA writer stores it (16 bytes, little-endian) */
typedef struct {
    uint32_t timestamp_us;
    uint8_t  source;        /* 1 UART, 2 SPI, 3 I2C, 4 serial engine */
    uint8_t  length;        /* valid data bytes 0..8 */
    uint16_t seq;
    uint8_t  data[8];
} ssc2_record_t;

/* ---------------- serial engine (0x800) ---------------- */
#define SE_CTRL             0x800u                /* [0] SM0_EN [1] SM1_EN, W1 [8]/[9] RESTART */
#define SE_FSTAT            0x804u
#define SE_CRC_CFG          0x808u                /* [15:0] POLY [31:16] INIT */
#define SE_TIME             0x80Cu
#define SM_CLKDIV(s)        (0x810u + 0x40u * (s))  /* [31:16] INT [15:8] FRAC */
#define SM_PINCTRL(s)       (0x814u + 0x40u * (s))
#define SM_SHIFTCTRL(s)     (0x818u + 0x40u * (s))
#define SM_TXF(s)           (0x81Cu + 0x40u * (s))
#define SM_RXF(s)           (0x820u + 0x40u * (s))
#define SM_EXEC(s)          (0x824u + 0x40u * (s))
#define SM_STATE(s)         (0x828u + 0x40u * (s))
#define SM_X(s)             (0x82Cu + 0x40u * (s))
#define SM_Y(s)             (0x830u + 0x40u * (s))
#define SM_CRC(s)           (0x834u + 0x40u * (s))
#define SM_PINS(s)          (0x838u + 0x40u * (s))
#define SE_PROG(i)          (0x900u + 4u * (i))
#define SE_CTRL_EN(s)       (1u << (s))
#define SE_CTRL_RESTART(s)  (1u << (8u + (s)))
#define SM_CLKDIV_VAL(i, f) ((((i) & 0xFFFFu) << 16) | (((f) & 0xFFu) << 8))
#define SE_FSTAT_TX_FULL(s) (1u << (16u + 2u * (s)))
#define SE_FSTAT_RX_EMPTY(s) (1u << (17u + 2u * (s)))
#define SM_STATE_STALLED    (1u << 8)

/* ---------------- misc (0xA00) ---------------- */
#define SSC2_VERSION        0xA00u                /* reads 0x00020000 */
#define SSC2_TIME_US        0xA04u
#define SSC2_INT2_STATUS    0xA08u                /* write 1 to clear */
#define SSC2_INT2_ENABLE    0xA0Cu
#define SSC2_APB_COUNT      0xA10u                /* bus transfers, write clears */
#define SSC2_IRQ_COUNT      0xA14u                /* interrupts, write clears */
#define SSC2_OLED_PWR       0xA18u                /* [0] VDD [1] VBAT, 1 = off */
#define SSC2_BOARD_IN       0xA1Cu                /* [7:0] switches [12:8] buttons */
#define SSC2_SE_LOOP        0xA20u                /* [0] SM1 pin 2 <- SM0 pin 1 */
#define SSC2_VERSION_VALUE  0x00020000u

#define INT2_DMA_BATCH      (1u << 0)
#define INT2_DMA_OVERFLOW   (1u << 1)
#define INT2_DMA_AXI_ERR    (1u << 2)
#define INT2_HUB_ERROR      (1u << 3)
#define INT2_HUB_RECORD     (1u << 4)
#define INT2_XBAR_SWITCH    (1u << 5)
#define INT2_SE_RX0         (1u << 6)            /* level */
#define INT2_SE_RX1         (1u << 7)            /* level */

#endif /* SSC2_REGS_H */

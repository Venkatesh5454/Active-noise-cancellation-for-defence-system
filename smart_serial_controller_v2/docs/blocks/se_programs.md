# Serial-engine programs (`programs/*.se`)

These are the protocols that run on the serial engine (slides 12, 13 and 24).
Each one is a short program in the assembler syntax of
`docs/blocks/serial_engine.md`. `tools/build_programs.sh` turns every `.se`
file into:

- `programs/<name>.hex`: one 16-bit word per line, for the testbenches,
- `programs/<name>.lst`: a listing with addresses, codes and source lines,
- one C header, `sw/se_programs.h`, with every program as an array plus
  `SE_PROG_<NAME>_ORIGIN / _LEN / _CLKDIV / _PINCTRL / _SHIFTCTRL`.

The point of the serial engine is that a new protocol is a new program, not
a new Verilog engine and bitstream. The programs below cover the three
protocols of v1 (so they can be compared with the v1 engines bit for bit),
one new protocol (Manchester) and the on-board OLED.

## 1. Overview

| program | words | origin | clock (INT + FRAC/256) | pins (Pmod 1..4 = pin 0..3) | runs on |
|---|---|---|---|---|---|
| `uart_tx` | 7 | 0 | 108 + 128 = 921.6 kHz = 8 x 115200 | pin 1 TXD | SM0 |
| `uart_rx` | 11 | 7 | 108 + 128 | pin 2 RXD | SM1 |
| `spi_master` | 14 | 18 | 25 = 4 MHz ticks, SCK 1 MHz | 0 CS#, 1 MOSI, 2 MISO, 3 SCK | SM0 |
| `i2c_master` | 32 | 0 | 125 = 800 kHz ticks, SCL 100 kHz | 2 SCL, 3 SDA (open drain) | SM0 |
| `manchester_tx` | 13 | 7 | 62 + 128 = 1.6 MHz ticks, 100 kbit/s | pin 1 TX | SM0 |
| `manchester_rx` | 12 | 20 | 62 + 128 | pin 2 RX | SM1 |
| `oled_spi` | 6 | 26 | 10 = 10 MHz ticks, SCLK 5 MHz | 0 DC, 1 SDIN, 2 RES#, 3 SCLK | SM1 |

The program memory has 32 words, shared by both state machines. The origins
were chosen so that the usual pairs fit together:

```
 0        7        18 20      26    31
 |uart_tx |uart_rx  |          |     |      uart_tx + uart_rx (+ oled_spi)
 |uart_tx |manchester_tx |manch._rx |      Manchester pair
 |uart_tx |uart_rx  |spi_master   |        UART + SPI
 |          i2c_master (all 32 words)  |      I2C alone
```

The pin numbers follow the same role table as the fixed engines
(`docs/SPEC.md` section 6), so a program can be routed by the crossbar to
the connector where its device is: the UART pins match the PmodUSBUART, the
SPI pins the PmodSF3, the I2C pins the PmodTMP2, and the OLED pins the
crossbar's OLED port.

## 2. How software uses a program

```c
#include "se_programs.h"
se_load(se_prog_uart_tx, SE_PROG_UART_TX_LEN, SE_PROG_UART_TX_ORIGIN);   /* PROG[] */
se_sm_config(0, SE_PROG_UART_TX_CLKDIV, SE_PROG_UART_TX_PINCTRL,
             SE_PROG_UART_TX_SHIFTCTRL);                                  /* SM0 regs */
ni_config(NODE_SE, 0);           /* the CPU feeds the FIFOs, not the network */
se_start(1);                     /* RESTART + enable SM0 */
xbar_select(PORT_JD, SRC_SM0, 0, 10000);   /* SM0's pins appear on JD */
se_put(0, 'A', 10000);           /* TX word -> SM0_TXF */
```

`sw/se_demos.c` does exactly this for every program (console commands
`se uart`, `se spi`, `se i2c`, `se man`, `se oled`, `switch`).

Data can also come from the network instead of the CPU: when `NI_CFG[0]` is
enabled, every byte of a packet sent to node 0 becomes one TX word of SM
`arg[0]` (in bits [7:0], or [31:24] if that SM's OUT shifts left), and RX
words go back out as DATA packets (`docs/blocks/ni_se.md`). That is how
the hub's "show the last value" task and the PC's RAW frames reach a
program.

## 3. The programs

### 3.1 `uart_tx`: the slide-13 program, unchanged

```
loop: PULL block          // wait for a byte
      SET  x, 7           // 8 data bits
      SET  pins, 0  [7]   // start bit: 8 ticks
bit:  OUT  pins, 1  [6]   // data bit, LSB first: 7 ticks
      JMP  x--, bit       // + 1 tick = 8 ticks per bit
      SET  pins, 1  [7]   // stop bit
      JMP  loop
```

- **TX word:** the byte in bits [7:0] (OUT shifts right, so the LSB leaves
  first).
- **Timing:** one tick is 100 MHz / 108.5 = 1.085 us, 8 ticks per bit =
  8.68 us = 115200 baud. The testbench measured 8680.0 ns per bit.
- **Pins:** TXD is pin 1, idle high (INIT_OUT, INIT_OE), PINCTRL 0x02200015.

### 3.2 `uart_rx`

Waits for the falling edge of the start bit, checks the middle of the start
bit (a glitch sends it back), samples 8 data bits in their middles (8 ticks
apart), checks the stop bit and pushes the byte.

- **RX word:** the byte in bits [31:24] (IN shifts right, so the first bit,
  the LSB, ends at bit 24). `ni_se` sends ISR[31:24] for such an SM, so a
  network packet simply carries the bytes.
- **Errors:** a low stop bit (framing error or break) drops the byte and
  waits for the idle line. A full RX FIFO drops the byte (PUSH noblock) and
  sets the overflow flag in SE_FSTAT.

### 3.3 `spi_master`: mode 0, MSB first, framed CS#

- **TX words:** the first word of a frame is N, the number of bytes, then N
  words with the bytes. Every byte sits in bits [31:24] (OUT shifts left).
  CS# goes low for the whole frame. N = 0 is ignored.
- **RX words:** one per byte sent, the byte read on MISO in bits [7:0].
- **Example, flash READ ID:** TX `0x04000000 0x9F000000 0 0 0` -> RX
  `0x00 0x20 0xBA 0x19`.
- If the TX FIFO runs empty inside a frame, the SM waits with CS# low and
  SCK low (a legal pause), so frames can be longer than the 4-word FIFO. A
  full RX FIFO also pauses the clock, so no byte is lost.
- SCK is the side-set pin: every instruction says whether SCK is high or
  low, so the clock needs no extra instructions. One bit = 4 ticks = 1 us.

### 3.4 `i2c_master`: 100 kHz, clock stretching, repeated START

The bus lines are open drain (OD_MASK = 1100): writing 0 pulls a line low,
writing 1 lets the pull-up raise it. SCL is the side-set pin. After every
release of SCL the program WAITs until SCL is really high, so a slave may
stretch the clock.

- **TX word = one byte on the bus:**

  | bits | meaning |
  |---|---|
  | [31] | START: send START (or a repeated START) before this byte |
  | [30] | STOP: send STOP after this byte |
  | [29:22] | the byte (0xFF for a read: SDA released) |
  | [21] | the 9th bit we drive: 1 for a write; for a read 0 = ACK, 1 = NACK |

- **RX word:** [8:1] the byte seen on SDA, [0] the 9th bit seen (0 = ACK).
- **PmodTMP2 read (0x4B, register 0):** 5 words:
  `START 0x96`, `0x00`, `START 0x97`, `0xFF + ACK`, `0xFF + NACK + STOP`
  -> RX `0x12C 0x000 0x12E 0x018 0x101`, so the bytes are 0x0C 0x80 =
  25.00 C (`se i2c` in `sw/se_demos.c`).
- **NACK:** the byte is pushed with bit 0 = 1 and STOP is sent at once, like
  the v1 engine; the rest of that transaction's TX words (up to the one
  with STOP) are thrown away.
- Timing: 8 ticks of 1.25 us per bit = 10 us = 100 kHz, SCL low 5 us, high
  5 us (the I2C standard asks for >= 4.7 and >= 4.0 us).

### 3.5 `manchester_tx` / `manchester_rx`: the new protocol

Manchester code (IEEE 802.3: bit 1 = low -> high in the middle of the bit,
bit 0 = high -> low) puts a transition in every bit, so the receiver can
recover the clock from the data. This is the "one new protocol" of
measurement 1 on slide 24: it needs no new Verilog, only these two programs.

- **Frame:** idle low, a start bit 1 (the sync), 8 data bits LSB first.
- **TX word:** the byte in [7:0]. **RX word:** the byte in [31:24].
- **CRC:** the receiver runs `CRC 8` after every byte, so SMs_CRC holds the
  CRC-16 (POLY 0x1021, INIT 0xFFFF, bits in wire order) of everything
  received since the last CRC reset (EXEC 0xE080). Software computes the
  same CRC over the bytes it sent and compares (`se man`).
- **Clock recovery:** the receiver re-locks on every middle edge, so the two
  ends may differ by several percent.
- **On the board:** SE_LOOP (0xA20 bit 0) connects SM0 pin 1 to SM1 pin 2
  inside the chip, so no wire is needed; the TX line is also visible on JD2
  for the logic analyser.

### 3.6 `oled_spi`: the on-board OLED (SSD1306)

- **TX word:** [31] DC (0 command, 1 data), [29] RES# (0 = reset), [27:20]
  the byte. In hex: `0x2AF00000` = command 0xAF, `0xA5500000` = data 0x55,
  `0x0E300000` = RES# low.
- `OUT pins, 4` sets DC and RES# (and keeps SCLK low) in one instruction,
  then 8 x (`OUT pins, 1` with SCLK low, `JMP` with SCLK high).
- SCLK = 5 MHz (the SSD1306 allows up to 10 MHz).
- The power-up order (VDD on, reset, charge pump, VBAT on, display on) and
  the 5x7 text font are in `sw/se_demos.c` (`se oled TEXT`).

## 4. How the programs are verified

`sim/unit/tb_se_programs.v` (`sh sim/run_unit.sh se_programs`, about 1
minute) runs every program in the real `se_engine` and uses the v1 engines
as reference models, as promised on slide 23:

| part | what is compared | result |
|---|---|---|
| A | `uart_tx` vs v1 `ssc_uart` TX: the line sampled at every bit centre, bit for bit; `uart_rx` receives what the PC model and the v1 UART send | identical, 8680.0 ns per bit |
| B | `spi_master` vs v1 `ssc_spi`, both reading the flash model: MOSI bits sampled on SCK edges, CS# framing, READ ID, WREN, RDSR | identical, 20 BA 19 |
| C | `i2c_master` vs v1 `ssc_i2c`, both reading the TMP2 model at 0x4B, with the I2C bus monitor decoding START / bytes / ACK / repeated START / STOP | same event sequence, 0C 80; SCL high and low 5 us |
| D | `manchester_tx` -> `manchester_rx`, random bytes, CRC-16 in the SM vs a software model | all bytes, CRC equal |
| E | `oled_spi`: DC/RES#/SCLK/SDIN against the expected bit stream | SCLK high >= 100 ns, period >= 200 ns |

The test ends with `ALL 69 CHECKS PASSED`. The way software uses the same
word formats on the board is in `sw/se_demos.c`.

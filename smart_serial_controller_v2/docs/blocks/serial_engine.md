# Serial engine (`rtl/se/se_engine.v`, `rtl/se/se_sm.v`)

Files: `rtl/se/se_engine.v` (registers, program memory, FIFOs, two SMs),
`rtl/se/se_sm.v` (one state machine).
Tools: `tools/se_asm.py` (assembler), `tools/build_programs.sh`.
Programs: `programs/*.se`. Test: `sim/unit/tb_se_engine.v`.
Contract: `docs/SPEC.md`, section 5.

This is the block behind slides 12 and 13. Slide 12 shows the idea: a tiny
processor that does nothing but wiggle and sample pins with exact timing.
Slide 13 shows its "hello world": a UART transmitter in 7 instructions.

This document is also the **programmer's reference**: with it you can
write, assemble, load and debug your own programs.

## 1. What the block does

The v1 controller has three fixed engines: UART, SPI and I2C. A sensor or
chip that speaks anything else (1-Wire, WS2812 LEDs, Manchester code, a
slow parallel bus, a non-standard SPI) needs a new engine, which means new
RTL and a new FPGA build.

The serial engine solves this in software. It has two small **state
machines** (SM0 and SM1). Each one runs a program of 16-bit instructions.
The instructions are made for serial protocols: shift bits out of a register
onto a pin, shift a pin into a register, wait for a pin level, delay an
exact number of ticks, compute a CRC. A new protocol is a new program, not
new hardware. The idea comes from the RP2040 "PIO".

Each SM has:

- a 4-word **TX FIFO** (CPU or network → SM) and a 4-word **RX FIFO**
  (SM → CPU or network), 32 bits wide,
- 4 **pins**, each with `out`, `oe` (output enable) and `in`,
- a **fractional clock divider** that sets its speed,
- registers **X** and **Y**, the shift registers **OSR** and **ISR**, and a
  16-bit **CRC** register.

The two SMs share a **program memory of 32 × 16 bits**, so they can run the
same program or two different ones. A 32-bit `time_us` input lets a program
time-stamp events.

On the board, the crossbar can connect SM0 (source 4) or SM1 (source 5) to
any Pmod port, the OLED port or the LEDs. SM pin i is pin i of the port.

## 2. Block diagram

```
                      +---------------------- se_engine ----------------------+
  CPU register port   | SE_CTRL  SE_FSTAT  SE_CRC_CFG  SE_TIME                |
  (reg_we/re/addr/ -->| per SM: CLKDIV PINCTRL SHIFTCTRL TXF RXF EXEC STATE   |
   wdata, rdata)      |         X Y CRC PINS                                  |
                      |                                                       |
                      |   PROG[0..31] (32 x 16 bit, 3 read ports)             |
                      |      |  prog[pc0]               |  prog[pc1]          |
                      |      v                          v                     |
  ni_tx_push/data0 -->| TX FIFO 0 --> +---------+   TX FIFO 1 --> +---------+ |
  (and SM0_TXF)       |  (4 x 32)     |  se_sm  |    (4 x 32)     |  se_sm  | |
                      |               |   SM0   |                 |   SM1   | |
  ni_rx_pop/data0 <---| RX FIFO 0 <-- +---------+   RX FIFO 1 <-- +---------+ |
  (and SM0_RXF)       |                 |    ^                      |    ^    |
                      |      sm0_out/oe |    | 2-FF sync  sm1_out/oe |    |    |
                      +-----------------|----|----------------------|----|----+
                                        v    sm0_in                 v    sm1_in
                                     crossbar (source 4)         crossbar (source 5)

  Inside one se_sm:

     tick <-- fractional divider (INT + FRAC/256 clocks)
       |
       v
     +----+   instruction   +------------------------------------------+
     | PC |---------------->| decode: JMP WAIT IN OUT PUSH/PULL SET OD  |
     +----+  (or EXEC word) |         CRC; delay / side-set field      |
       ^                    +---+-------+-------+-------+--------+-----+
       |   next PC              |       |       |       |        |
       +------------------------+       v       v       v        v
                                      X, Y    OSR     ISR      pins (out, oe)
                                              + out   + in     CRC unit
                                              count   count    (1 bit/clock)
```

## 3. The programmer's model

What a program can see and change in one SM:

| item | size | what it is |
|---|---|---|
| PC | 5 bits | address of the next instruction (0..31, wraps from 31 to 0) |
| X, Y | 32 bits each | scratch registers: loop counters, data, compare values |
| OSR | 32 bits | output shift register. `PULL` fills it from the TX FIFO, `OUT` shifts bits out of it. |
| out count | 0..32 | bits shifted out of the OSR since the last PULL. 32 = empty. |
| ISR | 32 bits | input shift register. `IN` shifts bits into it, `PUSH` sends it to the RX FIFO. |
| in count | 0..32 | bits shifted into the ISR since the last PUSH |
| CRC | 16 bits | the CRC register (per SM). POLY and INIT are shared by both SMs. |
| pins | 4 × (out, oe, in) | `in` comes through a 2-flip-flop synchroniser |
| delay counter | 5 bits | ticks still to wait after the last instruction |
| TX FIFO, RX FIFO | 4 × 32 bits each | first-word fall-through |

After reset: PC = 0, X = Y = ISR = OSR = 0, out count = 32 (empty),
in count = 0, CRC = 0xFFFF, all `out` = 0, all `oe` = 0, both SMs off.

## 4. Instruction set

### 4.1 Format

Every instruction is 16 bits:

```
 15  13 12       8 7              0
+------+----------+----------------+
|  op  | dly/side |   arguments    |
+------+----------+----------------+
```

The middle field depends on `SIDE_EN` in that SM's PINCTRL register:

| SIDE_EN | bit 12 | bits 11:8 | delay range |
|---|---|---|---|
| 0 | delay bit 4 | delay bits 3:0 | 0..31 |
| 1 | side-set value (driven on SIDE_PIN) | delay | 0..15 |

The **delay** is counted in ticks **after** the instruction completes. So an
instruction with delay `d` takes `1 + d` ticks (plus any stall).

### 4.2 Summary

| op | mnemonic | arguments [7:0] | example | code |
|---|---|---|---|---|
| 0 | `JMP` | [7:5] cond, [4:0] address | `JMP x--, 3` | `0x0043` |
| 1 | `WAIT` | [7] level, [1:0] pin | `WAIT 1, pin 2` | `0x2082` |
| 2 | `IN` | [7:5] source, [4:0] bit count (0 = 32) | `IN pins, 1` | `0x4001` |
| 3 | `OUT` | [7:5] destination, [4:0] bit count (0 = 32) | `OUT pins, 1 [6]` | `0x6601` |
| 4 | `PUSH` / `PULL` | [7] 1 = PULL, 0 = PUSH; [6] block | `PULL block` | `0x80C0` |
| 5 | `SET` | [7:5] destination, [4:0] value | `SET x, 7` | `0xA027` |
| 6 | `OD` | [7] 0 = drive low, 1 = release; [1:0] pin | `OD pin 3, Z` | `0xC083` |
| 7 | `CRC` | [7] 1 = reset; [4:0] bit count (0 = 32) | `CRC 8` | `0xE008` |

All codes in this document come from the assembler (`tools/se_asm.py`).
Bits not listed (for example [6:2] of WAIT) are 0.

### 4.3 `JMP [cond,] target`: jump

Sets PC = address if the condition is true. Otherwise PC + 1.

| cond [7:5] | syntax | jumps when | side effect | example | code |
|---|---|---|---|---|---|
| 0 | (none) | always | | `JMP 0` | `0x0000` |
| 1 | `!x` | X == 0 | | `JMP !x, 0` | `0x0020` |
| 2 | `x--` | X != 0 | X = X - 1 **always** (also when X is 0, it wraps to 0xFFFFFFFF) | `JMP x--, 0` | `0x0040` |
| 3 | `!y` | Y == 0 | | `JMP !y, 0` | `0x0060` |
| 4 | `y--` | Y != 0 | Y = Y - 1 always | `JMP y--, 0` | `0x0080` |
| 5 | `x!=y` | X != Y | | `JMP x!=y, 0` | `0x00A0` |
| 6 | `pin` | input pin JMP_PIN is 1 | | `JMP pin, 0` | `0x00C0` |
| 7 | `!osre` | OSR not empty (out count < 32) | | `JMP !osre, 0` | `0x00E0` |

`SET x, 7` followed by a loop that ends with `JMP x--, loop` runs the loop
**8** times (X = 7, 6, ..., 0).

### 4.4 `WAIT level, pin n`: wait for a pin

Stalls until input pin `n` (0..3, an absolute pin number) equals `level`.
Example: `WAIT 0, pin 2` (`0x2002`) waits for a UART start bit on pin 2.

### 4.5 `IN source, n`: shift n bits into the ISR

| src [7:5] | syntax | the data bits |
|---|---|---|
| 0 | `pins` | the input pins: bit j = pin (IN_BASE + j) mod 4. For n > 4 the 4 pins repeat. |
| 1 | `x` | X |
| 2 | `y` | Y |
| 3 | `null` | zeros |
| 4 | `time` | `time_us` |
| 5 | `crc` | `{16'b0, CRC}` |
| 6 | `isr` | the ISR itself (before the shift) |
| 7 | `osr` | the OSR (it is not changed) |

n = 1..32 (32 is coded as 0). The in count goes up by n and stops at 32.
Example: `IN pins, 1` (`0x4001`) samples one bit; `IN x, 32` (`0x4020`)
copies X into the ISR.

### 4.6 `OUT destination, n`: shift n bits out of the OSR

| dst [7:5] | syntax | where the n bits go |
|---|---|---|
| 0 | `pins` | output pins: bit j → pin (OUT_BASE + j) mod 4. Only bits 0..3 are used. |
| 1 | `x` | X = the bits (zero-extended) |
| 2 | `y` | Y = the bits |
| 3 | `null` | thrown away |
| 4 | `pindirs` | the `oe` bits, same pin mapping as `pins` |
| 5 | `pc` | jump to address = bits [4:0] |
| 6 | `isr` | ISR = the bits, and in count = n |
| 7 | `crc` | feed the n bits into the CRC, in the order they would go out on the wire |

The out count goes up by n and stops at 32. Examples: `OUT pins, 1 [6]`
(`0x6601`), `OUT pindirs, 4` (`0x6084`), `OUT pc, 5` (`0x60A5`),
`OUT crc, 8` (`0x60E8`).

### 4.7 `PUSH` / `PULL`

| syntax | code | what it does |
|---|---|---|
| `PULL` or `PULL block` | `0x80C0` | OSR = next TX FIFO word, out count = 0. If the FIFO is empty: **stall** until a word arrives. |
| `PULL noblock` | `0x8080` | the same, but if the FIFO is empty: OSR = X (out count = 0, no stall) |
| `PUSH` or `PUSH block` | `0x8040` | RX FIFO ← ISR, then ISR = 0 and in count = 0. If the FIFO is full: **stall** until there is room. |
| `PUSH noblock` | `0x8000` | the same, but if the FIFO is full: the word is dropped, ISR and in count are still cleared, and the sticky overflow flag is set (SE_FSTAT bit 28 or 29) |

`PULL noblock` with X as a default value is handy for "send the idle
pattern while there is no data".

### 4.8 `SET destination, value`: load a 5-bit constant

| dst [7:5] | syntax | what happens |
|---|---|---|
| 0 | `pins` | write the value to SET_COUNT pins starting at SET_BASE: bit j → pin (SET_BASE + j) mod 4 |
| 1 | `x` | X = value (zero-extended) |
| 2 | `y` | Y = value |
| 4 | `pindirs` | `oe` of those SET_COUNT pins = the value bits |
| 3, 5, 6, 7 | | nothing (the PC still moves on) |

Examples: `SET pins, 1 [7]` (`0xA701`), `SET x, 31` (`0xA03F`),
`SET pindirs, 0b1011` (`0xA08B`).

### 4.9 `OD pin n, 0|Z`: open-drain pin

Drives pin n (absolute, 0..3) as an open-drain pin, whatever OD_MASK says:
`0` → `oe = 1, out = 0` (pull low); `Z` → `oe = 0` (released, the pull-up
makes it high). Examples: `OD pin 3, 0` (`0xC003`), `OD pin 3, Z`
(`0xC083`). This is what I2C SDA/SCL need.

### 4.10 `CRC n` / `CRC reset`

- `CRC reset` (`0xE080`): CRC = INIT (from SE_CRC_CFG).
- `CRC n` (`CRC 8` = `0xE008`): feed the n newest ISR bits into the CRC, in
  wire order (oldest first). See section 7.

### 4.11 `NOP`

The assembler turns `NOP` into `JMP` to the next address (so it costs one
tick plus its delay). `NOP [5]` is a 6-tick pause.

## 5. Pins

### 5.1 Pin fields

All pin numbers are 0..3. Groups of pins **wrap**: with OUT_BASE = 3, an
`OUT pins, 2` writes pin 3 and then pin 0.

| field (PINCTRL) | used by | meaning |
|---|---|---|
| OUT_BASE | `OUT pins`, `OUT pindirs` | first pin |
| SET_BASE, SET_COUNT | `SET pins`, `SET pindirs` | first pin and number of pins (0 or 5..7 mean 4) |
| IN_BASE | `IN pins` | first pin |
| SIDE_PIN, SIDE_EN | side-set | the side-set pin, and whether bit 12 is side-set |
| JMP_PIN | `JMP pin` | the pin tested |
| OD_MASK | every pin write | pins that behave as open drain |
| INIT_OUT, INIT_OE | RESTART | starting `out` and `oe` |

`WAIT ..., pin n` and `OD pin n` give the pin number in the instruction.

### 5.2 Pin write rules

When an instruction (or side-set) writes value v to a pin:

- **pin in OD_MASK:** v = 0 → `oe = 1, out = 0` (pull low); v = 1 →
  `oe = 0` (release, `out` is kept).
- **any other pin:** `out = v`, `oe` is not changed.
- `pindirs` writes change only `oe`.
- `OD pin n` always uses the open-drain rule.
- If side-set and the instruction write the same pin, **side-set wins**.

A pin is only driven when its `oe` is 1, so a program (or INIT_OE) must set
the direction first. The crossbar puts `oe ? out : Z` on the board pin.

### 5.3 Side-set

With SIDE_EN = 1, every instruction carries one extra bit (bit 12) that is
written to SIDE_PIN **when the instruction starts**, even if it then
stalls. This lets one instruction both move a data bit and toggle a clock
pin, for example an SPI clock: `IN pins, 1 side 1` raises SCK and samples
MISO in the same tick.

### 5.4 Input timing

Every input pin passes a 2-flip-flop synchroniser (reset value 1 = idle
line). A program sees a pin change 2 clocks after it happens. `SMs_PINS[3:0]`
shows the synchronised value.

## 6. Shifting, with worked examples

The direction is set per SM in SHIFTCTRL: OUT_SHIFT_LEFT (bit 0) and
IN_SHIFT_LEFT (bit 1). Both are 0 (right) after reset.

| | right (default): LSB end | left: MSB end |
|---|---|---|
| `OUT dst, n` | data = OSR[n-1:0], then OSR >>= n | data = OSR[31:32-n], then OSR <<= n |
| `IN src, n` | ISR = (ISR >> n) \| (data[n-1:0] << (32-n)) | ISR = (ISR << n) \| data[n-1:0] |
| typical use | LSB-first protocols (UART) | MSB-first protocols (SPI, I2C) |

**OUT, worked example** (the testbench checks these exact values).
The TX FIFO word is `0xA5C30F96`, then `PULL`:

| instruction | right shift: result | OSR after | left shift: result | OSR after |
|---|---|---|---|---|
| `OUT x, 4` | X = `0x6` (OSR[3:0]) | `0x0A5C30F9` | X = `0xA` (OSR[31:28]) | `0x5C30F960` |
| `OUT y, 8` | Y = `0xF9` | `0x000A5C30` | Y = `0x5C` | `0x30F96000` |
| next | `OUT x, 20`: X = `0x000A5C30` | `0x00000000` | `OUT x, 32`: X = `0x30F96000` | `0x00000000` |

**IN, worked example.** X = `0xA5`, then `IN x, 8` and `PUSH`:

- IN right: the RX word is `0xA5000000` (8 bits end up in ISR[31:24]).
- IN left: the RX word is `0x000000A5` (8 bits end up in ISR[7:0]).

**A UART receiver** samples LSB first with `IN pins, 1` eight times and
right shift. The first bit on the wire (bit 0) enters at the top and is
pushed down 7 times, so after 8 bits it is in ISR[24] and the whole byte is
in **ISR[31:24]**. With left shift (MSB first, SPI) the byte is in
**ISR[7:0]**. This is why the SE network interface takes ISR[31:24] or
ISR[7:0] depending on IN_SHIFT_LEFT (SPEC 5.6).

**A transmitter** puts the byte where OUT takes bits first: in [7:0] for
OUT right (UART, `uart_tx.se`), in [31:24] for OUT left (SPI, `spi_rw.se`
in section 13.6).

## 7. CRC

Each SM has a 16-bit CRC register. POLY and INIT are global
(`SE_CRC_CFG`, reset POLY = 0x1021, INIT = 0xFFFF). One bit `b` is processed
as:

```
crc = {crc[14:0], 1'b0} ^ ((crc[15] ^ b) ? POLY : 16'h0000)
```

This is the normal (not reflected) CRC, MSB of the register first. There is
no final XOR; software can add one.

**Which bits are fed:**

- `CRC n` feeds the **n newest ISR bits**, oldest first (the order they came
  in on the wire):
  - IN left: ISR[n-1] down to ISR[0].
  - IN right: ISR[32-n] up to ISR[31].
- `OUT crc, n` feeds the n bits that OUT takes, in OUT order: OSR[0] upward
  for OUT right, OSR[31] downward for OUT left.

**Timing:** the CRC unit does one bit per **system clock**, not per tick.
While it works (n clocks) the SM is stalled (STATE[8] = 1). So `CRC 8`
costs 8 clocks after its own tick; at INT ≥ 9 this is hidden before the
next tick anyway.

**Worked example: CRC-16/CCITT-FALSE of "1".** POLY = 0x1021, INIT = 0xFFFF,
IN left, byte 0x31 (`0011 0001`) in ISR[7:0], then `CRC 8`. The bits go in
MSB first: 0, 0, 1, 1, 0, 0, 0, 1. The simulation shows the CRC register
after each clock:

```
FFFF -> EFDF -> CF9F -> 9F3E -> 3E7C -> 7CF8 -> F9F0 -> E3C1 -> C782
```

The result `0xC782` is CRC-16/CCITT-FALSE("1"). For the whole string
"123456789" the testbench gets the standard check value **0x29B1**, with IN
left and with OUT crc left. With POLY = 0x8005, INIT = 0 it gets 0xFEE8
(CRC-16/UMTS).

**Reflected CRCs** (MODBUS, KERMIT, ...) are computed on LSB-first data.
Feed the bits in wire order (IN right + `CRC 8`, as a UART receiver would
have them), use the normal POLY (0x8005 for MODBUS, 0x1021 for KERMIT) and
the bit-reversed INIT, and bit-reverse the 16-bit result in software. A
Python model of the formula above gives the catalogue values 0x4B37 (MODBUS)
and 0x2189 (KERMIT) this way.

## 8. Timing

### 8.1 Ticks

The fractional divider makes a one-clock **tick** every `INT + FRAC/256`
system clocks (section 9). In each tick the SM does exactly one of these
(in this priority):

1. a CRC is running → nothing (stalled, the tick is lost),
2. an EXEC instruction is pending → run it,
3. the delay counter is not 0 → count it down by one,
4. otherwise → run the instruction at PC.

"Run" means: drive the side-set pin, then check whether the instruction can
complete. `WAIT`, `PULL block` with an empty TX FIFO and `PUSH block` with a
full RX FIFO **cannot**: they stall and are tried again at the next tick.
When an instruction completes, its effects happen at the end of that clock
(pins, registers, PC), and the delay counter is loaded with its delay.

| what | cost |
|---|---|
| any instruction | 1 tick |
| delay `[d]` | d more ticks, after it completes |
| stall (WAIT / PULL / PUSH block) | until the condition is true, checked once per tick |
| `CRC n`, `OUT crc, n` | n system clocks of stall after the instruction's tick |
| EXEC | runs at the next tick (the next clock if the SM is off) |
| enabling an SM | the first instruction runs in the clock right after the SE_CTRL write |

### 8.2 Trace 1: the slide-13 UART sending 'A'

A simulation of `programs/uart_tx.se` with CLKDIV = 108.5 and the byte 0x41
('A') already in the TX FIFO. Clock 0 is the clock right after the SE_CTRL
write that enables SM0. Only the clocks with a tick are shown; TXD is pin 1.

| clock | PC | instruction | what happens | TXD after |
|---|---|---|---|---|
| 0 | 0 | `PULL block` | OSR = 0x41, out count = 0 | 1 (idle) |
| 108 | 1 | `SET x, 7` | X = 7 | 1 |
| 217 | 2 | `SET pins, 0 [7]` | **start bit**; delay = 7 | 0 |
| 325 .. 976 | 3 | (delay) | 7 delay ticks | 0 |
| 1085 | 3 | `OUT pins, 1 [6]` | bit 0 of 0x41 = 1; OSR = 0x20 | 1 |
| 1193 .. 1736 | 4 | (delay) | 6 delay ticks | 1 |
| 1844 | 4 | `JMP x--, bit` | X = 7 ≠ 0: jump to 3; X = 6 | 1 |
| 1953 | 3 | `OUT pins, 1 [6]` | bit 1 = 0; OSR = 0x10 | 0 |
| 2061 .. 2604 | 4 | (delay) | | 0 |
| 2712 | 4 | `JMP x--, bit` | X = 5 | 0 |
| 2821 | 3 | `OUT pins, 1 [6]` | bit 2 = 0 | 0 |

The ticks come 108 and 109 clocks apart in turn (108.5 on average). The
start bit edge is at the end of clock 217 and the bit-0 edge at the end of
clock 1085: **868 clocks = 8 ticks = 8.68 µs**, one bit at 115 200 baud.
Each data bit is `OUT [6]` (7 ticks) + `JMP` (1 tick) = 8 ticks. The stop
bit is `SET pins, 1 [7]` (8 ticks). When bytes are sent back to back,
`JMP loop`, `PULL` and `SET x, 7` add 3 ticks, so a frame takes 83 ticks
(90.06 µs) and the stop bit is 1.375 bits long, which every UART accepts.

### 8.3 Trace 2: side-set, delay, stall and CRC (INT = 1)

A program with `.side_set 1`, SIDE_PIN 0, IN shifting left, ticks on every
clock:

```
0: PULL block   side 1
1: IN   osr, 8  side 0 [1]
2: CRC  8       side 1
3: PUSH block   side 0
4: JMP  0       side 0
```

The TX FIFO is empty at first; the CPU writes 0x31 in clock 2. Each row
shows the values **during** that clock; what an instruction changes shows up
in the next row.

| clock | what the SM does | PC | stalled | pin 0 | ISR | CRC |
|---|---|---|---|---|---|---|
| 0 | `PULL` tries: TX empty → stall (side 1) | 0 | 0 | 1 | 0 | FFFF |
| 1, 2 | `PULL` tries again, stalls | 0 | 1 | 1 | 0 | FFFF |
| 3 | `PULL` completes: OSR = 0x31 | 0 | 1 | 1 | 0 | FFFF |
| 4 | `IN osr, 8 side 0 [1]` runs | 1 | 0 | 1 | 0 | FFFF |
| 5 | delay tick (side 0 now on the pin) | 2 | 0 | 0 | 0x31 | FFFF |
| 6 | `CRC 8 side 1` runs | 2 | 0 | 0 | 0x31 | FFFF |
| 7 .. 14 | CRC unit, one bit per clock; the SM waits | 3 | 1 | 1 | 0x31 | FFFF, EFDF, .. E3C1 |
| 15 | `PUSH side 0` runs (CRC done) | 3 | 0 | 1 | 0x31 | C782 |
| 16 | `JMP 0 side 0` runs; the RX FIFO has 1 word | 4 | 0 | 0 | 0 | C782 |
| 17 | `PULL side 1` tries: stall | 0 | 0 | 0 | 0 | C782 |
| 18 | still stalled, but pin 0 is already 1 | 0 | 1 | 1 | 0 | C782 |

What it shows:

- The **side-set** value reaches the pin at the end of the clock in which the
  instruction first runs, also when it stalls (clock 17 → 18).
- The **delay** `[1]` of `IN` adds one tick (clock 5).
- A word written while `PULL` stalls is taken at the next tick (written in
  clock 2, pulled in clock 3).
- `CRC 8` keeps the SM stalled for 8 clocks (7..14). The next instruction
  runs in clock 15, 9 clocks after the CRC instruction.

### 8.4 Other timing facts (all checked by the testbench)

- **WAIT reaction:** with INT = 1, a `WAIT 1, pin 2` followed by
  `SET pins, 1` changes the output 3.5 clocks (35 ns) after the input
  changes: 2 clocks of synchroniser, the WAIT tick and the SET tick.
- **CRC and slow ticks:** with INT = 20, `CRC 8` (8 clocks) ends before the
  next tick, so it costs nothing extra. `CRC 32` (32 clocks) costs one more
  tick.
- **Disabling** an SM freezes it: PC, stall state, delay and pins hold. When
  it is enabled again it continues where it was.
- **Two SMs enabled in the same SE_CTRL write** with the same divider run in
  lock-step (the testbench checks this for 600 clocks).

## 9. Clock divider

```
f_tick = 100 MHz / (INT + FRAC/256)          INT = 0 counts as 1
SMs_CLKDIV = (INT << 16) | (FRAC << 8)
```

How it works: a counter counts INT clocks between ticks. An 8-bit
accumulator adds FRAC at every tick; each time it overflows, that tick
period is one clock longer. Out of every 256 ticks, FRAC are `INT + 1`
clocks long and the rest are `INT` long. The jitter is at most one clock
(10 ns).

**How to choose INT and FRAC:** `D = 100 MHz / f_tick`, `INT = floor(D)`,
`FRAC = round((D - INT) × 256)`.

**Example 1: UART at 115 200 baud, 8 ticks per bit.** f_tick = 921 600 Hz,
D = 108.507:

| INT | FRAC | CLKDIV | f_tick | error |
|---|---|---|---|---|
| 108 | 130 (exact rounding) | `0x006C8200` | 921 592 Hz | -0.001 % |
| 108 | 128 (slide 13, `uart_tx.se`, boot ROM) | `0x006C8000` | 921 659 Hz | +0.006 % |

With 108.5 the ticks are 108 and 109 clocks long in turn, a bit is exactly
868 clocks = 8.680 µs (ideal 8.6806 µs). The testbench measures 8680.0 ns.

**Example 2: I2C at 400 kHz, 4 ticks per SCL period.** f_tick = 1.6 MHz,
D = 62.5, so INT = 62, FRAC = 128, CLKDIV = `0x003E8000`. Ticks are 62 and
63 clocks long in turn, so every SCL period (4 ticks) is exactly 250 clocks
= 2.5 µs.

**Example 3: SPI at 1 MHz, 4 ticks per SCK period** (`spi_rw.se`):
INT = 25, FRAC = 0, CLKDIV = `0x00190000`, 4 MHz ticks.

Limits: the fastest is a tick on every clock (INT = 0 or 1, FRAC = 0:
100 MHz). The slowest is INT = 65535, FRAC = 255: about 1526 ticks per
second. A CLKDIV write takes effect at the next tick. For long pauses, use
delays (up to 31 ticks) and loops with X or Y.

## 10. EXEC and RESTART

### 10.1 EXEC

Writing an instruction to `SMs_EXEC` makes that SM execute it **at its next
tick**, instead of the instruction at PC.

- The PC does not move, unless the instruction jumps (JMP, `OUT pc`).
- It works whether the SM is enabled or not. On a stopped SM the divider
  ticks every clock, so the instruction runs in the next clock.
- A pending EXEC beats a delay that is being counted (the rest of the delay
  is dropped). A running CRC is finished first.
- If the instruction stalls (for example `PULL` with an empty FIFO) it stays
  pending (`STATE[10] = 1`) and is tried at every tick. A new EXEC write
  replaces it.
- Its delay is loaded as usual. On a stopped SM the delay is only counted
  once the SM is enabled.

Uses:

| want | EXEC this |
|---|---|
| jump to address 12 | `JMP 12` = `0x000C` |
| load X with 5 | `SET x, 5` = `0xA025` |
| load X with a 32-bit value v | write v to TXF, then EXEC `PULL block` (`0x80C0`), then `OUT x, 32` (`0x6020`) |
| reset the CRC | `CRC reset` = `0xE080` |
| drive a pin by hand | `SET pins, 1` (`0xA001`), `OD pin 3, 0` (`0xC003`) |
| try out one instruction on a stopped SM | EXEC it, then read X, Y, PINS and STATE (the PC stays where it is) |

### 10.2 RESTART

Writing 1 to `SE_CTRL[8]` (SM0) or `SE_CTRL[9]` (SM1) restarts that SM:

- PC = START_PC (SHIFTCTRL[12:8]),
- X = Y = ISR = OSR = 0, out count = 32 (empty), in count = 0,
- the delay counter, the stall and a pending EXEC are cleared, and a
  running CRC is aborted (the CRC register itself is kept),
- the pins get INIT_OUT / INIT_OE,
- the clock divider restarts.

The FIFOs are **not** flushed. (Drain the RX FIFO by reading `SMs_RXF`
until SE_FSTAT says empty; a TX word is only removed by a PULL.)

**Careful:** SE_CTRL is one register. Every write also writes the enable
bits [1:0]. So:

| write | effect |
|---|---|
| `0x101` | restart SM0 and run it; SM1 is switched off |
| `0x103` | restart SM0, both SMs run |
| `0x100` | restart SM0 and switch **both** SMs off |
| `0x001` | run SM0 without restart |

The C driver keeps a copy of the enable bits for this (`se_start`,
`se_stop` in `sw/ssc2_driver.c`).

## 11. Registers (region 0x800-0x9FF, `reg_addr[8:0]`)

The CPU address is `0x800 + offset`. SM1's registers are 0x40 above SM0's.
Unused addresses and write-only registers read 0.

| offset | name | access | fields | reset |
|---|---|---|---|---|
| 0x000 | SE_CTRL | R/W | [0] SM0_EN, [1] SM1_EN. Write 1: [8] SM0_RESTART, [9] SM1_RESTART (one-shot, read 0) | 0 |
| 0x004 | SE_FSTAT | RO | see below | 0x000A0000 |
| 0x008 | SE_CRC_CFG | R/W | [15:0] POLY, [31:16] INIT | 0xFFFF1021 |
| 0x00C | SE_TIME | RO | `time_us` | |
| 0x010 + 0x40·s | SMs_CLKDIV | R/W | [31:16] INT, [15:8] FRAC | INT 1, FRAC 0 (0x00010000) |
| 0x014 + 0x40·s | SMs_PINCTRL | R/W | see below | 0 |
| 0x018 + 0x40·s | SMs_SHIFTCTRL | R/W | [0] OUT_SHIFT_LEFT, [1] IN_SHIFT_LEFT, [12:8] START_PC | 0 |
| 0x01C + 0x40·s | SMs_TXF | W | push a word into the TX FIFO (dropped if it is full) | |
| 0x020 + 0x40·s | SMs_RXF | R | pop a word from the RX FIFO (reads 0 if it is empty) | |
| 0x024 + 0x40·s | SMs_EXEC | W | [15:0] instruction to run at the next tick | |
| 0x028 + 0x40·s | SMs_STATE | RO | [4:0] PC, [8] stalled (WAIT / PULL / PUSH stall, or CRC running), [9] enabled, [10] EXEC pending | 0 |
| 0x02C + 0x40·s | SMs_X | RO | X | 0 |
| 0x030 + 0x40·s | SMs_Y | RO | Y | 0 |
| 0x034 + 0x40·s | SMs_CRC | RO | [15:0] CRC | 0xFFFF |
| 0x038 + 0x40·s | SMs_PINS | RO | [3:0] in (synchronised), [7:4] out, [11:8] oe | 0x0000000F |
| 0x100 + 4·i | PROG[i] | R/W | [15:0] instruction i, i = 0..31 (no reset value) | |

So SM0's registers are at offsets 0x010-0x038 (CPU 0x810-0x838), SM1's at
0x050-0x078 (CPU 0x850-0x878), and the program at 0x100-0x17C (CPU
0x900-0x97C).

**SE_FSTAT** (overflow bits clear when SE_FSTAT is read):

| bits | field |
|---|---|
| [2:0] | TX0 level (0..4) |
| [6:4] | RX0 level |
| [10:8] | TX1 level |
| [14:12] | RX1 level |
| [16] | TX0 full |
| [17] | RX0 empty |
| [18] | TX1 full |
| [19] | RX1 empty |
| [24] | SM0 stalled |
| [25] | SM1 stalled |
| [28] | SM0 RX overflow (a `PUSH noblock` dropped a word) |
| [29] | SM1 RX overflow |

**SMs_PINCTRL** (bits 7, 15 and 31:28 are unused and read 0):

| bits | field | meaning |
|---|---|---|
| [1:0] | OUT_BASE | first pin of `OUT pins` / `OUT pindirs` |
| [3:2] | SET_BASE | first pin of `SET pins` / `SET pindirs` |
| [6:4] | SET_COUNT | pins written by SET (0 or 5..7 = 4) |
| [9:8] | IN_BASE | first pin of `IN pins` |
| [11:10] | SIDE_PIN | the side-set pin |
| [12] | SIDE_EN | 1 = bit 12 of every instruction is side-set, delay ≤ 15 |
| [14:13] | JMP_PIN | the pin tested by `JMP pin` |
| [19:16] | OD_MASK | open-drain pins |
| [23:20] | INIT_OUT | `out` after RESTART |
| [27:24] | INIT_OE | `oe` after RESTART |

Example: `0x02200015` (uart_tx) = OUT_BASE 1, SET_BASE 1, SET_COUNT 1,
INIT_OUT = INIT_OE = 0010 (pin 1 is an output, idle high).

## 12. Ports of `se_engine`

```verilog
module se_engine (
    input clk, rst_n, input [31:0] time_us,
    input reg_we, reg_re, input [8:0] reg_addr, input [31:0] reg_wdata, output [31:0] reg_rdata,
    input  [1:0] ni_tx_push, input [31:0] ni_tx_data0, ni_tx_data1, output [1:0] ni_tx_full,
    input  [1:0] ni_rx_pop,  output [31:0] ni_rx_data0, ni_rx_data1, output [1:0] ni_rx_empty,
    output [1:0] out_shift_left, output [1:0] in_shift_left,
    output [3:0] sm0_out, sm0_oe, input [3:0] sm0_in,
    output [3:0] sm1_out, sm1_oe, input [3:0] sm1_in,
    output [1:0] sm_idle,
    output [1:0] rx_not_empty);
```

| port | dir | meaning |
|---|---|---|
| `clk`, `rst_n` | in | 100 MHz clock, asynchronous active-low reset |
| `time_us` | in | µs counter (`ssc2_timebase`), for `IN time` and SE_TIME |
| `reg_we`, `reg_re`, `reg_addr`, `reg_wdata`, `reg_rdata` | | register port (SPEC 0). `reg_re` pops SMs_RXF and clears the SE_FSTAT overflow bits. `reg_rdata` is combinational. |
| `ni_tx_push[s]`, `ni_tx_data0/1` | in | the SE network interface pushes a word into TX FIFO s |
| `ni_tx_full[s]` | out | TX FIFO s is full |
| `ni_rx_pop[s]` | in | the NI pops RX FIFO s |
| `ni_rx_data0/1` | out | front word of RX FIFO s (first-word fall-through, 0 when empty) |
| `ni_rx_empty[s]` | out | RX FIFO s is empty |
| `out_shift_left[s]`, `in_shift_left[s]` | out | SHIFTCTRL bits, so the NI knows where the byte sits |
| `sm0_out`, `sm0_oe`, `sm1_out`, `sm1_oe` | out | pin outputs (registers), to the crossbar sources 4 and 5 |
| `sm0_in`, `sm1_in` | in | pin inputs from the crossbar (synchronised inside) |
| `sm_idle[s]` | out | SM s is off, or stalled on `PULL block` with its TX FIFO empty. The crossbar only switches a port away from an SM while this is 1. |
| `rx_not_empty[s]` | out | RX FIFO s has data (INT2 bits 6 and 7, level) |

**Sharing the FIFOs with the network.** The CPU (`SMs_TXF`, `SMs_RXF`) and
the NI share each FIFO; the push and pop strobes are OR-ed. If both push in
the same clock the CPU's word is taken. The NI never does this, because it
does nothing while `stall` (the APB select) is 1.

`se_sm` (one SM) has the same signals per SM plus the program-memory read
port (`pc_out`, `prog_data`); it is not used on its own.

## 13. Writing, assembling and loading a program

### 13.1 Source syntax

One instruction per line:

```
label: OPCODE args [delay] side v      // comment, or ; comment
```

- Mnemonics and operands are not case-sensitive. Labels are made of
  letters, digits and `_`.
- Numbers can be decimal, `0x..` or `0b..`.
- `[delay]` and `side v` can be anywhere after the opcode.
- A label can stand on its own line; it then names the next instruction.

| mnemonic | syntax | notes |
|---|---|---|
| JMP | `JMP [cond,] target` | cond: `!x x-- !y y-- x!=y pin !osre`. target: a label, or a number = offset inside this program |
| WAIT | `WAIT level, pin n` | also `WAIT level, n` |
| IN | `IN src, n` | src: `pins x y null time crc isr osr`, n = 1..32 |
| OUT | `OUT dst, n` | dst: `pins x y null pindirs pc isr crc`, n = 1..32 |
| PUSH | `PUSH [block\|noblock]` | default block |
| PULL | `PULL [block\|noblock]` | default block |
| SET | `SET dst, v` | dst: `pins x y pindirs`, v = 0..31 |
| OD | `OD pin n, 0\|Z` | also `OD n, 0`; `1` means Z |
| CRC | `CRC n` or `CRC reset` | n = 1..32 |
| NOP | `NOP` | = JMP to the next address |

**Directives** (each on its own line, before the first instruction where
noted):

| directive | meaning |
|---|---|
| `.program name` | program name, used for the output files and C names (default: the file name) |
| `.side_set 1` | the program uses side-set: **every** instruction must have `side 0` or `side 1`, and delays are ≤ 15. `.side_set 0` (the default) = no side-set, delays ≤ 31. Before the first instruction. |
| `.origin N` | load address 0..31 (default 0). Labels become absolute addresses (origin + offset). Before the first instruction. |
| `.clkdiv INT FRAC` | only goes into the C header (`SE_PROG_<NAME>_CLKDIV`) |
| `.pinctrl 0x...` | only goes into the C header. Its SIDE_EN bit must match `.side_set` (error otherwise). |
| `.shiftctrl 0x...` | only goes into the C header. A warning is given if its START_PC is not the origin. |

### 13.2 Running the assembler

```
python3 tools/se_asm.py [-o sw/se_programs.h] [--outdir programs] a.se [b.se ...]
```

For every source file it writes `<name>.hex` and `<name>.lst` (next to the
source, or in `--outdir`). With `-o` it also writes **one** C header with all
the given programs. If any file has an error, **nothing** is written.

To rebuild everything in the project:

```
sh tools/build_programs.sh
```

This assembles every `programs/*.se` into `programs/<name>.hex`,
`programs/<name>.lst` and the shared header `sw/se_programs.h`. Run it after
adding or changing a program. It uses Python 3 and its standard library only.

Errors give the file and line. For example, this file `bad.se`:

```
.side_set 1
loop: PULL block side 0
      SET  pins, 1  [16] side 1
      OUT  pinz, 8 side 0
      JMP  nowhere   side 0
      WAIT 1, pin 2
```

gives:

```
bad.se:3: error: delay (with side-set) 16 is out of range 0..15
bad.se:4: error: unknown OUT destination 'pinz' (use pins x y null pindirs pc isr crc)
bad.se:5: error: unknown label 'nowhere'
bad.se:6: error: missing 'side 0|1' (the program uses .side_set 1)
se_asm: errors found, nothing written
```

### 13.3 Output files

**`.hex`**: one 4-digit hex word per line, only the program's words (no
address). Word i belongs at PROG[origin + i]. Verilog testbenches read it
with `$readmemh` (the unit test does this with `programs/uart_tx.hex`).

**`.lst`**: a listing. The header says the name, origin, size and side-set
mode. Then every source line, with its address (decimal) and code (hex) if
it is an instruction:

```
; se_asm.py listing of programs/uart_tx.se
; program uart_tx, origin 0, 7 words, side_set 0
;
; addr = word index in the program memory (decimal), code = hex
;
; addr code  line  source
              14  .program   uart_tx
              ...
  00   80c0   20  loop: PULL block
  01   a027   21        SET  x, 7
  02   a700   22        SET  pins, 0  [7]
  03   6601   23  bit:  OUT  pins, 1  [6]
  04   0043   24        JMP  x--, bit
  05   a701   25        SET  pins, 1  [7]
  06   0000   26        JMP  loop
```

Read a code like this: `a700` = `101 00111 000 00000` = op 5 (SET),
delay 7, dst 0 (pins), value 0.

**`sw/se_programs.h`**: one array and some defines per program:

```c
/* uart_tx: programs/uart_tx.se (7 words at 0) */
static const uint16_t se_prog_uart_tx[] = {
    0x80c0, 0xa027, 0xa700, 0x6601, 0x0043, 0xa701, 0x0000,
};
#define SE_PROG_UART_TX_ORIGIN    0
#define SE_PROG_UART_TX_LEN       7
#define SE_PROG_UART_TX_CLKDIV    0x006c8000u
#define SE_PROG_UART_TX_PINCTRL   0x02200015u
#define SE_PROG_UART_TX_SHIFTCTRL 0x00000000u
```

### 13.4 Loading and running a program from C

With the driver functions (`sw/ssc2_driver.c`):

```c
#include "ssc2_regs.h"
#include "ssc2_driver.h"
#include "se_programs.h"

/* 1. program memory, 2. SM set-up, 3. pins to a connector, 4. run */
se_load(se_prog_uart_tx, SE_PROG_UART_TX_LEN, SE_PROG_UART_TX_ORIGIN);
se_sm_config(0, SE_PROG_UART_TX_CLKDIV, SE_PROG_UART_TX_PINCTRL,
             SE_PROG_UART_TX_SHIFTCTRL);
ssc_wr(XBAR_PIN_SEL(PORT_JD), SRC_SM0);      /* SM0 pins on Pmod JD */
se_start(1u << 0);                           /* RESTART + enable SM0 */

se_put(0, 'A', 1000);                        /* one byte, 1 ms time-out */
```

The same with plain register writes:

```c
for (i = 0; i < SE_PROG_UART_TX_LEN; i++)
    ssc_wr(SE_PROG(SE_PROG_UART_TX_ORIGIN + i), se_prog_uart_tx[i]);
ssc_wr(SM_CLKDIV(0),    SE_PROG_UART_TX_CLKDIV);
ssc_wr(SM_PINCTRL(0),   SE_PROG_UART_TX_PINCTRL);
ssc_wr(SM_SHIFTCTRL(0), SE_PROG_UART_TX_SHIFTCTRL);
ssc_wr(SE_CTRL, SE_CTRL_EN(0) | SE_CTRL_RESTART(0));     /* 0x101 */
while (ssc_rd(SE_FSTAT) & SE_FSTAT_TX_FULL(0))
    ;                                                     /* wait for room */
ssc_wr(SM_TXF(0), 'A');
```

The order matters: load the program and the configuration **before**
RESTART + enable, because RESTART takes START_PC and the INIT pins from the
registers. The program memory has no reset value. Both SMs can be loaded at
different origins and run at the same time (for example `uart_tx` at 0..6
and `spi_rw` at 8..20).

Instead of the CPU, the network can feed an SM: the SE network interface
(node 0, SPEC 5.6, configured with `NI_CFG[0]`) turns every payload byte of
a packet into one TX FIFO word, and turns RX words into DATA packets. The
boot ROM uses this: hub task 1 sends the temperature to SM0, which sends it
out as two UART bytes.

### 13.5 How to debug a program

- `SMs_STATE`: where is the PC, is it stalled, is an EXEC pending?
- `SMs_X`, `SMs_Y`, `SMs_CRC`, `SMs_PINS`: the registers and the pins.
- `SE_FSTAT`: FIFO levels, and whether an RX word was dropped.
- Stop the SM (`SE_CTRL` EN = 0), then EXEC single instructions and read
  the registers after each one.

### 13.6 A second example: SPI master with side-set

This program was written for this document and checked in simulation
against the SPI flash model (`sim/models/tb_spi_flash.v`). It reads the
flash ID `20 BA 19`, like slide 17 does with the fixed SPI engine.

```
// spi_rw.se - SPI master, mode 0, MSB first, any number of bytes per CS frame
// pins (SPI master role): 0 = CS#, 1 = MOSI, 2 = MISO, 3 = SCK
// TX word: [31:24] = byte to send, [23] = 1 on the last byte of the frame
// RX word: [7:0] = byte received (one per byte sent)
.program   spi_rw
.side_set  1
.origin    8
.clkdiv    25 0           ; 4 MHz ticks, SCK = 4 ticks = 1 MHz
.pinctrl   0x0B101E11     ; OUT 1, SET 0 (1 pin), IN 2, side pin 3, INIT CS# high
.shiftctrl 0x00000803     ; OUT left, IN left, START_PC 8

start: PULL block      side 0     ; wait for the first byte of a frame
       SET  pins, 0    side 0     ; CS# low
byte:  SET  y, 7       side 0
bit:   OUT  pins, 1    side 0 [1] ; MOSI = next bit, SCK low for 2 ticks
       IN   pins, 1    side 1     ; SCK high, sample MISO
       JMP  y--, bit   side 1
       PUSH block      side 0     ; the received byte -> RX FIFO
       OUT  x, 1       side 0     ; X = bit 23 = "last byte"
       JMP  !x, next   side 0
       SET  pins, 1    side 0     ; CS# high: frame done
       JMP  start      side 0
next:  PULL block      side 0
       JMP  byte       side 0
```

Assembled (13 words at 8..20): `80c0 a000 a047 6101 5001 108b 8040 6021
0033 a001 0008 80c0 000a`.

How it works:

- PINCTRL `0x0B101E11`: OUT_BASE 1 (MOSI), SET_BASE 0 with SET_COUNT 1
  (CS#), IN_BASE 2 (MISO), SIDE_PIN 3 (SCK), SIDE_EN 1, INIT_OUT 0001 (CS#
  high), INIT_OE 1011 (pins 0, 1, 3 are outputs).
- OUT shifts left, so the byte goes in [31:24] and leaves MSB first. After
  8 bits, the original bit 23 is at the top of the OSR, and `OUT x, 1` reads
  it as the "last byte" flag.
- IN shifts left, so the received byte is in ISR[7:0] = RX word [7:0].
- SCK is low for 2 ticks (`OUT [1]`) and high for 2 ticks (`IN`, `JMP`):
  1 MHz. MISO is sampled in the tick in which SCK rises; the flash changes
  MISO on the falling edge, so it has been stable for 2 ticks.

From C, reading the flash ID (4 words fit in the 4-word FIFOs; for longer
frames read RX words while you write TX words):

```c
static const uint8_t cmd[4] = { 0x9F, 0x00, 0x00, 0x00 };
uint32_t w;
for (i = 0; i < 4; i++)
    se_put(0, ((uint32_t)cmd[i] << 24) | (i == 3 ? (1u << 23) : 0u), 1000);
for (i = 0; i < 4; i++) {
    se_get(0, &w, 1000);            /* 00, 20, BA, 19 */
    id[i] = (uint8_t)w;
}
```

In the simulation the RX words were `00000000 00000020 000000ba 00000019`,
CS# was low for 37.75 µs with exactly 32 SCK rising edges, and the SCK period
was 1000 ns. A second frame `05 00` (read status) returned `00 00`.

## 14. Design choices

The SPEC leaves these details open. Each is also listed in the header of
`se_sm.v` or `se_engine.v`.

1. **A disabled SM is frozen.** Its PC, registers, delay and pins hold. Its
   divider is parked so that it ticks on every clock: an EXEC on a stopped
   SM runs in the next clock, and the first instruction after SM_EN goes
   high runs one clock after the write. Two SMs enabled by the same write
   run in lock-step. RESTART also restarts the divider.
2. **EXEC beats a running delay** (the rest of the delay is dropped), but a
   running CRC is finished first. A stalled EXEC stays pending and a new
   EXEC write replaces it. RESTART drops a pending EXEC.
3. **OUT pins / OUT pindirs write at most 4 pins** (data bits 0..3).
   `IN pins` with n > 4 reads the 4 pins round and round. SET_COUNT 0 and
   5..7 mean 4.
4. **`OUT crc, n`** feeds the bits in the order OUT takes them (LSB first
   for right shift, MSB first for left shift), and takes n clocks like
   `CRC n`.
5. **`PUSH noblock` with a full RX FIFO** still clears the ISR and the in
   count, so the next word starts clean.
6. **RESTART keeps the CRC register** (the SPEC list of what RESTART resets
   does not include it) but aborts a CRC that is still running.
7. **SET with destination 3, 5, 6 or 7** does nothing (the PC moves on).
8. **Inputs are synchronised in the engine** (2 flip-flops, reset value 1 =
   idle line). The crossbar passes pins straight through, so this is the
   only synchroniser.
9. **The program memory has no reset**, so it can be built from LUT RAM.
   Load it before you enable an SM. The CPU may rewrite it at any time, also
   while an SM runs (the SM then runs the new word at its next fetch).
10. **CPU and NI push in the same clock:** the CPU's word is taken.
11. **SE_CTRL is a plain register for the enable bits.** The RESTART bits
    are one-shot. This follows the SPEC table, but it means every write must
    carry the enable bits you want (section 10.2).

## 15. Size

Yosys `synth_xilinx -flatten`:

| module | LUTs | flip-flops | other |
|---|---|---|---|
| `se_engine` (2 SMs, registers, program memory, 4 FIFOs) | 3076 | 706 | 32 RAM32M (program memory and FIFOs), 108 CARRY4, 142 MUXF7, 29 MUXF8 |
| one `se_sm` alone | 1379 | 256 | 52 CARRY4, 168 MUXF7, 34 MUXF8 |

The whole engine is about 6 % of the Zynq-7020's 53 200 LUTs. Most of an SM
is the variable shifters: OUT and IN shift by any n from 1 to 32, in both
directions, and the result goes to 8 different places. Yosys also lists INV
cells for the active-low reset; Vivado uses one inverter for `rst_n`.

## 16. The testbench (`sim/unit/tb_se_engine.v`)

Everything is done through the register port, the way the CPU would do it:
write the program into PROG[], configure the SM, then run single
instructions with SMs_EXEC or whole programs with the SM enabled. The
`exw` task writes EXEC and then polls STATE until the instruction is done.

| part | what it proves |
|---|---|
| A | reset values, register read-back (unused bits read 0), unused and write-only addresses read 0, PROG[0..31] write/read |
| B | TX FIFO levels and full flag, a 5th word is dropped, PULL returns the words in order, PULL block stalls and completes when a word arrives, PULL noblock copies X |
| C | SET x / y and every JMP condition (taken and not taken), `x--` / `y--` always decrement (also from 0), `!osre` with saturating out count, EXEC does not move the PC |
| D | OUT and IN in both directions (the worked examples of section 6), 48 random cases against a bit-by-bit model, every IN source, IN pins with wrap, OUT isr / pc / y |
| E | SET pins with SET_COUNT and wrap, SET/OUT pindirs, OUT pins, OD_MASK (pull low / release), the OD instruction, side-set (also during a stall, its priority, on an OD pin), a new EXEC replacing a stalled one |
| F | PUSH and RX levels, PUSH noblock overflow flag (cleared by the read), PUSH block stall, `rx_not_empty`, SM1's overflow bit |
| G | CRC-16/CCITT-FALSE("123456789") = 0x29B1 with IN left, IN right (reversed bytes), OUT crc left and right; CRC reset; POLY/INIT changed (0xFEE8); CRC 32 + CRC 5 against a model; CRC n stalls for n clocks (at INT = 1 and INT = 20); a running program computes the CRC of bytes from the TX FIFO |
| H | delays (also 31 and with side-set), the divider (108.5 gives 108/109 alternating, 2.25, 1.5, INT = 0), WAIT with the 2-clock synchroniser, stall status, `sm_idle`, freeze on SM_EN = 0, EXEC on a running SM (waits for the tick, beats a delay, replaces a stalled WAIT), RESTART (every item of 10.2, FIFOs kept, a running delay cleared), two SMs in lock-step |
| I | the `ni_*` FIFO ports, CPU priority when both push, `out/in_shift_left` per SM |
| J | `programs/uart_tx.hex` (the assembler's output) equals the slide-13 encoding and, run at CLKDIV 108.5, sends 41 00 FF 55 A5 3C to a TB UART receiver: start and stop bits right, bit time 8.68 µs ± 1 %, every edge of 'A' at k bit times, back-to-back frames every 83 ticks, `sm_idle` only after the last stop bit |

### How to run it and read the output

```
cd smart_serial_controller_v2
sh sim/run_unit.sh se_engine
# or by hand (from the project folder, the test reads programs/uart_tx.hex):
iverilog -g2005 -Wall -o build/tb_se_engine.vvp sim/unit/tb_se_engine.v \
    rtl/se/se_engine.v rtl/se/se_sm.v rtl/v1/ssc_fifo.v
vvp -n build/tb_se_engine.vvp
```

It runs in about 1 second (1 ms of simulated time). A good run prints:

```
A: reset values and registers
B: TX FIFO, PULL block / noblock
C: SET, JMP conditions
D: OUT / IN shifting
E: pins, OD, side-set
F: PUSH, RX levels, overflow
G: CRC
H: delays, side-set, divider, WAIT, EXEC, RESTART
I: ni_* ports
J: programs/uart_tx.hex at 115200 baud
  measured bit time 8680.0 ns (ideal 8680.6)
ALL 257 CHECKS PASSED
```

- Each letter line starts a part of the table above.
- A failing check prints `FAIL: <what>: got <value>, expected <value>
  (t=...)`, so you see at once which register or pin was wrong. The random
  shift test prints the failing case (`OUT x: w=... n=... left=...`), and
  the UART part prints a wrong frame bit by bit.
- `measured bit time` is the UART bit time from the edges of the 'A' frame.
- The last line is `ALL n CHECKS PASSED` or `m OF n CHECKS FAILED`.
- If the engine hangs, a watchdog prints `FAIL: watchdog time-out, the test
  hung` after 20 ms of simulated time. An EXEC that never finishes prints
  `FAIL: EXEC <code> on SM<s> never finished`.

## 17. Notes for the integrator

- `reg_addr` is `paddr[8:0]` for the 0x800-0x9FF region.
- `sm0_*` / `sm1_*` go to crossbar sources 4 and 5. `sm_idle` goes to the
  crossbar's `src_idle[3]` and `src_idle[4]`. A good `src_in_idle` for the
  SM inputs is all ones (an idle line), which matches the synchroniser's
  reset value.
- `rx_not_empty` drives INT2_STATUS bits 6 and 7 (level).
- The SE network interface (`ni_se`, node 0, SPEC 5.6) connects to the
  `ni_*` ports and to `out_shift_left` / `in_shift_left`.
- `SE_LOOP` (0xA20) in the core can feed SM0 pin 1 to SM1 pin 2, so a
  transmitter on SM0 and a receiver on SM1 can be tested without a wire.

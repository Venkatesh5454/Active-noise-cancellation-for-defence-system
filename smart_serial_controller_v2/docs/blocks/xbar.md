# Pin crossbar (`rtl/xbar/xbar.v`)

The pin crossbar decides which serial engine uses which connector on the
ZedBoard. It is section 6 of `docs/SPEC.md`, and it is the block behind
slide 14: "switch requests at random times; never switch in the middle of a
frame, and never let two engines drive one pin".

## 1. What the block does

The board has six **ports**. Each port has 4 pins:

| port | name | pins 0..3 |
|---|---|---|
| 0 | JA   | Pmod pins 1..4 |
| 1 | JB   | Pmod pins 1..4 |
| 2 | JC   | Pmod pins 1..4 |
| 3 | JD   | Pmod pins 1..4 |
| 4 | OLED | DC, SDIN, RES, SCLK |
| 5 | LED  | LD0..LD3 |

There are seven **sources** that can use a port:

| code | source | note |
|---|---|---|
| 0 | OFF  | pins not driven (Z) |
| 1 | UART | v1 engine |
| 2 | SPI  | v1 engine |
| 3 | I2C  | v1 engine |
| 4 | SM0  | serial engine state machine 0 |
| 5 | SM1  | serial engine state machine 1 |
| 6 | GPIO | the CPU drives the pins from registers |
| 7 | OFF  | same as 0 |

Each port has **one** select register, `PIN_SEL[p]`. It holds the code of
the source on that port. Because every pin has exactly one select, two
sources can never drive the same pin. One source may be chosen by several
ports at once. Its outputs then go to all of them.

When the CPU changes `PIN_SEL[p]`, the crossbar does not switch at once. It
waits until the old source has finished its frame, floats the pins for one
clock, hands the pins to the new source, and gives the new source 8 clocks
of clean "idle" input so its input filters can restart.

The crossbar also measures every switch. `SW_CYCLES[p]` is how long the
switch took. `SW_EDGE[p]` is when the new source first moved a pin.

## 2. Block diagram

```
                 register port (APB region 0x400)
                        |
     +------------------v-------------------------------------------+
     |  PIN_SEL[0..5] + one switch FSM per port                      |
     |  (IDLE -> WAIT_IDLE -> FLOAT -> HAND-OVER -> SETTLE -> IDLE)  |
     |  SW_CYCLES / SW_EDGE counters, XBAR_STATUS, GPIO registers    |
     +---------+-----------------------------------+-----------------+
               | cur[p], state[p]                  | cur[p], state[p]
               v                                   v
   src_out,  +-------------------+       +--------------------------+
   src_oe -->| output mux (x6)   |       | input mux (x5 sources)   |--> src_in
   GPIO_OUT  | port p = nibble of|       | source s = pad_in of the |
   GPIO_OE ->| its source, or 0  |       | lowest connected port    |<-- src_in_idle
             | (OFF / FLOAT)     |       | that selects s, else     |
             +---------+---------+       | src_in_idle              |
                       |                 +------------^-------------+
                       v                              |
                pad_out, pad_oe  ---> IOBUFs ---> pad_in
                                                  |
                                     2-FF sync -> GPIO_IN
```

Only the control (select, state, counters) is stored in flip-flops. The two
data muxes are **pure combinational logic**. A pin value goes through the
crossbar with no clock delay.

## 3. Ports

```verilog
module xbar #(parameter NP = 6) (
    input clk, rst_n,
    input reg_we, reg_re, input [7:0] reg_addr, input [31:0] reg_wdata, output [31:0] reg_rdata,
    input  [19:0] src_out, src_oe,     // sources 1..5, source s at [4*(s-1) +: 4]
    output [19:0] src_in,              // what each source sees on its inputs
    input  [4:0]  src_idle,            // bit s-1: source s is idle (safe to switch away from)
    input  [19:0] src_in_idle,         // level each source input sees while not connected / settling
    output [4*NP-1:0] pad_out, pad_oe,
    input  [4*NP-1:0] pad_in,
    output ev_switch);                 // 1-clock pulse at every hand-over
```

| port | dir | width | meaning |
|---|---|---|---|
| `clk` | in | 1 | 100 MHz clock |
| `rst_n` | in | 1 | asynchronous reset, active low |
| `reg_we` | in | 1 | register write strobe (one clock) |
| `reg_re` | in | 1 | read strobe. Not used: no register here changes when it is read. |
| `reg_addr` | in | 8 | byte offset inside the 0x400 region. Bits [1:0] are ignored. |
| `reg_wdata` | in | 32 | write data |
| `reg_rdata` | out | 32 | read data, combinational from `reg_addr` |
| `src_out` | in | 20 | output level of each source pin, source s at [4(s-1)+3 : 4(s-1)] |
| `src_oe` | in | 20 | output enable of each source pin (1 = drive) |
| `src_in` | out | 20 | what each source sees on its 4 input pins |
| `src_idle` | in | 5 | 1 = source is between frames, safe to switch away from |
| `src_in_idle` | in | 20 | input level a source sees while it is not connected or is settling |
| `pad_out` | out | 4·NP | level for each board pin, port p at [4p+3 : 4p] |
| `pad_oe` | out | 4·NP | drive enable for each board pin (0 = Z) |
| `pad_in` | in | 4·NP | level read back from each board pin |
| `ev_switch` | out | 1 | one-clock pulse in every clock where a port hands over |

`NP` can be 1..8 (the register map has room for 8 ports). The core uses 6.

The pin **roles** (for example "UART pin 1 is TXD") are set in the core, not
here. The crossbar moves whole 4-pin nibbles and does not care what each pin
means.

## 4. Registers

Region 0x400, `reg_addr[7:0]`. Unused addresses, and ports that do not
exist (p ≥ NP), read 0. Writes to read-only registers are ignored.

| offset | name | access | fields |
|---|---|---|---|
| 0x00 + 4p | `PIN_SEL[p]` | R/W | **W:** [2:0] new source, [8] FORCE. **R:** [2:0] current source, [6:4] requested source, [8] switching |
| 0x20 | `XBAR_STATUS` | RO | [NP-1:0] switching, one bit per port |
| 0x40 + 4p | `SW_CYCLES[p]` | RO | clocks from the request to the hand-over of the last switch |
| 0x60 + 4p | `SW_EDGE[p]` | RO | clocks from the request to the first output edge of the new source (0 = no edge yet) |
| 0x80 | `GPIO_OUT` | R/W | 4 bits per port, port p at [4p+3:4p] |
| 0x84 | `GPIO_OE` | R/W | 4 bits per port (1 = drive) |
| 0x88 | `GPIO_IN` | RO | `pad_in`, 4 bits per port, through a 2-flip-flop synchroniser |

Reset values:

| register | reset |
|---|---|
| `PIN_SEL[JA]` | 0x011 (UART) |
| `PIN_SEL[JB]` | 0x022 (SPI) |
| `PIN_SEL[JC]` | 0x033 (I2C) |
| `PIN_SEL[JD]` | 0x000 (OFF) |
| `PIN_SEL[OLED]` | 0x000 (OFF) |
| `PIN_SEL[LED]` | 0x066 (GPIO) |
| everything else | 0 |

"Switching" means the port is in WAIT_IDLE, FLOAT or SETTLE. A write to
`PIN_SEL[p]` while port p is switching is **ignored**. A write of the value
that is already current does **nothing** (no switch, no pulse, SW_EDGE kept).

## 5. How a switch works, step by step

Each port has its own small state machine:

```
          write PIN_SEL[p] != current
  IDLE ------------------------------> WAIT_IDLE ---- old source idle ----+
   ^       (old is OFF / GPIO, or FORCE: go straight to FLOAT)            |
   |                                                                      v
   +--- after 8 clocks --- SETTLE <--- HAND-OVER (current = new) <--- FLOAT
                                       SW_CYCLES, ev_switch           (1 clock, oe = 0)
```

1. **Request.** The CPU writes `PIN_SEL[p]` with a new value. The request
   is stored in "requested" and a counter starts. The counter is 0 in the
   first clock after the write and goes up by 1 every clock.
2. **WAIT_IDLE.** The old source still drives the pins and still sees its
   inputs (it may need them to finish, for example SPI MISO). The port waits
   for `src_idle` of the old source. This state is skipped when the old
   source is OFF (0 or 7) or GPIO (6), or when the write had FORCE (bit 8).
3. **FLOAT.** For exactly one clock all 4 pins have `oe = 0` (and
   `out = 0`). The old source no longer drives, and the new one does not
   drive yet. Nobody sees this port on their inputs.
4. **HAND-OVER.** current = new. This is the first clock in which the new
   source drives the pins. `ev_switch` pulses in this clock, and the
   counter value in this clock is stored in `SW_CYCLES[p]` (readable from
   the next clock).
5. **SETTLE.** For 8 clocks (the hand-over clock and the 7 after it) the new
   source already drives the pins, but it sees `src_in_idle` instead of this
   port's pins. Its input synchronisers and glitch filters restart from the
   idle level, so the switch cannot look like a false start bit or a false
   I2C START.
6. **IDLE.** The pins now also feed the new source's inputs.
7. **First edge.** From the clock after the hand-over on, the crossbar
   watches what the pins really show: for each pin, `oe` and, when `oe` is 1,
   `out`. The first clock in which this changes stores the counter in
   `SW_EDGE[p]`. A change of `out` on a pin that is not driven is not an
   edge. `SW_EDGE` is 0 until the edge happens, and every new request clears
   it to 0.

### Why this never cuts a frame

In WAIT_IDLE the port looks at `src_idle` of the old source. If it is 1 in
clock c, then clock c+1 is the FLOAT clock. So the **last clock in which the
old source drove the pins was an idle clock**. A frame is never cut in half
on the pins.

There is one corner case. If the engine starts a new frame in the very clock
the port floats, that frame never reaches this port at all (it is not cut,
it is simply not there). The crossbar cannot stop an engine from starting.
Software should stop feeding an engine before it moves it to another port.
The random test counts how often this happens (2 times in 60 000 clocks).

### Why two engines can never drive one pin

The output of port p is chosen by a multiplexer controlled by one 3-bit
register, `cur`. A multiplexer has exactly one selected input, so only the
selected source (or nothing) can reach the pin.

## 6. Timing examples

### Idle source: JA from UART (idle) to SM1

The CPU writes `PIN_SEL[0] = 5` in clock "w". Counter values are in the top
row.

```
counter      (w)    0       1       2       3 ... 9    10
state        IDLE   WAIT    FLOAT   SETTLE  SETTLE     IDLE
JA pins      UART   UART    oe=0    SM1     SM1        SM1
UART inputs  JA     JA      idle    idle    idle       idle   (no port selects UART now)
SM1 inputs   idle   idle    idle    idle    idle       JA
ev_switch    0      0       0       1       0          0
PIN_SEL[0]   0x011  0x151   0x151   0x155   0x155      0x055
SW_CYCLES                           <- 2 stored here
```

* Switch time for an idle source: **SW_CYCLES = 2 clocks (20 ns)**.
* From OFF or GPIO, or with FORCE: **1 clock (10 ns)** (no WAIT_IDLE clock).
* The new source sees the port's pins after 8 more clocks (counter 10,
  100 ns after the request).

### Slide-14 demo: JD from SPI to SM0 in the middle of a frame

The SPI master sends 0x9F with SCK = clk/8. Its frame is 68 clocks long
(CS# low, 8 bits, CS# high). SM0 is already running and toggles its pin 1
every 12 clocks. The CPU writes `PIN_SEL[3] = 4` in frame clock 25.

```
frame clock   25     26 ... 67   68      69      70       71      72
counter       (w)    0  ... 41   42      43      44       45      46
state         IDLE   WAIT ...    WAIT    FLOAT   SETTLE   SETTLE  SETTLE
SPI src_idle  0      0  ... 0    1       1       1        1       1
JD pins       SPI    SPI ...     SPI     oe=0    SM0      SM0     SM0, pin 1 toggles
ev_switch     0      0  ... 0    0       0       1        0       0
```

In counter 42 the SPI is idle for the first time, so the next clock is the
FLOAT clock.

* The whole SPI frame reaches JD. Nothing is cut.
* The port waits 42 clocks for the frame end, floats in counter 43 and hands
  over in counter 44: **SW_CYCLES = 44** (440 ns = remaining frame + 2).
* SM0's first pin change after the hand-over is in frame clock 72:
  **SW_EDGE = 46**, 2 clocks after the hand-over.

## 7. Design choices

* **No synchronisers on the data path.** `pad_in` goes straight to
  `src_in`. Every engine already has its own 2-FF synchroniser and glitch
  filter (`ssc_sync_filter`, or the serial engine's input stage). Extra
  flip-flops here would only add delay and change the engines' timing.
* **GPIO_IN has a 2-FF synchroniser.** The CPU reads it through the APB
  bus, so it must be a clean synchronous value. It shows `pad_in` from 2
  clocks earlier.
* **WAIT_IDLE is entered even when the old source is already idle.** This
  follows the SPEC state list exactly and keeps the decision ("is the old
  source idle?") in one place. It costs one clock (idle switch = 2 clocks).
* **SW_EDGE ignores the hand-over clock itself.** At hand-over the pins
  always change from "floating" to "driven by the new source". If that
  counted, SW_EDGE would always equal SW_CYCLES for any source that drives a
  pin. SW_EDGE measures when the new source first *moves* a pin.
* **"Effective output" = {oe, oe & out}.** A change of `out` on a pin
  that is not driven is invisible on the board, so it is not an edge.
* **Switching includes SETTLE.** A write during SETTLE is ignored too, so
  a port always finishes one switch before it starts the next.
* **SW_CYCLES keeps its value until the next hand-over.** SW_EDGE is
  cleared at every new request, because "0" must mean "no edge yet".
* **Source 7 is stored as 7.** It behaves exactly like OFF. A write of 7
  to a port holding 0 counts as "different" and does a harmless 1-clock
  switch. Reading back shows 7.
* **ev_switch is the OR of all ports.** If two ports hand over in the same
  clock there is one pulse. (It feeds the XBAR_SWITCH interrupt bit, which
  only needs "something switched".)
* **Counters are 32 bits and saturate** at 0xFFFFFFFF (about 43 s). The
  counter only runs while it is needed: while switching, or while waiting
  for the first edge.
* **Switching TO a busy source is allowed.** The SPEC protects only the old
  source. If the new source is already in the middle of a frame (because
  another port uses it), the new port shows the rest of that frame.

## 8. Resources

Yosys `synth_xilinx` (NP = 6): about **1137 LUTs and 792 flip-flops**
(54 CARRY4, 238 MUXF7). Most of it is the six 32-bit counters, SW_CYCLES
and SW_EDGE registers and their read multiplexer. This is about 2 % of the
LUTs of a Zynq-7020. The longest path is short (one 8-way mux from
`src_out` to `pad_out`, or from `pad_in` to `src_in`).

## 9. How the testbench proves it works

File: `sim/unit/tb_xbar.v`. Run it with:

```
cd smart_serial_controller_v2
sh sim/run_unit.sh xbar
# or by hand:
iverilog -g2005 -Wall -o build/tb_xbar.vvp sim/unit/tb_xbar.v rtl/xbar/xbar.v
vvp -n build/tb_xbar.vvp
```

It takes about 15 s.

### The parts of the test

Five TB "engines" stand in for UART, SPI, I2C, SM0 and SM1. An engine is
busy (`src_idle = 0`) during a frame and only changes its outputs while it
is busy. At the end of a frame it returns to its own idle pattern.

A **reference model** written from SPEC 6 runs next to the DUT. In **every
clock** the testbench checks:

* the DUT's state for every port (current source and IDLE / WAIT / FLOAT /
  SETTLE) equals the model,
* every pad nibble equals the out/oe of exactly the selected source
  (0 while FLOAT or OFF), so no pin ever shows another source's value,
* every `src_in` nibble equals `pad_in` of the lowest connected port that
  selects the source, or `src_in_idle` (settling and floating ports
  excluded),
* `ev_switch` equals "some port is in its hand-over clock",
* the register read in that clock (a random address) equals the model.

It also checks some **properties directly on the DUT**, without the model,
so a mistake in the model cannot hide a bug:

* **no switch mid-frame:** a port only floats away from a source that was
  idle in the last clock it drove the pins (unless FORCE, OFF or GPIO),
* FLOAT lasts exactly 1 clock with `oe = 0`, and the current source only
  changes right after a FLOAT clock,
* `ev_switch` pulses exactly in the clocks where a port hands over.

Then there are directed parts:

| part | what it shows |
|---|---|
| A | reset values of every register and pad, unused addresses read 0 |
| B | idle switch JA UART → SM1, clock by clock: WAIT, FLOAT, hand-over, 8 settle clocks, SW_CYCLES = 2, write during SETTLE ignored |
| C | SW_EDGE: 0 while nothing moves, an undriven pin change is no edge, the first real edge is latched and kept |
| D | FORCE while SPI is busy: straight to FLOAT, SW_CYCLES = 1 |
| E | JC I2C (busy) → UART: I2C keeps the pins while busy, writes during WAIT_IDLE (with and without FORCE) ignored, SW_CYCLES = wait + 2 |
| F | writing the current value (also with FORCE) does nothing |
| G | SM0 on three ports: outputs on all, input from the lowest connected port, a WAIT port still feeds, FLOAT and SETTLE ports do not |
| H | GPIO_OUT / GPIO_OE on the LED port and on JD, GPIO_IN is 2 clocks late |
| I | source 7 is OFF |
| J | slide-14 demo, JD SPI → SM0 in the middle of a frame |
| K | 60 000 random clocks: frames at random times (some up to 700 clocks), random PIN_SEL writes with and without FORCE (also to ports that do not exist and with junk in unused bits), random GPIO writes, random writes to any address, random `pad_in` and `src_in_idle` |
| L | reset in the middle of activity |

The test was also run against 11 deliberately broken copies of the RTL
(no WAIT_IDLE, FLOAT of 2 clocks, highest port wins instead of lowest, no
SETTLE, SETTLE of 7 clocks, a WAIT port not feeding its source, writes
accepted while switching, GPIO not skipping WAIT, hand-over counted as an
edge, undriven pins counted as edges, counter stopping too early). Every one
of them was caught.

### How to read the output

A passing run ends like this:

```
E: JC I2C -> UART requested mid-frame: waited 32 clocks, SW_CYCLES = 33
SWITCH TIME: idle old source (JA UART -> SM1): SW_CYCLES = 2 clocks (20 ns);
             the new source sees its pins after 8 more settle clocks.
SWITCH TIME: FORCE (JB SPI busy -> SM0) = 1 clock, from GPIO/OFF = 1 clock
SWITCH TIME: slide-14 demo, JD SPI -> SM0 written in clock 25 of a 68-clock SPI
             frame (0x9F, SCK = clk/8): waited 42 clocks for the frame end, SW_CYCLES = 44,
             first SM0 edge on JD at SW_EDGE = 46 (2 clocks after hand-over)
COVERAGE: 1586 switches (380 waited for a frame end, 72 forced while busy), 1478 hand-overs, 1478 ev_switch clocks
COVERAGE: 433 writes ignored while switching, 274 same-value writes, 1018 SW_EDGE latches
COVERAGE: 79075 source-clocks on 2+ ports, 2491 settle-shadow clocks, 2 frames started in a FLOAT clock
ALL 1217806 CHECKS PASSED
```

* **SWITCH TIME** lines are the measured switch times (section 6).
* **COVERAGE** lines show that the random part really hit every case:
  - "waited for a frame end": switches that had to wait in WAIT_IDLE.
  - "forced while busy": FORCE used while the old source was mid-frame.
  - "hand-overs" equals "ev_switch clocks": one pulse per hand-over clock.
  - "writes ignored while switching" and "same-value writes".
  - "source-clocks on 2+ ports": a source used on several ports at once.
  - "settle-shadow clocks": a source whose lower port was settling, so its
    input came from a higher port.
  - "frames started in a FLOAT clock": the corner case of section 5.
  The test fails if any of these counts is too low.
* The last line is `ALL n CHECKS PASSED`, or `m OF n CHECKS FAILED`. Every
  failed check prints a `FAIL:` line with its name and clock number (the
  first 30 are printed).
* If the simulation hangs, a watchdog prints `FAIL: watchdog time-out`.

## 10. Notes for the integrator

* `reg_addr` is the byte offset inside the 0x400 region (`paddr[7:0]`).
* `pad_out`/`pad_oe` go to IOBUFs: pin = `oe ? out : Z`. I2C pins are open
  drain in the core (`out = 0`, `oe` = drive low), so they need pull-ups.
* `src_idle` must be 1 only between frames. The crossbar samples it every
  clock in WAIT_IDLE.
* There is no combinational path from `src_idle` to any output. So the
  core may build `src_idle` from `src_in` (the SPI slave does this).
* `src_in` is combinational from `pad_in`. Every engine input must go
  through the engine's own synchroniser.

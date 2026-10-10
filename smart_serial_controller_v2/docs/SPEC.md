# Smart Serial Controller v2: design specification

This document is the contract between all v2 blocks. Every module must match
the ports, formats and behaviour described here exactly.

## 0. Conventions

- **Language:** Verilog-2001, no SystemVerilog. Every file starts with a
  header comment and then `` `timescale 1ns / 1ps``.
- **Tools:** the code must compile with `iverilog -g2005 -Wall` and must be
  clean under `verilator --lint-only -Wall` (allowed waivers:
  `-Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY`). It must be
  synthesisable by Vivado for the Zynq-7020 at 100 MHz.
- **Clock and reset:** one clock `clk` (100 MHz) and one asynchronous
  active-low reset `rst_n`, used as `always @(posedge clk or negedge rst_n)`.
- **Comments:** short and student-friendly, matching the v1 style in
  `rtl/v1/*.v`.
- **Register port.** Every block that owns registers has the same generic
  port:
  - `reg_we`: write strobe, one clock wide, during the APB access phase.
  - `reg_re`: read strobe, one clock wide. Use it for pop-on-read registers.
  - `reg_addr`: the **byte offset inside the block's region**, word aligned,
    so bits [1:0] are ignored.
  - `reg_wdata`: write data.
  - `reg_rdata`: read data. It is combinational from `reg_addr` and is
    sampled when `reg_re` is high.
  - Unused addresses read 0.
- **CPU priority / `stall`.** The CPU and a NoC network interface (NI) may
  share an engine FIFO.
  - The core drives `stall = psel`: high whenever an APB transfer is in
    progress.
  - An NI **must not** push, pop or issue an engine command in a cycle where
    `stall` is 1.
  - Engines OR the CPU and NI strobes together.

## 1. Node map (2 × 3 mesh)

```
            x=0                  x=1                  x=2
   y=0   node 0  SERIAL ENGINE   node 1  SENSOR HUB   node 2  DMA WRITER
   y=1   node 3  UART            node 4  SPI          node 5  I2C
```

- The node id is `y*W + x`, with W = 3 and H = 2.
- XY routing: move along x (east/west) first, then along y (south is y+1,
  north is y-1).
- The **engine code** used in records and in the crossbar is: 1 UART, 2 SPI,
  3 I2C, 4 SM0, 5 SM1.

## 2. Time base: `rtl/top/ssc2_timebase.v`

```
module ssc2_timebase #(parameter CLK_HZ = 100_000_000,
                       parameter US_PER_MS = 1000) (   // simulations may lower it
    input clk, rst_n,
    output reg [31:0] time_us,   // microseconds since reset (wraps after 71 min)
    output reg        us_tick,   // 1-clock pulse every microsecond
    output reg        ms_tick);  // 1-clock pulse every millisecond
```

## 3. Network-on-chip

### 3.1 Flits

A flit is **34 bits**: `{kind[1:0], data[31:0]}`. The kind is sideband
information, so the payload is 32 bits wide.

| kind | name   | meaning |
|---|---|---|
| 2'b00 | HEAD   | first flit of a packet that has a payload |
| 2'b01 | BODY   | middle payload flit |
| 2'b10 | TAIL   | last payload flit; it carries data too |
| 2'b11 | SINGLE | header-only packet (length 0); it is both head and tail |

Head flit `data[31:0]` (the order matches slide 15):

| bits | field | meaning |
|---|---|---|
| [31:29] | type   | packet type (3.2) |
| [28:26] | dest   | destination node id |
| [25:23] | source | sender node id |
| [22]    | prio   | 1 = urgent (alarm), wins arbitration |
| [21:16] | length | payload bytes, 0..63 |
| [15:8]  | tag    | free for the sender; copied into replies |
| [7:0]   | arg    | "spare": I2C 7-bit address, SPI chip-select, SM index |

Payload rules:

- The payload is packed **little-endian** into BODY/TAIL flits: byte `i` of
  a flit sits at `data[8*i+7 : 8*i]`.
- There are `ceil(length/4)` payload flits. The last one has kind TAIL, and
  its unused bytes are 0.
- length = 0 is sent as one SINGLE flit.

### 3.2 Packet types and payloads

| type | name | payload |
|---|---|---|
| 0 | DATA      | raw bytes. To an engine: bytes to transmit. From an engine: bytes it received. |
| 1 | XFER_REQ  | `[wlen][rlen][w0 .. w(wlen-1)]`: one transaction, write wlen bytes then read rlen bytes. The head `arg` selects the I2C address or SPI chip-select. |
| 2 | XFER_RESP | `[status][r0 .. r(n-1)]`. Status 0 = OK, 1 = NACK, 2 = arbitration lost, 3 = time-out, 4 = bad request. n = rlen if status is 0, otherwise 0. The tag is copied from the request. |
| 3 | RECORD    | 16 bytes, see 3.3 |
| 4 | ALARM     | like DATA, sent with prio = 1 (reserved for hub use) |
| 5-7 | (reserved) | ignored by all NIs, but must still be consumed |

### 3.3 RECORD (16 bytes, little-endian, as the DMA writer stores it)

| byte | field |
|---|---|
| 0..3 | timestamp, in µs (`time_us` when the hub task started) |
| 4 | source: engine code of the responder (1 UART, 2 SPI, 3 I2C, 4 SE) |
| 5 | length: number of valid data bytes (0..8) |
| 6..7 | sequence number (16 bit, the first record is 1) |
| 8..15 | data bytes (unused bytes are 0) |

### 3.4 Links and credits

Each direction of a link carries three signals:

- `valid`: one clock per flit.
- `flit[33:0]`.
- `credit`: a one-clock pulse sent back by the receiver every time it frees
  one input-buffer slot.

The sender keeps a credit counter. It starts at DEPTH (4), the receiver's
buffer size, and the sender may send only while the counter is above 0.
Nothing is ever dropped.

### 3.5 Router: `rtl/noc/noc_router.v`

```
module noc_router #(
    parameter X = 0, parameter Y = 0, parameter W = 3, parameter H = 2,
    parameter IDW = 3, parameter DEST_LSB = 26, parameter PRIO_BIT = 22,
    parameter DEPTH = 4
) (
    input  wire            clk, rst_n,
    input  wire [4:0]      in_valid,     // ports: 0 LOCAL 1 NORTH 2 EAST 3 SOUTH 4 WEST
    input  wire [5*34-1:0] in_flit,      // port p at [34*p +: 34]
    output wire [4:0]      in_credit,    // pulse: a slot of input buffer p was freed
    output wire [4:0]      out_valid,    // registered
    output wire [5*34-1:0] out_flit,     // registered
    input  wire [4:0]      out_credit);  // pulse: the downstream buffer freed a slot
```

The router's ports face these directions:

- NORTH goes to (x, y-1).
- SOUTH goes to (x, y+1).
- EAST goes to (x+1, y).
- WEST goes to (x-1, y).

How the router works:

- **Input buffers:** a DEPTH-flit FIFO per input. Every time a flit leaves
  it, the router pulses `in_credit[p]`.
- **Routing:** the route is computed from HEAD/SINGLE flits as
  `dest = flit[DEST_LSB +: IDW]`, with dx = dest % W and dy = dest / W:
  - dx > X → EAST
  - dx < X → WEST
  - otherwise dy > Y → SOUTH
  - otherwise dy < Y → NORTH
  - otherwise LOCAL

  A dest of W*H or more goes LOCAL.
- **Wormhole switching:** an output, once granted to an input, stays locked
  to that input until that packet's TAIL or SINGLE flit has passed. Body
  flits follow the locked path.
- **Arbitration:** this is done per free output, among the inputs whose
  head flit wants that output. Inputs with `prio` (`flit[PRIO_BIT]`) = 1 win
  first, and round-robin among equals.
- **Throughput:** at most one flit per input and one flit per output each
  cycle. A flit is sent only while that output has a credit.
- **Latency target:** ≤ 3 cycles per hop.
- **Correctness:** no deadlock (XY routing) and no loss or duplication.

### 3.6 Mesh: `rtl/noc/noc_mesh.v`

```
module noc_mesh #(parameter W = 3, parameter H = 2, parameter N = 6,
                  parameter IDW = 3, parameter DEST_LSB = 26,
                  parameter PRIO_BIT = 22, parameter DEPTH = 4) (
    input  wire            clk, rst_n,
    // node -> network (into router LOCAL input)
    input  wire [N-1:0]    tx_valid,
    input  wire [N*34-1:0] tx_flit,
    output wire [N-1:0]    tx_credit,
    // network -> node (from router LOCAL output)
    output wire [N-1:0]    rx_valid,
    output wire [N*34-1:0] rx_flit,
    input  wire [N-1:0]    rx_credit);
```

- N must equal W*H.
- On boundary ports, `in_valid` is tied to 0 and `out_credit` to 0, so the
  starting credits are never refilled. XY routing never uses those ports for
  valid destinations.

### 3.7 Packet helpers (already written and tested): `noc_pkt_tx.v`, `noc_pkt_rx.v`

```
module noc_pkt_tx #(parameter DEPTH = 4) (
    input clk, rst_n,
    input        start,        // take a new packet (only while hdr_ready)
    output       hdr_ready,
    input  [2:0] ptype, dest, src,
    input        prio,
    input  [5:0] len,
    input  [7:0] tag, arg,
    input        b_valid,      // then exactly len payload bytes
    input  [7:0] b_data,
    output       b_ready,
    output reg        out_valid,   // to the router (mesh tx_*)
    output reg [33:0] out_flit,
    input             out_credit);

module noc_pkt_rx #(parameter DEPTH = 4) (
    input clk, rst_n,
    input         in_valid,        // from the router (mesh rx_*)
    input  [33:0] in_flit,
    output        in_credit,
    output        hdr_valid,       // header of the packet at the front
    output [2:0]  ptype, dest, src,
    output        prio,
    output [5:0]  len,
    output [7:0]  tag, arg,
    input         hdr_ready,       // consumer takes the header
    output        b_valid,         // then len payload bytes
    output [7:0]  b_data,
    output        b_last,
    input         b_ready);
```

All NIs, the hub and the DMA writer use these helpers. In every node module,
`in_*` means **from** the network and `out_*` means **to** the network.

## 4. Network interfaces for the fixed engines (`rtl/noc/ni_*.v`)

Common ports for every NI:

- `clk, rst_n`
- `stall`
- `en`
- `my_id[2:0]`
- `in_valid, in_flit[33:0], in_credit`
- `out_valid, out_flit[33:0], out_credit`
- `pkts_in[15:0], pkts_out[15:0]` (counters)

When `en` = 0, an NI must still **consume and discard** every packet that
reaches it, so the network never blocks.

### 4.1 `ni_uart` (node 3)

Extra ports:

```
input [1:0] mode,      // 0 RAW, 1 FRAME, 2 ADDRESSED FRAME
input [2:0] dest, input prio, input [7:0] arg,
input resp_status,     // 1: also send the XFER_RESP status byte to the PC
input [15:0] timeout_us, input us_tick,
output tx_push, output [7:0] tx_data, input tx_full,      // UART engine TX FIFO
output rx_pop,  input [7:0] rx_data, input rx_empty       // UART engine RX FIFO (first-word fall-through)
```

From the PC (UART RX FIFO) to the network:

- **RAW:** collect received bytes and send them as a DATA packet to `dest`
  (with `arg`, `prio`).
  - A packet is sent when 16 bytes have been collected, or when no byte has
    arrived for `timeout_us` µs.
  - The bytes are buffered in the NI, so a packet is only sent when it is
    complete.
- **FRAME:** parse PC frames `[wlen][w bytes][rlen]` (slide 17:
  `01 9F 03`), with wlen 0..60 and rlen 0..60.
  - Send `XFER_REQ` to `dest`, with `arg` and a payload of
    `[wlen][rlen][w...]`.
  - If a frame is still incomplete `timeout_us` µs after its last byte, it
    is thrown away so the parser re-synchronises.
- **ADDRESSED FRAME:** the frame is `[dest][arg][wlen][w...][rlen]`. The
  dest and arg come from the frame instead of from the configuration.

From the network to the PC (all modes): every payload byte of an incoming
packet is pushed into the UART TX FIFO. The one exception is XFER_RESP,
whose status byte is skipped unless `resp_status` = 1.

### 4.2 `ni_spi` (node 4)

Extra ports:

```
output tx_push, output [7:0] tx_data, input tx_full,
output rx_pop, input [7:0] rx_data, input rx_empty,
input  busy,                       // SPI master busy
output active,                     // the NI owns the engine (core forces 8-bit words, manual CS, master)
output cs_level, output [1:0] cs_sel
```

- **XFER_REQ (cs = arg[1:0]):**
  1. Raise `active`, assert CS (`cs_level` = 1), and clock out the wlen
     write bytes and then rlen 0x00 bytes.
  2. The engine returns one byte for each byte sent. Keep the last rlen
     received bytes in a buffer.
  3. Wait for `!busy`, release CS, then send
     `XFER_RESP [0][r...]` to the requester with the same tag.

  Keep at most 16 bytes in flight in the engine FIFOs.
- **DATA (cs = arg[1:0]):** send all the payload bytes in one CS frame,
  throw away the received bytes, and send no reply.
- **A bad request** (for example a payload shorter than 2) gets status 4.

### 4.3 `ni_i2c` (node 5)

Extra ports:

```
output tx_push, output [7:0] tx_data, input tx_full,
output rx_pop, input [7:0] rx_data, input rx_empty,
output cmd_valid, output [7:0] cmd_len, output cmd_read, output cmd_stop,
output abort,
input  busy, input holding, input nack_flag, input arb_flag, input ev_done,
input  [15:0] timeout_us, input us_tick,      // per command
output active, output [6:0] addr7
```

- **XFER_REQ (addr = arg[6:0]):**
  1. If wlen > 0:
     - Push the w bytes. The engine FIFO holds 16, so wlen ≤ 16; a larger
       wlen gets status 4.
     - Issue `cmd(len = wlen, read = 0, stop = (rlen == 0))` and wait for
       `ev_done`.
  2. If NACK or arb is reported: the engine has already sent STOP (v1
     behaviour), so reply with status 1 or 2.
  3. If rlen > 0: issue `cmd(len = rlen, read = 1, stop = 1)` and pop the
     bytes as they arrive (rlen ≤ 60).
  4. Wait for `ev_done`, then reply `XFER_RESP [status][r...]`.
- **Time-out:** if `ev_done` does not come within `timeout_us` per command,
  pulse `abort`, reply status 3 and continue.
- **Commands:** only issue a command while `!busy`.
- **DATA (addr = arg):** one write transaction with all the bytes (≤ 16) and
  STOP. No reply.

### 4.4 `ni_se` (node 0)

Owned by the serial-engine author. See 5.6.

## 5. Serial engine: `rtl/se/se_engine.v`, `rtl/se/se_sm.v`, `rtl/se/ni_se.v`

The engine has two state machines (SM0 and SM1) sharing a 32 × 16-bit
program memory. Each SM has:

- a 4-word TX FIFO and a 4-word RX FIFO,
- 4 pins: `out`, `oe`, `in`,
- a fractional clock divider,
- X and Y registers, OSR and ISR, and a CRC16 register.

There is also a 32-bit µs timestamp input.

### 5.1 Instruction encoding (16 bit)

`[15:13] opcode | [12:8] delay/side-set | [7:0] arguments`

The delay/side-set field depends on the SM's PINCTRL.SIDE_EN:

- SIDE_EN = 1: bit [12] is the side-set value, driven on SIDE_PIN when the
  instruction starts (even if it then stalls), and [11:8] is the delay
  (0..15).
- SIDE_EN = 0: [12:8] is the delay (0..31).
- The delay is counted in SM ticks after the instruction completes.

| op | mnemonic | args |
|---|---|---|
| 0 | JMP  | [7:5] cond, [4:0] address |
| 1 | WAIT | [7] level, [1:0] pin |
| 2 | IN   | [7:5] src, [4:0] bit count (0 = 32) |
| 3 | OUT  | [7:5] dst, [4:0] bit count (0 = 32) |
| 4 | PUSH/PULL | [7] 1 = PULL, 0 = PUSH; [6] block |
| 5 | SET  | [7:5] dst, [4:0] value |
| 6 | OD   | [7] 0 = drive low, 1 = release (Z); [1:0] pin |
| 7 | CRC  | [7] 1 = reset the CRC to INIT (count ignored); [4:0] n (0 = 32) |

The operands:

- **JMP cond:** 0 always, 1 `!x` (X==0), 2 `x--` (jump if X≠0, X is always
  decremented), 3 `!y`, 4 `y--`, 5 `x!=y`, 6 `pin` (input JMP_PIN==1),
  7 `!osre` (OSR not empty: out count < 32).
- **WAIT:** stall until `in[pin] == level`.
- **IN src:** 0 pins (n bits from IN_BASE, pin index wraps mod 4), 1 X, 2 Y,
  3 null (zeros), 4 time (`time_us`), 5 crc (`{16'b0, crc}`), 6 ISR, 7 OSR.
- **OUT dst:** 0 pins (n bits from OUT_BASE, wraps), 1 X, 2 Y, 3 null,
  4 pindirs, 5 PC (jump to data[4:0]), 6 ISR (ISR = data, in count = n),
  7 crc (feed the n bits into the CRC in wire order).
- **SET dst:** 0 pins (SET_COUNT bits from SET_BASE), 1 X, 2 Y, 4 pindirs.
  The value is zero-extended.

**Pin writes:**

- On a pin in OD_MASK, value 0 means oe = 1, out = 0 (pull low), and value 1
  means oe = 0 (release).
- On any other pin, out = value and oe is unchanged.
- PINDIRS writes the oe bits.
- `OD pin, v` forces the open-drain behaviour on that pin whatever OD_MASK
  says.

**Shifting.** In right shift (the default) data leaves or enters at the LSB
end; in left shift, at the MSB end:

- **OUT, right (LSB first):** data = OSR[n-1:0], then OSR >>= n.
- **OUT, left (MSB first):** data = OSR[31:32-n], then OSR <<= n.
- **IN, right:** ISR = (ISR >> n) | (data[n-1:0] << (32-n)). The newest bits
  sit at the top, so 8 bits end up in ISR[31:24].
- **IN, left:** ISR = (ISR << n) | data[n-1:0]. 8 bits end up in ISR[7:0].
- The out count goes up by n (it saturates at 32, and PULL sets it to 0).
  The in count goes up by n (it saturates at 32, and PUSH sets it to 0).

**PUSH/PULL:**

- **PULL:** OSR = TX FIFO word and the out count becomes 0. If the FIFO is
  empty: block = 1 stalls; block = 0 copies X into OSR.
- **PUSH:** RX FIFO ← ISR, then ISR = 0 and the in count becomes 0. If the
  FIFO is full: block = 1 stalls; block = 0 drops the word and sets a sticky
  overflow flag.

**CRC (16 bit):**

- Each bit is processed as
  `crc = {crc[14:0],1'b0} ^ ((crc[15]^bit) ? POLY : 16'h0)`.
- `CRC n` feeds the n most recent ISR bits in wire order (oldest first):
  - IN left: ISR[n-1] down to ISR[0].
  - IN right: ISR[32-n] up to ISR[31].
- It takes n system clocks, during which the SM stalls.
- POLY and INIT are global registers.

**Clock divider:** the SM ticks every INT + FRAC/256 system clocks, using a
fractional accumulator. INT = 0 is treated as 1. Example: INT = 108,
FRAC = 128 gives 921.6 kHz.

**EXEC:** writing SMx_EXEC makes the SM execute that instruction at its next
tick, instead of the one at PC. The PC does not advance unless the
instruction jumps. This works whether the SM is enabled or not.

**RESTART:**

- PC = START_PC, X = Y = ISR = OSR = 0, out count = 32 (empty), in count = 0.
- The delay counter and stall are cleared.
- The pins are set from INIT_OUT/INIT_OE.

The FIFOs are not flushed.

### 5.2 Program example (slide 13; it must run unmodified)

```
loop: PULL block
      SET  x, 7
      SET  pins, 0  [7]
bit:  OUT  pins, 1  [6]
      JMP  x--, bit
      SET  pins, 1  [7]
      JMP  loop
```

### 5.3 Registers (region 0x800–0x9FF, reg_addr[8:0])

| off | name | fields |
|---|---|---|
| 0x000 | SE_CTRL | [0] SM0_EN [1] SM1_EN. W1: [8] SM0_RESTART [9] SM1_RESTART |
| 0x004 | SE_FSTAT (RO) | [2:0] TX0 level [6:4] RX0 level [10:8] TX1 level [14:12] RX1 level [16] TX0 full [17] RX0 empty [18] TX1 full [19] RX1 empty [24] SM0 stalled [25] SM1 stalled [28] SM0 RX overflow [29] SM1 RX overflow (overflows clear on read) |
| 0x008 | SE_CRC_CFG | [15:0] POLY (reset 0x1021), [31:16] INIT (reset 0xFFFF) |
| 0x00C | SE_TIME (RO) | time_us |
| 0x010 + 0x40*s | SMs_CLKDIV | [31:16] INT, [15:8] FRAC (reset INT = 1, FRAC = 0) |
| 0x014 + 0x40*s | SMs_PINCTRL | [1:0] OUT_BASE [3:2] SET_BASE [6:4] SET_COUNT (0 = 4) [9:8] IN_BASE [11:10] SIDE_PIN [12] SIDE_EN [14:13] JMP_PIN [19:16] OD_MASK [23:20] INIT_OUT [27:24] INIT_OE |
| 0x018 + 0x40*s | SMs_SHIFTCTRL | [0] OUT_SHIFT_LEFT [1] IN_SHIFT_LEFT [12:8] START_PC |
| 0x01C + 0x40*s | SMs_TXF (W) | push a word (dropped if full) |
| 0x020 + 0x40*s | SMs_RXF (R) | pop a word (0 if empty) |
| 0x024 + 0x40*s | SMs_EXEC (W) | [15:0] instruction |
| 0x028 + 0x40*s | SMs_STATE (RO) | [4:0] PC, [8] stalled, [9] enabled, [10] exec pending |
| 0x02C + 0x40*s | SMs_X (RO) | |
| 0x030 + 0x40*s | SMs_Y (RO) | |
| 0x034 + 0x40*s | SMs_CRC (RO) | [15:0] |
| 0x038 + 0x40*s | SMs_PINS (RO) | [3:0] in, [7:4] out, [11:8] oe |
| 0x100 + 4*i | PROG[i] | [15:0] instruction i (i = 0..31) |

### 5.4 Ports

```
module se_engine (
    input clk, rst_n, input [31:0] time_us,
    input reg_we, reg_re, input [8:0] reg_addr, input [31:0] reg_wdata, output [31:0] reg_rdata,
    // FIFO ports for ni_se (CPU register access has priority)
    input  [1:0] ni_tx_push, input [31:0] ni_tx_data0, ni_tx_data1, output [1:0] ni_tx_full,
    input  [1:0] ni_rx_pop,  output [31:0] ni_rx_data0, ni_rx_data1, output [1:0] ni_rx_empty,
    output [1:0] out_shift_left, output [1:0] in_shift_left,
    output [3:0] sm0_out, sm0_oe, input [3:0] sm0_in,
    output [3:0] sm1_out, sm1_oe, input [3:0] sm1_in,
    output [1:0] sm_idle,        // !enabled, or stalled on a blocking PULL with the TX FIFO empty
    output [1:0] rx_not_empty);
```

### 5.5 Assembler: `tools/se_asm.py`

- **Syntax:** the slide syntax, `label: OPCODE args [delay] side v // or ; comments`.
- **Directives:**
  - `.side_set 1`: the program uses side-set, so delays are ≤ 15.
  - `.origin N`: the program's load address.
- **Mnemonics:**
  - `JMP [cond,] label`, with cond one of `!x`, `x--`, `!y`, `y--`,
    `x!=y`, `pin`, `!osre`
  - `WAIT level, pin n`
  - `IN src, n`
  - `OUT dst, n`
  - `PUSH [block|noblock]`
  - `PULL [block|noblock]`
  - `SET dst, v`
  - `OD pin n, 0|Z`
  - `CRC n | CRC reset`
  - `NOP` (= `SET y, y`? No: NOP assembles to `JMP` to the next address)
- **Output:** `programs/<name>.hex` (one 4-hex-digit word per line),
  `programs/<name>.lst`, and `sw/se_programs.h` (C arrays with origin and
  length).

### 5.6 `ni_se` (node 0)

```
module ni_se (
    input clk, rst_n, input stall, input en, input [2:0] my_id,
    input [2:0] dest0, dest1, input prio0, prio1, input word4_0, word4_1,
    input [1:0] out_shift_left, in_shift_left,
    output [1:0] tx_push, output [31:0] tx_data0, tx_data1, input [1:0] tx_full,
    output [1:0] rx_pop, input [31:0] rx_data0, rx_data1, input [1:0] rx_empty,
    input in_valid, input [33:0] in_flit, output in_credit,
    output out_valid, output [33:0] out_flit, input out_credit,
    output [15:0] pkts_in, pkts_out);
```

- **Network to SM:** every payload byte of an incoming packet (any type)
  becomes one word in the TX FIFO of SM `arg[0]`. The byte goes in [7:0],
  or in [31:24] when that SM shifts OUT to the left.
- **SM to network:**
  - RX words become DATA packets to destX/prioX, with `arg` = SM index.
  - Each word gives 1 byte, or 4 bytes (little-endian) if `word4`. The one
    byte is ISR[7:0] when IN shifts left and ISR[31:24] when IN shifts
    right.
  - Up to 16 bytes go in one packet. The packet is sent as soon as the RX
    FIFO is empty or 16 bytes have been collected (bytes are buffered first,
    because the header needs the length).

## 6. Pin crossbar: `rtl/xbar/xbar.v`

Ports (pads) are numbered 0 JA, 1 JB, 2 JC, 3 JD, 4 OLED, 5 LED. Each port
has 4 pins: Pmod pins 1..4, the OLED's DC/SDIN/RES/SCLK, or LD0..LD3.

Sources are numbered: 0 OFF (Z), 1 UART, 2 SPI, 3 I2C, 4 SM0, 5 SM1,
6 GPIO (from registers), 7 = OFF.

Pin roles (pin 0..3) are mapped in the core, not in the xbar:

| source | pin 0 | pin 1 | pin 2 | pin 3 |
|---|---|---|---|---|
| UART | CTS# in | TXD out | RXD in | RTS# out |
| SPI (master) | CS0# out | MOSI out | MISO in | SCK out |
| SPI (slave) | CS# in | MOSI in | MISO out | SCK in |
| I2C | – | – | SCL (open drain) | SDA (open drain) |
| SM0 / SM1 | direct | direct | direct | direct |

```
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

**Switching (per port, slide 14):**

1. Writing `PIN_SEL[p]` with a value different from the current one starts
   a switch. A cycle counter starts at 0.
2. **WAIT_IDLE:** the port waits until the old source is idle. It does not
   wait if the old source is OFF or GPIO, or if FORCE (bit 8 of the write)
   is set.
3. **FLOAT:** the port's pins have oe = 0 for exactly 1 clock.
4. **HAND-OVER:** current = new. `SW_CYCLES[p]` = counter, and `ev_switch`
   pulses.
5. **SETTLE:** for 8 clocks the new source sees `src_in_idle` on this
   port's inputs (the filters restart). Its outputs already drive the pins.
6. **First edge:** the first change of the port's effective output after
   hand-over latches `SW_EDGE[p]`, the cycles since the request (0 until it
   happens). A change means a pin's driven level or its oe changed.

While a switch is in progress, writes to that port's PIN_SEL are ignored.

**Other rules:**

- A source may be selected by several ports. Its outputs go to all of them,
  and its inputs come from the lowest-numbered port that selects it and is
  not settling. Otherwise the source sees `src_in_idle`.
- Two sources can never drive the same pin, because each pin has exactly
  one select.

**Reset values:** JA = 1, JB = 2, JC = 3, JD = 0, OLED = 0, LED = 6.

**Registers (region 0x400, reg_addr[7:0]):**

| off | name | fields |
|---|---|---|
| 0x00 + 4p | PIN_SEL[p] | W: [2:0] new source, [8] FORCE. R: [2:0] current, [6:4] requested, [8] switching |
| 0x20 | XBAR_STATUS (RO) | [5:0] switching per port |
| 0x40 + 4p | SW_CYCLES[p] (RO) | |
| 0x60 + 4p | SW_EDGE[p] (RO) | |
| 0x80 | GPIO_OUT | 4 bits per port, [4p+3:4p] |
| 0x84 | GPIO_OE | 4 bits per port |
| 0x88 | GPIO_IN (RO) | pad_in, 4 bits per port |

## 7. Sensor hub: `rtl/hub/hub.v` (node 1)

```
module hub (
    input clk, rst_n, input [31:0] time_us, input ms_tick, input [2:0] my_id,
    input reg_we, reg_re, input [7:0] reg_addr, input [31:0] reg_wdata, output [31:0] reg_rdata,
    input in_valid, input [33:0] in_flit, output in_credit,
    output out_valid, output [33:0] out_flit, input out_credit,
    output ev_error, output ev_record);
```

There are 8 tasks. Each task has 4 words at `0x40 + 16*t + 4*w`.

**W0 TASK_CFG:**

| bits | field |
|---|---|
| [0] | EN |
| [3:1] | dest node |
| [6:4] | packet type (1 = XFER_REQ, 0 = DATA) |
| [7] | prio |
| [15:8] | arg |
| [19:16] | wlen (0..8) |
| [23:20] | rlen (0..8) |
| [24] | RECORD: make a record from a successful response |
| [25] | SEND_LAST: append the last-value bytes |
| [26] | ADDR_INC: w bytes 1..3 are a big-endian address, add 256 after each run |
| [27] | TRIG_RECORDS: period counts records, not ms |
| [28] | APPEND_REC: append the last 16-byte record |

**W1 BYTES0:** write bytes 0..3 (byte 0 in [7:0]).

**W2 BYTES1:** write bytes 4..7.

**W3 TIMING:** [15:0] period (ms or records), [31:16] phase (ms before the
first run; 0 = one period).

**Scheduling:**

- When EN rises, or when a task's W3 is written, that task's countdown
  becomes phase, or period if phase is 0.
- On every ms_tick, the countdown decrements. At 0 the task becomes due and
  the countdown reloads with the period.
- TRIG_RECORDS tasks become due when PERIOD records have been sent since
  their last run.
- Due tasks run one at a time, in index order. `HUB_CTRL[8+t]` (W1) makes
  task t due at once.

**Running a task:**

1. Record `t_start = time_us`.
2. Send the request (from `my_id`) with `tag = {roll[4:0], t[2:0]}`. The
   payload is:
   - XFER_REQ: `[wtotal][rlen][w bytes][last value if SEND_LAST][last record if APPEND_REC]`
   - DATA: the same bytes, without the two length bytes.
3. **XFER_REQ waits for the XFER_RESP with the same tag.** A response with
   any other tag is thrown away. If nothing arrives within HUB_TIMEOUT ms,
   `ev_error` pulses and the time-out counter increments.
4. **On status 0:**
   - If the response has data, last value = the data and last length = n.
   - If RECORD is set, send a RECORD packet to REC_DEST:
     `{t_start, code(src), n, seq, data}`.
   - Then seq++ and `ev_record` pulses.
5. **On status ≠ 0:** `ev_error` pulses, the error counter increments, and
   no record is sent.
6. DATA tasks do not wait for a reply.

**Registers (region 0x600, reg_addr[7:0]):**

| off | name | fields |
|---|---|---|
| 0x00 | HUB_CTRL | [0] EN, W1 [15:8] RUN_NOW |
| 0x04 | HUB_STATUS (RO) | [0] busy, [3:1] task, [8] waiting |
| 0x08 | HUB_REC_DEST | [2:0], reset 2 |
| 0x0C | HUB_TIMEOUT | ms, reset 50 |
| 0x10 | HUB_SEQ (RO) | next sequence number, reset 1 |
| 0x14 | HUB_LAST (RO) | [3:0] length, [15:8] status, [23:16] task |
| 0x18 / 0x1C | HUB_LAST_LO/HI (RO) | |
| 0x20–0x30 | counters (RO) | REQ, RESP_OK, ERR, TMO, REC |
| 0x40.. | task table | |

## 8. DMA writer: `rtl/hub/dma_writer.v` (node 2)

```
module dma_writer (
    input clk, rst_n, input ms_tick, input [2:0] my_id,
    input reg_we, reg_re, input [7:0] reg_addr, input [31:0] reg_wdata, output [31:0] reg_rdata,
    input in_valid, input [33:0] in_flit, output in_credit,
    output out_valid, output [33:0] out_flit, input out_credit,      // never sends: out_valid = 0
    output [31:0] m_axi_awaddr, output [7:0] m_axi_awlen, output [2:0] m_axi_awsize,
    output [1:0] m_axi_awburst, output [3:0] m_axi_awcache, output [2:0] m_axi_awprot,
    output m_axi_awvalid, input m_axi_awready,
    output [31:0] m_axi_wdata, output [3:0] m_axi_wstrb, output m_axi_wlast,
    output m_axi_wvalid, input m_axi_wready,
    input [1:0] m_axi_bresp, input m_axi_bvalid, output m_axi_bready,
    output ev_batch, output ev_overflow, output ev_axi_err);
```

**Behaviour:**

- **RECORD packet:** take the first 16 bytes (pad with 0 if fewer).
  - If the ring is full (`(wr - rd) mod size == size - 1`), drop the record
    and increment OVERFLOW.
  - Otherwise write one AXI4 INCR burst: 4 beats × 32 bit, to
    `BASE + wr*16`.
    - `awlen` = 3, `awsize` = 2, `awburst` = 1, `awcache` = 4'b0011,
      `awprot` = 0, `wstrb` = F.
    - Beat k carries record bytes 4k..4k+3.
  - Wait for B. A non-OKAY response increments AXI_ERR.
  - Then wr = (wr + 1) mod size, COUNT++, PENDING++.
- **Other packet types:** consume them and increment IGNORED.
- **Batch:** when PENDING reaches BATCH, pulse `ev_batch` and set
  PENDING = 0.
- **Time-out:** if PENDING > 0 and TIMEOUT ms have passed since the oldest
  pending record was written, pulse `ev_batch` and set PENDING = 0.
  TIMEOUT = 0 disables this.

**Registers (region 0x700, reg_addr[7:0]):**

| off | name | fields |
|---|---|---|
| 0x00 | DMA_CTRL | [0] EN, W1 [8] RESET (wr = 0, pending = 0, counters = 0) |
| 0x04 | DMA_BASE | |
| 0x08 | DMA_SIZE_LOG2 | [3:0] 1..12, reset 6 |
| 0x0C | DMA_WR_IDX (RO) | |
| 0x10 | DMA_RD_IDX | |
| 0x14 | DMA_BATCH | [7:0], reset 32 |
| 0x18 | DMA_TIMEOUT | ms, reset 1000 |
| 0x1C | COUNT (RO) | |
| 0x20 | OVERFLOW (RO) | |
| 0x24 | AXI_ERR (RO) | |
| 0x28 | PENDING (RO) | |
| 0x2C | STATUS (RO) | [0] busy [1] full [2] empty |
| 0x30 | IGNORED (RO) | |

## 9. Core register map (4 KB, APB, 12-bit address)

| range | block |
|---|---|
| 0x000–0x3FF | v1 registers (ID = 0x53534302; BRIDGE_CTRL has no effect in v2) |
| 0x400–0x4FF | crossbar |
| 0x500–0x5FF | NI configuration (core) |
| 0x600–0x6FF | hub |
| 0x700–0x7FF | DMA writer |
| 0x800–0x9FF | serial engine |
| 0xA00–0xAFF | v2 misc + interrupts (core) |

**NI configuration (0x500):**

`NI_CFG[n]` at `0x500 + 4n`:

| bits | field |
|---|---|
| [0] | EN |
| [2:1] | MODE (UART) |
| [6:4] | DEST |
| [7] | PRIO |
| [15:8] | ARG |
| [16] | RESP_STATUS (UART) |
| [17] | WORD4 SM0 |
| [18] | WORD4 SM1 |
| [22:20] | DEST of SM1 |
| [23] | PRIO of SM1 |

`NI_STAT[n]` at `0x520 + 4n` (RO): [15:0] packets in, [31:16] packets out.
The core counts these itself from the head flits on node n's mesh link, so
they also work for the hub (n = 1) and the DMA writer (n = 2). `NI_CFG[1]` and
`NI_CFG[2]` do not exist (those nodes have their own CTRL registers).

Timeouts:

- `0x540`: UART NI timeout, µs (reset 2000).
- `0x544`: I2C NI timeout, µs (reset 50000).

**Misc (0xA00):**

| off | name | fields |
|---|---|---|
| 0x00 | VERSION (RO) | 0x00020000 |
| 0x04 | TIME_US (RO) | |
| 0x08 | INT2_STATUS (W1C) | 0 DMA_BATCH, 1 DMA_OVERFLOW, 2 DMA_AXI_ERR, 3 HUB_ERROR, 4 HUB_RECORD, 5 XBAR_SWITCH, 6 SE_RX0 (level), 7 SE_RX1 (level) |
| 0x0C | INT2_ENABLE | |
| 0x10 | APB_COUNT (RO, write clears) | number of APB transfers |
| 0x14 | IRQ_COUNT (RO, write clears) | rising edges of irq |
| 0x18 | OLED_PWR | [0] OLED_VDD pin, [1] OLED_VBAT pin (reset 1, 1 = off) |
| 0x1C | BOARD_IN (RO) | [7:0] switches, [12:8] buttons (C, D, L, R, U) |
| 0x20 | SE_LOOP | [0] SM1 pin 2 listens to SM0 pin 1 (Manchester demo without a wire) |

`irq = v1_irq | |(INT2_STATUS & INT2_ENABLE)`.

## 10. Verification required for every block

Each block author writes `sim/unit/tb_<block>.v`:

- It is self-checking and ends with `ALL n CHECKS PASSED` or
  `m OF n CHECKS FAILED`.
- It runs with `iverilog -g2005`.
- It is listed in `sim/run_unit.sh`.

The suite of required checks follows slide 23.

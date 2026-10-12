# SPI and I2C network interfaces (`ni_spi`, node 4 and `ni_i2c`, node 5)

Files: `rtl/noc/ni_spi.v`, `rtl/noc/ni_i2c.v`.
Tests: `sim/unit/tb_ni_spi.v`, `sim/unit/tb_ni_i2c.v`.
Contract: `docs/SPEC.md`, sections 3.1, 3.2, 3.7, 4, 4.2 and 4.3.

These two blocks are behind slides 16 to 18. On slide 17 the PC asks the
SPI flash for its ID through the network ("flow A"). On slides 16 and 18 the
sensor hub reads the PmodTMP2 temperature sensor through the I2C node and
the reply crosses the mesh back to the hub.

## 1. What the blocks do

In v1 only the CPU could use the SPI and I2C engines. The two network
interfaces (NIs) let **any node of the network** use them, without the CPU.
A node sends a request packet; the NI runs the whole bus transaction on the
v1 engine and sends the answer back as a packet.

Both NIs understand the same three kinds of packet:

| packet | payload | what the NI does | reply |
|---|---|---|---|
| XFER_REQ (type 1) | `[wlen][rlen][w0 .. w(wlen-1)]` | one transaction: write wlen bytes, then read rlen bytes | XFER_RESP `[status][r0 .. r(rlen-1)]` |
| DATA (type 0) | raw bytes | write all the bytes in one transaction | none |
| anything else (2..7) | | read and thrown away | none |

The head flit's `arg` byte selects the device: the chip-select for SPI
(`arg[1:0]`), the 7-bit address for I2C (`arg[6:0]`).

The reply goes back to the node that sent the request (its `source` field),
with the **same tag**, so the requester can match it. Status codes:

| status | meaning | used by |
|---|---|---|
| 0 | OK, the read bytes follow | both |
| 1 | NACK: the device did not answer | I2C |
| 2 | arbitration lost (another master was on the bus) | I2C |
| 3 | time-out: the engine did not finish in time | I2C |
| 4 | bad request (wrong lengths) | both |

When the status is not 0 the reply is only the status byte.

**Examples.**

- Slide 17, SPI flash ID: XFER_REQ to node 4, arg 0 (CS0), payload
  `[01][03][9F]` (write 9F, read 3). Reply `[00][20 BA 19]`.
- Slide 18, TMP2 temperature: XFER_REQ to node 5, arg 0x4B, payload
  `[01][02][00]` (write the register pointer 00, read 2). Reply
  `[00][0C 80]` = 25.0 °C.
- SPI write enable: DATA to node 4, arg 0, payload `[06]`. No reply.

## 2. Block diagrams

### 2.1 `ni_spi`

```
                                    ni_spi
            +---------------------------------------------------------------+
 in_valid   |  +-------------+  header, bytes  +--------------------------+  |
 in_flit -->|->| noc_pkt_rx  |---------------->| state machine            |  |
 in_credit<-|<-| (4 flits)   |                 | IDLE WLEN RLEN DROP PREP |  |
            |  +-------------+                 | XFER GAP REPLY RBYTE     |  |
            |                                  |  ns = bytes pushed       |  |
            |                                  |  nr = bytes popped       |  |
            |                                  +---+---------+--------+---+  |
            |     write bytes, then 0x00 fill      |         ^        |      |  tx_push, tx_data
            |     (only while ns - nr < 16)        +---------|--------|----->|-------------> SPI
            |                                                |        |      |  rx_pop    <-- engine
            |     rbuf (64 x 8): the last rlen  <------------+--------|------|<-- rx_data    FIFOs
            |     bytes that came back                                |      |
            |            |                                            v      |  active, cs_level,
            |            v                                                   |  cs_sel -> core forces
            |  +-------------+  [status][r...]                               |  the SPI engine
 out_valid<-|<-| noc_pkt_tx  |<---- (XFER_RESP to the requester)             |
 out_flit   |  +-------------+                                               |  busy <-- engine
 out_credit>|                                                               |
            +---------------------------------------------------------------+
```

### 2.2 `ni_i2c`

```
                                    ni_i2c
            +---------------------------------------------------------------+
 in_*  ---->|  noc_pkt_rx --> state machine                                 |
            |                 IDLE WLEN RLEN DROP WRITE READ ABORT FIN       |
            |                 REPLY RBYTE                                    |
            |                    |        |          |          ^           |
            |   write bytes -----+        |          |          |           |  tx_push/tx_data
            |   (WRITE phase) ------------|----------|----------|---------->|--------------> I2C
            |   cmd(len, read, stop) -----+          |          |           |  cmd_valid/len/   engine
            |                                        |          |           |  read/stop
            |   abort (after an error) --------------+          |           |  abort
            |   per-command timer: us_tick count vs timeout_us  |           |
            |                                                   |           |  busy, holding,
            |   rbuf (64 x 8) <-- rx_pop (READ phase) ----------+-----------|<- nack, arb,
            |        |                                                      |  ev_done, rx_data
            |        v                                                      |
 out_* <----|  noc_pkt_tx <-- XFER_RESP [status][r...]                      |  active, addr7 ->
            |                                                               |  core forces the
            +---------------------------------------------------------------+  I2C engine
```

## 3. Ports

### 3.1 Common NI ports (SPEC 4)

| port | dir | width | meaning |
|---|---|---|---|
| `clk` | in | 1 | 100 MHz clock |
| `rst_n` | in | 1 | asynchronous reset, active low |
| `stall` | in | 1 | 1 = the CPU is doing an APB access. The NI does not push, pop, issue a command or abort in that clock. |
| `en` | in | 1 | NI enable (`NI_CFG[4]` / `NI_CFG[5]` bit 0). 0 = consume and drop every packet, never touch the engine, no reply. |
| `my_id` | in | 3 | own node id (4 or 5), the `source` of the replies |
| `in_valid`, `in_flit` | in | 1, 34 | flits from the network (router LOCAL output) |
| `in_credit` | out | 1 | pulse: one slot of the NI's 4-flit input buffer was freed |
| `out_valid`, `out_flit` | out | 1, 34 | flits to the network (router LOCAL input) |
| `out_credit` | in | 1 | pulse: the router freed one slot |
| `pkts_in` | out | 16 | packets received (every header, also dropped ones) |
| `pkts_out` | out | 16 | replies sent |

### 3.2 Extra ports of `ni_spi` (SPEC 4.2)

| port | dir | width | meaning | connected in the core to |
|---|---|---|---|---|
| `tx_push` | out | 1 | push `tx_data` into the SPI TX FIFO | OR-ed with the CPU strobe |
| `tx_data` | out | 8 | byte to send | muxed with the CPU data (CPU first) |
| `tx_full` | in | 1 | SPI TX FIFO full | `ssc_spi` |
| `rx_pop` | out | 1 | pop the front byte of the SPI RX FIFO | OR-ed with the CPU strobe |
| `rx_data` | in | 8 | front byte of the RX FIFO (first-word fall-through) | `ssc_spi` |
| `rx_empty` | in | 1 | SPI RX FIFO empty | `ssc_spi` |
| `busy` | in | 1 | the SPI master is busy | `ssc_spi` |
| `active` | out | 1 | the NI owns the engine | forces: enabled, master, 8-bit words, manual CS |
| `cs_level` | out | 1 | 1 = assert the chip select | engine `cs_level` while `active` |
| `cs_sel` | out | 2 | which chip select (from `arg[1:0]`) | engine `cs_sel` while `active` |

### 3.3 Extra ports of `ni_i2c` (SPEC 4.3)

| port | dir | width | meaning | connected in the core to |
|---|---|---|---|---|
| `tx_push`, `tx_data`, `tx_full` | | 1, 8, 1 | I2C TX FIFO, as for SPI | OR / mux with the CPU |
| `rx_pop`, `rx_data`, `rx_empty` | | 1, 8, 1 | I2C RX FIFO (first-word fall-through) | OR with the CPU |
| `cmd_valid` | out | 1 | start a command (one clock) | OR-ed with the CPU's CMD write |
| `cmd_len` | out | 8 | bytes to write or read | muxed with the CPU |
| `cmd_read` | out | 1 | 1 = read, 0 = write | muxed |
| `cmd_stop` | out | 1 | 1 = STOP at the end, 0 = keep the bus (repeated START next) | muxed |
| `abort` | out | 1 | one-clock pulse: engine back to idle, let go of SCL/SDA, flush TX | OR-ed with the CPU's abort |
| `busy` | in | 1 | the engine is running a command (or has one pending) | `ssc_i2c` |
| `holding` | in | 1 | the engine keeps the bus after a command without STOP | `ssc_i2c` |
| `nack_flag`, `arb_flag` | in | 1 | result of the command, valid with `ev_done` | `ssc_i2c` |
| `ev_done` | in | 1 | pulse: the command finished | `ssc_i2c` |
| `timeout_us` | in | 16 | time-out per command, in µs | register 0x544 (reset 50000) |
| `us_tick` | in | 1 | one-clock pulse every µs | `ssc2_timebase` |
| `active` | out | 1 | the NI owns the engine | forces: enabled, no auto-write, 7-bit address |
| `addr7` | out | 7 | device address (from `arg[6:0]`) | engine `addr` while `active` |

### 3.4 Registers

The NIs have no registers of their own. The core holds their settings:

| CPU address | name | used bits |
|---|---|---|
| 0x510 | `NI_CFG[4]` (SPI) | [0] EN |
| 0x514 | `NI_CFG[5]` (I2C) | [0] EN |
| 0x544 | I2C NI time-out | µs, reset 50000 (50 ms) |
| 0x530 / 0x534 | `NI_STAT[4]` / `NI_STAT[5]` (RO) | [15:0] packets in, [31:16] packets out |

The SPI clock, the SPI mode and the I2C speed come from the v1 engine
registers, which the CPU (or the boot ROM) sets once. The boot ROM uses
SPI mode 0 at 1 MHz and I2C at 100 kHz.

## 4. How `ni_spi` works

### 4.1 States

| state | what happens |
|---|---|
| IDLE | wait for a header. Take it at once and remember source, tag, arg, prio and length. `cs_sel = arg[1:0]`. Decide: `en = 0` or another type → DROP; DATA → PREP (wlen = length, rlen = 0); XFER_REQ with length < 2 → status 4, DROP; XFER_REQ → WLEN. |
| WLEN | take payload byte 0 = wlen. Check `wlen + 2 == length`. |
| RLEN | take payload byte 1 = rlen. If the length check failed or rlen > 60 → status 4, DROP. Else → PREP. |
| DROP | read and throw away the rest of the payload. Then REPLY (XFER_REQ) or IDLE. |
| PREP | wait until the engine is not busy, popping any old bytes left in the RX FIFO. Then raise `active` and `cs_level` (CS low), clear ns and nr → XFER. |
| XFER | push the wlen write bytes (straight from the packet), then rlen bytes 0x00; pop one returned byte for every byte sent; keep the last rlen of them in `rbuf`. When all bytes are back and the engine is not busy: `cs_level = 0` → GAP. |
| GAP | 8 clocks with CS high, then drop `active` → REPLY (XFER_REQ) or IDLE (DATA). |
| REPLY | start XFER_RESP to the requester: length = rlen + 1 (status 0) or 1, same tag, arg and prio. |
| RBYTE | give the status byte, then the rlen bytes from `rbuf` → IDLE. |

### 4.2 At most 16 bytes in flight

The SPI engine has a 16-byte TX FIFO and a 16-byte RX FIFO. The NI counts
`ns` (bytes pushed) and `nr` (bytes popped) and only pushes while
`ns - nr < 16`. So the TX FIFO, the shifter and the RX FIFO together never
hold more than 16 bytes. Nothing can overflow, and a transfer of any length
(up to 61 write + 60 read bytes) streams through in one CS frame.

### 4.3 Which bytes are kept

For every byte sent the engine returns one byte. Byte number `nr` (counting
from 0) is a read byte when `nr >= wlen`; it is stored at
`rbuf[nr - wlen]`. The bytes that come back during the write part (for the
flash these are just 0x00 or 0xFF) are thrown away. For a DATA packet all
returned bytes are thrown away.

### 4.4 Sharing the engine with the CPU

While `active = 1`, the core forces the SPI engine to: enabled, master,
8-bit words, manual CS, with the NI's `cs_level` and `cs_sel`. The CPU's
own settings come back when `active` drops. The TX/RX FIFO strobes of the
CPU and the NI are OR-ed. The core drives `stall = psel`, and the NI never
pushes or pops while `stall = 1`, so the two can never collide on a FIFO.

## 5. How `ni_i2c` works

### 5.1 States

| state | what happens |
|---|---|
| IDLE | take a header. `addr7 = arg[6:0]`, clear the timer. `en = 0` or another type → DROP. DATA → `active = 1`, WRITE (wlen = length, a write with STOP). XFER_REQ with length < 2 → status 4, DROP. XFER_REQ → WLEN. |
| WLEN | take wlen. Check `wlen <= 16` and `wlen + 2 == length`. |
| RLEN | take rlen. Bad lengths or rlen > 60 → status 4, DROP. Else `active = 1`, timer cleared, and: wlen = 0 and rlen > 0 → READ; otherwise → WRITE. |
| WRITE | push the wlen bytes (only while the engine is not busy, so a CPU command cannot take them). When all are queued and the engine is free: `cmd(len = wlen, read = 0, stop = (rlen == 0))`. Wait for `ev_done`. NACK / arbitration → status 1 / 2, ABORT. OK and rlen > 0 → READ (timer cleared). OK otherwise → FIN. Timer runs out → status 3, ABORT. |
| READ | first throw away old bytes in the RX FIFO. Then `cmd(len = rlen, read = 1, stop = 1)`, pop every byte as soon as it arrives into `rbuf`. When `ev_done` has come and all rlen bytes are popped → FIN. NACK / arbitration → ABORT with 1 / 2. Timer → ABORT with 3. |
| ABORT | one `abort` pulse, `active = 0` → DROP (throws away write bytes not yet taken, then REPLY) |
| FIN | `active = 0` (and an `abort` pulse if the engine still holds the bus, which should not happen) → REPLY or IDLE |
| REPLY, RBYTE | send XFER_RESP `[status][r...]`, as for SPI |

### 5.2 The I2C transaction

For `XFER_REQ 0x4B [01][02][00]` the bus shows (log of the test's bus
monitor):

```
S 96 A 00 A Sr 97 A 0c A 80 N P
```

- `S 96 A`: START, address 0x4B + write, ACK.
- `00 A`: the register pointer. The write command had `stop = 0`, so the
  engine keeps the bus (`holding = 1`).
- `Sr 97 A`: the read command starts with a **repeated START**, address
  0x4B + read.
- `0c A 80 N P`: two data bytes; the master NACKs the last one and sends
  STOP.

Other shapes:

| request | bus |
|---|---|
| wlen > 0, rlen = 0 | `S addr+W A w0 A .. P` (one write command with STOP) |
| wlen = 0, rlen > 0 | `S addr+R A r0 A .. N P` (one read command) |
| wlen = 0, rlen = 0 | `S addr+W A P`: an **address probe**. Status 0 = a device answered, 1 = NACK. |
| DATA, n bytes | `S addr+W A b0 A .. P` |

### 5.3 Errors and time-out

- **NACK** (no device, or the device refuses a byte) and **arbitration lost**:
  the v1 engine has already sent STOP or let go of the bus. The NI pulses
  `abort` once (this also flushes write bytes the engine did not send, so
  they cannot leak into the next transaction) and replies status 1 or 2.
- **Time-out:** each command has its own timer. It counts `us_tick` pulses
  from the start of the phase (waiting for the engine, pushing bytes, the
  command itself). When it reaches `timeout_us` without `ev_done`, the NI
  pulses `abort` and replies status 3. Because `us_tick` comes once per µs,
  the real time-out is between `timeout_us - 1` and `timeout_us` µs (the
  test measures 49.1 µs for 50). `timeout_us = 0` times out at once.
- A command is only issued while `busy = 0` (and `stall = 0`). If the CPU is
  using the engine, the NI waits; if the CPU's command never finishes, the NI
  times out, aborts it and replies 3.

### 5.4 Sharing the engine with the CPU

While `active = 1` the core forces the I2C engine to: enabled, no
auto-write, 7-bit address = `addr7`. The FIFO strobes, `cmd_*` and `abort`
are OR-ed / muxed with the CPU's, and the NI never drives them while
`stall = 1`.

## 6. Timing examples

These traces come from a small testbench that wires each NI to the real v1
engine exactly like the core does, at the board's speeds (SPI 1 MHz, I2C
100 kHz), with `stall = 0` and an always-ready reply collector. Clock 0 is
the clock in which the request's HEAD flit reaches the NI.

### 6.1 SPI: slide 17, `XFER_REQ cs0 [01][03][9F]`

| clock | event |
|---|---|
| 0 | HEAD flit `0_30835A00` arrives: XFER_REQ, dest 4, src 1, len 3, tag 5A, arg 00 |
| 1 | the header is at the front of `noc_pkt_rx`; the NI takes it → WLEN |
| 4 | TAIL flit `2_009F0301` arrives: bytes 01, 03, 9F |
| 5 | wlen = 1 taken (1 + 2 = length 3: OK) → RLEN |
| 6 | rlen = 3 taken (≤ 60: OK) → PREP |
| 7 | PREP: the engine is not busy and the RX FIFO is empty |
| 8 | XFER: `active = 1`, `cs_level = 1`; `tx_push 9F` (the write byte) |
| 9 | CS0# goes low; `tx_push 00` |
| 10, 11 | `tx_push 00`, `tx_push 00` (4 bytes in flight) |
| 10 | the engine is busy |
| 861 | `rx_pop 00` (the byte that came back during 9F: thrown away) |
| 1761 | `rx_pop 20` → `rbuf[0]` |
| 2661 | `rx_pop BA` → `rbuf[1]` |
| 3561 | `rx_pop 19` → `rbuf[2]` |
| 3660 | all 4 bytes are back and the engine is idle (`busy = 0`) → GAP, `cs_level = 0` |
| 3662 | CS0# goes high |
| 3669 | after 8 gap clocks: `active = 0` → REPLY |
| 3671 | reply HEAD `0_46045A00`: XFER_RESP, dest 1, src 4, len 4, tag 5A |
| 3676 | reply TAIL `2_19BA2000`: bytes 00 20 BA 19 |

At 1 MHz one byte comes back every 900 clocks (9 µs: 8 SCLK periods plus
the engine's set-up and hold half-periods). The whole request, from the
head flit in to the reply's tail flit out, takes **3676 clocks = 36.8 µs**.
The NI itself adds about 10 clocks at the start and 16 at the end; the rest
is the SPI bus.

### 6.2 I2C: slide 18, `XFER_REQ 0x4B [01][02][00]`

| clock | event |
|---|---|
| 0 | HEAD flit `0_3483214B` arrives: XFER_REQ, dest 5, src 1, len 3, tag 21, arg 4B |
| 1 | header taken → WLEN |
| 4 | TAIL flit `2_00000201`: bytes 01, 02, 00 |
| 5, 6 | wlen = 1, rlen = 2 taken, both OK |
| 7 | → WRITE: `active = 1`, `tx_push 00` (the register pointer) |
| 8 | `cmd len 1 read 0 stop 0` |
| 9 | the engine is busy: `S 96 A 00 A` on the bus |
| 18923 | `ev_done` (no NACK); `holding = 1` (no STOP) |
| 18924 | → READ: `cmd len 2 read 1 stop 1` (repeated START, `97`) |
| 37089 | `rx_pop 0C` |
| 46170 | `rx_pop 80` |
| 48437 | `ev_done` after NACK + STOP |
| 48438 | → FIN; 48439: `active = 0` → REPLY |
| 48441 | reply HEAD `0_4683214B`: XFER_RESP, dest 1, src 5, len 3, tag 21 |
| 48445 | reply TAIL `2_00800C00`: bytes 00 0C 80 |

At 100 kHz each byte with its ACK takes 9 SCL periods = 90 µs. The whole
read takes **48445 clocks = 484 µs**. The NI adds about 8 clocks at the
start and 8 at the end; the reply then needs 6 clocks of network to reach
the hub (see `docs/blocks/noc.md`, section 6).

## 7. Design choices

The SPEC leaves these details open. They are also listed in the module
headers.

**Both NIs:**

1. **The reply copies the request's prio and arg**, and comes from `my_id`.
   The tag is copied as the SPEC asks.
2. **`en` is looked at when a header arrives.** A transfer that has already
   started is finished even if `en` goes to 0. With `en = 0` an XFER_REQ
   gets no reply (the packet is "consumed and discarded"); the requester's
   own time-out handles that.
3. **A bad request never touches the engine.** The rest of its payload is
   read and dropped, and the reply is `[04]`.
4. **Reserved and other types** (XFER_RESP, RECORD, ALARM, 5..7) are read
   and dropped, so the network behind the NI never blocks.
5. **Old bytes in the RX FIFO are thrown away** before a transfer reads
   anything (left there by the CPU, for example), so they cannot be taken
   for answers.
6. **`pkts_in` counts every header**, also dropped ones; `pkts_out` counts
   replies.

**`ni_spi`:**

7. **The length check is exact:** an XFER_REQ must have exactly
   `2 + wlen` payload bytes. Longer or shorter gets `[04]`. The largest
   transfer is wlen = 61 (a 63-byte packet) and rlen = 60.
8. **CS gap:** after CS is released, `active` stays 1 for 8 more clocks,
   so two NI frames in a row always have at least 80 ns of CS high (the
   test measures 100 ns).
9. **A DATA packet with no payload does nothing.** An XFER_REQ with
   wlen = rlen = 0 gives a one-clock CS pulse (no SCLK) and the reply `[00]`.
10. **The NI waits for `!busy` before it releases CS**, so the CS hold time
    after the last SCLK edge is at least one SCLK period (the test measures
    120 ns at 10 MHz).

**`ni_i2c`:**

11. **wlen ≤ 16** (the engine's TX FIFO), rlen ≤ 60. The read bytes are
    popped as they arrive, so a 60-byte read works with a 16-byte RX FIFO.
12. **A DATA packet longer than 16 bytes is still one write transaction.**
    The command is issued when 16 bytes are queued, and the rest are pushed
    while the engine sends. If the engine runs out of bytes it holds SCL
    low, which is a legal I2C pause.
13. **Every failed transaction ends with exactly one `abort` pulse.** It
    flushes unsent write bytes and leaves the engine idle and the bus free.
14. **The time-out is per command** (write phase and read phase each get
    `timeout_us`), and it fires between `timeout_us - 1` and `timeout_us`
    µs after the phase started.
15. **wlen = rlen = 0 is an address probe** (`S addr A/N P`).

## 8. Size

Yosys `synth_xilinx -flatten`, each NI with its `noc_pkt_tx` and
`noc_pkt_rx`:

| module | LUTs | flip-flops | other |
|---|---|---|---|
| `ni_spi` | 447 | 241 | 6 RAM32M (4-flit input buffer), 3 RAM64M (64-byte `rbuf`), 50 CARRY4 |
| `ni_i2c` | 480 | 255 | 6 RAM32M, 3 RAM64M, 47 CARRY4 |

About 270 LUTs and 135 flip-flops of each are the two packet helpers
(`noc_pkt_tx` 232 / 116, `noc_pkt_rx` 40 / 19). So the SPI NI's own logic
is about 175 LUTs and the I2C NI's about 210 LUTs. Yosys also lists INV
cells for the active-low reset; Vivado uses one inverter for `rst_n`.

## 9. The testbenches

### 9.1 `tb_ni_spi.v`

Set-up (the same wiring as `ssc2_core.v`):

```
  TB requester --noc_pkt_tx--> ni_spi --noc_pkt_rx--> TB reply collector
                                  |
           CPU strobes (idle) --OR/mux--> ssc_spi --> tb_spi_flash  (CS0)
                                             \-----> tb_spi_slave  (CS1)
```

- The real v1 `ssc_spi` runs at 10 MHz SCLK. `tb_spi_flash` answers `9F`
  with `20 BA 19`, has a status register, write enable and page program.
- The "CPU" settings of the engine are deliberately wrong (disabled, slave
  mode, 16-bit words), to prove that `active` forces the right ones.
- A stall generator asserts `stall` at random (light, or heavy with
  30-clock bursts). The reply collector is ready only 3 clocks in 4, or
  blocked completely.

| test | what it proves |
|---|---|
| 1 | slide 17: `[01][03][9F]` → `[00][20 BA 19]` to the requester (node 3), tag 5A; one CS-low window, 32 SCLK edges, CS released and engine handed back |
| 2 | DATA `[06]`: one CS frame of 8 clocks, the flash's write-enable latch is set, no reply |
| 3 | DATA page program, 44 bytes in **one** CS frame (352 SCLK edges); the in-flight limit of 16 is reached and never passed; the returned bytes are thrown away |
| 4 | XFER_REQ `[01][01][05]` status polling until the flash is ready |
| 5 | wlen 4 + rlen 40 (44 bytes): one CS window, at most 16 in flight, the 41-byte reply |
| 6 | the longest read, rlen 60: a 61-byte reply |
| 7 | DATA to CS1 (a generic slave), 20 and 8 bytes back to back: bytes arrive in order, two CS1 windows, CS0 untouched |
| 8 | the CPU sends 3 bytes itself and leaves the answers in the RX FIFO; the next NI transfer throws them away first and its reply is still correct |
| 9 | bad requests (payload of 1, no payload, wlen too big, rlen 61, one byte too many) → `[04]`, no CS, no push, `active` never raised |
| 10 | RECORD, ALARM, reserved and a stray XFER_RESP: consumed, no reply, engine untouched |
| 11 | `en = 0`: packets consumed and dropped, no reply, engine untouched, all credits back; then `en = 1` works again |
| 12 | heavy stall + 3 back-to-back requests while the reply path is blocked: nothing is lost, three separate CS frames |
| 13 | counters and global monitors |

The **global monitors** run on every clock and must stay at 0: an engine
strobe while `stall = 1`, an SCLK edge while every CS is high, a FIFO
overflow (or TX + RX above 16), a reply head flit before CS is released.
The shortest CS-high gap between frames (≥ 80 ns) and the shortest CS hold
after the last SCLK edge (≥ 100 ns) are measured and checked.

### 9.2 `tb_ni_i2c.v`

Set-up:

```
  TB requester --noc_pkt_tx--> ni_i2c --noc_pkt_rx--> TB reply collector
                                  |
         CPU strobes --OR/mux--> ssc_i2c ==SCL/SDA== tb_i2c_slave 0x4B (TMP2)
                                 (open drain,         tb_i2c_monitor (bus log)
                                  pull-ups)           tb_i2c_master  (2nd master)
                                                      "rogue" device holding SCL
```

- The real v1 `ssc_i2c` runs at 1 MHz (100 kHz for the arbitration test, to
  match the second master model). `tb_i2c_slave` behaves like the PmodTMP2:
  registers 0 and 1 hold `0C 80`.
- The "CPU" settings are deliberately wrong (disabled, auto-write on,
  10-bit address 0x2A5), to prove that `active` forces the right ones.
- `tb_i2c_monitor` writes the bus log (`S 96 A 00 A Sr 97 A ...`).
- The same stall generator and reply collector as for SPI.

| test | what it proves |
|---|---|
| 1 | slides 16-18: `0x4B [01][02][00]` → `[00][0C 80]`, bus log `S 96 A 00 A Sr 97 A 0c A 80 N P` (repeated START); engine handed back, bus free |
| 2 | write only `[03][00][02 AA BB]` → `[00]`; the TMP2 registers 2 and 3 are written |
| 3 | write then read back → `[00][AA BB]` |
| 4 | read only `[00][04]` → `[00][BB 44 45 46]`, a single read command |
| 5 | rlen 60 with heavy stall: one transaction (S, 00, Sr, 97, 60 bytes, P), bytes popped as they arrive. 5b: a 300 µs stall fills the RX FIFO (16) and the engine waits. 5c: the stall covers the end of a read, so `ev_done` comes with 3 bytes still in the FIFO; the reply is still complete. |
| 6 | address probe: ACK → `[00]` (`S 96 A P`), no device → `[01]` |
| 7 | NACK on the write address and on the read address → `[01]`; one abort pulse flushed the unsent byte; the next read is correct |
| 8 | arbitration lost against a second master that starts at the same moment → `[02]`; the other master finishes undisturbed |
| 9 | a device holds SCL low: abort 48..51 µs after the command for a 50 µs time-out, reply `[03]`, SCL/SDA released, the next request works. `timeout_us = 0` → `[03]` at once, bus untouched. |
| 10 | bad requests (wlen > 16, rlen > 60, wrong lengths, too short) → `[04]`, engine untouched |
| 11 | DATA: a 5-byte write, a 20-byte write as **one** transaction (1 START, 1 STOP), DATA with NACK (abort, no reply) |
| 12 | the CPU leaves 2 old bytes in the RX FIFO (the NI throws them away); NI requests that arrive while a CPU command runs wait for it; a stuck CPU command (2 bytes announced, 1 pushed) does not get the NI's byte, and the NI times out |
| 13 | other packet types and `en = 0`: consumed, nothing on the bus, all credits back |
| 14 | heavy stall + 3 back-to-back requests + a blocked reply path |
| 15 | counters and global monitors |

The **global monitors** must stay at 0: an engine strobe, command or abort
while `stall = 1`; a command while the engine is busy; a TX FIFO overflow;
a reply head flit before the engine is idle and free.

### 9.3 How to run them and read the output

```
cd smart_serial_controller_v2
sh sim/run_unit.sh ni_spi
sh sim/run_unit.sh ni_i2c
```

or by hand, for example:

```
iverilog -g2005 -Wall -o build/tb_ni_i2c.vvp sim/unit/tb_ni_i2c.v rtl/noc/ni_i2c.v \
    rtl/noc/noc_pkt_tx.v rtl/noc/noc_pkt_rx.v rtl/top/ssc2_timebase.v rtl/v1/ssc_i2c.v \
    rtl/v1/ssc_fifo.v rtl/v1/ssc_sync_filter.v sim/models/tb_i2c_slave.v \
    sim/models/tb_i2c_monitor.v sim/models/tb_i2c_master.v
vvp -n build/tb_ni_i2c.vvp
```

`tb_ni_spi` runs in under 1 second, `tb_ni_i2c` in about 5 seconds.
The SPI test prints:

```
[1] XFER_REQ cs0 [01][03][9F] -> XFER_RESP [00][20 BA 19]
    reply: tag 5a, dest 3, 4 bytes: 00 20 ba 19
[2] DATA [06] (write enable): one CS frame, no reply
...
[4] XFER_REQ [01][01][05] status polling until the flash is ready
    flash ready after 8 polls, status 00
[5] long read: wlen 4 + rlen 40 (44 bytes, more than 16 in flight)
    reply: tag a5, dest 0, 41 bytes: 00 00 8c 5a 5c 7b 9f bd 04 a3 03 b6 ...
...
[13] counters and monitors: pkts_in 34 (sent 34), pkts_out 21 (replies 21)
    shortest CS-high gap between frames: 100 ns
    shortest CS hold after the last SCLK edge: 120 ns
ALL 50 CHECKS PASSED
```

The I2C test prints the bus log of every transaction:

```
[1] XFER_REQ 0x4B [01][02][00] -> XFER_RESP [00][0C 80] (repeated START)
    reply: tag 21, dest 1, 3 bytes: 00 0c 80
        bus: S 96 A 00 A Sr 97 A 0c A 80 N P
[2] write only: [03][00][02 AA BB] -> [00]
        bus: S 96 A 02 A aa A bb A P
...
[9] time-out: a device holds SCL low during the address byte, timeout 50 us
    command to abort: 49.1 us
...
[15] counters and monitors: pkts_in 42 (sent 42), pkts_out 32 (replies 32)
ALL 77 CHECKS PASSED
```

- `[n]` starts test n of the tables above.
- `reply:` shows a reply's tag, destination and payload bytes.
- `bus:` is the I2C bus log: `S` START, `Sr` repeated START, `P` STOP,
  `A` ACK, `N` NACK, bytes in hex (the first byte is address × 2 + R/W).
- A failing check prints `FAIL: <what> (t=<time>)`. A wrong reply also
  prints all its fields and bytes just before.
- The last line is `ALL n CHECKS PASSED` or `m OF n CHECKS FAILED`.
- If the NI hangs, a watchdog prints `FAIL: watchdog - simulation took too
  long`. Waiting tasks print `(time-out waiting for reply n)` or
  `(time-out waiting for the NI to go idle)` before the checks fail.

## 10. Notes for the integrator

- Wire each NI to its mesh port: `in_*` = mesh `rx_*[n]`, `out_*` = mesh
  `tx_*[n]`, `my_id` = 4 (SPI) or 5 (I2C).
- OR the FIFO strobes, `cmd_valid` and `abort` with the CPU's, mux the data
  with the CPU first, and use `active` to force the engine settings
  (sections 4.4 and 5.4). `stall` must be the APB `psel`.
- The CPU and an NI should not run transfers on the same engine at the same
  time. The NIs wait for `busy = 0` before they start a transfer, but while
  they wait they already empty the RX FIFO, so a CPU read that is running at
  that moment loses its bytes. And a CPU transfer that starts while an NI
  transfer runs mixes bytes with it. Software should leave an engine to its
  NI while the NI is enabled.
- The requester must handle a missing reply (`en = 0`): the hub has its
  own time-out (HUB_TIMEOUT).

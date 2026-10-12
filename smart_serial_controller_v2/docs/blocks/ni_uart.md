# `ni_uart`: the UART network interface (NoC node 3)

File: `rtl/noc/ni_uart.v`. Test: `sim/unit/tb_ni_uart.v`.
Contract: `docs/SPEC.md`, sections 3.1, 3.2, 3.7, 4 and 4.1.

## 1. What the block does

The PC is connected to the board through the v1 UART. The `ni_uart`
block lets the PC talk to the other nodes of the network-on-chip (NoC),
for example the SPI flash behind node 4. It works in both directions:

- **PC to network.** Bytes typed or sent by the PC arrive in the UART RX
  FIFO. The NI reads them, groups them into a packet and sends the packet
  into the network.
- **Network to PC.** Packets that arrive for node 3 are unpacked. Their
  payload bytes are written into the UART TX FIFO, so the PC receives them.

This is "flow A" on slide 17. The PC sends `01 9F 03`, which means
"write 1 byte (0x9F, the JEDEC ID command), then read 3 bytes". The NI
turns this into an XFER_REQ packet for the SPI node. The SPI node answers
with an XFER_RESP, and the PC receives `20 BA 19`.

## 2. Block diagram

```
                         ni_uart
            +------------------------------------------------------+
  UART RX   |  rx_pop   +-----------------+   +--------------+     |  out_valid
  FIFO ---->|---------->| parser / RAW    |-->| 64-byte      |     |  out_flit
 (from PC)  |  rx_data  | collector (FSM) |   | packet buffer|     |------------> network
            |           +-----------------+   +------+-------+     |
            |                 |  silence timer       |             |
            |   us_tick ----->|  (quiet, µs)         v             |
            |                                   +-------------+    |  out_credit
            |                                   | noc_pkt_tx  |<---|<----------- network
            |                                   +-------------+    |
            |                                                      |
  UART TX   |  tx_push  +-----------------+   +-------------+      |  in_valid
  FIFO <----|<----------| byte filter     |<--| noc_pkt_rx  |<-----|<----------- network
 (to PC)    |  tx_data  | (keep / skip)   |   | (4 flits)   |      |  in_flit
            |           +-----------------+   +-------------+----->|-----------> in_credit
            |                                                      |
            |  pkts_in / pkts_out: one count per HEAD/SINGLE flit  |
            +------------------------------------------------------+
     stall (CPU is using the FIFOs) and en gate every rx_pop / tx_push
```

The two directions are independent and run at the same time.

## 3. Ports

### Common NI ports (SPEC 4)

| port | dir | width | meaning |
|---|---|---|---|
| `clk` | in | 1 | 100 MHz clock |
| `rst_n` | in | 1 | asynchronous reset, active low |
| `stall` | in | 1 | 1 = the CPU is doing an APB access. The NI must not push or pop. |
| `en` | in | 1 | NI enable (NI_CFG[3] bit 0). 0 = do not touch the UART FIFOs, but still drain the network. |
| `my_id` | in | 3 | own node id (3). Used as the `source` field. |
| `in_valid`, `in_flit` | in | 1, 34 | flits from the network (router LOCAL output) |
| `in_credit` | out | 1 | pulse: one slot of the NI's 4-flit input buffer was freed |
| `out_valid`, `out_flit` | out | 1, 34 | flits to the network (router LOCAL input) |
| `out_credit` | in | 1 | pulse: the router freed one slot of its input buffer |
| `pkts_in` | out | 16 | packets received (HEAD or SINGLE flits seen on `in_*`) |
| `pkts_out` | out | 16 | packets sent (HEAD or SINGLE flits seen on `out_*`) |

### Extra ports (SPEC 4.1)

| port | dir | width | meaning | comes from (core) |
|---|---|---|---|---|
| `mode` | in | 2 | 0 RAW, 1 FRAME, 2 ADDRESSED FRAME (3 behaves as RAW) | NI_CFG[3] [2:1] (0x50C) |
| `dest` | in | 3 | destination node for RAW and FRAME | NI_CFG[3] [6:4] |
| `prio` | in | 1 | prio bit of every packet sent | NI_CFG[3] [7] |
| `arg` | in | 8 | `arg` field for RAW and FRAME | NI_CFG[3] [15:8] |
| `resp_status` | in | 1 | 1 = also send the XFER_RESP status byte to the PC | NI_CFG[3] [16] |
| `timeout_us` | in | 16 | silence time-out in µs | 0x540 (reset 2000) |
| `us_tick` | in | 1 | 1-clock pulse every µs | `ssc2_timebase` |
| `tx_push` | out | 1 | write `tx_data` into the UART TX FIFO | OR-ed with the CPU strobe |
| `tx_data` | out | 8 | byte for the UART TX FIFO | muxed with the CPU data |
| `tx_full` | in | 1 | UART TX FIFO is full | `ssc_uart` |
| `rx_pop` | out | 1 | remove the front byte of the UART RX FIFO | OR-ed with the CPU strobe |
| `rx_data` | in | 8 | front byte of the RX FIFO (first-word fall-through) | `ssc_uart` |
| `rx_empty` | in | 1 | UART RX FIFO is empty | `ssc_uart` |

The NI has no registers of its own. The CPU reads `pkts_in`/`pkts_out`
through NI_STAT[3] at 0x52C.

## 4. How it works

### 4.1 PC to network

A state machine reads the RX FIFO one byte per clock. It only pops a byte
(`rx_pop = 1`) when `en = 1`, `stall = 0`, the FIFO is not empty and the
NI is collecting (not busy sending a packet).

The header of a packet holds the payload length. So the NI first stores the
whole payload in a 64-byte buffer, and starts the packet only when it is
complete. Then it hands the header and the bytes to `noc_pkt_tx`, which
makes the flits and handles the credits.

States:

| state | used in | what happens on the next byte |
|---|---|---|
| `S_RAW` | RAW | store it at `buf[cnt]`; on the 16th byte the packet is complete |
| `S_DEST` | ADDRESSED | byte 0..7 is the destination; a bigger value is a bad frame |
| `S_ARG` | ADDRESSED | the byte is the `arg` field |
| `S_WLEN` | FRAME, ADDRESSED | wlen (0..60), stored at `buf[0]`; a bigger value is a bad frame |
| `S_WDATA` | FRAME, ADDRESSED | write byte i is stored at `buf[2+i]` |
| `S_RLEN` | FRAME, ADDRESSED | rlen (0..60), stored at `buf[1]`; the frame is complete |
| `S_SKIP` | FRAME, ADDRESSED | bad frame: the byte is thrown away |
| `S_HDR` | all | wait for `noc_pkt_tx` to be free, then give it the header |
| `S_BYTES` | all | give `noc_pkt_tx` one buffer byte each time it is ready |

Because of the buffer layout (`wlen` at 0, `rlen` at 1, write bytes from 2),
the buffer already holds the XFER_REQ payload `[wlen][rlen][w...]` in the
right order. The PC sends rlen last, but it simply goes into slot 1.

**The three modes:**

- **RAW (mode 0).** Every byte goes into the buffer. With 16 bytes, a DATA
  packet of 16 bytes is sent at once. If fewer bytes are waiting and the
  line has been quiet for `timeout_us`, a shorter DATA packet is sent. Dest,
  arg and prio come from the configuration.
- **FRAME (mode 1).** The PC sends `[wlen][w...][rlen]`. The NI sends
  XFER_REQ to `dest`, with `arg`, and the payload `[wlen][rlen][w...]`. The
  length is wlen + 2.
- **ADDRESSED FRAME (mode 2).** The PC sends `[dest][arg][wlen][w...][rlen]`.
  It is the same as FRAME, but dest and arg come from the frame.

**Tag.** `tag_cnt` counts the packets this NI has sent (0, 1, 2, ... after
reset). It goes into the `tag` field, so consecutive XFER_REQs have
increasing tags.

**Silence timer.** `quiet` counts `us_tick` pulses. It is cleared while a
byte is waiting in (or being taken from) the RX FIFO, while `en = 0`, and
while a packet is being sent. The time-out (`tmo`) fires on the first
`us_tick` after `quiet` has reached `timeout_us`. That is between
`timeout_us` and `timeout_us + 1` µs after the last byte. Then:

- in RAW with some bytes collected, those bytes are sent;
- in a half-received frame, the frame is thrown away and the parser goes
  back to its start state (`S_WLEN` for FRAME, `S_DEST` for ADDRESSED);
- in `S_SKIP`, the parser goes back to its start state.

### 4.2 Network to PC

`noc_pkt_rx` keeps the incoming flits in a 4-flit buffer and shows the NI
the header and then the payload bytes one by one. The NI always takes the
header at once (`hdr_ready = 1`) and remembers two flags:

- `n_keep`: the packet type is 0..4 (DATA, XFER_REQ, XFER_RESP, RECORD,
  ALARM), so its bytes go to the PC. Types 5..7 are reserved: they are
  consumed and dropped.
- `n_skip`: the packet is an XFER_RESP and `resp_status = 0`. The first
  byte (the status) is thrown away.

For each payload byte:

- If the byte is to be dropped (`!en`, `!n_keep` or `n_skip`), it is taken
  at once and nothing is pushed.
- Otherwise it is pushed (`tx_push = 1`) in the first clock where
  `tx_full = 0` and `stall = 0`. Until then the byte waits in `noc_pkt_rx`.
  Its buffer fills up and the credits stop, so the network waits too. No
  byte is ever lost.

A header-only packet (SINGLE flit) has no payload, so it pushes nothing.

### 4.3 Sharing the UART with the CPU

The CPU can also use the UART FIFOs through the v1 registers. The core ORs
the CPU and NI strobes and muxes the data. The core drives `stall = psel`,
and the CPU only touches the FIFOs during an APB access. The NI never pushes
or pops while `stall = 1`, so the two can never collide.

## 5. Timing example: slide 17

The time-out is 20 µs, the UART runs at 1.5625 Mbaud (6.4 µs per byte),
and `dest = 4`, `arg = 01`. This trace is taken from the testbench.

**PC to network.** The PC has sent `01` and `9F`. The parser is in `S_RLEN`.
Clock c0 is the clock where byte `03` is popped.

| clock | what happens | `out_valid` / `out_flit` |
|---|---|---|
| c0 | `rx_pop = 1`, `rx_data = 03`. rlen is valid: `buf[1] = 03`, header fields latched, next state `S_HDR` | 0 |
| c1 | `S_HDR`: `start = 1`, `noc_pkt_tx` is free and takes the header. `tag_cnt` becomes 1. | 0 |
| c2 | `S_BYTES`: `noc_pkt_tx` sends the head flit (it has a credit) | 0 |
| c3 | head flit on the link; byte 0 (`01`) taken | 1 / `0_31830001` (HEAD) |
| c4 | byte 1 (`03`) taken | 0 |
| c5 | byte 2 (`9F`) taken: last byte, the NI goes back to `S_WLEN` | 0 |
| c7 | tail flit on the link | 1 / `2_009F0301` (TAIL) |

The head flit `0x31830001` decodes as type 1 (XFER_REQ), dest 4, source 3,
prio 0, length 3, tag 0, arg 01. The TAIL flit holds the payload
little-endian: byte 0 = `01` (wlen), byte 1 = `03` (rlen), byte 2 = `9F`.

**Network to PC.** The SPI node answers with XFER_RESP, len 4, payload
`00 20 BA 19`, and `resp_status = 0`.

| clock | what happens |
|---|---|
| m0 | HEAD flit `0_4E040000` arrives (type 2, dest 3, src 4, len 4) |
| m1 | header taken: `n_keep = 1`, `n_skip = 1` |
| m5 | TAIL flit `2_19BA2000` arrives |
| m6 | byte `00` (status) is dropped at once, `n_skip` clears |
| m7, m8 | byte `20` is ready but `stall = 1`: it waits |
| m9 | `stall = 0`: `tx_push = 1`, `tx_data = 20` |

Bytes `BA` and `19` follow in the same way. The PC receives `20 BA 19`.
With `resp_status = 1` it would receive `00 20 BA 19`.

## 6. Design choices

The SPEC leaves some details open. These are the choices made, and why.

1. **The packet buffer is 64 bytes, single-buffered.** The largest payload
   is 62 bytes (wlen = 60 plus 2). While a packet is being sent, the NI
   does not pop the RX FIFO. Sending takes only a few tens of clocks, and
   the UART RX FIFO holds 16 bytes meanwhile. The NI leaves `S_BYTES` as soon
   as `noc_pkt_tx` has taken the last byte. So even when the network is
   slow, it can already collect the next packet.
2. **Bad frames.** A frame is bad if wlen > 60, rlen > 60, or (ADDRESSED) the
   dest byte is above 7. The frame is thrown away, and so is every byte
   that follows it without a pause. After `timeout_us` of silence the parser
   waits for a new frame. This is the same re-synchronisation rule as for
   an incomplete frame. It never turns the rest of a broken frame into
   wrong requests.
3. **Time-out resolution.** The time-out fires between `timeout_us` and
   `timeout_us + 1` µs after the last byte, because `us_tick` comes once per
   µs. `timeout_us = 0` is not "off". It fires at the next `us_tick`, in
   under 1 µs.
4. **What counts as silence.** A byte that is already in the RX FIFO has
   "arrived", even if the NI could not pop it yet (for example during a
   long `stall`). So the timer only runs while the RX FIFO is empty.
5. **`en = 0` and mode changes.** Both throw away a half-collected RAW
   packet or frame. A packet that is already being sent is always finished,
   because its bytes come from the NI's own buffer, not from the UART.
6. **Mode 3** is not defined by the SPEC. It behaves like RAW.
7. **Tag.** One 8-bit counter of all packets this NI sends, for both RAW
   and FRAME packets. It starts at 0.
8. **Counters.** `pkts_in` and `pkts_out` count HEAD and SINGLE flits on the
   links, exactly as the core's NI_STAT counters do. Packets that are dropped
   (reserved types, or `en = 0`) are still counted in `pkts_in`.
9. **All forwarded types.** DATA, XFER_REQ, XFER_RESP, RECORD and ALARM
   payloads all go to the PC, following the SPEC rule "every payload byte".

## 7. Size

Yosys `synth_xilinx`, flattened with the two helpers:
402 LUTs (LUT1..LUT6), 250 flip-flops, 39 CARRY4, 57 MUXF7/F8,
3 RAM64M (the 64-byte packet buffer) and 6 RAM32M (the 4-flit input buffer).
Most of the logic is in `noc_pkt_tx`. The NI's own logic is roughly
130 LUTs and 115 flip-flops.

Yosys also lists about 270 INV cells. These come from the active-low
asynchronous reset: yosys puts one inverter in front of each flip-flop's
clear pin. Vivado uses a single inverter for `rst_n`.

## 8. The testbench (`sim/unit/tb_ni_uart.v`)

### Set-up

```
 tb_uart_term (PC) <-- rxd/txd --> ssc_uart (v1, 8N1) <-- FIFOs --> ni_uart
                                                                    |    ^
             TB network:   u_cap (noc_pkt_rx)  <---- out link ------+    |
                           u_inj (noc_pkt_tx)  ----- in link ------------+
```

- The real v1 `ssc_uart` is used, with `baud_int = 4` (64 clocks per bit,
  6.4 µs per byte). `tb_uart_term` plays the PC.
- `u_cap` captures every packet the NI sends. It is ready only 75 % of the
  time, so the NI's credit counter is exercised. It can also be stopped
  completely (`cap_hold`).
- `u_inj` injects packets into the NI, as if they came from the network.
- `timeout_us = 20`.
- `stall` is random for the whole run: 25 % of the clocks, plus a
  50-clock burst now and then. It can also be held at 1 (`stall_force`).

### What is checked

| test | what it proves |
|---|---|
| 1 | Slide 17: `01 9F 03` gives XFER_REQ to node 4, src 3, arg 01, payload `01 03 9F`, sent at once |
| 2 | XFER_RESP `[00][20 BA 19]` gives exactly `20 BA 19` at the PC; with `resp_status = 1` it gives `00 20 BA 19`; a status-only response gives nothing |
| 3 | wlen = 0; wlen = rlen = 60 (62-byte packet); pauses shorter than the time-out; tags 0, 1, 2, 3 |
| 4 | An incomplete frame is dropped between 20 and 21 µs after its last byte, and the next frame is parsed correctly. A 40 µs stall with bytes waiting does not drop the frame. |
| 5 | wlen = 61 (complete or not), rlen = 61, and the bytes right after a bad frame: nothing is sent. After a pause a good frame works. |
| 6 | ADDRESSED: dest and arg come from the frame; a dest byte of 9 and an incomplete frame are dropped |
| 7 | RAW: 16 bytes give one packet at once; 5 bytes give a packet 20..21 µs after the last byte; 40 bytes give 16 + 16 + 8; while the network is blocked, bytes wait in the RX FIFO and none are lost; a mode change drops a half frame |
| 8 | DATA, RECORD, ALARM and XFER_REQ payloads reach the PC |
| 9 | Reserved types 5, 6, 7 are consumed (all credits come back) and nothing is pushed |
| 10 | Header-only packets push nothing |
| 11 | 4 × 63 bytes arrive at the PC complete and in order, while the NI waits on `tx_full` many times |
| 12 | `en = 0`: 4 packets are consumed, no push, no pop, the PC bytes stay in the RX FIFO. `en = 1` sends them as a RAW packet. `en = 0` also drops a half frame. |
| 13 | Random frames from the PC and random DATA packets to the PC, both at the same time |
| 14 | `pkts_in` / `pkts_out` match the TB's own counts. Global assertions are checked here. |

Monitors run on every clock for the whole test. These counts must all be 0:

- `bad_stall`: a push or pop while `stall = 1`;
- `bad_en`: a push or pop while `en = 0`;
- `bad_full` / `bad_empty`: a push into a full FIFO or a pop from an empty one;
- UART overflow and framing errors.

There are also coverage counters, to prove the stress really happened.
`cov_stall` counts clocks where stall held the NI back. `cov_bp` counts
clocks where the NI waited on `tx_full`.

### How to run it and read the output

```
cd smart_serial_controller_v2
sh sim/run_unit.sh ni_uart
```

or by hand:

```
iverilog -g2005 -Wall -o build/niu_tb_ni_uart.vvp sim/unit/tb_ni_uart.v rtl/noc/ni_uart.v \
    rtl/noc/noc_pkt_tx.v rtl/noc/noc_pkt_rx.v rtl/top/ssc2_timebase.v rtl/v1/ssc_uart.v \
    rtl/v1/ssc_fifo.v rtl/v1/ssc_baud_gen.v rtl/v1/ssc_sync_filter.v sim/models/tb_uart_term.v
vvp -n build/niu_tb_ni_uart.vvp
```

The run takes about 10 s (5.4 ms of simulated time). The output looks like
this:

```
--- 1: slide 17 flow A, PC -> network  (t = 0 us, 1 checks so far)
    PC 01 9F 03 -> type 1 dest 4 src 3 tag 0 arg 01 len 3 payload 01 03 9f
--- 2: reply 20 BA 19 back to the PC  (t = 19 us, 4 checks so far)
    XFER_RESP 00 20 BA 19 -> PC received 3 bytes: 20 ba 19
--- 3: FRAME corner cases  (t = 124 us, 8 checks so far)
...
--- 14: counters and global assertions  (t = 5414 us, 57 checks so far)
pkts_in 31  pkts_out 27  pushes 480  pops 327  stall-blocked 1768  tx_full-waits 267383
ALL 65 CHECKS PASSED
```

- Each `---` line starts a test. It shows the simulated time and how many
  checks have run so far.
- A failing check prints `FAIL: <what> (t=...)`, sometimes with a detail
  line above it, such as the header of a wrong packet, or the first PC byte
  that differs.
- The last summary line shows the totals: packets in and out, UART pushes
  and pops, and the two coverage counters.
- The final line is `ALL n CHECKS PASSED` or `m OF n CHECKS FAILED`.
- If the test hangs, a watchdog stops it after 40 ms of simulated time
  and prints a failure.

The testbench was also checked against 16 deliberately broken copies of
the RTL. Examples: no stall gating, no status skip, timer one µs early,
`tx_full` ignored, `en` not dropping bytes, reserved types kept, wrong
wlen/rlen/dest limits, a bad frame re-synchronising too early, wrong RAW
size, tag not counting, and wrong ADDRESSED dest. Every broken copy made
at least one check fail.

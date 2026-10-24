# Sensor hub (`rtl/hub/hub.v`)

The sensor hub is node 1 of the network-on-chip. It is section 7 of
`docs/SPEC.md` and the block behind slides 18 to 20. It takes the boring,
repeating sensor work away from the CPU. A typical job is:

> Every 100 ms, ask the I2C node to read 2 bytes from the TMP2 sensor at
> address 0x4B. Put the answer in a time-stamped record and send the record
> to the DMA writer.

The CPU only sets the job up once. After that the hub does it on its own,
for as long as the board runs.

## 1. What the block does

The hub holds a table of **8 tasks**. Each task describes one small network
transaction:

- **who** to talk to: the destination node, plus an `arg` byte. For the I2C
  node the arg is the 7-bit address; for the SPI node it is the chip-select.
- **what** to send: up to 8 bytes, set by the CPU. Optionally the hub adds
  the last value it read (SEND_LAST) and the last record it made
  (APPEND_REC).
- **how many bytes to read back** (rlen).
- **when** to run: every N ms, after every N records, or only when the CPU
  asks (RUN_NOW).
- **what to do with the answer:** keep it as the "last value" and,
  optionally, turn it into a 16-byte **RECORD** for the DMA writer.

There are two kinds of task:

| task type | packet sent | waits for a reply? |
|---|---|---|
| XFER_REQ (type 1) | `[wtotal][rlen][write bytes ...]` | yes: the XFER_RESP with the same tag |
| DATA (type 0) | the write bytes only, no length bytes | no |

The hub runs **one task at a time**. When several tasks are due together,
the lowest task number goes first.

The RECORD layout (SPEC 3.3, 16 bytes, little-endian):

| byte | field |
|---|---|
| 0..3 | `t_start`: `time_us` when the task started |
| 4 | source: engine code of the node that answered (1 UART, 2 SPI, 3 I2C, 4 SE) |
| 5 | number of valid data bytes (0..8) |
| 6..7 | sequence number, the first record is 1 |
| 8..15 | response data, padded with 0 |

## 2. Block diagram

```
                 reg_we/addr/wdata                         time_us   ms_tick
                        |                                     |         |
     +------------------v-------------------------------------|---------|----+
     |  register decode                                       |         |    |
     |     |                                                  |         |    |
     |     v                                                  |         |    |
     |  +-------------------- 8 task slots --------------------+         |    |
     |  | slot t:  W0 TASK_CFG  W1 BYTES0  W2 BYTES1  W3 TIMING |<--------+    |
     |  |          countdown cd (ms or records)   due flag      |  ms_tick     |
     |  +-------------------------+-----------------------------+              |
     |                            | due_v[7:0]          ^ rec_evt (a record    |
     |                            v                     |  was sent)           |
     |              lowest due task = "pick"            |                      |
     |                            |                     |                      |
     |                            v                     |                      |
     |  +--------------------- main state machine ------+---------------+      |
     |  |  IDLE -> HDR -> PAY -> WAIT -> RESP -> (HDR -> PAY: RECORD)  |      |
     |  |  copies of the task: cur_cfg, cur_w, t_start, tag            |      |
     |  |  last value (8 bytes), last record (16 bytes), seq, counters |      |
     |  +------+---------------------------------------------+---------+      |
     |         | header + payload bytes                      ^ status + data  |
     |         v                                             |                |
     |  +-------------+   payload generator:          +------+-------+        |
     |  | noc_pkt_tx  |<- [len][rlen][w][last][rec]   | noc_pkt_rx   |        |
     |  |  (credits)  |                               | (4-flit buf) |        |
     |  +------+------+                               +------^-------+        |
     +---------|---------------------------------------------|----------------+
               v out_valid/out_flit, out_credit              | in_valid/in_flit, in_credit
                              to / from the router (local port of node 1)

     outputs: reg_rdata (combinational), ev_error, ev_record (1-clock pulses)
```

## 3. Ports

```
module hub (
    input clk, rst_n, input [31:0] time_us, input ms_tick, input [2:0] my_id,
    input reg_we, reg_re, input [7:0] reg_addr, input [31:0] reg_wdata, output [31:0] reg_rdata,
    input in_valid, input [33:0] in_flit, output in_credit,
    output out_valid, output [33:0] out_flit, input out_credit,
    output ev_error, output ev_record);
```

| port | dir | meaning |
|---|---|---|
| `clk`, `rst_n` | in | 100 MHz clock, asynchronous active-low reset |
| `time_us` | in | microseconds since reset, from `ssc2_timebase`. Copied into `t_start`. |
| `ms_tick` | in | 1-clock pulse every millisecond. Drives the task countdowns and the time-out. |
| `my_id` | in | the hub's node id (1 in the core). Used as the source of every packet. |
| `reg_we` | in | register write strobe (1 clock) |
| `reg_re` | in | read strobe. Not used: no hub register changes when it is read. |
| `reg_addr` | in | byte offset inside the hub region 0x600..0x6FF |
| `reg_wdata` | in | write data |
| `reg_rdata` | out | read data, combinational from `reg_addr`. Unused addresses read 0. |
| `in_valid`, `in_flit`, `in_credit` | in/in/out | link **from** the network, into the hub's `noc_pkt_rx` |
| `out_valid`, `out_flit`, `out_credit` | out/out/in | link **to** the network, from the hub's `noc_pkt_tx` |
| `ev_error` | out | 1-clock pulse: a reply had status ≠ 0, or a time-out happened (INT2 bit 3) |
| `ev_record` | out | 1-clock pulse: a RECORD packet was sent (INT2 bit 4) |

## 4. Registers (region 0x600, `reg_addr[7:0]`)

### 4.1 Global registers

| off | name | access | fields |
|---|---|---|---|
| 0x00 | HUB_CTRL | RW | [0] EN. W1: [15:8] RUN_NOW, bit 8+t makes task t due. Read: [15:8] = the due flags (tasks waiting to run). |
| 0x04 | HUB_STATUS | RO | [0] busy, [3:1] current (or last) task, [8] waiting for a reply |
| 0x08 | HUB_REC_DEST | RW | [2:0] node that gets the RECORDs. Reset 2 (DMA writer). |
| 0x0C | HUB_TIMEOUT | RW | [15:0] reply time-out in ms. Reset 50. |
| 0x10 | HUB_SEQ | RO | sequence number of the next record. Reset 1. |
| 0x14 | HUB_LAST | RO | [3:0] last value length, [15:8] status of the last XFER_REQ run (0xFF = time-out), [23:16] its task |
| 0x18 | HUB_LAST_LO | RO | last value bytes 0..3 (byte 0 in [7:0]) |
| 0x1C | HUB_LAST_HI | RO | last value bytes 4..7 |
| 0x20 | REQ | RO | requests sent (every task run, XFER_REQ and DATA) |
| 0x24 | RESP_OK | RO | replies with status 0 |
| 0x28 | ERR | RO | replies with status ≠ 0 |
| 0x2C | TMO | RO | time-outs |
| 0x30 | REC | RO | RECORD packets sent |

The counters are 32 bits and wrap. Writes to read-only registers are
ignored.

### 4.2 Task table: task t at `0x40 + 16*t`

| off | name | fields |
|---|---|---|
| +0x0 | W0 TASK_CFG | see below |
| +0x4 | W1 BYTES0 | write bytes 0..3, byte 0 in [7:0] |
| +0x8 | W2 BYTES1 | write bytes 4..7 |
| +0xC | W3 TIMING | [15:0] period, [31:16] phase (0 = one period). In ms, or in records for a TRIG_RECORDS task. |

W0 TASK_CFG (bits [31:29] read 0):

| bits | field | meaning |
|---|---|---|
| [0] | EN | task on |
| [3:1] | dest | destination node |
| [6:4] | type | 1 = XFER_REQ (waits for a reply), 0 = DATA. Other values are sent like DATA; 4 (ALARM) always goes out with prio 1. |
| [7] | prio | head flit prio bit |
| [15:8] | arg | head flit arg byte (I2C address, SPI chip-select, SM index) |
| [19:16] | wlen | write bytes taken from W1/W2 (0..8; 9..15 act as 8) |
| [23:20] | rlen | bytes to read, sent as written. Only the first 8 reply bytes are kept. |
| [24] | RECORD | make a RECORD from a good reply |
| [25] | SEND_LAST | append the last value bytes |
| [26] | ADDR_INC | W1 bytes 1..3 are a big-endian address; add 256 after each run |
| [27] | TRIG_RECORDS | the period counts records, not ms |
| [28] | APPEND_REC | append the last 16-byte record |

Example: the slide-18 task, as written by `rtl/top/ssc2_boot.v`:

```
W0 = 0x0121_4B1B   EN=1, dest=5 (I2C), type=1 (XFER_REQ), prio=0, arg=0x4B,
                   wlen=1, rlen=2, RECORD=1
W1 = 0x0000_0000   write byte 0 = 0x00 (the TMP2 temperature register)
W3 = 0x0000_0064   period 100 ms, phase 0 (first run after one period)
```

## 5. How it works, step by step

### 5.1 Scheduling (one small block per task, all 8 in parallel)

Each task slot has a 16-bit countdown `cd` and a `due` flag.

1. **Start of the schedule.** When EN goes from 0 to 1, or when W3 is
   written, `cd` gets the phase. If the phase is 0 it gets the period.
2. **Counting.** For a normal task, every `ms_tick` is one step. For a
   TRIG_RECORDS task, every RECORD the hub sends is one step. On each step
   `cd` goes down by one. When it goes from 1 to 0, the task becomes due and
   `cd` reloads with the period.
3. **Records since the last run.** A TRIG_RECORDS task also reloads `cd`
   with the period every time it starts. So it becomes due when PERIOD
   records were sent **since it last ran**, even if it last ran by RUN_NOW.
4. **RUN_NOW.** Writing 1 to `HUB_CTRL[8+t]` makes task t due at once (only
   if the task is enabled).
5. **Stopped countdown.** `cd = 0` means stopped. A period of 0 therefore
   means "never periodic": the task runs only by RUN_NOW. A phase with
   period 0 gives one single, delayed run.

Because the countdown loads on every W3 write, the order of the set-up
writes does not matter: W0 first (like the boot ROM does) or W3 first both
work.

### 5.2 Choosing a task

When the main state machine is idle, `HUB_CTRL.EN = 1` and at least one task
is due, it takes the lowest due task number (`pick`). In that same clock:

- the due flag of that task is cleared,
- `t_start = time_us`,
- the tag becomes `{roll[4:0], t[2:0]}`, and `roll` goes up by one (it
  counts every run, so it wraps after 32 runs),
- W0 and the 8 write bytes are copied (`cur_cfg`, `cur_w`), so a CPU write
  during the run cannot mix two configurations,
- for ADDR_INC: the address in W1 bytes 1..3 is increased by 256 (the copy
  keeps the old address for this run).

### 5.3 Sending the request (states HDR and PAY)

**HDR** hands the header to `noc_pkt_tx`: type, dest, `src = my_id`, prio,
length, tag, arg. The length is

```
wtotal = wlen + (SEND_LAST ? last length : 0) + (APPEND_REC ? 16 : 0)
len    = wtotal + 2   (XFER_REQ)       or   wtotal   (DATA)
```

The largest request is 2 + 8 + 8 + 16 = 34 bytes, well below the 63-byte
limit.

**PAY** gives the payload bytes one by one. The payload is made of four
parts, and a small counter pair `(seg, idx)` walks through them:

| seg | part | bytes | used when |
|---|---|---|---|
| 0 | lengths | `wtotal`, `rlen` | XFER_REQ only |
| 1 | write bytes | W1/W2 bytes 0..wlen-1 | always |
| 2 | last value | last length bytes | SEND_LAST |
| 3 | last record | 16 bytes | APPEND_REC |

An empty part is skipped in one clock. When the last byte of part 3 has
been taken (or part 3 is empty), the request is complete:

- a DATA task is finished: back to IDLE (no reply is expected),
- an XFER_REQ task goes to WAIT and loads the time-out counter with
  HUB_TIMEOUT.

### 5.4 Waiting for the reply (states WAIT and RESP)

The hub's receiver **always** takes every header and every byte, so the
network behind it never blocks. For each header:

- If the hub is in WAIT, the type is XFER_RESP (2) **and** the tag equals
  the run's tag, this is our reply: go to RESP and keep its bytes.
- Anything else is read and thrown away: other types (DATA, RECORD,
  XFER_REQ, reserved 5..7), replies with another tag, and late replies to an
  old run (their roll value is different).

In RESP the first payload byte is the status. The next bytes (at most 8) go
into `rsp_data`; any more are read and dropped. When the last byte is in,
the reply is judged:

- **Status 0:** RESP_OK + 1. If the reply has data, the last value becomes
  that data (padded with 0) and the last length becomes n. If RECORD is set,
  go to step 5.5. Otherwise back to IDLE.
- **Status ≠ 0:** ERR + 1, `ev_error` pulses, no record, the last value is
  kept. Back to IDLE.

HUB_LAST gets the status and the task number in both cases.

**Time-out.** In WAIT, every `ms_tick` counts the time-out counter down.
When a tick comes while it is already 0, the hub gives up: TMO + 1,
`ev_error` pulses, HUB_LAST.status = 0xFF, back to IDLE. With HUB_TIMEOUT =
T, this happens at the (T+1)-th tick after the request, so the hub always
waits **at least T ms** (and less than T+1 ms).

### 5.5 Making a record

The new record is built in the 128-bit `last_rec` register:

```
last_rec = { data[63:0], seq[15:0], {4'b0, n}, code(src), t_start[31:0] }
            bytes 15..8   bytes 7..6   byte 5     byte 4     bytes 3..0
```

The source code comes from the reply's source node: 3 → 1 (UART),
4 → 2 (SPI), 5 → 3 (I2C), 0 → 4 (SE), any other → 0.

Then the state machine goes back to HDR with `rec_mode = 1`. It sends a
RECORD packet (type 3) to HUB_REC_DEST with `prio = 0`, `len = 16`, the
run's tag and `arg = task number`. The payload generator only uses part 3,
which now holds the new record. When the 16 bytes are handed over:

- `seq` + 1, REC + 1, and `ev_record` pulses,
- every TRIG_RECORDS task takes one countdown step.

`seq` only moves when a record is really sent. A good reply of a task
without RECORD updates only the last value and RESP_OK.

## 6. Timing example: one run of the slide-18 task

This is taken from the unit test (`US_PER_MS = 5`, so 1 ms = 500 clocks; the
test's receiver adds random back-pressure, which spaces the record flits).
Clock 0 is the clock where `ms_tick` is high and task 0's countdown is 1.

| clock | what happens |
|---|---|
| 0 | `ms_tick` = 1 and `cd` = 1, so the task becomes due |
| 1 | `due[0]` = 1 |
| 2 | IDLE → HDR: `t_start = time_us = 500`, tag = 0x00 |
| 3 | HDR → PAY (header taken by `noc_pkt_tx`) |
| 4 | HEAD flit `0x3483004B` = XFER_REQ, dest 5, src 1, len 3, tag 00, arg 4B |
| 8 | TAIL flit `0x00000201` = bytes 01 02 00 (wtotal 1, rlen 2, write 00) |
| 9 | PAY → WAIT, time-out counter = 50 |
| 315 | reply HEAD arrives: XFER_RESP from node 5, len 3, tag 00 |
| 317 | WAIT → RESP |
| 319 | reply TAIL: status 00, data 0C 80 |
| 324 | RESP → HDR (rec_mode): last value = 0C 80, RESP_OK + 1 |
| 326 | RECORD HEAD `0x68900000` = type 3, dest 2, src 1, len 16 |
| 333 | body `0x000001F4`: time stamp 500 |
| 338 | body `0x00010203`: code 3, n 2, seq 0x0001 |
| 343 | body `0x0000800C`: data 0C 80 |
| 347 | last byte handed over: `ev_record`, seq = 2, back to IDLE (the TAIL flit with bytes 12..15 = 0 follows) |

The run takes 9 clocks to send the request. The rest is the responder's
time (300 clocks in the test) plus about 25 clocks for the record. The next
run starts exactly 100 ms after the first, so consecutive time stamps differ
by exactly 100 ms.

## 7. Worked example: logging records into SPI flash pages

ADDR_INC, APPEND_REC and TRIG_RECORDS exist for this use (one 16-byte record
per flash page, every 16 records):

| task | W0 | W1 | W3 | what it does |
|---|---|---|---|---|
| A (lower index, e.g. 4) | `0x0801_0009` | `0x0000_0006` | `0x0000_0010` | DATA to the SPI node, byte 06 (WREN), TRIG_RECORDS period 16 |
| B (higher index, e.g. 7) | `0x1C04_0019` | `0x0000_0002` | `0x0000_0010` | XFER_REQ to the SPI node, wlen 4: `02 A2 A1 A0` (page program at address 0x000000), APPEND_REC, ADDR_INC, TRIG_RECORDS period 16 |

After 16 records both become due together. A runs first because of index
order, so WREN is clocked out first. Then B sends

```
[20][00][02 A2 A1 A0][16 bytes of the last record]
```

The SPI node clocks out these 20 bytes in one CS frame and replies `[00]`.
After each run of B the address in W1 has gone up by 256 (one flash page):
`00 00 00`, `00 01 00`, `00 02 00`, and so on, with a carry from A1 into A2.

## 8. Design choices

These are the places where the SPEC leaves room. Each one is also listed in
the header of `hub.v`.

1. **Period 0 = never periodic.** The task only runs by RUN_NOW. With a
   phase and period 0, it runs once after the phase.
2. **TRIG_RECORDS uses the same countdown.** The phase also counts records
   (the first run after PHASE records; 0 = one period). The countdown
   restarts at every run, which gives exactly "PERIOD records since the
   task last ran". A TRIG_RECORDS task that makes records itself with
   period 1 would trigger itself forever, so do not configure that.
3. **A due flag is not a queue.** If a task becomes due again before it
   ran, it still runs once. Disabling a task clears its due flag. RUN_NOW is
   ignored for a disabled task.
4. **HUB_CTRL.EN = 0** only stops new tasks from starting. A running task
   finishes, including its record. The countdowns keep running, so due tasks
   wait and run, in index order, as soon as EN returns.
5. **Index order** means: each time the hub is free it takes the lowest due
   task. A low task that becomes due while a high one waits will overtake
   it.
6. **Time-out** counts whole ms ticks: at least HUB_TIMEOUT ms, at most one
   ms more. HUB_TIMEOUT = 0 gives about 1 ms. HUB_LAST.status = 0xFF marks a
   time-out (0xFF is not a network status code).
7. **Long replies.** rlen is sent as written (0..15). The hub keeps only the
   first 8 data bytes and reports length 8. wlen 9..15 acts as 8.
8. **Reply without a status byte** (a matching XFER_RESP of length 0, which
   a correct NI never sends) counts as status 4 (bad request).
9. **Without RECORD** a good reply updates the last value and RESP_OK only.
   seq, REC and `ev_record` do not move.
10. **HUB_LAST** describes the last XFER_REQ run (good, error or time-out).
    DATA runs do not touch it, so a forwarding task (like task 1 of the boot
    ROM) does not hide the result of the sensor task.
11. **The last value and the last record are shared by all tasks.** That is
    what lets one task read a sensor and another forward the value.
12. **ADDR_INC is applied when the task starts**, on the register, after the
    bytes of this run were copied. The register shows the next address
    during the run, and a CPU write to W1 always wins.
13. **Other packet types in TASK_CFG.** Only type 1 waits for a reply. Every
    other type is sent like DATA. Type 4 (ALARM) always gets prio 1.
14. **The RECORD packet** has prio 0, the run's tag and `arg` = task
    number. The DMA writer ignores tag and arg; they help when debugging.
15. **seq** is 16 bits and wraps from 0xFFFF to 0x0000.
16. **Area.** The task table is stored in flip-flops (8 × 125 bits), because
    all 8 schedulers need their own words at the same time. The ADDR_INC
    adder is shared by all tasks.

## 9. Resources

`yosys synth_xilinx -flatten` (hub + its `noc_pkt_tx`/`noc_pkt_rx`):
**2363 LUTs** and **1915 flip-flops**, plus 6 RAM32M for the 4-flit receive
buffer (about 4 % of the Zynq-7020's LUTs). The INV cells yosys also reports
come from the active-low reset and disappear in Vivado.

## 10. How the testbench proves it works (`sim/unit/tb_hub.v`)

### 10.1 Set-up

```
                 +---------+  out link (random back-pressure)  +-----------------+
   registers --->|   hub   |---------------------------------->| TB noc_pkt_rx   |--> packet log
   (wr / rd)     | my_id=1 |<----------------------------------| TB noc_pkt_tx   |<-- send queue
                 +---------+            in link                +-----------------+
                      ^                                                ^
                ssc2_timebase (US_PER_MS = 5: 1 ms = 500 clocks)       |
                                                    responder model for nodes 3, 4, 5
```

- The hub's out link goes straight into a TB `noc_pkt_rx`, and a TB
  `noc_pkt_tx` drives the hub's in link. A direct link like this is a valid
  one-hop network.
- The TB receiver takes headers and bytes only 2 clocks out of 3 (random,
  fixed seed 7), so credit flow control is exercised all the time.
- **Packet log:** every packet from the hub is stored (header fields,
  payload) together with the ms count, clock and `time_us` at the moment its
  head flit left the hub.
- **Responder model:** every XFER_REQ to node 3, 4 or 5 is answered with an
  XFER_RESP that has the same tag, after `rsp_delay` clocks, with
  `rsp_status` and `rlen` bytes of per-node data (node 3: `5A 11 22 ...`,
  node 4: `20 BA 19`, node 5: `0C 80`). With `rsp_junk = 1` it first sends 7
  junk packets: a DATA packet, an XFER_RESP with a wrong roll, one with
  another task's index, a RECORD, an XFER_REQ, and reserved types 6 and 7.
- **Global monitors:** every packet must come from node 1, every request
  must carry `tag = {roll, task}` with roll counting runs, and every XFER_REQ
  length must equal `wtotal + 2`. The event pulses are counted and must be 1
  clock wide.

### 10.2 What each part proves

| part | what is checked |
|---|---|
| A | reset values of every register, W0..W3 read back (W0 keeps 29 bits), read-only registers, unused addresses read 0, nothing is sent while idle |
| B | slide 18: task 0 = I2C 0x4B, write 00, read 2, every 100 ms. Three runs exactly at +100, +200, +300 ms, request `[01][02][00]`, tag `{roll, 0}`. Each RECORD is checked field by field: dest 2, time stamp = `time_us` at task start, source 3, length 2, seq 1, 2, 3, data `0C 80 00 ...`. Time stamps exactly 100 ms apart. HUB_SEQ, HUB_LAST, LO/HI and counters. |
| C | DATA task with phase 3, period 7: runs at +3, +10, +17 ms with payload `AA BB` and no length bytes; the hub is not waiting after a DATA run; a W3 rewrite (phase 0, period 4) restarts the countdown (+4, +8 ms); no run after EN = 0; DATA runs leave RESP_OK and HUB_LAST alone |
| D | period 0 never runs by itself; RUN_NOW of a disabled task is ignored; RUN_NOW runs the SPI read-ID task (`[01][03][9F]` → record with source 2, data `20 BA 19`); HUB_STATUS = busy, task 2, waiting during the wait; the RUN_NOW bit clears when the task starts |
| E | tasks 3, 5, 6 made due while HUB_CTRL.EN = 0: nothing starts, HUB_CTRL shows them; switching a due task off clears its due flag; after EN = 1 they run one at a time in order 3, 5, 6, and task 5 starts only after task 3's reply. SEND_LAST in an XFER_REQ (`[05][01][55 66][20 BA 19]`) and in a DATA task (`0C 80`). EN = 0 during a run: the run finishes with its record, the next task waits for EN. |
| F | flash logging: a 3 ms record maker plus task A (WREN) and task B (page program, ADDR_INC, APPEND_REC), both TRIG_RECORDS period 2. Three rounds: two records, then A, then B. B's payload is `[20][00][02 A2 A1 A0]` + a byte-exact copy of the last record. The address goes 01FE00 → 01FF00 → 020000 (carry) and W1 then holds 020100. A good reply without data keeps the last value. |
| F2 | a TRIG_RECORDS phase of 1 record; then the period of 2; RUN_NOW restarts the count, so one record after it is not enough and the second one is |
| G | junk and wrong-tag packets are read and ignored: the hub keeps waiting, counts nothing, and the record and last value come from the right reply only |
| H | HUB_TIMEOUT = 10, the reply comes after 15 ms: `ev_error` 10..11 ms after the request, TMO = 1, ERR unchanged, HUB_LAST.status = 0xFF, no record; the late reply is ignored; the next run works again |
| I | status 1 (NACK): `ev_error`, ERR = 1, no record, HUB_SEQ and the last value unchanged, HUB_LAST.status = 1 |
| J | a 9-byte reply (rlen 9): the record has length 8 and the first 8 bytes, HUB_LAST length 8, LO/HI; the RECORD goes to the new HUB_REC_DEST (7) |
| L | ALARM (type 4) goes out with prio 1, raw bytes, no wait; the TASK_CFG prio bit sets the head prio |
| K | totals: REQ, RESP_OK, ERR, TMO, REC and HUB_SEQ equal the TB's own count; one `ev_record` per record and one `ev_error` per error/time-out, all 1 clock wide; the roll wrapped past 32; every packet on the link was received; all credits came back |

Every check was also tried against deliberately broken copies of the RTL
(tag not compared, timeout one tick early, highest task first, no ADDR_INC
carry, no TRIG restart, seq stuck, no APPEND_REC bytes, EN not gating, phase
ignored, wrong source code, length 9 kept, due flag not cleared, ALARM prio,
...). Each broken copy fails at least one check.

### 10.3 How to run it and read the output

```
cd smart_serial_controller_v2
sh sim/run_unit.sh hub
# or by hand:
iverilog -g2005 -Wall -o build/tb_hub.vvp sim/unit/tb_hub.v rtl/hub/hub.v \
    rtl/noc/noc_pkt_tx.v rtl/noc/noc_pkt_rx.v rtl/top/ssc2_timebase.v
vvp -n build/tb_hub.vvp
```

It runs about 450 simulated "ms" (about 1.1 million clocks) in about 4
seconds. A good run ends with:

```
hub: 59 packets, 40 requests, 19 records, 25 XFER_REQ answered
ALL 197 CHECKS PASSED
```

- The `hub:` line is a summary: packets the hub sent, task runs (requests),
  RECORDs, and replies the responder model gave.
- A failing check prints `FAIL: <part>: <what> (t=<time>)`. The letter at
  the start is the part from the table above. For a wrong record it also
  prints the sequence number and data it got and expected.
- The last line is `m OF n CHECKS FAILED` if anything went wrong.
- If the hub hangs, a watchdog stops the run after 1000 simulated ms with
  `FAIL: watchdog - the test did not finish`.

## 11. Notes for the integrator

- Instantiate with `my_id = 1` (the core uses `N_HUB`). There are no
  parameters. The helpers use DEPTH = 4, as the mesh does.
- `in_*` is the mesh `rx_*` port of node 1 and `out_*` its `tx_*` port.
- The hub never blocks the network: it always takes incoming packets.
- `ev_error` and `ev_record` are 1-clock pulses for INT2_STATUS bits 3 and
  4.
- The boot ROM's tasks work as written. W0 is written before W3, and the W3
  write loads the countdown.

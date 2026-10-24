# DMA writer (`rtl/hub/dma_writer.v`) and AXI memory model (`sim/models/axi_mem_model.v`)

The DMA writer is node 2 of the network-on-chip. It is section 8 of
`docs/SPEC.md`, and it is the block behind slides 18 and 23: "An AXI memory
model checks that every record lands at the right address, the pointer
wraps, and the interrupt comes after 32 records".

## 1. What the block does

The sensor hub (node 1) reads the sensors. For every good reading it sends a
16-byte **RECORD** packet to node 2. The DMA writer copies each record into
memory (DDR on the Zynq, or the 4 KB block RAM in the FPGA-only build). It
uses one AXI4 burst per record.

The records go into a **ring buffer**. The ARM does not have to look at
every record. The DMA writer raises an interrupt event (`ev_batch`) when 32
records are waiting (the default), or when the oldest waiting record is
1 second old. The ARM then reads all new records in one go.

A record has this layout (SPEC 3.3). It is stored exactly like this, byte 0
at the lowest address:

| byte | field |
|---|---|
| 0..3  | time stamp in µs (little-endian) |
| 4     | source engine code (1 UART, 2 SPI, 3 I2C, 4 SE) |
| 5     | number of valid data bytes (0..8) |
| 6..7  | sequence number (little-endian) |
| 8..15 | data bytes |

The DMA writer does not look inside the record. It just copies 16 bytes.

## 2. The ring buffer

```
   DMA_BASE                                          DMA_BASE + 16*SIZE
      |                                                      |
      v                                                      v
      +------+------+------+------+------+------+------+------+
      | slot | slot | slot | slot | slot | slot | slot | slot |   SIZE = 2^SIZE_LOG2
      |  0   |  1   |  2   |  3   |  4   |  5   |  6   |  7   |   slots of 16 bytes
      +------+------+------+------+------+------+------+------+
                ^                         ^
                RD_IDX = 1                WR_IDX = 5
                (next slot the ARM        (next slot the DMA
                 will read)                writer will fill)

   unread records: slots 1, 2, 3, 4   ->   used = (wr - rd) mod SIZE = 4
```

- Slot `i` is at address `DMA_BASE + 16*i`.
- The hardware moves `WR_IDX` after every record it writes.
- Software moves `RD_IDX` after it has read records.
- **Empty:** `wr == rd`.
- **Full:** `(wr - rd) mod SIZE == SIZE - 1`. One slot always stays free.
  Without that free slot, "full" and "empty" would both look like `wr == rd`.
- When the ring is full, a new record is **dropped** and `OVERFLOW` goes
  up. Records the ARM has not read yet are never overwritten.

Example with SIZE = 8, rd = 1: slots 1..7 can be filled (7 records). Then
`wr = 0`, `(0 - 1) mod 8 = 7 = SIZE - 1`, so the ring is full. The next
record is dropped. If software sets `RD_IDX = 4`, then `(0 - 4) mod 8 = 4`,
so three more records fit.

## 3. Block diagram

```
     NoC (from router port LOCAL of node 2)
        in_valid, in_flit[33:0]           in_credit
              |                               ^
              v                               |
     +------------------------------------------------+
     | noc_pkt_rx (4-flit buffer, returns credits)    |
     |   hdr_valid / ptype / len    b_valid / b_data  |
     +----------+------------------------+------------+
                | header                 | bytes
                v                        v
     +-------------------------+   +---------------------------+
     | record state machine    |-->| record buffer, 128 bits   |
     | IDLE RECV CHECK AXI RESP|   | byte i at bits 8i+7..8i   |
     +----+-------------+------+   +-------------+-------------+
          |             |                        | [31:0], shifts 32 bits per beat
          |             |                        v
          |             |        AW: awaddr = BASE + wr*16, len 3, size 2, INCR
          |             +------> W : wdata, wstrb F, wlast on beat 3   --> AXI4 master
          |                      B : bresp                              <--  (to HP0 / BRAM)
          v
     +--------------------------------------------------+
     | WR_IDX, COUNT, OVERFLOW, AXI_ERR, IGNORED         |
     | PENDING + batch / time-out logic   <-- ms_tick    |---> ev_batch, ev_overflow, ev_axi_err
     | registers (APB region 0x700)       <-> reg_*     |
     +--------------------------------------------------+

     out_valid = 0, out_flit = 0: node 2 never sends packets.
```

## 4. How it works, step by step

The state machine handles one packet at a time.

| state | what happens | leaves when |
|---|---|---|
| IDLE  | Wait for a packet header from `noc_pkt_rx` and take it. Clear the record buffer to 16 zero bytes. A RECORD (type 3) while EN = 1 is kept. Any other packet, or any packet while EN = 0, is counted in `IGNORED` and only consumed. | a header is taken |
| RECV  | Take the payload bytes, one per clock. Byte `k < 16` of a kept record goes into byte `k` of the buffer. Bytes 16 and up are thrown away. A short record keeps its zeros at the end. | the last byte (`b_last`) |
| CHECK | Is the ring full? Yes: drop the record, `OVERFLOW++`, pulse `ev_overflow`. No: put `BASE + wr*16` into the AW register and raise `awvalid` and `wvalid` together. | after 1 clock |
| AXI   | AW and W are two separate handshakes. Each one finishes on its own. W sends 4 beats. After each beat the buffer shifts right by 32 bits, so the next word is always at bits 31..0. `wlast` is high on beat 3. | AW and all 4 beats are done |
| RESP  | `bready = 1`. Wait for `bvalid`. A `bresp` other than OKAY counts in `AXI_ERR` and pulses `ev_axi_err`. Then `wr = (wr + 1) mod SIZE`, `COUNT++`, `PENDING++`. | B is taken |

Only one burst is ever on the bus. The next packet header is not taken
until the B response of the current record has come back.

A packet with no payload (length 0) goes from IDLE straight to CHECK (a
RECORD, written as 16 zero bytes) or stays in IDLE (anything else).

### 4.1 Timing example

One RECORD of 16 bytes `10 11 12 ... 1F`, `DMA_BASE = 0x100`, `wr = 0`. The
memory model is always ready and answers B at once. This trace is from a
real simulation; clock numbers count from the clock in which the sender
started.

| clock | what you see |
|---|---|
| 3     | the HEAD flit arrives on `in_valid` |
| 4     | `hdr_valid = 1`, the DMA writer takes the header (IDLE -> RECV) |
| 9..27 | 16 payload bytes, one per clock while a flit is in the buffer (the sender only makes one flit every 5 clocks) |
| 28    | CHECK: the ring is not full |
| 29    | `awvalid = wvalid = 1`, `awaddr = 0x100`, `wdata = 0x13121110`: AW and beat 0 accepted |
| 30    | beat 1, `wdata = 0x17161514` |
| 31    | beat 2, `wdata = 0x1B1A1918` |
| 32    | beat 3, `wdata = 0x1F1E1D1C`, `wlast = 1` |
| 33    | RESP: `bvalid = bready = 1`, OKAY |
| 34    | `COUNT = 1`, `WR_IDX = 1`, `PENDING = 1` |

Notice the little-endian order: byte 0 (`0x10`) is in `wdata[7:0]` of
beat 0, so it lands at address `0x100`.

The AXI part takes 6 clocks (CHECK + 4 beats + B) when the slave never
waits. A whole record takes at least 23 clocks, so the DMA writer could
store more than 4 million records per second. The hub makes a few hundred.

### 4.2 Batches and the time-out

`PENDING` counts records written since the last `ev_batch`.

- **Count trigger:** when a record makes `PENDING` reach `BATCH`,
  `ev_batch` pulses in the next clock and `PENDING` goes to 0. With the
  reset value 32, the event comes right after record 32, 64, 96, ...
  `PENDING` never reads as 32: it goes from 31 straight to 0.
- **Time trigger:** a small counter (`age`) starts at 0 when the first
  pending record is written (PENDING goes from 0 to 1). It counts
  `ms_tick`s. On the TIMEOUT-th tick, `ev_batch` pulses and `PENDING` = 0.
  Later records do not restart the timer, because the timer belongs to the
  **oldest** pending record.
- `ms_tick` is not lined up with the record, so the event comes between
  TIMEOUT - 1 and TIMEOUT ms after the oldest record. With TIMEOUT = 1000
  that is 999..1000 ms.
- `TIMEOUT = 0` turns the time trigger off. `BATCH = 0` turns the count
  trigger off.
- If software lowers `BATCH` below the current `PENDING`, `ev_batch` comes
  at once (the next clock).

The core turns `ev_batch` into bit 0 of `INT2_STATUS` (DMA_BATCH), so the ARM
gets an interrupt.

### 4.3 RESET

Writing 1 to `DMA_CTRL[8]` empties the ring: `WR_IDX`, `RD_IDX`, `PENDING`,
`COUNT`, `OVERFLOW`, `AXI_ERR` and `IGNORED` become 0. Bit 0 of the same
write still sets EN, so `0x101` means "reset and keep running".

An AXI burst cannot be stopped half way. If RESET comes while a record is on
the bus, the burst finishes normally, but it is **not counted**: `WR_IDX`
stays 0 and `COUNT` and `PENDING` stay 0. The data does reach memory (in the
old slot), but the ring says it is empty, so software will not read it.

## 5. Ports

| port | dir | width | meaning |
|---|---|---|---|
| `clk` | in | 1 | 100 MHz clock |
| `rst_n` | in | 1 | asynchronous reset, active low |
| `ms_tick` | in | 1 | 1-clock pulse every millisecond (from `ssc2_timebase`) |
| `my_id` | in | 3 | node id (2). Not used: the node never sends |
| `reg_we` | in | 1 | register write strobe |
| `reg_re` | in | 1 | register read strobe. Not used (no pop-on-read registers) |
| `reg_addr` | in | 8 | byte offset inside region 0x700 (bits 1..0 ignored) |
| `reg_wdata` | in | 32 | write data |
| `reg_rdata` | out | 32 | read data, combinational from `reg_addr` |
| `in_valid`, `in_flit` | in | 1, 34 | flits from the router (LOCAL output of node 2) |
| `in_credit` | out | 1 | one pulse per freed slot of the 4-flit input buffer |
| `out_valid`, `out_flit` | out | 1, 34 | always 0 |
| `out_credit` | in | 1 | not used |
| `m_axi_awaddr` | out | 32 | `BASE + 16*wr`, always 16-byte aligned |
| `m_axi_awlen` | out | 8 | 3 (4 beats) |
| `m_axi_awsize` | out | 3 | 2 (4 bytes per beat) |
| `m_axi_awburst` | out | 2 | 1 (INCR) |
| `m_axi_awcache` | out | 4 | 0011 (normal memory, bufferable, not cached) |
| `m_axi_awprot` | out | 3 | 0 |
| `m_axi_awvalid`, `m_axi_awready` | out, in | 1, 1 | AW handshake |
| `m_axi_wdata` | out | 32 | beat k = record bytes 4k..4k+3, little-endian |
| `m_axi_wstrb` | out | 4 | F |
| `m_axi_wlast` | out | 1 | high on beat 3 |
| `m_axi_wvalid`, `m_axi_wready` | out, in | 1, 1 | W handshake |
| `m_axi_bresp`, `m_axi_bvalid` | in | 2, 1 | write response |
| `m_axi_bready` | out | 1 | high while waiting for B |
| `ev_batch` | out | 1 | 1-clock pulse: a batch of records is ready |
| `ev_overflow` | out | 1 | 1-clock pulse: a record was dropped (ring full) |
| `ev_axi_err` | out | 1 | 1-clock pulse: a burst got SLVERR or DECERR |

## 6. Registers (region 0x700)

| off | name | access | reset | fields |
|---|---|---|---|---|
| 0x00 | DMA_CTRL | RW | 0 | [0] EN. [8] RESET: write 1 to clear the ring and the counters (reads 0) |
| 0x04 | DMA_BASE | RW | 0 | ring start address. Bits 3..0 are ignored and read 0 |
| 0x08 | DMA_SIZE_LOG2 | RW | 6 | [3:0] ring size = 2^n slots. Writes are clamped to 1..12 (2..4096 slots) |
| 0x0C | DMA_WR_IDX | RO | 0 | next slot the hardware writes |
| 0x10 | DMA_RD_IDX | RW | 0 | next slot software reads. [11:0], used modulo the ring size |
| 0x14 | DMA_BATCH | RW | 32 | [7:0] records per `ev_batch`. 0 = count trigger off |
| 0x18 | DMA_TIMEOUT | RW | 1000 | [15:0] ms. 0 = time trigger off |
| 0x1C | COUNT | RO | 0 | records written (32 bits) |
| 0x20 | OVERFLOW | RO | 0 | records dropped because the ring was full |
| 0x24 | AXI_ERR | RO | 0 | bursts answered with a non-OKAY `bresp` |
| 0x28 | PENDING | RO | 0 | records written since the last `ev_batch` |
| 0x2C | STATUS | RO | 4 | [0] busy (handling a packet) [1] full [2] empty |
| 0x30 | IGNORED | RO | 0 | packets consumed but not written (other types, or EN = 0) |
| other | | | | read 0 |

All counters are 32 bits and wrap around.

**How software uses it.** Set `DMA_BASE`, `DMA_SIZE_LOG2` and write
`DMA_CTRL = 0x101`. On every DMA_BATCH interrupt: read `DMA_WR_IDX`, read
the slots from `RD_IDX` up to `WR_IDX - 1` (wrapping at SIZE), then write
the new `RD_IDX`. On the Zynq the ARM must invalidate its data cache for
those addresses first, because the DMA writer writes DDR behind the cache.

## 7. Design choices

The SPEC leaves some details open. These are the choices made, and why.

1. **BASE[3:0] are ignored.** Every slot is 16-byte aligned, so a 16-byte
   burst can never cross a 4 KB page (an AXI rule).
2. **EN is checked when the header arrives.** With EN = 0 a RECORD is
   consumed and counted in `IGNORED`, and nothing is written. The network
   never blocks. A record already being handled when EN goes to 0 is
   finished.
3. **A short record is padded with zeros. A RECORD with length 0 is
   written as 16 zero bytes.** That is what "pad with 0 if fewer" says.
4. **An AXI error still uses the slot.** The SPEC says "then wr = wr + 1"
   after any B. So a failed record moves `WR_IDX`, `COUNT` and `PENDING`
   like a good one. Software sees the problem in `AXI_ERR` and the
   interrupt. This keeps the ring simple: a bad memory region gives
   counted errors instead of a stuck DMA.
5. **RESET also clears RD_IDX.** The SPEC lists wr, pending and the
   counters. If RD_IDX stayed, the ring would look partly full of old
   records right after a reset. With RD_IDX = 0 too, the ring is empty.
6. **RESET during a burst:** the burst finishes (AXI does not allow a cut),
   but it is not counted (section 4.3). Events that happen in the same clock
   as a RESET are not counted either.
7. **SIZE_LOG2 is clamped to 1..12.** The indexes are used modulo the
   current size, and `WR_IDX`/`RD_IDX` read back modulo the size. Change
   SIZE_LOG2 together with a RESET.
8. **AW and W are offered in the same clock.** AXI allows the slave to take
   them in any order. `ssc2_axi_bram` takes W only after AW, the Zynq HP
   port may take W first. Both work, because the two handshakes are
   independent.
9. **Write data comes straight from a register.** The record buffer shifts
   by one word per beat, so `wdata` is always `buffer[31:0]`. There is no
   4:1 multiplexer, and `wdata` stays stable while `wready` is low, as AXI
   requires.
10. **One packet at a time.** The DMA writer could take the next header
    while it waits for B, but records come every few milliseconds, so the
    simpler design is fast enough (more than 4 million records/s).
11. **Self-repair:** if a header ever appears while the DMA writer waits for
    payload bytes (only a broken sender can cause that), it stops waiting,
    so it can never lock up.

## 8. The AXI memory model (`sim/models/axi_mem_model.v`)

The model is an AXI4 write slave for simulation. It stands in for DDR or the
block RAM. The system testbench can use it too.

### 8.1 Ports and parameters

Ports: `clk`, `rst_n`, and the slave side of the AW, W and B channels with
the same names as the DMA writer's master ports, prefixed `s_axi_`
(`s_axi_awaddr`, `s_axi_awlen`, `s_axi_awsize`, `s_axi_awburst`,
`s_axi_awcache`, `s_axi_awprot`, `s_axi_awvalid`, `s_axi_awready`,
`s_axi_wdata`, `s_axi_wstrb`, `s_axi_wlast`, `s_axi_wvalid`,
`s_axi_wready`, `s_axi_bresp`, `s_axi_bvalid`, `s_axi_bready`). The data bus
is 32 bits.

| parameter | default | meaning |
|---|---|---|
| `MEM_BYTES` | 65536 | memory size in bytes |
| `BASE_ADDR` | 0 | first address of the memory |
| `SEED` | 1 | random seed (runs are repeatable) |
| `AW_READY_PCT` | 100 | chance (%) that `awready` is high in a clock |
| `W_READY_PCT` | 100 | chance (%) that `wready` is high in a clock |
| `B_MAX_DELAY` | 0 | B comes 0..this many clocks after the last beat |
| `FILL` | 0x00 | memory contents at time 0 |
| `MAX_PRINT` | 20 | how many errors are printed (all are counted) |
| `VERBOSE` | 0 | 1 = print every burst |
| `NAME` | "axi_mem" | name used in messages |

### 8.2 How it works

- AW requests and W beats go into two separate queues (16 deep). So a W
  beat may come before its AW, together with it, or after it. AXI allows
  all three.
- As soon as a burst has its AW and a W beat, the beat is written into the
  memory, byte by byte as `wstrb` says.
- After the last beat, the B response is queued and given after a random
  0..`b_max_delay` clocks. `bvalid` never comes before the AW and the last
  W beat of that burst. It stays high until `bready`.
- `awready` and `wready` are random every clock, so the master sees
  back-pressure in many different patterns.

### 8.3 Protocol checks

Every problem prints `<NAME>: PROTOCOL ERROR: ...` and adds 1 to
`proto_errors`:

- `awvalid`/`wvalid` dropped before the handshake, or AW/W signals changed
  while waiting for ready;
- X on `awvalid`, `wvalid`, `bready`, on the AW fields, or on strobed data;
- burst type not INCR; `awsize` wider than the bus;
- a burst that crosses a 4 KB boundary;
- `wlast` early or missing, so the beat count must be exactly `awlen + 1`;
- `wstrb` bits outside the byte lanes of a narrow or unaligned beat.

An address outside the memory is answered with DECERR, counts in
`range_errors` and is not written.

### 8.4 Testbench controls

| call | what it does |
|---|---|
| `u_mem.set_ready(aw_pct, w_pct, b_max)` | change the back-pressure at run time |
| `u_mem.inject_bresp(n, resp)` | the next n bursts answer `resp` (2 = SLVERR, 3 = DECERR); their data is not stored |
| `u_mem.rd8(addr)`, `u_mem.rd32(addr)` | backdoor read (`rd32` is little-endian) |
| `u_mem.wr8(addr, v)`, `u_mem.fill(v)` | backdoor write, fill all memory |
| `u_mem.report` | print one line of statistics |
| `u_mem.proto_errors`, `range_errors`, `n_bursts`, `n_beats`, `n_aw`, `n_b`, `err_bresp` | counters to check |

Example for a system testbench (DDR at 0x1000_0000):

```verilog
axi_mem_model #(.MEM_BYTES(65536), .BASE_ADDR(32'h1000_0000), .SEED(5),
                .AW_READY_PCT(70), .W_READY_PCT(70), .B_MAX_DELAY(3)) u_ddr (
    .clk(clk), .rst_n(rst_n),
    .s_axi_awaddr(m_axi_awaddr), .s_axi_awlen(m_axi_awlen), .s_axi_awsize(m_axi_awsize),
    .s_axi_awburst(m_axi_awburst), .s_axi_awcache(m_axi_awcache), .s_axi_awprot(m_axi_awprot),
    .s_axi_awvalid(m_axi_awvalid), .s_axi_awready(m_axi_awready),
    .s_axi_wdata(m_axi_wdata), .s_axi_wstrb(m_axi_wstrb), .s_axi_wlast(m_axi_wlast),
    .s_axi_wvalid(m_axi_wvalid), .s_axi_wready(m_axi_wready),
    .s_axi_bresp(m_axi_bresp), .s_axi_bvalid(m_axi_bvalid), .s_axi_bready(m_axi_bready));
// at the end:  if (u_ddr.proto_errors != 0) ... fail
```

## 9. The testbench (`sim/unit/tb_dma_writer.v`)

### 9.1 Set-up

```
   noc_pkt_tx  --link-->  dma_writer  --AXI4-->  axi_mem_model (16 KB at 0x1000_0000)
   (the TB sends packets)    (DUT)                random ready / B delays

   axi_mem_model "selftest"  <-- driven by hand by the TB (section M)
```

The testbench keeps three things next to the DUT:

1. a **model of the registers** (wr, rd, COUNT, OVERFLOW, PENDING, ...),
   updated every time it sends a packet;
2. a **list of expected bursts** (address and 16 bytes), in order;
3. a **shadow copy** of the whole memory.

A **monitor** watches the AXI bus every clock. It checks each burst against
the list (address and every data beat), the fixed fields (awlen 3, awsize 2,
INCR, awcache 0011, awprot 0, wstrb F, wlast on beat 3), that a new burst
never starts before the B of the previous one, that `out_valid` stays 0, and
that every event is exactly one clock long. Comparing the memory with the
shadow copy proves that each record landed at `BASE + i*16` with the right
16 bytes **and that nothing else in memory changed**.

### 9.2 What each part proves

| part | what it does | what it proves |
|---|---|---|
| T0  | read every register after reset | reset values; unused addresses read 0 |
| T1  | EN = 0, send a RECORD and a DATA packet | both consumed, IGNORED = 2, no AXI traffic |
| T2  | BASE = 0x1000_100C, send 32 records, then 8 more | BASE[3:0] ignored; `ev_batch` exactly once, the clock after the B of record 32; PENDING back to 0, then 8 |
| T3  | RD_IDX = 40, send 30 records | WR_IDX wraps (40 + 30) mod 64 = 6; record 64 is in slot 0; second `ev_batch` after record 64 |
| T4  | send 36 records into a ring with 33 free slots | 3 dropped, OVERFLOW = 3, 3 `ev_overflow` pulses, STATUS = full, slot rd-1 untouched; after RD_IDX moves, writing resumes |
| T5  | inject SLVERR, then DECERR | AXI_ERR = 1, then 2, `ev_axi_err` pulses; COUNT and WR_IDX still move |
| T6  | send types 0, 1, 2, 4, 5, 6, 7 with 0..63 bytes, then a RECORD | all counted in IGNORED, no burst; the RECORD after them is correct |
| T7  | RECORDs of 0, 5, 1, 15, 17, 40, 63, 16 bytes | zero padding of short records, cut of long ones, stream stays in step |
| T8  | RESET; then RESET while AW is held off (W beats already taken) | all counters, WR_IDX and RD_IDX cleared; the stuck burst finishes but is not counted; W-before-AW works |
| T9  | SIZE_LOG2 = 0, 15, 13, 2; rings of 4 and 2 slots; BATCH = 0 and lowered | clamp to 1..12; tiny rings fill, drop and wrap; a short record after a dropped one is still zero padded; BATCH = 0 gives no event; lowering BATCH fires at once |
| T10 | TIMEOUT = 3 with 1 ms = 200 clocks | `ev_batch` on the 3rd ms tick after the **oldest** record (a later record does not restart it); TIMEOUT = 0 gives no event for 10 ms; setting it again fires at the next tick |
| T11 | 30 rounds of random packets (types, lengths 0..63, gaps), random back-pressure, 8-slot ring, BATCH = 7, random RD_IDX moves, one round with EN = 0 | registers match the model after every round; one `ev_batch` per 7 records; memory matches |
| end | whole-run checks | monitor saw no problem, every expected burst happened and no extra one, zero protocol errors, no address outside the memory |
| M   | the second model is driven by hand with good and bad bursts | the checker accepts good bursts (also W before AW and narrow bursts) and catches each kind of protocol error exactly once; SLVERR injection and DECERR work |

### 9.3 How to run it and read the output

```
cd smart_serial_controller_v2
sh sim/run_unit.sh dma_writer
```

The log is in `build/tb_dma_writer.log`. A good run looks like this:

```
T0: reset values
T1: EN = 0, packets are consumed and ignored
...
T10: time-out batch (1 ms = 200 clocks)
    time-out ev_batch 527 clocks after the oldest record
T11: random traffic, random back-pressure, 8-slot ring, BATCH 7
Whole run
axi_mem: 217 bursts, 868 beats, 217 B responses (2 not OKAY), 0 protocol errors, 0 range errors
M: memory model self-test (the PROTOCOL ERROR lines below are deliberate)
axi_mem_selftest: PROTOCOL ERROR: wlast missing on beat awlen (t=...)
...
ALL 242 CHECKS PASSED
```

- Each `Tn:` line starts a part of the test.
- `FAIL: <what>` lines name the check that failed. Lines starting with
  `MONITOR:` come from the bus monitor, and `mem[...] = .., expected ..`
  lines show the first wrong bytes in memory.
- The `axi_mem:` line must show **0 protocol errors**. The `PROTOCOL ERROR`
  lines after "M:" come from the self-test model and are deliberate: the
  test checks that each one is counted.
- The last line is `ALL n CHECKS PASSED` or `m OF n CHECKS FAILED`. If the
  test hangs, the watchdog prints `WATCHDOG: ...` and a failure line.

The test runs in under one second of wall time.

The tests were also checked against deliberately broken copies of the RTL
(wrong full test, `wlast` on the wrong beat, byte order swapped, batch one
record late, time-out restarted by every record, RESET not cutting the
count, no zero padding, wrong awcache, BASE bits shifted, ...). Every broken
copy fails at least one check.

## 10. Synthesis

`yosys synth_xilinx` (with `noc_pkt_rx`): about 1070 LUTs, 450 flip-flops,
6 RAM32M (the 4-flit input buffer) and 80 CARRY4. Most of the logic is the
128-bit record buffer, the five 32-bit counters and the register read
multiplexer. That is about 2 % of the Zynq-7020. The longest paths are a
32-bit add and compare (PENDING against BATCH) and a 28-bit add (the burst
address), well inside 10 ns.

# Network-on-chip: router, mesh and packet helpers

Files: `rtl/noc/noc_router.v`, `rtl/noc/noc_mesh.v`, `rtl/noc/noc_pkt_tx.v`,
`rtl/noc/noc_pkt_rx.v`.
Tests: `sim/unit/tb_noc_router.v`, `sim/unit/tb_noc_mesh.v`,
`sim/unit/tb_noc_pkt.v`.
Contract: `docs/SPEC.md`, sections 1 and 3.

This is the block behind slides 15 to 17: the flit format (slide 15), a
packet crossing the mesh from the I2C node to the hub (slide 16), and the
PC asking the SPI flash for its ID (slide 17).

## 1. What the block does

In v1 every engine (UART, SPI, I2C) talked only to the CPU. If the PC wanted
the flash ID, the CPU had to read the UART, drive the SPI and write the
answer back. In v2 the engines talk to **each other** over a small
network-on-chip (NoC). The CPU only sets things up.

The network has six **nodes** in a 3 × 2 grid (a "mesh"):

```
            x=0                  x=1                  x=2
   y=0   node 0  SERIAL ENGINE   node 1  SENSOR HUB   node 2  DMA WRITER
   y=1   node 3  UART            node 4  SPI          node 5  I2C
```

- The node id is `y*3 + x`.
- Every node has one **router**. Neighbouring routers are joined by two
  **links**, one in each direction.
- A node sends a **packet** (a header plus 0..63 payload bytes). The
  packet is cut into 34-bit **flits**, and the flits hop from router to
  router until they reach the destination node.

The NoC has four pieces:

| file | what it is |
|---|---|
| `noc_router.v` | one 5-port router (LOCAL, NORTH, EAST, SOUTH, WEST) |
| `noc_mesh.v` | six routers wired into the 3 × 2 mesh (only wires, no logic) |
| `noc_pkt_tx.v` | packet sender: turns "header + bytes" into flits for the network |
| `noc_pkt_rx.v` | packet receiver: turns flits back into "header + bytes" |

Every node module (the NIs, the hub, the DMA writer) uses one
`noc_pkt_tx` and one `noc_pkt_rx`. So the flit format lives in one place,
and no node has to know about flits or credits.

**Why a network and not a bus?** On a shared bus only one transfer can
happen at a time. In the mesh, the SPI reply to the UART (4 → 3) and the
I2C reply to the hub (5 → 4 → 1) use different links, so they move at the
same time. The mesh also grows: the same router works in a 4 × 4 mesh
(the testbench checks that).

## 2. Block diagram

### 2.1 The mesh

```
          tx/rx 0               tx/rx 1               tx/rx 2
             |                     |                     |
       +-----L-----+         +-----L-----+         +-----L-----+
       | router 0  |E <--> W | router 1  |E <--> W | router 2  |
       |  (0,0)    |         |  (1,0)    |         |  (2,0)    |
       +-----S-----+         +-----S-----+         +-----S-----+
             ^                     ^                     ^
             v                     v                     v
       +-----N-----+         +-----N-----+         +-----N-----+
       | router 3  |E <--> W | router 4  |E <--> W | router 5  |
       |  (0,1)    |         |  (1,1)    |         |  (2,1)    |
       +-----L-----+         +-----L-----+         +-----L-----+
             |                     |                     |
          tx/rx 3               tx/rx 4               tx/rx 5

   L = LOCAL port (the node itself)    <--> = two links, one each way
   Ports on the edge (router 0 NORTH, router 2 EAST, ...) are tied off.
```

### 2.2 Inside one router

```
              input side (x5)                          output side (x5)
            +---------------------+              +---------------------------+
 in_valid ->| 4-flit FIFO         |  front flit  | arbiter: prio first, then |
 in_flit  ->| (first-word         |------------->| round-robin; wormhole lock|
            |  fall-through)      |  + route     | credit counter (0..4)     |--> out_valid
 in_credit<-| credit pulse when a |  (XY table)  | output register           |--> out_flit
            | flit leaves         |<-------------| (34 bit)                  |<-- out_credit
            +---------------------+   "taken"    +---------------------------+
```

Each of the 5 inputs has a FIFO. Each of the 5 outputs has an arbiter, a
lock, a credit counter and an output register. Any input can reach any
output through a 5 × 5 crossbar (the multiplexer in front of each output
register).

### 2.3 A node and its helpers

```
         node logic (NI, hub, DMA writer)
     header + bytes                header + bytes
          |                              ^
          v                              |
   +--------------+                +--------------+
   | noc_pkt_tx   |                | noc_pkt_rx   |
   | packs bytes  |                | 4-flit buffer|
   | credit count |                | unpacks bytes|
   +------+-------+                +------+-------+
    out_valid/out_flit  ^ out_credit   in_valid/in_flit   | in_credit
          v             |                     ^           v
        mesh tx_* (into router LOCAL)       mesh rx_* (from router LOCAL)
```

In every node module `out_*` means **to** the network and `in_*` means
**from** the network.

## 3. Flits and packets

A flit is 34 bits: `{kind[1:0], data[31:0]}`.

| kind | name | meaning |
|---|---|---|
| 00 | HEAD | first flit of a packet that has a payload |
| 01 | BODY | middle payload flit |
| 10 | TAIL | last payload flit (it carries data too) |
| 11 | SINGLE | header-only packet (length 0) |

The head flit's `data` holds the header (slide 15):

| bits | field | meaning |
|---|---|---|
| [31:29] | type | 0 DATA, 1 XFER_REQ, 2 XFER_RESP, 3 RECORD, 4 ALARM |
| [28:26] | dest | destination node |
| [25:23] | source | sending node |
| [22] | prio | 1 = urgent, wins arbitration |
| [21:16] | length | payload bytes, 0..63 |
| [15:8] | tag | free for the sender, copied into replies |
| [7:0] | arg | I2C address, SPI chip-select, SM index |

The payload goes into BODY/TAIL flits, 4 bytes each, **little-endian**:
byte 0 of a flit is in `data[7:0]`. A packet with `length` bytes has
`ceil(length/4)` payload flits. Unused bytes of the TAIL are 0.

**Example (slide 16).** The I2C node answers the hub's temperature request
with `XFER_RESP [00][0C 80]` (status OK, then the two TMP2 bytes). It is
two flits:

```
HEAD  0_4683214B   type 2, dest 1, src 5, prio 0, length 3, tag 21, arg 4B
TAIL  2_00800C00   byte 0 = 00, byte 1 = 0C, byte 2 = 80, byte 3 = 0 (unused)
```

## 4. Ports and parameters

The NoC has **no CPU registers**. The core counts packets on each node's
link for `NI_STAT[n]` (SPEC 9), outside these modules.

### 4.1 `noc_router`

```verilog
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

| parameter | default | meaning |
|---|---|---|
| `X`, `Y` | 0, 0 | position of this router in the mesh |
| `W`, `H` | 3, 2 | mesh size (nodes = W × H) |
| `IDW` | 3 | width of the dest field (node ids 0 .. 2^IDW - 1) |
| `DEST_LSB` | 26 | dest = `flit[DEST_LSB +: IDW]` |
| `PRIO_BIT` | 22 | prio = `flit[PRIO_BIT]` |
| `DEPTH` | 4 | flits per input FIFO = starting credits of every sender |

| port | dir | width | meaning |
|---|---|---|---|
| `clk`, `rst_n` | in | 1 | 100 MHz clock, asynchronous active-low reset |
| `in_valid[p]` | in | 5 | a flit arrives on input p this clock |
| `in_flit` | in | 170 | the flits, port p at `[34p +: 34]` |
| `in_credit[p]` | out | 5 | one-clock pulse: a flit left input FIFO p (a register) |
| `out_valid[p]` | out | 5 | a flit leaves on output p this clock (a register) |
| `out_flit` | out | 170 | the flits, port p at `[34p +: 34]` (a register) |
| `out_credit[p]` | in | 5 | pulse from the next router (or node): it freed one slot |

The port numbers are 0 LOCAL, 1 NORTH (to y-1), 2 EAST (to x+1),
3 SOUTH (to y+1), 4 WEST (to x-1).

There are also two debug wires, `dbg_level` (FIFO level per input) and
`dbg_cred` (credits per output), 8 bits per port. Nothing inside uses them.
The testbenches read them to check that every counter is back at its start
value at the end.

### 4.2 `noc_mesh`

```verilog
module noc_mesh #(parameter W = 3, parameter H = 2, parameter N = 6,
                  parameter IDW = 3, parameter DEST_LSB = 26,
                  parameter PRIO_BIT = 22, parameter DEPTH = 4) (
    input  wire            clk, rst_n,
    input  wire [N-1:0]    tx_valid,     // node -> network (router LOCAL input)
    input  wire [N*34-1:0] tx_flit,
    output wire [N-1:0]    tx_credit,
    output wire [N-1:0]    rx_valid,     // network -> node (router LOCAL output)
    output wire [N*34-1:0] rx_flit,
    input  wire [N-1:0]    rx_credit);
```

| port | dir | meaning |
|---|---|---|
| `tx_valid[n]`, `tx_flit[34n +: 34]` | in | node n sends a flit (from its `noc_pkt_tx`) |
| `tx_credit[n]` | out | router n freed a slot of its LOCAL input FIFO |
| `rx_valid[n]`, `rx_flit[34n +: 34]` | out | a flit for node n (into its `noc_pkt_rx`) |
| `rx_credit[n]` | in | node n freed a slot of its receive buffer |

`N` must equal `W*H`. Router n sits at `x = n % W`, `y = n / W`. The mesh
has no logic: it only connects `EAST out` of router (x,y) to `WEST in` of
router (x+1,y), `SOUTH out` to the `NORTH in` below, and so on, with each
credit wire going back the other way. On the edge of the mesh `in_valid`,
`in_flit` and `out_credit` are tied to 0.

### 4.3 `noc_pkt_tx` (packet sender)

```verilog
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
```

| port | dir | meaning |
|---|---|---|
| `start` | in | take the header fields now. Only looked at while `hdr_ready = 1`. |
| `hdr_ready` | out | 1 = idle, a new packet can start |
| `ptype`, `dest`, `src`, `prio`, `len`, `tag`, `arg` | in | header fields, sampled in the `start` clock |
| `b_valid`, `b_data` | in | the next payload byte |
| `b_ready` | out | 1 = a byte is taken in this clock if `b_valid = 1` |
| `out_valid`, `out_flit` | out | flits to the router (registers) |
| `out_credit` | in | credit pulses from the router's LOCAL input |

How to use it: wait for `hdr_ready`, put the header on the inputs and pulse
`start`. Then give exactly `len` bytes; each one is taken in a clock where
`b_valid` and `b_ready` are both 1. With `len = 0` no byte is asked for and
one SINGLE flit is sent.

### 4.4 `noc_pkt_rx` (packet receiver)

```verilog
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

| port | dir | meaning |
|---|---|---|
| `in_valid`, `in_flit` | in | flits from the router's LOCAL output |
| `in_credit` | out | one-clock pulse every time a flit leaves the buffer (a register) |
| `hdr_valid` | out | the header of the next packet is on `ptype .. arg` |
| `ptype .. arg` | out | header fields, straight from the flit at the front |
| `hdr_ready` | in | the node takes the header (in a clock where `hdr_valid = 1`) |
| `b_valid`, `b_data` | out | the next payload byte |
| `b_last` | out | this is the last byte of the packet |
| `b_ready` | in | the node takes the byte |

## 5. How it works, step by step

### 5.1 One hop through the router: 2 clocks

```
clock c    in_valid[p] = 1: the flit is written into FIFO p
clock c+1  the flit is at the FIFO front (first-word fall-through):
           route it, arbitrate, and if it wins and the output has a credit,
           copy it into the output register
clock c+2  out_valid = 1 on the chosen output (= clock c of the next router)
           in_credit[p] = 1 (its FIFO slot is free again)
```

So every router adds **2 clocks** (20 ns). The SPEC target is ≤ 3.

### 5.2 Routing (XY)

The route is decided only for HEAD and SINGLE flits, from
`dest = flit[28:26]`:

```
dx = dest % W, dy = dest / W
dx > X  -> EAST        dx < X  -> WEST          (first move along x)
dy > Y  -> SOUTH       dy < Y  -> NORTH         (then along y)
else    -> LOCAL       dest >= W*H -> LOCAL      (bad dest: delivered here)
```

The `%` and `/` are never built as hardware. For each possible dest value
(8 of them with IDW = 3) the output port is a constant computed when the
design is elaborated. The router only looks the dest up in this small
table.

Example: router 4 is at (1,1). A packet for node 1 (dx = 1, dy = 0) has
`dx == X`, then `dy < Y`, so it goes NORTH.

### 5.3 Wormhole switching

When a HEAD flit wins an output, that output is **locked** to its input.
The BODY and TAIL flits of the packet then follow on the same output,
without arbitration. When the TAIL has passed, the lock opens. A SINGLE
flit passes without locking. So the flits of two packets are never mixed on
one link, and the receiver can rebuild packets with no packet number.

The cost: a packet that stalls in the middle keeps its path locked. Another
packet that needs the same output waits, even if it is urgent.

### 5.4 Arbitration

Each **free** (unlocked) output looks at all inputs whose front flit is a
HEAD/SINGLE that wants this output:

1. If any of them has `prio = 1`, only those take part.
2. Among them it is **round-robin**: the search starts at the input after
   the one that won this output last time.

Each output has its own pointer, shared by both priority classes. One input
can be granted by at most one output (it has only one front flit), and
each output takes at most one flit per clock. The five outputs work in
parallel, so up to 5 flits cross the router in one clock.

### 5.5 Credits (flow control)

Nothing is ever dropped. Instead, each output keeps a **credit counter** =
the number of free slots in the FIFO on the other end of the link:

- It starts at DEPTH (4).
- Sending a flit costs one credit.
- Each `out_credit` pulse gives one back.
- A flit is only sent while the counter is above 0.

The `noc_pkt_tx` of each node has the same counter for the router's LOCAL
input, and `noc_pkt_rx` sends the credits for its own 4-flit buffer.

**The credit loop is 4 clocks:**

```
clock 0  the flit is on the link; it is written into the next FIFO
clock 1  it is at the front of that FIFO and leaves it (if it wins)
clock 2  in_credit pulses back to the sender
clock 3  the sender's counter has the credit again: a new flit is loaded
clock 4  the new flit is on the link, in the same slot
```

So each slot is used once every 4 clocks. With 4 credits the sender can keep
sending one flit per clock for ever. DEPTH = 4 is exactly the smallest size
that gives full speed.

### 5.6 The packet sender (`noc_pkt_tx`)

A small state machine with four states:

| state | `hdr_ready` / `b_ready` | what happens |
|---|---|---|
| IDLE | 1 / 0 | on `start`: build the head word, `left = len`, go to HEAD |
| HEAD | 0 / 0 | when a credit is free: send HEAD (or SINGLE if len = 0) |
| FILL | 0 / 1 | take one byte per clock into the 32-bit word (byte i at `[8i+7:8i]`); after 4 bytes or the last byte go to SEND |
| SEND | 0 / 0 | when a credit is free: send BODY (or TAIL if no byte is left); back to FILL, or IDLE after the TAIL |

So a full 4-byte flit takes 5 clocks (4 FILL + 1 SEND), and a packet needs 2
clocks before its first payload byte (IDLE, HEAD). This sets the speed of a
single stream (section 7.3).

### 5.7 The packet receiver (`noc_pkt_rx`)

- Incoming flits go into a DEPTH-entry FIFO. This is the buffer that the
  router's LOCAL output counts credits for. A flit is visible at the front
  one clock after it arrives.
- **Expecting a header:** if the front flit is HEAD/SINGLE, `hdr_valid = 1`
  and the header fields are shown. `hdr_ready` takes it and removes the head
  flit from the FIFO.
- **In the payload:** `b_valid = 1` while the front flit has bytes left.
  `b_data` is byte `idx` of it. Each `b_ready` takes one byte. When the 4th
  byte (or the last byte of the packet) is taken, the flit leaves the FIFO.
  `b_last` marks the last byte.
- Every flit that leaves makes `in_credit` pulse one clock later.
- **Self-repair:** a BODY/TAIL flit that shows up where a header is
  expected (a correct sender never does that) is thrown away, and its
  credit is still returned. A HEAD flit with length 0 is treated as
  header-only. So the receiver always gets back in step at the next head.

## 6. Timing example: slide 16, node 5 → node 1

The I2C node (5) sends `XFER_RESP [00][0C 80]` (3 bytes, tag 21, arg 4B) to
the hub (1). The XY path is 5 → 4 (WEST), then 4 → 1 (NORTH): **2 hops,
3 routers (R5, R4, R1)**. Nothing else is in the network.

This trace comes from a small measurement testbench (the 3 × 2 `noc_mesh`,
a `noc_pkt_tx` at node 5 and an always-ready `noc_pkt_rx` at node 1). Each
row is one clock. A 1 under a link means a flit is on it in that clock; "cr"
is the credit pulse going back on that link.

```
clk  | start st | node5->R5 link   cr  | R5->R4 cr | R4->R1 cr | R1->node1 link   cr  | hv bv byte
   7 |  1    0  |  0                0  |  0   0   |  0   0   |  0                0  | 0  0
   8 |  1    1  |  0                0  |  0   0   |  0   0   |  0                0  | 0  0
   9 |  0    2  |  1 0_4683214B     0  |  0   0   |  0   0   |  0                0  | 0  0
  10 |  0    2  |  0                0  |  0   0   |  0   0   |  0                0  | 0  0
  11 |  0    2  |  0                1  |  1   0   |  0   0   |  0                0  | 0  0
  12 |  0    3  |  0                0  |  0   0   |  0   0   |  0                0  | 0  0
  13 |  0    0  |  1 2_00800C00     0  |  0   1   |  1   0   |  0                0  | 0  0
  14 |  0    0  |  0                0  |  0   0   |  0   0   |  0                0  | 0  0
  15 |  0    0  |  0                1  |  1   0   |  0   1   |  1 0_4683214B     0  | 0  0
  16 |  0    0  |  0                0  |  0   0   |  0   0   |  0                0  | 1  0
  17 |  0    0  |  0                0  |  0   1   |  1   0   |  0                1  | 0  0
  18 |  0    0  |  0                0  |  0   0   |  0   0   |  0                0  | 0  0
  19 |  0    0  |  0                0  |  0   0   |  0   1   |  1 2_00800C00     0  | 0  0
  20 |  0    0  |  0                0  |  0   0   |  0   0   |  0                0  | 0  1  00
  21 |  0    0  |  0                0  |  0   0   |  0   0   |  0                0  | 0  1  0C
  22 |  0    0  |  0                0  |  0   0   |  0   0   |  0                0  | 0  1  80
  23 |  0    0  |  0                0  |  0   0   |  0   0   |  0                1  | 0  0
```

(`st` is the `noc_pkt_tx` state: 0 IDLE, 1 HEAD, 2 FILL, 3 SEND.)

Step by step:

| clock | what happens |
|---|---|
| 7 | node 5 pulses `start`; `noc_pkt_tx` takes the header (IDLE → HEAD) |
| 8 | HEAD state: a credit is free, so the head word goes into the output register |
| 9 | the HEAD flit is on the link into R5. `noc_pkt_tx` starts FILL and takes byte `00` |
| 10 | the head is at the front of R5's LOCAL FIFO. Dest 1 has dx = 1 < X = 2: WEST. It wins and is copied to the WEST register. Byte `0C` taken. |
| 11 | head on the R5 → R4 link. R5 gives node 5 its credit back. Byte `80` (the last one) taken. |
| 12 | SEND: the TAIL word goes into the output register |
| 13 | TAIL on the link into R5. In R4, dx = X and dy = 0 < Y = 1: NORTH, so the head is on the R4 → R1 link. R4 returns R5's credit. |
| 15 | R1 sends the head to its LOCAL port (dest = own id): it is on the link to node 1. The TAIL is one hop behind. |
| 16 | node 1's `noc_pkt_rx` shows `hdr_valid` (type 2, src 5, length 3, tag 21); the hub takes the header |
| 17 | node 1 returns the head's credit to R1 |
| 19 | the TAIL reaches node 1 |
| 20-22 | bytes `00`, `0C`, `80` come out, one per clock; `b_last` is 1 with `80` |
| 23 | the TAIL's credit goes back to R1. Every counter is back at 4. |

Results:

- **Per hop:** the head enters R5 in clock 9, R4 in 11, R1 in 13 and leaves
  R1 in 15: exactly **2 clocks per router**.
- **Head latency** (link into the first router → link out of the last):
  6 clocks = **60 ns**.
- **Whole reply** (`start` at node 5 → last byte taken at node 1):
  **15 clocks = 150 ns**. Of these, 6 are the network; the rest is the
  sender packing 3 bytes (the TAIL leaves node 5 4 clocks after the HEAD)
  and the receiver giving out 3 bytes.

## 7. Measured performance

### 7.1 Per-hop latency

`tb_noc_router` sends one packet from every input of ten routers to every
possible dest: the latency is **2 clocks for every case** (in_valid → out_valid).

### 7.2 Zero-load latency for all node pairs

From `tb_noc_mesh` (head flit: `tx_valid` at the source → `rx_valid` at the
dest, one packet at a time, in clocks of 10 ns):

```
      dest:  0  1  2  3  4  5
    src 0:   2  4  6  4  6  8
    src 1:   4  2  4  6  4  6
    src 2:   6  4  2  8  6  4
    src 3:   4  6  8  2  4  6
    src 4:   6  4  6  4  2  4
    src 5:   8  6  4  6  4  2
```

The rule is `latency = 2 × (hops + 1)` clocks: 2, 4, 6, 8 clocks for 0, 1,
2, 3 hops. A packet to its own node (0 hops) still goes through its router
(2 clocks). The longest path in the 3 × 2 mesh (0 ↔ 5, 3 hops) takes 80 ns.

The slide-16 path 5 → 1 is 2 hops: **6 clocks (60 ns)** for the head, and
**15 clocks (150 ns)** from the start of the reply to its last byte at the
hub (section 6).

### 7.3 Single-stream throughput

Measured with the same measurement testbench, one stream from node 5 to
node 1 (2 hops), receiver always ready:

| what | result |
|---|---|
| raw flits, ideal sink (credit back 1 clock after each flit) | 400 flits in 400 clocks = **1 flit/clock** = 4 payload bytes/clock = **400 MB/s** (3.2 Gbit/s) |
| `noc_pkt_tx` → `noc_pkt_rx`, 63-byte packets back to back | **0.773 bytes/clock** (77 MB/s), 81.5 clocks per packet |
| same, 16-byte packets | 0.718 bytes/clock (22.3 clocks per packet) |
| same, 4-byte packets | 0.549 bytes/clock (7.3 clocks per packet) |
| same, 1-byte packets | 0.238 bytes/clock (4.2 clocks per packet) |
| same, 0-byte (SINGLE) packets | one packet every 2.2 clocks |

So **the network itself runs at 1 flit per clock on every link**, and a
single stream between two nodes that use the helpers is limited by the
sender: `noc_pkt_tx` needs 5 clocks for each 4-byte flit and 2 clocks per
header, which gives at most 0.8 bytes/clock. The receiver gives out 1 byte
per clock. Both are far faster than any serial engine (a 1 MHz SPI moves
0.00125 bytes/clock).

Other numbers from the tests:

- One router, five streams that do not collide (a permutation): **5
  flits/clock** in total (`tb_noc_router` T1).
- Two full-speed streams into one output share it exactly 1:1 and the
  output still runs at 1 flit/clock (T3).
- Hot spot: all six nodes send 100 packets each to node 1 at once. Node 1
  receives **0.96 bytes/clock**, close to its receiver's limit of 1 byte
  per clock (`tb_noc_mesh` A4).

## 8. Design choices

1. **Registered outputs, registered credits.** `out_valid`, `out_flit` and
   `in_credit` come straight from flip-flops. There is no combinational
   path from one router into the next, so the clock speed does not depend
   on the mesh size. The price is 2 clocks per hop instead of 1.
2. **DEPTH = 4.** With the 4-clock credit loop (5.5), 4 slots are exactly
   enough for 1 flit per clock. A bigger FIFO would not be faster; a smaller
   one would leave bubbles.
3. **The route table is a constant**, built at elaboration from X, Y, W, H.
   No divider is synthesised, and the router is the same code at every
   position.
4. **Prio only matters at arbitration.** An urgent packet wins a free
   output, but it does not overtake the packet in front of it in the same
   FIFO, and it does not break a wormhole lock. The SPEC asks only for
   "wins arbitration".
5. **One round-robin pointer per output**, shared by both priority classes.
   It only moves when a new packet is granted (not for body flits).
6. **Stray flits.** A BODY/TAIL at the front of an input that owns no
   output can only come from a broken sender. The router throws it away and
   returns its credit, so that input never blocks for ever (the same rule as
   `noc_pkt_rx`).
7. **A flit that arrives at a full FIFO is ignored.** This can only happen
   if a sender ignores its credits. Correct senders (all of ours) never do.
8. **A dest of W*H or more goes LOCAL** (SPEC). With 3-bit ids, dest 6 and 7
   are delivered to the router where they entered. The node there consumes
   them (every NI must consume every packet).
9. **No deadlock.** XY routing never turns from y back to x, so no cycle of
   links can wait on itself. This only holds if every node keeps consuming
   its packets, which is why every NI must drain packets even with `en = 0`.
10. **Boundary ports** have `in_valid = 0` and `out_credit = 0`. Their 4
    starting credits are never refilled, which is harmless because XY
    routing never sends a valid dest there. Synthesis removes their FIFOs.
11. **The mesh is fully parameterised** (W, H, IDW, DEST_LSB, PRIO_BIT,
    DEPTH) for the scalability study; `tb_noc_mesh` also runs a 4 × 4 mesh
    with 4-bit ids.
12. **The helpers keep the format in one place.** No node builds flits
    itself. A node that wants to send a packet whose length it does not know
    yet must buffer the bytes first, because the length is in the header
    (the UART NI and the hub do this).

## 9. Size

Yosys `synth_xilinx -flatten` (Zynq-7020 cells):

| module | LUTs | flip-flops | other |
|---|---|---|---|
| `noc_router` (alone, all 5 ports) | 762 | 259 | 30 RAM32M (the five 4 × 34 FIFOs), 25 CARRY4, 142 MUXF7, 52 MUXF8 |
| `noc_mesh` 3 × 2 (6 routers) | 3268 | 1160 | 120 RAM32M (20 used inputs), 160 CARRY4, 557 MUXF7, 188 MUXF8 |
| `noc_pkt_tx` | 232 | 116 | 13 CARRY4, 47 MUXF7 |
| `noc_pkt_rx` | 40 | 19 | 6 RAM32M (the 4-flit buffer) |

The whole 3 × 2 mesh is about 6 % of the 53 200 LUTs of the Zynq-7020 (7 %
counting the LUTs used as RAM32M). In the mesh, the 10 boundary input FIFOs
disappear, because their `in_valid` is tied to 0. Most of a router is the
five 34-bit, 5-way multiplexers in front of the output registers and the
five 34-bit output registers.

Yosys also lists INV cells (one per flip-flop with an asynchronous
active-low reset). Vivado uses a single inverter for `rst_n` instead.

## 10. The testbenches

### 10.1 `tb_noc_pkt.v`: the helpers on one link

`noc_pkt_tx` is wired straight to `noc_pkt_rx` (valid/flit one way, credit
back), which is exactly one NoC link.

| part | what it proves |
|---|---|
| A | 399 random packets (length 0..63 with the corner cases 0, 1, 3, 4, 5, 8, 63 first, every header field random, random gaps on both sides): every field and byte arrives in order. A link monitor checks the flit kinds, the flit count per packet and that unused TAIL bytes are 0. |
| B | credit flow control: the receiver stops, the 400th packet (63 bytes = 17 flits) is sent, and only DEPTH = 4 flits cross the link. After release the packet is complete and all credits come back. |
| C | a stray BODY flit in front of a header is thrown away (with its credit) |
| D | `ssc2_timebase`: `us_tick` every 100 clocks, `ms_tick` every US_PER_MS µs |

Output: `A: 399 packets, 3548 flits, 399 good` and `ALL 19 CHECKS PASSED`.

### 10.2 `tb_noc_router.v`: one router with raw flits

- **Part 1 (routing):** ten routers at different places in different meshes
  (all six of the 3 × 2 mesh, plus 4 × 4, 4 × 2, 2 × 2 and 3 × 3 positions,
  IDW 2..4). Every input of every router gets one packet for every possible
  dest. The flit must leave on the XY port, unchanged, the TAIL on the same
  port, after exactly 2 clocks, with one `in_credit` pulse per flit.
- **Part 2:** the centre router of a 3 × 3 mesh (all five outputs used).
  Every input has a source that obeys the credits and every output a sink
  that models a 4-flit buffer and returns credits at a random rate (or not
  at all). A **scoreboard** checks every flit that leaves: right output,
  unchanged, packets never mixed, in order per (input, output) flow,
  delivered exactly once.

| test | what it proves |
|---|---|
| T1 | 1 flit/clock for a 200-flit packet, 100 SINGLEs, 50 back-to-back 4-flit packets; five parallel streams at 1 flit/clock each |
| T2 | prio beats round-robin: the grant order of 20 queued packets equals a reference model |
| T3 | round-robin fairness: every 5 grants in a row serve 5 different inputs; two full-speed streams alternate 1:1 |
| T4 | wormhole: two 7-flit packets never mix; 9 slow packets from 3 inputs never mix; an urgent packet waits for the TAIL of the packet that holds the output |
| T5 | no credits: exactly DEPTH flits leave, then nothing more for 200 clocks, and the source stops after 2 × DEPTH flits (4 delivered to the stopped sink, 4 waiting in the router's input FIFO); after release all 12 arrive |
| T6 | 2000 random packets (random dest, 1..8 flits, 20 % urgent) with random rates and stalls on all ports |
| T7 | a stray BODY at an idle input is dropped and its credit returned |
| T8 | end: `in_credit` pulses = flits sent, every credit counter = DEPTH, every FIFO empty |

Output:

```
R: 500 packets routed (135 with dest >= W*H -> LOCAL), latency 2..2 clocks per hop
T1: one 200-flit packet LOCAL->EAST in 200 clocks = 1.000 flits/clock
T6: 2000 random packets (9336 flits total so far) in 8281 clocks
T8: 2539 packets, 9339 flits through the main router
ALL 1547 CHECKS PASSED
```

### 10.3 `tb_noc_mesh.v`: the whole mesh with real helpers

- **Part A** uses `noc_mesh` with the SPEC parameters and a real
  `noc_pkt_tx` / `noc_pkt_rx` at every node. Each node owns a list of
  packets; the receiver checks every packet against the sender's list
  (right node, header and payload unchanged, in order per flow).
  - A1: zero-load latency for all 36 (source, dest) pairs (the table in 7.2).
  - A2: the slide-16 reply 5 → 1 (head 6 clocks, start → last byte 15).
  - A3: more than 5000 random packets, all-to-all (also to itself), lengths
    0..63, random prio/type/tag/arg, random sender gaps and receiver stalls.
    A **deadlock watchdog** stops the run if nothing arrives for 20000 clocks.
  - A4: hot spot, every node sends 100 packets to node 1.
  - A5: end state: all 30 router credit counters and all 6 sender counters
    back at 4, flits in = flits out, and no flit ever left on a boundary port.
- **Part B:** a 4 × 4 mesh (IDW 4, dest at bit 24) with raw flit sources
  and sinks, random rates and stalls, 3200 packets of 1..8 flits: every
  packet delivered once, unchanged, in order, never mixed; all 80 router and
  16 source credit counters back at 4.

Output:

```
A1: zero-load network latency (head flit: tx_valid -> rx_valid), clocks:
      dest:  0  1  2  3  4  5
    src 0:  2  4  6  4  6  8
    ...
A1: hops 0/1/2/3 -> 2/4/6/8 clocks = 2 + 2 per hop (2 per router)
A2: node 5 -> node 1 (2 hops), 3-byte XFER_RESP = HEAD + TAIL:
    head in -> head out 6 clocks, head in -> tail out 10 clocks
    start at node 5 -> last byte taken at node 1: 15 clocks
A3: 5105 random packets, 46950 flits, in 74342 clocks
A4: hot spot: 600 packets (19182 bytes) into node 1 in 19846 clocks = 0.96 bytes/clock
A: 5742 packets, 52609 flits through the 3x2 mesh
B: 4x4 mesh: 3200 packets, 12608 flits in 4620 clocks
ALL 19 CHECKS PASSED
```

### 10.4 How to run them and read the output

```
cd smart_serial_controller_v2
sh sim/run_unit.sh noc        # runs tb_noc_pkt, tb_noc_router and tb_noc_mesh
```

or one by hand:

```
iverilog -g2005 -Wall -o build/tb_noc_mesh.vvp sim/unit/tb_noc_mesh.v rtl/noc/noc_mesh.v \
    rtl/noc/noc_router.v rtl/noc/noc_pkt_tx.v rtl/noc/noc_pkt_rx.v
vvp -n build/tb_noc_mesh.vvp
```

`run_unit.sh` prints `### tb_noc_router` and then the last lines of each
log (the full log is in `build/<name>.log`), and at the end
`UNIT TESTS: 3 passed`. `tb_noc_pkt` and `tb_noc_router` take about 2 s;
`tb_noc_mesh` takes about 1.5 minutes (it moves 5742 + 3200 packets).

- Lines starting with a letter (`R:`, `T1:`, `A1:`, `B:`) are measurements
  and summaries.
- A failing check prints `FAIL: <what> (t=<time>)`. The scoreboards also
  print a detail line before it, for example `out 2: payload flit wrong ...`
  or `node 1: header mismatch src 5 len 3 tag 21`.
- The last line is `ALL n CHECKS PASSED` or `m OF n CHECKS FAILED`.
- If the network locks up, the mesh test prints
  `FAIL: DEADLOCK - nothing delivered for 20000 clocks` with a per-node list
  of packets started and received. Each test also has a watchdog on the
  total simulated time.

## 11. Notes for the integrator

- Connect node n's `noc_pkt_tx` to mesh `tx_*[n]` and its `noc_pkt_rx` to
  mesh `rx_*[n]`; credits go the other way (`tx_credit[n]` into the sender,
  `rx_credit[n]` from the receiver).
- Use the same DEPTH (4) everywhere: the sender's starting credits must
  equal the receiver's buffer size.
- Every node must keep consuming packets, also when it is disabled.
  Otherwise the packets behind it block, and in the worst case the whole
  mesh stops.
- The core counts head flits on each node link for `NI_STAT`; the mesh
  does not need to know about it.

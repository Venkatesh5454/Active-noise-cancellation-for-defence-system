# Serial-engine network interface (`rtl/se/ni_se.v`, node 0)

`ni_se` connects the serial engine's two state machines to the
network-on-chip (node 0 at (0,0), `docs/SPEC.md` section 5.6). It lets any
node send bytes to a program and lets a program's received data travel to
any node, without the CPU.

## 1. What it does

```
         network (router (0,0) LOCAL port)
            |  in_*                     ^ out_*
            v                           |
     +--------------+            +--------------------------+
     | noc_pkt_rx   |            | noc_pkt_tx               |
     +------+-------+            +-----------^--------------+
            | header: SM = arg[0]            | DATA packet: dest/prio of the SM,
            | bytes                          | arg = SM, up to 16 bytes
            v                                |
     byte -> TX word                   16-byte buffer  <- RX words
     ([7:0] or [31:24])                (1 or 4 bytes per word)
            |                                ^
            v                                |
       SM0/SM1 TX FIFO                 SM0/SM1 RX FIFO        (se_engine)
```

**Network to state machine.** Every payload byte of a packet that reaches
node 0, whatever its type, becomes one word in the TX FIFO of SM `arg[0]`.
The byte goes in bits [7:0], or in [31:24] when that SM's OUT shifts left,
so an LSB-first program (`uart_tx`) and an MSB-first program (`spi_master`)
both get the byte where they shift it out from.

**State machine to network.** Words from an SM's RX FIFO are collected in a
16-byte buffer:

- one byte per word: ISR[7:0] if the SM's IN shifts left, ISR[31:24] if it
  shifts right (where an 8-bit right-shifted byte ends up);
- or four bytes per word, little-endian, when `WORD4` is set for that SM.

A DATA packet goes to `DEST`/`PRIO` of that SM with `arg` = SM index as
soon as the RX FIFO is empty or the buffer holds 16 bytes. The header needs
the length, so the bytes are buffered first. SM0 and SM1 take turns.

## 2. Ports and configuration

The port list is the one in SPEC 5.6. The core drives the configuration
from `NI_CFG[0]` (0x500):

| bits | field | meaning |
|---|---|---|
| [0] | EN | 0: the NI never touches the FIFOs (the CPU owns them) but still drains packets |
| [6:4] | DEST | destination node of SM0's packets |
| [7] | PRIO | priority flag of SM0's packets |
| [17] | WORD4 SM0 | 4 bytes per RX word |
| [18] | WORD4 SM1 | 4 bytes per RX word |
| [22:20] | DEST SM1 | destination node of SM1's packets |
| [23] | PRIO SM1 | priority flag of SM1's packets |

`out_shift_left` / `in_shift_left` come straight from the SMs' SHIFTCTRL
registers.

## 3. Sharing with the CPU

The CPU can read and write the same FIFOs through SMs_TXF / SMs_RXF. The
core drives `stall = psel`, and `ni_se` never pushes or pops in a clock
where `stall` is 1, so the two never collide. Software should still use one
or the other: `sw/se_demos.c` writes `NI_CFG[0] = 0` before it feeds a
program itself.

## 4. Design choices

- One packet at a time per direction; the two directions are independent.
- A packet already being collected is finished when EN drops (what is in
  the buffer is sent); nothing new is popped.
- The first SM with data is served, then the other one gets the next turn,
  so a busy SM cannot starve the other.
- Size: about 300 LUTs and 200 flip-flops with its two packet helpers
  (yosys estimate).

## 5. Verification (`sim/unit/tb_ni_se.v`)

`sh sim/run_unit.sh ni_se` (a few seconds). The FIFOs are behavioural
models of the engine's 4-word FIFOs, so every push and pop is checked:

| part | what is checked |
|---|---|
| A | 300 random packets (all types, lengths 0..63, both SMs, OUT shift right/left): every byte becomes exactly one TX word of the right SM, in [7:0] or [31:24] |
| B | random RX words on both SMs, IN shift right/left, 1 or 4 bytes per word: each SM's byte stream arrives unchanged and in order, in DATA packets with the right dest/prio, arg = SM, source = 0, 1..16 bytes |
| C | a lone word is sent at once (1-byte packet); 20 queued words make packets of 16 and 4 bytes |
| D | EN = 0: nothing pushed or popped, every packet drained; after EN = 1 the waiting word is sent |
| all | random `stall`: no push/pop while stall = 1 or EN = 0, no push into a full FIFO, no pop from an empty one |

Expected end of the log: `ALL 16 CHECKS PASSED`. The system testbench also
uses `ni_se` end to end: the PC's "Hi!" travels UART -> network -> SM0 and
comes out of the `uart_tx` program on JD (`tb_ssc2_system`, test [6]), and
in the FPGA-only image the hub sends the last temperature to SM0 every
100 ms (`tb_zed2_standalone`, test [5]).

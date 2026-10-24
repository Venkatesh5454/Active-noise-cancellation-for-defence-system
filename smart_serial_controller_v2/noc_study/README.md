# NoC scalability study (slide 24, measurement 6)

Slide 24 promises to compare "bus vs crossbar vs NoC for 4, 8 and 16
nodes: LUTs, top clock, latency, throughput", and to report the result as
it is, "if the NoC only wins at 16 nodes, or not at all on this FPGA".
This folder is that study. The numbers are in [results.md](results.md).

## 1. What is compared

Three fabrics with exactly the same node interface as the v2 network
(`rtl/noc/noc_mesh.v`): 34-bit flits, credit flow control, a 4-flit
receive buffer at every node, priority bit, round-robin arbitration,
wormhole switching. So one traffic generator can drive all three, and every
fabric gets exactly the same packets.

| fabric | file | how it works | flits per clock (whole fabric) |
|---|---|---|---|
| bus | `rtl/study_bus.v` | one shared medium, one round-robin arbiter, a node keeps the bus for its whole packet | 1 |
| crossbar | `rtl/study_xbar.v` | N x N switch: one arbiter, lock and output register per output, a 4-flit FIFO per input (one big router) | up to N |
| mesh NoC | `rtl/study_mesh.v` -> `rtl/noc/noc_mesh.v` | the real v2 routers, XY routing; 2x2, 4x2 and 4x4 for N = 4, 8, 16 | up to (links in use) |

`study_mesh.v` is only a wrapper that picks W x H and the id width; the
routers are the same Verilog as in the v2 controller.

## 2. How it is measured

`tb_study.v` (see its header for every detail):

- **Traffic:** every node makes 4-flit packets (head, 2 body, tail) at a
  given rate, to uniformly random destinations. Packets wait in a source
  queue (64 packets), so a slow fabric cannot reduce the offered load.
- **Rates:** 0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7 and 1.0 flits per node per
  clock.
- **Latency:** clock the tail arrives minus clock the packet was made
  (includes queueing). Zero-load latency is 6 clocks for the bus and the
  crossbar and 6 + 2 x hops for the mesh.
- **Throughput:** flits that arrive in the measurement window divided by
  N x window. "sat" means the source queues filled up.
- **Correctness:** every flit is rebuilt and checked at the sink, so each
  run also proves "no lost, repeated or reordered packets" (each run ends
  with `ALL 24 CHECKS PASSED`).
- **Area:** yosys `synth_xilinx` for the Zynq-7000 (LUTs, FFs and the
  longest logic chain as a stand-in for clock speed). These are estimates;
  `vivado_study.tcl` gives the real LUTs and Fmax.

## 3. How to run it

Linux or WSL (Icarus Verilog and yosys installed):

```
cd smart_serial_controller_v2
bash noc_study/run_study.sh            # everything, about 10 minutes
bash noc_study/run_study.sh sim        # only the simulations
bash noc_study/run_study.sh synth      # only yosys
JOBS=4 bash noc_study/run_study.sh     # 4 simulations at a time
```

Vivado (real LUTs and Fmax, 20-60 minutes):

```
vivado -mode batch -nojournal -source noc_study/vivado_study.tcl            (synthesis)
vivado -mode batch -nojournal -source noc_study/vivado_study.tcl -tclargs impl   (routed)
```

or double-click `windows\5_noc_study_vivado.bat`. It writes
`noc_study/vivado_reports/summary.txt`; then `bash noc_study/run_study.sh
report` adds that table to `results.md`.

A quick sanity run of the study testbench is also part of the unit tests
(`sh sim/run_unit.sh study`).

## 4. What the numbers say (from results.md)

**Area.** The mesh is the largest fabric at every size: about 1.9k LUTs at
4 nodes (bus 0.25k, crossbar 0.45k), 5.7k at 8 and 13.5k at 16 (crossbar
6.4k, bus 1.5k). Each mesh router is a full 5-port switch with five input
FIFOs, so a mesh costs about 5 router ports per node, while a crossbar costs
one input FIFO per node plus an N-way output mux.

**Latency at low load.** The bus and the crossbar need 6 clocks; the mesh
needs 2 more per hop: 8.5 clocks at 4 nodes, 11.9 at 16. In the v2
controller this is irrelevant: one I2C read takes about 120 us = 12 000
clocks, the network adds about 10.

**Throughput.** The bus saturates at 1 flit per clock for the whole system
(0.25, 0.125 and 0.0625 flits per node per clock at 4, 8, 16 nodes); it is
already saturated at a 10 % load with 16 nodes. The crossbar and the mesh
both scale: about 0.6-0.7 flits per node per clock at saturation. With
uniform random traffic the crossbar is a little better (0.61 vs 0.50 at 16
nodes), because the mesh's middle links are shared by many flows.

**Clock speed.** This is where the mesh gains: its longest logic chain stays
at 22-24 levels for every size, because a router only ever arbitrates
among 5 ports. The bus (13 -> 21 -> 31 levels) and the crossbar (13 -> 18 ->
30) grow with N, because their arbiters and muxes have N inputs. At 16 nodes
the mesh has the shortest chain, so it should reach the highest Fmax
(confirm with `vivado_study.tcl`).

**Conclusion, said plainly.** On the Zynq-7020 at the sizes of this project
(6 nodes) the NoC does **not** win on area, latency or raw throughput: a
crossbar would be smaller and as fast for this traffic. The NoC's advantage
only appears at about 16 nodes and only in clock speed (logic depth that
does not grow with N) and in modularity: adding a node means adding a
router at the edge instead of widening every arbiter and mux. That is the
honest result slide 24 asked for. For v2 itself, the network (6 routers and the packet helpers) costs about
9.7k of the design's 21k LUTs (yosys estimate, per module), which is
affordable on the 53k-LUT Zynq-7020.

## 5. Files

| file | what |
|---|---|
| `rtl/study_bus.v` | the shared bus |
| `rtl/study_xbar.v` | the crossbar |
| `rtl/study_mesh.v` | wrapper: `noc_mesh` with W x H for N |
| `tb_study.v` | traffic generator, checker and measurement |
| `run_study.sh` | runs all simulations and yosys, writes `results.md` |
| `vivado_study.tcl` | Vivado out-of-context synthesis (+ optional routing) for real LUTs and Fmax |
| `results.md` | the generated tables |
| `vivado_reports/` | created by `vivado_study.tcl` |

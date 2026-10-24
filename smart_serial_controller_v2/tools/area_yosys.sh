#!/bin/sh
# -----------------------------------------------------------------------------
# area_yosys.sh - quick area estimate of every v2 block with yosys
#     sh tools/area_yosys.sh
# Synthesises ssc2_core for the Zynq-7000 (synth_xilinx) and prints LUTs and
# flip-flops per block.  These are ESTIMATES (yosys maps wide muxes less well
# than Vivado); the real numbers are in Vivado's utilization_hier.rpt after a
# build (build/ps_system/ or build/standalone/).  Needs yosys and python3.
# Takes 5-10 minutes.  Result: build/area_yosys.txt
# -----------------------------------------------------------------------------
cd "$(dirname "$0")/.." || exit 1
mkdir -p build
RTL=$(ls rtl/v1/*.v rtl/noc/*.v rtl/se/*.v rtl/xbar/*.v rtl/hub/*.v rtl/top/ssc2_core.v \
         rtl/top/ssc2_timebase.v | tr '\n' ' ')
echo "running yosys (5-10 minutes) ..."
yosys -q -l build/area_yosys.log -p "read_verilog $RTL; synth_xilinx -top ssc2_core; stat" \
      > /dev/null 2>&1 || { echo "yosys failed, see build/area_yosys.log"; exit 1; }
python3 - build/area_yosys.log <<'PY' | tee build/area_yosys.txt
import re, sys
log = open(sys.argv[1]).read()
log = log[log.rfind("Printing statistics"):]          # the final 'stat'
parts = re.split(r"\n=== (.+?) ===\n", log)
cells = {}
for i in range(1, len(parts) - 1, 2):
    name, body = parts[i], parts[i + 1]
    c = dict((k, int(v)) for v, k in [(m.group(2), m.group(1)) for m in
             re.finditer(r"^\s+(\S+)\s+(\d+)\s*$", body, re.M)])
    cells[name] = c
hier = parts[-1]
def short(n):
    n = re.sub(r"^\$paramod\$[0-9a-f]+\\\\?", "", n)
    n = re.sub(r"^\$paramod\\\\?", "", n)
    return n.split("\\")[0]
def own(c):
    lut = sum(v for k, v in c.items() if re.match(r"LUT\d$", k))
    lut += 4 * c.get("RAM32M", 0) + 4 * c.get("RAM64M", 0)
    ff = sum(v for k, v in c.items() if re.match(r"FD[A-Z]*$", k))
    return lut, ff
# instance counts from the hierarchy section
count = {}
for m in re.finditer(r"^\s{5,}(\S+)\s+(\d+)\s*$", hier, re.M):
    count[m.group(1)] = count.get(m.group(1), 0) + int(m.group(2))
groups = {}
for name, c in cells.items():
    if name == "design hierarchy" or not c:
        continue
    lut, ff = own(c)
    n = count.get(name, 1)
    g = short(name)
    a = groups.setdefault(g, [0, 0, 0])
    a[0] += lut * n; a[1] += ff * n; a[2] += n
tl = sum(v[0] for v in groups.values()); tf = sum(v[1] for v in groups.values())
print("%-22s %6s %7s %7s" % ("module (all instances)", "count", "LUTs", "FFs"))
for g, (l, f, n) in sorted(groups.items(), key=lambda x: -x[1][0]):
    print("%-22s %6d %7d %7d" % (g, n, l, f))
print("%-22s %6s %7d %7d" % ("TOTAL", "", tl, tf))
print("Zynq-7020: 53200 LUTs, 106400 FFs -> %.1f %% LUTs, %.1f %% FFs (yosys estimate)"
      % (100.0 * tl / 53200, 100.0 * tf / 106400))
PY

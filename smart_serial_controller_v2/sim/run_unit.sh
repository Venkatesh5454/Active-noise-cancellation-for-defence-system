#!/bin/sh
# -----------------------------------------------------------------------------
# Run every unit testbench with Icarus Verilog.
#     cd smart_serial_controller_v2
#     sh sim/run_unit.sh              # all unit tests
#     sh sim/run_unit.sh noc_pkt      # only tests whose name contains "noc_pkt"
# Each test prints "ALL n CHECKS PASSED" or "m OF n CHECKS FAILED".
# The script exits with an error if any test fails.
# -----------------------------------------------------------------------------
cd "$(dirname "$0")/.."
mkdir -p build
FILTER="$1"
FAILED=""
PASSED=0

# run_tb <name> <files...>
run_tb() {
    name="$1"; shift
    case "$name" in *"$FILTER"*) ;; *) return ;; esac
    echo "### $name"
    if ! iverilog -g2005 -Wall -o "build/$name.vvp" "$@"; then
        FAILED="$FAILED $name(compile)"; return
    fi
    vvp -n "build/$name.vvp" > "build/$name.log" 2>&1
    tail -n 3 "build/$name.log" | grep -v '\$finish'
    if grep -q "CHECKS PASSED" "build/$name.log"; then
        PASSED=$((PASSED + 1))
    else
        FAILED="$FAILED $name"
    fi
}

run_tb tb_noc_pkt  sim/unit/tb_noc_pkt.v rtl/noc/noc_pkt_tx.v rtl/noc/noc_pkt_rx.v \
                   rtl/top/ssc2_timebase.v
run_tb tb_xbar     sim/unit/tb_xbar.v rtl/xbar/xbar.v
run_tb tb_dma_writer sim/unit/tb_dma_writer.v rtl/hub/dma_writer.v sim/models/axi_mem_model.v \
                   rtl/noc/noc_pkt_tx.v rtl/noc/noc_pkt_rx.v

echo
if [ -n "$FAILED" ]; then
    echo "UNIT TESTS FAILED:$FAILED"
    exit 1
fi
echo "UNIT TESTS: $PASSED passed"

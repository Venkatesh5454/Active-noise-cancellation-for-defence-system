#!/bin/sh
# -----------------------------------------------------------------------------
# Run the two whole-design testbenches with Icarus Verilog.
#     cd smart_serial_controller_v2
#     sh sim/run_system.sh              # both (about 10 minutes)
#     sh sim/run_system.sh system       # only tb_ssc2_system   (Way 2, ~3 min)
#     sh sim/run_system.sh standalone   # only tb_zed2_standalone (Way 1, ~7 min)
#     sh sim/run_system.sh system vcd   # also write build/tb_ssc2_system.vcd
# Each prints "ALL n CHECKS PASSED" or "m OF n CHECKS FAILED"; the script
# exits with an error if any check failed.
# -----------------------------------------------------------------------------
cd "$(dirname "$0")/.."
mkdir -p build
WHICH="${1:-all}"
PLUS=""
[ "$2" = "vcd" ] && PLUS="+vcd"
FAILED=""
RTL="rtl/v1/*.v rtl/noc/*.v rtl/se/*.v rtl/xbar/*.v rtl/hub/*.v rtl/top/*.v"

run_sys() {
    name="$1"; shift
    echo "### $name"
    if ! iverilog -g2005 -Wall -o "build/$name.vvp" $RTL "$@" sim/models/*.v "sim/$name.v"; then
        FAILED="$FAILED $name(compile)"; return
    fi
    vvp -n "build/$name.vvp" $PLUS | tee "build/$name.log" | grep -v '\$finish'
    grep -q "CHECKS PASSED" "build/$name.log" || FAILED="$FAILED $name"
}

if [ "$WHICH" = "all" ] || [ "$WHICH" = "system" ]; then
    run_sys tb_ssc2_system boards/zedboard/zed2_pads.v
fi
if [ "$WHICH" = "all" ] || [ "$WHICH" = "standalone" ]; then
    run_sys tb_zed2_standalone boards/zedboard/zed2_pads.v boards/zedboard/zed2_top_standalone.v
fi

echo
if [ -n "$FAILED" ]; then
    echo "SYSTEM TESTS FAILED:$FAILED"
    exit 1
fi
echo "SYSTEM TESTS PASSED"

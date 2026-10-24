#!/bin/sh
# -----------------------------------------------------------------------------
# Run all three testbenches with the free Icarus Verilog simulator (optional - the
# same testbenches also run in Vivado XSim, see vivado/run_sim.tcl).
#     cd smart_serial_controller
#     sh sim/run_iverilog.sh            # all testbenches
#     sh sim/run_iverilog.sh vcd        # also write waveforms (open with GTKWave)
# -----------------------------------------------------------------------------
set -e
cd "$(dirname "$0")/.."
mkdir -p build
PLUS=""
[ "$1" = "vcd" ] && PLUS="+vcd"

echo "### tb_ssc_top: the controller on its own (APB, all protocols)"
iverilog -g2005 -Wall -o build/tb_ssc_top.vvp \
    rtl/*.v sim/models/*.v sim/tb_ssc_top.v
vvp -n build/tb_ssc_top.vvp $PLUS

echo
echo "### tb_ssc_axi: the Way 2 hardware path (AXI4-Lite -> APB -> controller)"
iverilog -g2005 -Wall -o build/tb_ssc_axi.vvp \
    rtl/*.v sim/models/*.v sim/tb_ssc_axi.v
vvp -n build/tb_ssc_axi.vvp

echo
echo "### tb_zed_standalone: the complete Way 1 ZedBoard image"
iverilog -g2005 -Wall -o build/tb_zed_standalone.vvp \
    rtl/*.v boards/zedboard/zed_top_standalone.v boards/zedboard/zed_pmod_pads.v \
    sim/models/*.v sim/tb_zed_standalone.v
vvp -n build/tb_zed_standalone.vvp

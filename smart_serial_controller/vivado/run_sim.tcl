# =============================================================================
# run_sim.tcl - run a testbench in Vivado's simulator (XSim), no GUI needed
# -----------------------------------------------------------------------------
# Windows : double-click windows\1_simulate.bat   (runs all three testbenches)
# Linux   : source <Vivado>/settings64.sh ; cd <...>/smart_serial_controller
#           vivado -mode batch -source vivado/run_sim.tcl                       (tb_ssc_top)
#           vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_ssc_axi
#           vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_zed_standalone
# Vivado GUI Tcl Console / Vivado Tcl Shell (forward slashes, no "vivado" word):
#           cd C:/ssc/smart_serial_controller
#           set argv tb_ssc_axi ; source vivado/run_sim.tcl
#
# The testbench prints every test and finishes with
#     ALL n CHECKS PASSED      or      m OF n CHECKS FAILED
# and this script stops with an error (non-zero exit code) if any check failed.
# To look at waveforms instead, open the project in the GUI
# (build/sim/ssc_sim.xpr), click Run Simulation > Run Behavioral Simulation,
# then type  log_wave -r /  and  run all  in the Tcl console.
# =============================================================================
set tb tb_ssc_top
if {[info exists argv] && [llength $argv] > 0} { set tb [lindex $argv 0] }

set root [file normalize [file join [file dirname [info script]] ..]]
set pdir [file join $root build sim]

# close anything left open from an earlier run in the same Vivado session
while {[llength [get_projects -quiet]] > 0} { close_project }
create_project ssc_sim $pdir -part xc7z020clg484-1 -force
add_files -norecurse [glob [file join $root rtl *.v]]
add_files -norecurse [list \
    [file join $root boards zedboard zed_top_standalone.v] \
    [file join $root boards zedboard zed_pmod_pads.v]]
set_property top zed_top_standalone [current_fileset]

add_files -fileset sim_1 -norecurse [glob [file join $root sim models *.v]]
add_files -fileset sim_1 -norecurse [file join $root sim ${tb}.v]
set_property top $tb [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1

launch_simulation -simset sim_1 -mode behavioral
run all

# read the testbench's own counters so a failure also fails this script
set nerr ""
set nchk ""
if {[catch {
    set nerr [get_value -radix unsigned /$tb/errors]
    set nchk [get_value -radix unsigned /$tb/checks]
} msg]} {
    puts "NOTE: could not read the result counters ($msg)."
    puts "      Look for 'ALL n CHECKS PASSED' in the output above."
}
close_sim

if {$nerr ne "" && $nchk ne ""} {
    puts ""
    puts "$tb: $nchk checks, $nerr failed"
    if {$nerr != 0 || $nchk == 0} {
        error "$tb FAILED ($nerr of $nchk checks failed)"
    }
}

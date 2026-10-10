# =============================================================================
# run_sim.tcl - run one v2 testbench in Vivado's simulator (XSim), no GUI needed
# -----------------------------------------------------------------------------
# Windows : double-click windows\1_simulate.bat   (runs the main testbenches)
# Linux   : source <Vivado>/settings64.sh ; cd <...>/smart_serial_controller_v2
#           vivado -mode batch -source vivado/run_sim.tcl                        (tb_ssc2_system)
#           vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_noc_mesh   (any unit test)
# Vivado GUI Tcl Console (forward slashes, no "vivado" word):
#           cd C:/ssc/smart_serial_controller_v2
#           set argv tb_xbar ; source vivado/run_sim.tcl
#
# The testbench finishes with  ALL n CHECKS PASSED  or  m OF n CHECKS FAILED
# and this script stops with an error if any check failed.
# To look at waveforms, open build/sim/ssc2_sim.xpr in the GUI, click
# Run Simulation > Run Behavioral Simulation, then type  log_wave -r /  and
# run all  in the Tcl console.
# =============================================================================
set tb tb_ssc2_system
if {[info exists argv] && [llength $argv] > 0} { set tb [lindex $argv 0] }

set root [file normalize [file join [file dirname [info script]] ..]]
set pdir [file join $root build sim]
source [file join $root vivado sources.tcl]

# the testbench file: sim/<tb>.v, sim/unit/<tb>.v or noc_study/<tb>.v
set tbfile ""
foreach d [list [file join $root sim] [file join $root sim unit] [file join $root noc_study]] {
    if {[file exists [file join $d ${tb}.v]]} { set tbfile [file join $d ${tb}.v] }
}
if {$tbfile eq ""} { error "testbench $tb not found in sim/, sim/unit/ or noc_study/" }

while {[llength [get_projects -quiet]] > 0} { close_project }
create_project ssc2_sim $pdir -part xc7z020clg484-1 -force
add_files -norecurse $rtl_files
add_files -norecurse [list \
    [file join $root boards zedboard zed2_top_standalone.v] \
    [file join $root boards zedboard zed2_pads.v]]
foreach f [glob -nocomplain [file join $root noc_study rtl *.v]] { add_files -norecurse $f }
set_property top zed2_top_standalone [current_fileset]

add_files -fileset sim_1 -norecurse $sim_model_files
add_files -fileset sim_1 -norecurse $tbfile
set_property top $tb [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
# testbenches load the assembled programs from here
set_property verilog_define [list "PROG_DIR=\"[file join $root programs]/\""] [get_filesets sim_1]
update_compile_order -fileset sim_1

launch_simulation -simset sim_1 -mode behavioral
run all

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

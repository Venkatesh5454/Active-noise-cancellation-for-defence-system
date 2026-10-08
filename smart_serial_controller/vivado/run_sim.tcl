# =============================================================================
# run_sim.tcl - run a testbench in Vivado's simulator (XSim), no GUI needed
# -----------------------------------------------------------------------------
#     cd <...>/smart_serial_controller
#     vivado -mode batch -source vivado/run_sim.tcl                      (whole controller)
#     vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_ssc_axi
#     vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_zed_standalone
#
# The testbench prints every test and finishes with
#     ALL n CHECKS PASSED      or      m OF n CHECKS FAILED
# To look at waveforms instead, open the project in the GUI
# (build/sim/ssc_sim.xpr) and click Run Simulation > Run Behavioral Simulation,
# then type  run all  in the Tcl console.
# =============================================================================
set tb tb_ssc_top
if {[info exists argv] && [llength $argv] > 0} { set tb [lindex $argv 0] }

set root [file normalize [file join [file dirname [info script]] ..]]
set pdir [file join $root build sim]

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
close_sim

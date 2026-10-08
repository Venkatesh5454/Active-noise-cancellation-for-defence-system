# =============================================================================
# build_standalone.tcl - "Way 1": FPGA-only bitstream (no ARM, no software)
# -----------------------------------------------------------------------------
# From a terminal (Linux) or the "Vivado Tcl Shell" (Windows):
#     cd <...>/smart_serial_controller
#     vivado -mode batch -source vivado/build_standalone.tcl
# or from the Tcl console inside the Vivado GUI:
#     cd <...>/smart_serial_controller
#     source vivado/build_standalone.tcl
#
# Result:
#     build/standalone/ssc_standalone.xpr       (open it in the GUI if you like)
#     build/standalone/zed_top_standalone.bit   (program this into the ZedBoard)
#     build/standalone/timing_summary.rpt, utilization.rpt
# =============================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set proj ssc_standalone
set pdir [file join $root build standalone]

create_project $proj $pdir -part xc7z020clg484-1 -force

# ---------------- design sources ----------------
add_files -norecurse [glob [file join $root rtl *.v]]
add_files -norecurse [list \
    [file join $root boards zedboard zed_top_standalone.v] \
    [file join $root boards zedboard zed_pmod_pads.v]]
set_property top zed_top_standalone [current_fileset]

# ---------------- constraints ----------------
add_files -fileset constrs_1 -norecurse [list \
    [file join $root boards zedboard zed_pmods.xdc] \
    [file join $root boards zedboard zed_standalone.xdc]]

# ---------------- simulation sources (for "Run Behavioral Simulation") ----------------
add_files -fileset sim_1 -norecurse [glob [file join $root sim models *.v]]
add_files -fileset sim_1 -norecurse [list \
    [file join $root sim tb_ssc_top.v] \
    [file join $root sim tb_ssc_axi.v] \
    [file join $root sim tb_zed_standalone.v]]
set_property top tb_ssc_top [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

# ---------------- synthesis ----------------
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    error "Synthesis failed - open the project and look at the Messages tab"
}

# ---------------- implementation + bitstream ----------------
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "Implementation failed - open the project and look at the Messages tab"
}

open_run impl_1
report_timing_summary -file [file join $pdir timing_summary.rpt]
report_utilization    -file [file join $pdir utilization.rpt]
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

set bit [file join [get_property DIRECTORY [get_runs impl_1]] zed_top_standalone.bit]
file copy -force $bit [file join $pdir zed_top_standalone.bit]

puts ""
puts "================================================================"
puts " Way 1 build finished"
puts "   worst setup slack (WNS) : $wns ns   (must be >= 0)"
puts "   bitstream               : [file join $pdir zed_top_standalone.bit]"
puts "================================================================"

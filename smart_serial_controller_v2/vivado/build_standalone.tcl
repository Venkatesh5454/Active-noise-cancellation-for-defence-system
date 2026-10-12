# =============================================================================
# build_standalone.tcl - "Way 1": FPGA-only bitstream (no ARM, no software)
# -----------------------------------------------------------------------------
# Windows : double-click windows\2_build_way1_fpga_only.bat
# Linux   : source <Vivado>/settings64.sh ; cd <...>/smart_serial_controller_v2
#           vivado -mode batch -source vivado/build_standalone.tcl
# Vivado GUI Tcl Console (forward slashes!):
#           cd C:/ssc/smart_serial_controller_v2 ; source vivado/build_standalone.tcl
#
# Result:
#     build/standalone/ssc2_standalone.xpr
#     build/standalone/zed2_top_standalone.bit   (program this into the ZedBoard)
#     build/standalone/timing_summary.rpt, utilization.rpt, utilization_hier.rpt
# =============================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set proj ssc2_standalone
set pdir [file join $root build standalone]
source [file join $root vivado sources.tcl]

while {[llength [get_projects -quiet]] > 0} { close_project }
create_project $proj $pdir -part xc7z020clg484-1 -force

# ---------------- design sources ----------------
add_files -norecurse $rtl_files
add_files -norecurse [list \
    [file join $root boards zedboard zed2_top_standalone.v] \
    [file join $root boards zedboard zed2_pads.v]]
set_property top zed2_top_standalone [current_fileset]

# ---------------- constraints ----------------
add_files -fileset constrs_1 -norecurse [list \
    [file join $root boards zedboard zed2_pins.xdc] \
    [file join $root boards zedboard zed2_standalone.xdc]]

update_compile_order -fileset sources_1

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
report_utilization -hierarchical -hierarchical_depth 6 -file [file join $pdir utilization_hier.rpt]
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

set bit [file join [get_property DIRECTORY [get_runs impl_1]] zed2_top_standalone.bit]
file copy -force $bit [file join $pdir zed2_top_standalone.bit]

puts ""
puts "================================================================"
puts " Way 1 build finished"
puts "   worst setup slack (WNS) : $wns ns   (must be >= 0)"
puts "   bitstream               : [file join $pdir zed2_top_standalone.bit]"
puts "   area per block          : [file join $pdir utilization_hier.rpt]"
puts "================================================================"

# =============================================================================
# create_project.tcl - one complete Vivado project for working in the GUI
# -----------------------------------------------------------------------------
# Windows : double-click windows\0_create_project_gui.bat
# Vivado GUI Tcl Console (forward slashes!):
#           cd D:/smart_serial_controller_v2 ; source vivado/create_project.tcl
# Linux   : vivado -source vivado/create_project.tcl        (opens the GUI)
#
# It creates build/gui/ssc2_gui.xpr with EVERYTHING set up the right way, so
# Run Synthesis, Run Implementation, Generate Bitstream and every kind of
# simulation work from the Flow Navigator:
#
#   Design Sources  every RTL file (rtl/v1, noc, se, xbar, hub, top) and the
#                   ZedBoard files; top = zed2_top_standalone (the Way 1 board
#                   top level - it has the real ZedBoard pins)
#   Constraints     zed2_pins.xdc + zed2_standalone.xdc (pins, 100 MHz clock)
#   Simulation      one simulation set per testbench, each with its own
#                   testbench and models (the design files come from Design
#                   Sources automatically):
#                     sim_1          tb_zed2_gate  (pins only: behavioural,
#                                    post-synthesis AND post-implementation)
#                     sim_system     tb_ssc2_system   (Way 2, as the ARM sees it)
#                     sim_board      tb_zed2_standalone (Way 1, inside view)
#                     sim_xbar, sim_noc_router, sim_noc_mesh, sim_noc_pkt,
#                     sim_se_engine, sim_se_programs, sim_ni_se, sim_ni_uart,
#                     sim_ni_spi, sim_ni_i2c, sim_hub, sim_dma, sim_study
#                   To run one: in the Sources window right-click the set
#                   (e.g. sim_xbar) -> Make Active, then Run Simulation.
#
# Every simulation set runs to the end ($finish) by itself (run time "all");
# the testbench prints "ALL n CHECKS PASSED" in the Tcl Console.
#
# Nothing is synthesised by this script; it only builds the project.  The
# project uses the files where they are (no copies), so edits to rtl/ are
# picked up.  Running the script again deletes and re-creates build/gui.
# =============================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set proj ssc2_gui
set pdir [file join $root build gui]
source [file join $root vivado sources.tcl]

while {[llength [get_projects -quiet]] > 0} { close_project }
create_project $proj $pdir -part xc7z020clg484-1 -force
set_property target_language Verilog [current_project]
set_property simulator_language Verilog [current_project]

# ---------------- design sources: ALL RTL + the board files ----------------
add_files -norecurse $rtl_files
add_files -norecurse [list \
    [file join $root boards zedboard zed2_top_standalone.v] \
    [file join $root boards zedboard zed2_pads.v]]
set_property top zed2_top_standalone [get_filesets sources_1]

# ---------------- constraints ----------------
add_files -fileset constrs_1 -norecurse [list \
    [file join $root boards zedboard zed2_pins.xdc] \
    [file join $root boards zedboard zed2_standalone.xdc]]

update_compile_order -fileset sources_1

# ---------------- simulation sets ----------------
# name  testbench  extra files (besides the models in sim/models)
set sim_sets [list \
    sim_1           tb_zed2_gate        [list sim/tb_zed2_gate.v] \
    sim_system      tb_ssc2_system      [list sim/tb_ssc2_system.v] \
    sim_board       tb_zed2_standalone  [list sim/tb_zed2_standalone.v] \
    sim_xbar        tb_xbar             [list sim/unit/tb_xbar.v] \
    sim_noc_router  tb_noc_router       [list sim/unit/tb_noc_router.v] \
    sim_noc_mesh    tb_noc_mesh         [list sim/unit/tb_noc_mesh.v] \
    sim_noc_pkt     tb_noc_pkt          [list sim/unit/tb_noc_pkt.v] \
    sim_se_engine   tb_se_engine        [list sim/unit/tb_se_engine.v] \
    sim_se_programs tb_se_programs      [list sim/unit/tb_se_programs.v] \
    sim_ni_se       tb_ni_se            [list sim/unit/tb_ni_se.v] \
    sim_ni_uart     tb_ni_uart          [list sim/unit/tb_ni_uart.v] \
    sim_ni_spi      tb_ni_spi           [list sim/unit/tb_ni_spi.v] \
    sim_ni_i2c      tb_ni_i2c           [list sim/unit/tb_ni_i2c.v] \
    sim_hub         tb_hub              [list sim/unit/tb_hub.v] \
    sim_dma         tb_dma_writer       [list sim/unit/tb_dma_writer.v] \
    sim_study       tb_study            [list noc_study/tb_study.v noc_study/rtl/study_bus.v \
                                              noc_study/rtl/study_xbar.v noc_study/rtl/study_mesh.v] \
]

foreach {set tb files} $sim_sets {
    if {[llength [get_filesets -quiet $set]] == 0} {
        create_fileset -simset $set
    }
    set full {}
    foreach f $files { lappend full [file join $root $f] }
    add_files -fileset $set -norecurse $full
    add_files -fileset $set -norecurse $sim_model_files
    set_property top     $tb               [get_filesets $set]
    set_property top_lib xil_defaultlib    [get_filesets $set]
    # run until the testbench calls $finish
    if {[catch {set_property -name xsim.simulate.runtime -value all -objects [get_filesets $set]} msg]} {
        puts "NOTE: could not set the run time of $set to 'all' ($msg); type 'run all' after launching"
    }
    update_compile_order -fileset $set
    puts "simulation set $set -> $tb"
}
current_fileset -simset [get_filesets sim_1]

puts ""
puts "================================================================"
puts " Project ready: [file join $pdir ${proj}.xpr]"
puts "   synthesis top : zed2_top_standalone   (Way 1 board design)"
puts "   simulation    : sim_1 = tb_zed2_gate  (also post-synthesis /"
puts "                   post-implementation); right-click another"
puts "                   simulation set -> Make Active for the others"
puts " Flow Navigator: Run Simulation / Run Synthesis / Run Implementation"
puts "                 / Generate Bitstream"
puts "================================================================"

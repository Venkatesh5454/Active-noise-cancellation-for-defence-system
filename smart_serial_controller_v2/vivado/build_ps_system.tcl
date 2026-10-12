# =============================================================================
# build_ps_system.tcl - "Way 2": Zynq ARM + v2 controller, bitstream + XSA for Vitis
# -----------------------------------------------------------------------------
# Windows : double-click windows\3_build_way2_arm_fpga.bat
# Linux   : source <Vivado>/settings64.sh ; cd <...>/smart_serial_controller_v2
#           vivado -mode batch -source vivado/build_ps_system.tcl
# Vivado GUI Tcl Console (forward slashes!):
#           cd C:/ssc/smart_serial_controller_v2 ; source vivado/build_ps_system.tcl
#
# Block design "system":
#
#   processing_system7_0 (ZedBoard preset: DDR3, UART1 on MIO 48/49, FCLK0 100 MHz)
#        M_AXI_GP0 --> AXI interconnect --> ssc_0/S_AXI     (0x43C0_0000, 4 KB)
#        S_AXI_HP0 <-- AXI interconnect <-- ssc_0/M_AXI     (DMA writer -> DDR)
#        IRQ_F2P[0] <-- ssc_0/irq                             (interrupt ID 61)
#   ssc_0 = module reference to rtl/top/ssc2_axi_top.v
#   all controller pins become block-design ports; boards/zedboard/zed2_top_ps.v
#   wraps the generated system_wrapper and adds the pads.
#
# Results:
#     build/ps_system/ssc2_ps.xpr
#     build/ps_system/zed2_top_ps.bit
#     build/ps_system/ssc2_system.xsa   <- give this to Vitis
#     build/ps_system/utilization.rpt, utilization_hier.rpt, timing_summary.rpt
# =============================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set proj ssc2_ps
set pdir [file join $root build ps_system]
source [file join $root vivado sources.tcl]

while {[llength [get_projects -quiet]] > 0} { close_project }
create_project $proj $pdir -part xc7z020clg484-1 -force

# ---------------- ZedBoard board files (needed for the DDR3 settings) ----------------
set bp [get_board_parts -quiet -latest_file_version {*zedboard*}]
if {$bp eq ""} { set bp [get_board_parts -quiet -latest_file_version {*:zed:*}] }
if {$bp eq ""} {
    puts "ERROR: the ZedBoard board files are not installed."
    puts "       In Vivado: Tools > Vivado Store... > Boards > Avnet > ZedBoard > Install"
    puts "       (or copy Digilent's vivado-boards 'zedboard' folder into"
    puts "       <Vivado install>/data/boards/board_files) and run this script again."
    error "ZedBoard board part not found"
}
set bp [lindex $bp end]
puts "Using board part $bp"
set_property board_part $bp [current_project]

# ---------------- sources ----------------
add_files -norecurse $rtl_files
add_files -norecurse [list \
    [file join $root boards zedboard zed2_top_ps.v] \
    [file join $root boards zedboard zed2_pads.v]]
add_files -fileset constrs_1 -norecurse [list \
    [file join $root boards zedboard zed2_pins.xdc] \
    [file join $root boards zedboard zed2_buttons_ps.xdc]]
update_compile_order -fileset sources_1

# ---------------- block design ----------------
create_bd_design system

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps
set_property -dict [list \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {32} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
    CONFIG.PCW_IRQ_F2P_INTR {1} \
] $ps

# the controller (module reference: no IP packaging needed)
create_bd_cell -type module -reference ssc2_axi_top ssc_0

# find the two AXI interfaces by their role, so their exact names do not matter
set ssc_s [get_bd_intf_pins -quiet -of_objects [get_bd_cells ssc_0] -filter {MODE == "Slave"}]
set ssc_m [get_bd_intf_pins -quiet -of_objects [get_bd_cells ssc_0] -filter {MODE == "Master"}]
if {[llength $ssc_s] != 1 || [llength $ssc_m] != 1} {
    error "ssc_0: expected one AXI slave and one AXI master interface, found: [get_bd_intf_pins -quiet -of_objects [get_bd_cells ssc_0]]"
}
set ssc_s_path [get_property PATH $ssc_s]
set ssc_m_path [get_property PATH $ssc_m]
puts "ssc_0 AXI slave: $ssc_s_path   AXI master: $ssc_m_path"

# registers: PS M_AXI_GP0 -> ssc_0/S_AXI
if {[catch {
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
        -config [list Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} \
                      Master {/processing_system7_0/M_AXI_GP0} Slave $ssc_s_path \
                      intc_ip {New AXI Interconnect} master_apm {0}] \
        $ssc_s
} msg]} {
    puts "Newer AXI automation syntax failed ($msg) - trying the older one"
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
        -config {Master "/processing_system7_0/M_AXI_GP0" Clk "Auto"} $ssc_s
}

# DMA: ssc_0/M_AXI -> PS S_AXI_HP0 (DDR)
set hp0 [get_bd_intf_pins processing_system7_0/S_AXI_HP0]
if {[catch {
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
        -config [list Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} \
                      Master $ssc_m_path Slave {/processing_system7_0/S_AXI_HP0} \
                      intc_ip {New AXI Interconnect} master_apm {0}] \
        $hp0
} msg]} {
    puts "Newer AXI automation syntax failed ($msg) - trying the older one"
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
        -config [list Master $ssc_m_path Clk "Auto"] $hp0
}

# interrupt to the ARM, and a copy for the top level
connect_bd_net [get_bd_pins ssc_0/irq] [get_bd_pins processing_system7_0/IRQ_F2P]
create_bd_port -dir O irq_out
connect_bd_net [get_bd_ports irq_out] [get_bd_pins ssc_0/irq]

# fabric clock out, for the switch/button synchronisers in the top level
create_bd_port -dir O -type clk fclk
set_property CONFIG.FREQ_HZ 100000000 [get_bd_ports fclk]
connect_bd_net [get_bd_ports fclk] [get_bd_pins processing_system7_0/FCLK_CLK0]

# controller pins -> block-design ports (same names)
foreach {name dir msb} {
    pad_out O 23  pad_oe O 23  pad_in I 23
    spi_cs_hi_n O 2  mon O 3  board_sw I 7  board_btn I 4
    oled_pwr O 1  status_led O 3
} {
    create_bd_port -dir $dir -from $msb -to 0 $name
    connect_bd_net [get_bd_ports $name] [get_bd_pins ssc_0/$name]
}

# addresses: registers at 0x43C0_0000 (4 KB); the DMA master sees all of DDR
assign_bd_address
foreach seg [get_bd_addr_segs -quiet processing_system7_0/Data/*] {
    if {[string match -nocase *ssc_0* $seg]} {
        set_property range  4K         $seg
        set_property offset 0x43C00000 $seg
        puts "ssc_0 registers mapped at [get_property offset $seg]"
    }
}
foreach seg [get_bd_addr_segs -quiet ssc_0/*] {
    puts "ssc_0 DMA master segment: $seg offset [get_property -quiet offset $seg] range [get_property -quiet range $seg]"
}

regenerate_bd_layout
validate_bd_design
save_bd_design

# ---------------- wrapper + top level ----------------
set bd_file [get_files system.bd]
generate_target all $bd_file
set wrapper [make_wrapper -fileset sources_1 -files $bd_file -top]
add_files -norecurse -fileset sources_1 $wrapper
set_property top zed2_top_ps [current_fileset]
update_compile_order -fileset sources_1

# ---------------- synthesis, implementation, bitstream ----------------
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

set bit [file join [get_property DIRECTORY [get_runs impl_1]] zed2_top_ps.bit]
file copy -force $bit [file join $pdir zed2_top_ps.bit]

# ---------------- hardware hand-off for Vitis ----------------
set xsa [file join $pdir ssc2_system.xsa]
write_hw_platform -fixed -include_bit -force -file $xsa

puts ""
puts "================================================================"
puts " Way 2 build finished"
puts "   worst setup slack (WNS) : $wns ns   (must be >= 0)"
puts "   bitstream               : [file join $pdir zed2_top_ps.bit]"
puts "   hardware for Vitis      : $xsa"
puts "   area per block          : [file join $pdir utilization_hier.rpt]"
puts "================================================================"

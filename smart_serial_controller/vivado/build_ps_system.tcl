# =============================================================================
# build_ps_system.tcl - "Way 2": Zynq ARM + controller, bitstream + XSA for Vitis
# -----------------------------------------------------------------------------
# Windows : double-click windows\3_build_way2_arm_fpga.bat
# Linux   : source <Vivado>/settings64.sh ; cd <...>/smart_serial_controller
#           vivado -mode batch -source vivado/build_ps_system.tcl
# Vivado GUI Tcl Console (forward slashes!):
#           cd C:/ssc/smart_serial_controller ; source vivado/build_ps_system.tcl
#
# Block design "system":
#
#   processing_system7_0 (ZedBoard preset: DDR3, UART1 on MIO 48/49, FCLK0 100 MHz)
#        M_AXI_GP0 --> AXI interconnect --> ssc_0/S_AXI   (0x43C0_0000, 4 KB)
#        IRQ_F2P[0] <-- ssc_0/irq                           (interrupt ID 61)
#   ssc_0 = module reference to rtl/ssc_axi_top.v (AXI-Lite -> APB -> controller)
#   all controller pins become block-design ports; boards/zedboard/zed_top_ps.v
#   wraps the generated system_wrapper and adds the tri-state pads.
#
# Results:
#     build/ps_system/ssc_ps.xpr
#     build/ps_system/zed_top_ps.bit
#     build/ps_system/ssc_system.xsa   <- give this to Vitis
# =============================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set proj ssc_ps
set pdir [file join $root build ps_system]

# close anything left open from an earlier run in the same Vivado session
while {[llength [get_projects -quiet]] > 0} { close_project }
create_project $proj $pdir -part xc7z020clg484-1 -force

# ---------------- ZedBoard board files (needed for the DDR3 settings) ----------------
set bp [get_board_parts -quiet -latest_file_version {*zedboard*}]
if {$bp eq ""} { set bp [get_board_parts -quiet -latest_file_version {*:zed:*}] }
if {$bp eq ""} {
    puts "ERROR: the ZedBoard board files are not installed."
    puts "       In Vivado: Tools > Vivado Store... > Boards > Avnet > ZedBoard > Install"
    puts "       (Vivado 2020.x: File > New Project > Default Part > Boards tab > Refresh,"
    puts "        then the download icon next to ZedBoard)"
    puts "       (or copy Digilent's vivado-boards 'zedboard' folder into"
    puts "       <Vivado install>/data/boards/board_files) and run this script again."
    error "ZedBoard board part not found"
}
set bp [lindex $bp end]
puts "Using board part $bp"
set_property board_part $bp [current_project]

# ---------------- sources ----------------
add_files -norecurse [glob [file join $root rtl *.v]]
add_files -norecurse [list \
    [file join $root boards zedboard zed_top_ps.v] \
    [file join $root boards zedboard zed_pmod_pads.v]]
add_files -fileset constrs_1 -norecurse [file join $root boards zedboard zed_pmods.xdc]
update_compile_order -fileset sources_1

# ---------------- block design ----------------
create_bd_design system

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps
set_property -dict [list \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
    CONFIG.PCW_IRQ_F2P_INTR {1} \
] $ps

# the controller (module reference: no IP packaging needed)
create_bd_cell -type module -reference ssc_axi_top ssc_0

# AXI connection: interconnect + processor reset block are added automatically.
# The slave interface is found by its role, so its exact name does not matter.
set ssc_axi [get_bd_intf_pins -quiet -of_objects [get_bd_cells ssc_0] -filter {MODE == "Slave"}]
if {[llength $ssc_axi] != 1} {
    error "ssc_0: expected one AXI slave interface, found: [get_bd_intf_pins -quiet -of_objects [get_bd_cells ssc_0]]"
}
set ssc_axi_path [get_property PATH $ssc_axi]
puts "ssc_0 AXI slave interface: $ssc_axi_path"
if {[catch {
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
        -config [list Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} \
                      Master {/processing_system7_0/M_AXI_GP0} Slave $ssc_axi_path \
                      intc_ip {New AXI Interconnect} master_apm {0}] \
        $ssc_axi
} msg]} {
    puts "Newer AXI automation syntax failed ($msg) - trying the older one"
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
        -config {Master "/processing_system7_0/M_AXI_GP0" Clk "Auto"} $ssc_axi
}

# interrupt to the ARM, and a copy for an LED
connect_bd_net [get_bd_pins ssc_0/irq] [get_bd_pins processing_system7_0/IRQ_F2P]
create_bd_port -dir O irq_out
connect_bd_net [get_bd_ports irq_out] [get_bd_pins ssc_0/irq]

# fabric clock out, for the LED logic in the top level
create_bd_port -dir O -type clk fclk
set_property CONFIG.FREQ_HZ 100000000 [get_bd_ports fclk]
connect_bd_net [get_bd_ports fclk] [get_bd_pins processing_system7_0/FCLK_CLK0]

# controller pins -> block-design ports (same names)
foreach {name dir} {
    uart_rxd   I  uart_txd     O  uart_cts_n     I  uart_rts_n O
    spi_sclk   O  spi_mosi     O  spi_miso       I
    spis_sclk  I  spis_mosi    I  spis_cs_n      I
    spis_miso  O  spis_miso_oe O  spi_slave_mode O
    i2c_scl_in I  i2c_scl_oe   O  i2c_sda_in     I  i2c_sda_oe O
} {
    create_bd_port -dir $dir $name
    connect_bd_net [get_bd_ports $name] [get_bd_pins ssc_0/$name]
}
create_bd_port -dir O -from 3 -to 0 spi_cs_n
connect_bd_net [get_bd_ports spi_cs_n] [get_bd_pins ssc_0/spi_cs_n]

# address: 0x43C0_0000, 4 KB (matches SSC_BASEADDR in sw/ssc_regs.h)
assign_bd_address
foreach seg [get_bd_addr_segs -quiet processing_system7_0/Data/*] {
    if {[string match -nocase *ssc_0* $seg]} {
        set_property range  4K         $seg
        set_property offset 0x43C00000 $seg
        puts "ssc_0 mapped at [get_property offset $seg]"
    }
}

regenerate_bd_layout
validate_bd_design
save_bd_design

# ---------------- wrapper + top level ----------------
set bd_file [get_files system.bd]
generate_target all $bd_file
set wrapper [make_wrapper -fileset sources_1 -files $bd_file -top]
add_files -norecurse -fileset sources_1 $wrapper
set_property top zed_top_ps [current_fileset]
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
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

set bit [file join [get_property DIRECTORY [get_runs impl_1]] zed_top_ps.bit]
file copy -force $bit [file join $pdir zed_top_ps.bit]

# ---------------- hardware hand-off for Vitis ----------------
set xsa [file join $pdir ssc_system.xsa]
write_hw_platform -fixed -include_bit -force -file $xsa

puts ""
puts "================================================================"
puts " Way 2 build finished"
puts "   worst setup slack (WNS) : $wns ns   (must be >= 0)"
puts "   bitstream               : [file join $pdir zed_top_ps.bit]"
puts "   hardware for Vitis      : $xsa"
puts "================================================================"

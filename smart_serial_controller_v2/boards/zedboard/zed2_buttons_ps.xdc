## =============================================================================
## zed2_buttons_ps.xdc - push buttons for "Way 2" (all five go to BOARD_IN)
## Bank 34, powered from VADJ (2.5 V jumper setting -> LVCMOS25)
## =============================================================================
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS25} [get_ports {btn[0]}]   ;# BTNC
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS25} [get_ports {btn[1]}]   ;# BTND
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS25} [get_ports {btn[2]}]   ;# BTNL
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS25} [get_ports {btn[3]}]   ;# BTNR
set_property -dict {PACKAGE_PIN T18 IOSTANDARD LVCMOS25} [get_ports {btn[4]}]   ;# BTNU
set_false_path -from [get_ports {btn[*]}]

## =============================================================================
## zed2_standalone.xdc - extra pins for "Way 1" (FPGA only)
## (use together with zed2_pins.xdc)
## =============================================================================

## 100 MHz oscillator GCLK (bank 13, 3.3 V)
set_property -dict {PACKAGE_PIN Y9 IOSTANDARD LVCMOS33} [get_ports clk_100m]
create_clock -period 10.000 -name clk_100m [get_ports clk_100m]

## centre push button BTNC = reset; the other four go to BOARD_IN
## (bank 34, VADJ 2.5 V -> LVCMOS25; use LVCMOS18 if J18 is set to 1.8 V)
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS25} [get_ports btn_reset]   ;# BTNC
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS25} [get_ports {btn[1]}]    ;# BTND
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS25} [get_ports {btn[2]}]    ;# BTNL
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS25} [get_ports {btn[3]}]    ;# BTNR
set_property -dict {PACKAGE_PIN T18 IOSTANDARD LVCMOS25} [get_ports {btn[4]}]    ;# BTNU
set_false_path -from [get_ports {btn_reset btn[*]}]

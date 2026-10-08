## =============================================================================
## zed_standalone.xdc - extra pins for "Way 1" (FPGA only)
## (use together with zed_pmods.xdc)
## =============================================================================

## 100 MHz oscillator GCLK (bank 13, 3.3 V)
set_property -dict {PACKAGE_PIN Y9 IOSTANDARD LVCMOS33} [get_ports clk_100m]
create_clock -period 10.000 -name clk_100m [get_ports clk_100m]

## centre push button BTNC = reset (bank 34, powered from VADJ).
## LVCMOS25 matches the ZedBoard's 2.5 V VADJ jumper setting (J18); if your
## board is set to 1.8 V use LVCMOS18 instead.
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS25} [get_ports btn_reset]
set_false_path -from [get_ports btn_reset]

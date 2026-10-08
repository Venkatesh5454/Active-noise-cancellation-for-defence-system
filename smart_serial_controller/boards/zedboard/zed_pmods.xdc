## =============================================================================
## zed_pmods.xdc - ZedBoard pins used by BOTH designs (Way 1 and Way 2)
## Pin numbers come from the ZedBoard master XDC (Rev C/D).  Pmods JA..JD are in
## bank 13, which is fixed at 3.3 V; the LEDs are in bank 33 (3.3 V).
##
##  Pmod pin numbering (looking at the connector on the board):
##     top row    : 1 2 3 4 GND VCC        bottom row : 7 8 9 10 GND VCC
## =============================================================================

## ---------------- JA : PmodUSBUART (UART to the PC) ----------------
## The Pmod's own labels are from ITS point of view: its RXD is our TXD.
set_property -dict {PACKAGE_PIN Y11  IOSTANDARD LVCMOS33} [get_ports uart_cts_n]   ;# JA1  RTS# of the Pmod -> our CTS#
set_property -dict {PACKAGE_PIN AA11 IOSTANDARD LVCMOS33} [get_ports uart_txd]     ;# JA2  RXD of the Pmod  <- our TXD
set_property -dict {PACKAGE_PIN Y10  IOSTANDARD LVCMOS33} [get_ports uart_rxd]     ;# JA3  TXD of the Pmod  -> our RXD
set_property -dict {PACKAGE_PIN AA9  IOSTANDARD LVCMOS33} [get_ports uart_rts_n]   ;# JA4  CTS# of the Pmod <- our RTS#
set_property PULLUP true [get_ports uart_rxd]      ;# idle-high if nothing is plugged in
set_property PULLDOWN true [get_ports uart_cts_n]  ;# "clear to send" if nothing is plugged in

## ---------------- JB : PmodSF3 (SPI flash) / SPI slave port ----------------
set_property -dict {PACKAGE_PIN W12 IOSTANDARD LVCMOS33} [get_ports spi_cs_n]      ;# JB1  CS#
set_property -dict {PACKAGE_PIN W11 IOSTANDARD LVCMOS33} [get_ports spi_mosi]      ;# JB2  MOSI (DQ0)
set_property -dict {PACKAGE_PIN V10 IOSTANDARD LVCMOS33} [get_ports spi_miso]      ;# JB3  MISO (DQ1)
set_property -dict {PACKAGE_PIN W8  IOSTANDARD LVCMOS33} [get_ports spi_sclk]      ;# JB4  SCK
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports spi_wp_n]      ;# JB7  WP#   (held high)
set_property -dict {PACKAGE_PIN W10 IOSTANDARD LVCMOS33} [get_ports spi_hold_n]    ;# JB8  HOLD# (held high)
set_property -dict {PACKAGE_PIN V9  IOSTANDARD LVCMOS33} [get_ports spi_cs1_n]     ;# JB9  CS1# (spare)
set_property -dict {PACKAGE_PIN V8  IOSTANDARD LVCMOS33} [get_ports spi_cs2_n]     ;# JB10 CS2# (spare)
set_property PULLUP true [get_ports spi_miso]

## ---------------- JC : PmodTMP2 (I2C temperature sensor) ----------------
## Plug the Pmod's 2x4 I2C header into the RIGHT-HAND four columns of JC
## (pins 3,4,5,6 / 9,10,11,12) so that SCL = JC3 and SDA = JC4.
set_property -dict {PACKAGE_PIN Y4  IOSTANDARD LVCMOS33} [get_ports i2c_scl]       ;# JC3 (JC2_P)
set_property -dict {PACKAGE_PIN AA4 IOSTANDARD LVCMOS33} [get_ports i2c_sda]       ;# JC4 (JC2_N)
## weak internal pull-ups as a backup; the PmodTMP2 has its own resistors
set_property PULLUP true [get_ports i2c_scl]
set_property PULLUP true [get_ports i2c_sda]

## ---------------- JD : copy of all bus lines for a logic analyser ----------------
set_property -dict {PACKAGE_PIN V7 IOSTANDARD LVCMOS33} [get_ports {la[0]}]        ;# JD1  UART TX
set_property -dict {PACKAGE_PIN W7 IOSTANDARD LVCMOS33} [get_ports {la[1]}]        ;# JD2  UART RX
set_property -dict {PACKAGE_PIN V5 IOSTANDARD LVCMOS33} [get_ports {la[2]}]        ;# JD3  SPI SCLK
set_property -dict {PACKAGE_PIN V4 IOSTANDARD LVCMOS33} [get_ports {la[3]}]        ;# JD4  SPI MOSI
set_property -dict {PACKAGE_PIN W6 IOSTANDARD LVCMOS33} [get_ports {la[4]}]        ;# JD7  SPI MISO
set_property -dict {PACKAGE_PIN W5 IOSTANDARD LVCMOS33} [get_ports {la[5]}]        ;# JD8  SPI CS#
set_property -dict {PACKAGE_PIN U6 IOSTANDARD LVCMOS33} [get_ports {la[6]}]        ;# JD9  I2C SCL
set_property -dict {PACKAGE_PIN U5 IOSTANDARD LVCMOS33} [get_ports {la[7]}]        ;# JD10 I2C SDA

## ---------------- LEDs LD0..LD7 ----------------
set_property -dict {PACKAGE_PIN T22 IOSTANDARD LVCMOS33} [get_ports {led[0]}]      ;# heartbeat
set_property -dict {PACKAGE_PIN T21 IOSTANDARD LVCMOS33} [get_ports {led[1]}]      ;# UART TX activity
set_property -dict {PACKAGE_PIN U22 IOSTANDARD LVCMOS33} [get_ports {led[2]}]      ;# UART RX activity
set_property -dict {PACKAGE_PIN U21 IOSTANDARD LVCMOS33} [get_ports {led[3]}]      ;# SPI activity
set_property -dict {PACKAGE_PIN V22 IOSTANDARD LVCMOS33} [get_ports {led[4]}]      ;# I2C activity
set_property -dict {PACKAGE_PIN W22 IOSTANDARD LVCMOS33} [get_ports {led[5]}]      ;# SPI slave mode
set_property -dict {PACKAGE_PIN U19 IOSTANDARD LVCMOS33} [get_ports {led[6]}]      ;# W1: error  W2: IRQ
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVCMOS33} [get_ports {led[7]}]      ;# running

## ---------------- timing ----------------
## Every serial input goes through a 2-flip-flop synchroniser and every output
## changes thousands of clocks apart, so the pins have no timing relationship
## to the 100 MHz clock that the tools need to check.
set_false_path -from [get_ports {uart_rxd uart_cts_n spi_cs_n spi_mosi spi_miso spi_sclk i2c_scl i2c_sda}]
set_false_path -to   [get_ports {uart_txd uart_rts_n spi_cs_n spi_mosi spi_miso spi_sclk spi_wp_n spi_hold_n spi_cs1_n spi_cs2_n i2c_scl i2c_sda la[*] led[*]}]

## unused pins float (the PmodTMP2 repeats SCL/SDA on JC9/JC10)
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullnone [current_design]

## =============================================================================
## zed2_pins.xdc - ZedBoard pins used by BOTH v2 designs (Way 1 and Way 2)
## Pin numbers come from the ZedBoard master XDC (Rev C/D).
## Pmods JA..JD, the OLED and the LEDs are in banks 13/33 (fixed 3.3 V).
## Switches and buttons are in banks 34/35, powered from VADJ: LVCMOS25 matches
## the ZedBoard's 2.5 V VADJ jumper setting (J18); for 1.8 V use LVCMOS18.
##
##  Pmod pin numbering (looking at the connector on the board):
##     top row    : 1 2 3 4 GND VCC        bottom row : 7 8 9 10 GND VCC
##  The top row of every Pmod is one crossbar port (pin index 0..3 = Pmod 1..4).
## =============================================================================

## ---------------- JA : crossbar port 0 (reset: UART -> PmodUSBUART) ----------------
## UART roles: JA1 CTS# in, JA2 TXD out, JA3 RXD in, JA4 RTS# out
set_property -dict {PACKAGE_PIN Y11  IOSTANDARD LVCMOS33} [get_ports {ja[0]}]
set_property -dict {PACKAGE_PIN AA11 IOSTANDARD LVCMOS33} [get_ports {ja[1]}]
set_property -dict {PACKAGE_PIN Y10  IOSTANDARD LVCMOS33} [get_ports {ja[2]}]
set_property -dict {PACKAGE_PIN AA9  IOSTANDARD LVCMOS33} [get_ports {ja[3]}]

## ---------------- JB : crossbar port 1 (reset: SPI -> PmodSF3 flash) ----------------
## SPI roles: JB1 CS#, JB2 MOSI, JB3 MISO, JB4 SCK
set_property -dict {PACKAGE_PIN W12 IOSTANDARD LVCMOS33} [get_ports {jb[0]}]
set_property -dict {PACKAGE_PIN W11 IOSTANDARD LVCMOS33} [get_ports {jb[1]}]
set_property -dict {PACKAGE_PIN V10 IOSTANDARD LVCMOS33} [get_ports {jb[2]}]
set_property -dict {PACKAGE_PIN W8  IOSTANDARD LVCMOS33} [get_ports {jb[3]}]
## JB7..10 (fixed): spare CS1#, PmodSF3 RST#, WP#, HOLD# (held high)
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports {jb_lo[0]}]
set_property -dict {PACKAGE_PIN W10 IOSTANDARD LVCMOS33} [get_ports {jb_lo[1]}]
set_property -dict {PACKAGE_PIN V9  IOSTANDARD LVCMOS33} [get_ports {jb_lo[2]}]
set_property -dict {PACKAGE_PIN V8  IOSTANDARD LVCMOS33} [get_ports {jb_lo[3]}]

## ---------------- JC : crossbar port 2 (reset: I2C -> PmodTMP2) ----------------
## I2C roles: JC3 SCL, JC4 SDA (open drain).  Plug the PmodTMP2's 2x4 header
## into the RIGHT-HAND four columns of JC (pins 3,4,5,6 / 9,10,11,12) so that
## SCL = JC3 and SDA = JC4.  JC7..10 are left unused (the TMP2 repeats SCL/SDA
## on JC9/JC10).  Fit 4.7k pull-ups from JC3 and JC4 to 3.3 V (JC6) if your
## TMP2 has none; the internal PULLUPs below are only a weak fallback.
set_property -dict {PACKAGE_PIN AB7 IOSTANDARD LVCMOS33} [get_ports {jc[0]}]
set_property -dict {PACKAGE_PIN AB6 IOSTANDARD LVCMOS33} [get_ports {jc[1]}]
set_property -dict {PACKAGE_PIN Y4  IOSTANDARD LVCMOS33} [get_ports {jc[2]}]
set_property -dict {PACKAGE_PIN AA4 IOSTANDARD LVCMOS33} [get_ports {jc[3]}]

## ---------------- JD : crossbar port 3 (reset: OFF) - the switching demo ----------------
set_property -dict {PACKAGE_PIN V7 IOSTANDARD LVCMOS33} [get_ports {jd[0]}]
set_property -dict {PACKAGE_PIN W7 IOSTANDARD LVCMOS33} [get_ports {jd[1]}]
set_property -dict {PACKAGE_PIN V5 IOSTANDARD LVCMOS33} [get_ports {jd[2]}]
set_property -dict {PACKAGE_PIN V4 IOSTANDARD LVCMOS33} [get_ports {jd[3]}]
## JD7..10 (fixed probe copies): UART TXD, UART RXD, I2C SCL, I2C SDA
set_property -dict {PACKAGE_PIN W6 IOSTANDARD LVCMOS33} [get_ports {jd_lo[0]}]
set_property -dict {PACKAGE_PIN W5 IOSTANDARD LVCMOS33} [get_ports {jd_lo[1]}]
set_property -dict {PACKAGE_PIN U6 IOSTANDARD LVCMOS33} [get_ports {jd_lo[2]}]
set_property -dict {PACKAGE_PIN U5 IOSTANDARD LVCMOS33} [get_ports {jd_lo[3]}]

## every crossbar pin idles high when nobody drives it (UART idle, I2C pull-up)
set_property PULLUP true [get_ports {ja[*] jb[*] jc[*] jd[*]}]

## ---------------- on-board OLED (SSD1306): crossbar port 4 (reset: OFF) ----------------
set_property -dict {PACKAGE_PIN U10  IOSTANDARD LVCMOS33} [get_ports {oled[0]}]   ;# DC
set_property -dict {PACKAGE_PIN AA12 IOSTANDARD LVCMOS33} [get_ports {oled[1]}]   ;# SDIN
set_property -dict {PACKAGE_PIN U9   IOSTANDARD LVCMOS33} [get_ports {oled[2]}]   ;# RES
set_property -dict {PACKAGE_PIN AB12 IOSTANDARD LVCMOS33} [get_ports {oled[3]}]   ;# SCLK
set_property -dict {PACKAGE_PIN U12  IOSTANDARD LVCMOS33} [get_ports oled_vdd]    ;# 1 = logic supply off
set_property -dict {PACKAGE_PIN U11  IOSTANDARD LVCMOS33} [get_ports oled_vbat]   ;# 1 = panel supply off

## ---------------- LEDs ----------------
## LD0..3 = crossbar port 5 (reset: GPIO), LD4 heartbeat, LD5 DMA batch toggle,
## LD6 hub error seen, LD7 interrupt line
set_property -dict {PACKAGE_PIN T22 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN T21 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN U22 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN U21 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN V22 IOSTANDARD LVCMOS33} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN W22 IOSTANDARD LVCMOS33} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN U19 IOSTANDARD LVCMOS33} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVCMOS33} [get_ports {led[7]}]

## ---------------- switches SW0..7 ----------------
set_property -dict {PACKAGE_PIN F22 IOSTANDARD LVCMOS25} [get_ports {sw[0]}]
set_property -dict {PACKAGE_PIN G22 IOSTANDARD LVCMOS25} [get_ports {sw[1]}]
set_property -dict {PACKAGE_PIN H22 IOSTANDARD LVCMOS25} [get_ports {sw[2]}]
set_property -dict {PACKAGE_PIN F21 IOSTANDARD LVCMOS25} [get_ports {sw[3]}]
set_property -dict {PACKAGE_PIN H19 IOSTANDARD LVCMOS25} [get_ports {sw[4]}]
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS25} [get_ports {sw[5]}]
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS25} [get_ports {sw[6]}]
set_property -dict {PACKAGE_PIN M15 IOSTANDARD LVCMOS25} [get_ports {sw[7]}]

## ---------------- timing ----------------
## Every serial input goes through a synchroniser and every output changes
## many clocks apart, so the pins have no timing relationship to the clock.
set_false_path -from [get_ports {ja[*] jb[*] jc[*] jd[*] oled[*] sw[*]}]
set_false_path -to   [get_ports {ja[*] jb[*] jb_lo[*] jc[*] jd[*] jd_lo[*] oled[*] oled_vdd oled_vbat led[*]}]

set_property BITSTREAM.CONFIG.UNUSEDPIN Pullnone [current_design]

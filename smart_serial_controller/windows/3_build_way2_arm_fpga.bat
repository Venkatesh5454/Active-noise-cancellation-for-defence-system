@echo off
rem Builds the Way 2 hardware (Zynq ARM + FPGA) and exports build\ps_system\ssc_system.xsa for Vitis
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
call vivado -mode batch -nojournal -log build\build_way2.log -source vivado\build_ps_system.tcl
echo.
echo Hardware for Vitis: build\ps_system\ssc_system.xsa   (log: build\build_way2.log)
pause

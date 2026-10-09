@echo off
rem Builds the Way 1 bitstream (FPGA only): build\standalone\zed_top_standalone.bit
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
call vivado -mode batch -nojournal -log build\build_way1.log -source vivado\build_standalone.tcl
echo.
echo Bitstream: build\standalone\zed_top_standalone.bit   (log: build\build_way1.log)
pause

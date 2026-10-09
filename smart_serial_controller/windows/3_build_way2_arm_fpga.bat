@echo off
rem Builds the Way 2 hardware (Zynq ARM + FPGA) and exports build\ps_system\ssc_system.xsa for Vitis
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
if exist build\ps_system\ssc_system.xsa del build\ps_system\ssc_system.xsa
echo This takes 10-20 minutes. Do not close this window.
call vivado -mode batch -nojournal -log build\build_way2.log -source vivado\build_ps_system.tcl
if errorlevel 1 goto failed
if not exist build\ps_system\ssc_system.xsa goto failed
echo.
echo SUCCESS. Hardware for Vitis: build\ps_system\ssc_system.xsa   (log: build\build_way2.log)
pause
exit /b 0
:failed
echo.
echo *** BUILD FAILED *** Open build\build_way2.log and search for "ERROR:"
echo (missing ZedBoard board files is the most common cause - README section 7)
pause
exit /b 1

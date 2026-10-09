@echo off
rem Builds the Way 1 bitstream (FPGA only): build\standalone\zed_top_standalone.bit
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
if exist build\standalone\zed_top_standalone.bit del build\standalone\zed_top_standalone.bit
echo This takes 5-15 minutes. Do not close this window.
call vivado -mode batch -nojournal -log build\build_way1.log -source vivado\build_standalone.tcl
if errorlevel 1 goto failed
if not exist build\standalone\zed_top_standalone.bit goto failed
echo.
echo SUCCESS. Bitstream: build\standalone\zed_top_standalone.bit   (log: build\build_way1.log)
pause
exit /b 0
:failed
echo.
echo *** BUILD FAILED *** Open build\build_way1.log and search for "ERROR:"
echo (README section 7 lists the usual causes)
pause
exit /b 1

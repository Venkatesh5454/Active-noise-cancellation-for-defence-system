@echo off
rem Opens the Way 1 project in the Vivado GUI (run 2_build_way1_fpga_only.bat first)
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build\standalone\ssc_standalone.xpr (
    echo Run 2_build_way1_fpga_only.bat first.
    pause & exit /b 1
)
start "Vivado - do not close this window" /min cmd /c vivado -nojournal -nolog build\standalone\ssc_standalone.xpr

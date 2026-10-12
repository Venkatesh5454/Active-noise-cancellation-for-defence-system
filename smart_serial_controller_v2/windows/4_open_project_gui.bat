@echo off
rem Opens the Way 2 project in the Vivado GUI if it exists, otherwise the Way 1 project
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
set XPR=
if exist build\standalone\ssc2_standalone.xpr set XPR=build\standalone\ssc2_standalone.xpr
if exist build\ps_system\ssc2_ps.xpr set XPR=build\ps_system\ssc2_ps.xpr
if "%XPR%"=="" (
    echo Run 2_build_way1_fpga_only.bat or 3_build_way2_arm_fpga.bat first.
    pause & exit /b 1
)
start "Vivado - do not close this window" /min cmd /c vivado -nojournal -nolog %XPR%

@echo off
rem NoC scalability study (slide 24, measurement 6): synthesises bus, crossbar
rem and mesh for 4, 8 and 16 nodes and writes LUTs / Fmax to noc_study\vivado_reports
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
echo This takes 20-60 minutes. Do not close this window.
call vivado -mode batch -nojournal -log build\noc_study.log -source noc_study\vivado_study.tcl
if errorlevel 1 (
    echo *** FAILED *** see build\noc_study.log
    pause
    exit /b 1
)
echo.
echo SUCCESS. Results: noc_study\vivado_reports\summary.txt
pause

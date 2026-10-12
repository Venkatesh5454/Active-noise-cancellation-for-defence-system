@echo off
rem Runs the three self-checking testbenches in Vivado's simulator (XSim).
rem Every testbench must end with "ALL n CHECKS PASSED".  Takes about 5-15 minutes.
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
set RESULT=
for %%T in (tb_ssc_top tb_ssc_axi tb_zed_standalone) do (
    echo.
    echo ================= %%T =================
    call vivado -mode batch -nojournal -log build\sim_%%T.log -source vivado\run_sim.tcl -tclargs %%T
    if errorlevel 1 (
        call set "RESULT=%%RESULT%%   %%T: FAILED  (see build\sim_%%T.log)"
    ) else (
        call set "RESULT=%%RESULT%%   %%T: passed"
    )
)
echo.
echo ================= SUMMARY =================
echo %RESULT%
echo (each testbench's own report says "ALL n CHECKS PASSED" - scroll up)
pause

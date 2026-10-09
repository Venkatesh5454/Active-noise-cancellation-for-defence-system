@echo off
rem Runs the three self-checking testbenches in Vivado's simulator (XSim).
rem Every testbench must end with "ALL n CHECKS PASSED".
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
for %%T in (tb_ssc_top tb_ssc_axi tb_zed_standalone) do (
    echo.
    echo ================= %%T =================
    call vivado -mode batch -nojournal -log build\sim_%%T.log -source vivado\run_sim.tcl -tclargs %%T
)
echo.
echo Done. Scroll up for "ALL ... CHECKS PASSED" (logs are in the build folder).
pause

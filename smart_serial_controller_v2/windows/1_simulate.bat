@echo off
rem Runs the main self-checking testbenches in Vivado's simulator (XSim):
rem   tb_ssc2_system      the whole controller as the ARM sees it (Way 2)
rem   tb_zed2_standalone  the FPGA-only ZedBoard image (Way 1)
rem   tb_noc_mesh, tb_se_programs, tb_xbar, tb_dma_writer   key unit tests
rem Every testbench must end with "ALL n CHECKS PASSED".  Takes about 15-40 minutes.
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
set RESULT=
for %%T in (tb_ssc2_system tb_zed2_standalone tb_noc_mesh tb_se_programs tb_xbar tb_dma_writer) do (
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

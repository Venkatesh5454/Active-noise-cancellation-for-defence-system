@echo off
rem Creates ONE complete Vivado project (build\gui\ssc2_gui.xpr) with every design
rem file, the pin constraints, the right top level and one simulation set per
rem testbench, then opens it in the Vivado GUI.  Use this instead of making a
rem project by hand.  Running it again re-creates the project.
setlocal
call "%~dp0_find_vivado.bat"
where vivado >nul 2>nul || (pause & exit /b 1)
cd /d "%~dp0.."
if not exist build mkdir build
echo Creating build\gui\ssc2_gui.xpr and opening Vivado - do not close this window.
start "Vivado - do not close this window" /min cmd /c vivado -nojournal -log build\create_project.log -source vivado\create_project.tcl

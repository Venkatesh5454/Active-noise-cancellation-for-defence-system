@echo off
rem ---------------------------------------------------------------------------
rem Helper used by the other .bat files: puts Vivado on the PATH.
rem If Vivado is not found, write the full path of YOUR settings64.bat below,
rem with ONE pair of quotes around the whole NAME=value, for example:
rem     set "MY_VIVADO_SETTINGS=C:\Xilinx\2025.1\Vivado\settings64.bat"
rem 2025.1 and newer : <install root>\<version>\Vivado\settings64.bat
rem 2024.2 and older : C:\Xilinx\Vivado\<version>\settings64.bat
rem ---------------------------------------------------------------------------
set "MY_VIVADO_SETTINGS="

where vivado >nul 2>nul
if not errorlevel 1 goto :eof

if defined MY_VIVADO_SETTINGS if exist "%MY_VIVADO_SETTINGS%" (
    call "%MY_VIVADO_SETTINGS%"
    goto :eof
)

rem try the usual install folders, newest version first
for %%V in (2026.2 2026.1 2025.2 2025.1 2024.2 2024.1 2023.2 2023.1 2022.2 2022.1 2021.2 2021.1 2020.2) do (
    if exist "C:\Xilinx\%%V\Vivado\settings64.bat" (
        call "C:\Xilinx\%%V\Vivado\settings64.bat"
        goto :eof
    )
    if exist "C:\Xilinx\Vivado\%%V\settings64.bat" (
        call "C:\Xilinx\Vivado\%%V\settings64.bat"
        goto :eof
    )
    if exist "C:\AMDDesignTools\%%V\Vivado\settings64.bat" (
        call "C:\AMDDesignTools\%%V\Vivado\settings64.bat"
        goto :eof
    )
)
echo.
echo  Could not find Vivado.
echo  Open windows\_find_vivado.bat in Notepad and set MY_VIVADO_SETTINGS to the
echo  full path of settings64.bat inside your Vivado install folder:
echo    2025.1 and newer : ^<install root^>\^<version^>\Vivado\settings64.bat
echo    2024.2 and older : C:\Xilinx\Vivado\^<version^>\settings64.bat
echo.

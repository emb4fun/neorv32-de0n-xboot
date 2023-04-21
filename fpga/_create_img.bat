@echo off
cls

::
:: Set project name and offset here
::
set HW_IMAGE=neorv32-de0n

:: ------------------------------------

mkdir .\build >NUL 2>&1
del .\build\*.* /S /Q >NUL 2>&1

copy .\output_files\*.sof .\build >NUL 2>&1

echo *******************************
echo * Starting Nios Command Shell *
echo *******************************
echo Please wait...

::
:: Create hwimage.hex and swimage.hex
::
call "%ALTERA_PROGRAM_DIR_V15%\nios2eds\Nios II Command Shell.bat" ./cfg/convert.sh %HW_IMAGE%

::
:: Convert HEX files to POF, RPD and JIC image files
::
%ALTERA_PROGRAM_DIR_V11%\quartus\bin\quartus_cpf -c .\cfg\de0n_pof.cof
%ALTERA_PROGRAM_DIR_V11%\quartus\bin\quartus_cpf -c .\cfg\de0n_rpd.cof
%ALTERA_PROGRAM_DIR_V11%\quartus\bin\quartus_cpf -c .\cfg\de0n_jic.cof

::
:: Delete files which does not needed anymore
::
del .\build\*.flash
del .\build\*.hex
del .\build\*.pof

::
:: Rename image files to "project" files
::
copy .\build\image.jic .\build\%HW_IMAGE%.jic >NUL 2>&1
copy .\build\image.rpd .\build\%HW_IMAGE%.rpd >NUL 2>&1
copy .\build\image.map .\build\%HW_IMAGE%.map >NUL 2>&1
del .\build\image.??? >NUL 2>&1

::
:: Create XBOOT file for XModem update support
::
.\tools\rpd2xboot -i:.\build\%HW_IMAGE%.rpd  -y
del .\build\*.rpd
echo.

pause


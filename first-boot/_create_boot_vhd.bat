@echo off
cls

::
:: Project name
::
set ORG_NAME=neorv32-first-boot
set PRJ_NAME=first_stage_bootloader

:: ------------------------------------

::
:: Delete files which does not needed anymore
::
mkdir build >NUL 2>&1
del build\*.* /Q >NUL 2>&1

::
:: Copy output file and create new image file
::
copy "prj\ROM Release\%ORG_NAME%.bin" .\build >NUL 2>&1
.\tools\bin2vhd -i .\build\%ORG_NAME%.bin -o .\build\%PRJ_NAME%.vhd

echo.
pause

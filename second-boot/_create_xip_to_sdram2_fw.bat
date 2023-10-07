@echo off
cls

::
:: Project name
::
set ORG_NAME=neorv32-second-boot
set PRJ_NAME=second-boot

:: ------------------------------------

::
:: Delete files which does not needed anymore
::
mkdir .\build >NUL 2>&1
del .\build\*.* /Q >NUL 2>&1

::
:: Copy output file and create new image file
::
copy "prj\XIP to SDRAM2 Release\%ORG_NAME%.bin" .\build >NUL 2>&1
.\tools\bin2xboot -i .\build\%ORG_NAME%.bin -o .\build\%PRJ_NAME%.xbo -s 0xE07E0000

echo.
pause
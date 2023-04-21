@echo off
cls

::
:: Project name
::
set ORG_NAME=%1
set PRJ_DIR=%~2
set STAR_ADDR=%3

:: ------------------------------------

::
:: Delete files which does not needed anymore
::
mkdir ..\build >NUL 2>&1
del ..\build\*.* /Q >NUL 2>&1

::
:: Copy output file and create new image file
::
copy "%PRJ_DIR%\%ORG_NAME%.bin" ..\build >NUL 2>&1
..\tools\bin2xboot -i ..\build\%ORG_NAME%.bin -s %STAR_ADDR%

echo.
:: pause
@echo off

del *.bak /S

cd bin2vhd
call _clean.bat
cd ..

cd blinky
call _clean.bat
cd ..

cd first-boot
call _clean.bat
cd ..

cd first-xboot
call _clean.bat
cd ..

cd fpga
call _clean.bat
cd ..

cd second-boot
call _clean.bat
cd ..

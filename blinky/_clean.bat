del *.bak /S
rmdir build  /S /Q
rmdir "prj\TCM Debug" /S /Q
rmdir "prj\TCM Release" /S /Q
rmdir "prj\SDRAM Debug" /S /Q
rmdir "prj\SDRAM Release" /S /Q
rmdir "prj\XIP Release" /S /Q
rmdir "prj\XIP to TCM Release" /S /Q
rmdir "prj\XIP to SDRAM Release" /S /Q
del prj\*.jlink


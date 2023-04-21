#!/bin/bash

#
# Convert SOF and ELF to HEX files, for more information take a look here:
# https://www.altera.com/support/support-resources/knowledge-base/solutions/rd12092009_471.html
#

printf "\n"
printf "Convert SOF to Flash...";
sof2flash --input=./build/$1.sof --output=./build/hwimage.flash --epcs
printf "done\n"

printf "Convert Flash to HEX..."
nios2-elf-objcopy -I srec -O ihex ./build/hwimage.flash  ./build/hwimage.hex
printf "done\n"
printf "\n"

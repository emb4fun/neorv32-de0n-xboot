#### README for neorv32-de0n-xboot
This project here is a further development of an existing XModem Bootloader which was also
created for a DE0-Nano board but for a NIOS II CPU. With this project it should be possible
to load a program into one of the different memories and start it there:

- XIP (Execute in Place memory, existing SPI flash on the DE0-Nano)
- IMEM (TCM, Tightly Coupled Memory)
- SDRAM
- SD Card to SDRAM (under development)

In addition, it should also be possible to update the FPGA image.

#### Project information:

| Board    | [Terasic DE0-Nano](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=593) |
| :------- | :------------- |
| FPGA     | Cyclone IV `EP4CE22F17C6N` |
| Quartus  | 15.0.2         |
| clk_i    | 100 MHz        |
| Terminal | 115200, 8, N, 1 |


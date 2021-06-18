.\execute.exe -flashreset -dll=.\usbdbgport.dll -port=USB2:DEBUGPORT_ONLY:SYSFREQ=324000000:STARTBAUD=38400:BAUDRATE=2000000 -elf=.\uart.elf -bin=.\u-boot-dtb.bin@RAW,0x23f00000,,LOAD -reg=.\ddr_init.rvs -wait=polling


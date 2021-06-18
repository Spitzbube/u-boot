.\execute.exe -flashreset -dll=.\usbdbgport.dll -port=USB2:DEBUGPORT_ONLY:SYSFREQ=324000000:STARTBAUD=38400:BAUDRATE=2000000 -elf=.\spl\u-boot-spl -reg=.\ddr_init.rvs -wait=polling
#-elf=.\uart.elf -bin=.\spl\u-boot-spl.bin,0x23f00000,0x23f00000,EXEC
#-elf=.\spl\u-boot-spl
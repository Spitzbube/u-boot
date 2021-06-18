// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2013 Atmel Corporation
 *		      Bo Shen <voice.shen@atmel.com>
 */

#include <asm/io.h>
#include <spl.h>
#include <dt-bindings/pinctrl/mb86h60.h>


u32 spl_boot_device(void)
{
	return BOOT_DEVICE_SPI;
//	return BOOT_DEVICE_NONE;
}

static void uart_init(void)
{
	   //Configure UART0
	   //Set Baudrate to 115200
	   *((volatile int*)0xc2000024) = 43; //FREG_UART_SetIbrd_BaudDivint(0, 43);
	   *((volatile int*)0xc2000028) = 60; //FREG_UART_SetFbrd_BaudDivfrac(0, 60);

	   //Set Databits to 8 //FREG_UART_SetLcrH_Wlen(0, 8-5);
	   {
	      int b = 8-5;
	      b = (b << 5) & (3 << 5);
	      b |= (*((volatile int*)0xc200002c) & ~(3 << 5));
	      *((volatile int*)0xc200002c) = b;
	   }

	   //Set Stopbits to 1 //FREG_UART_SetLcrH_Stp2(0, 1-1);
	   {
	      int b = 1-1;
	      b = (b << 3) & (1 << 3);
	      b |= (*((volatile int*)0xc200002c) & ~(1 << 3));

	      *((volatile int*)0xc200002c) = b;
	   }

	   //Set Parity to 0 //FREG_UART_SetLcrH_Pen(0, 0);
	   {
	      int b = 0;
	      b = (b << 1) & (1 << 1);
	      b |= (*((volatile int*)0xc200002c) & ~(1 << 1));

	      *((volatile int*)0xc200002c) = b;
	   }

	   //Set Flow Control to 0
	   //FREG_UART_SetCr_Ctsen(0, 0);
	   {
	      int b = 0;
	      b = (b << 15) & (1 << 15);
	      b |= (*((volatile int*)0xc2000030) & ~(1 << 15));

	      *((volatile int*)0xc2000030) = b;
	   }
	   //FREG_UART_SetCr_Rtsen(0, 0);
	   {
	      int b = 0;
	      b = (b << 14) & (1 << 14);
	      b |= (*((volatile int*)0xc2000030) & ~(1 << 14));

	      *((volatile int*)0xc2000030) = b;
	   }


	   //FREG_UART_SetCr_Rxe(0, 1);
	   {
	      int b = 1;
	      b = (b << 9) & (1 << 9);
	      b |= (*((volatile int*)0xc2000030) & ~(1 << 9));

	      *((volatile int*)0xc2000030) = b;
	   }

	   //FREG_UART_SetCr_Txe(0, 1);
	   {
	      int b = 1;
	      b = (b << 8) & (1 << 8);
	      b |= (*((volatile int*)0xc2000030) & ~(1 << 8));

	      *((volatile int*)0xc2000030) = b;
	   }

	   //FREG_UART_SetLcrH_Fen(0, 1);
	   {
	      int b = 1;
	      b = (b << 4) & (1 << 4);
	      b |= (*((volatile int*)0xc200002c) & ~(1 << 4));

	      *((volatile int*)0xc200002c) = b;
	   }

	   //FREG_UART_SetCr_Uarten(0, 1);
	   {
	      int b = 1;
	      b = (b << 0) & (1 << 0);
	      b |= (*((volatile unsigned*)0xc2000030) & ~(1 << 0));

	      *((volatile unsigned*)0xc2000030) = b;
	   }
}

static void gpio_uart_rx_init(int index, int function)
{
	   //FAPI_GPIO_SetPinFunction(32, 0x115); //UART0 RX
	   {
	      //FREG_GPIO_SetDataOut_DataEnSel(index, 1);
	      {
	    	   ((volatile unsigned*)0xc3000000)[index] = 1 << 8;
	      }
	      //FREG_GPIO_SetDataIn_DataInSel(r17, index + 2);
	      {
	    	   ((volatile unsigned*)0xc3000200)[function - 0x100] = index + 2;
	      }
	   }
}

static void gpio_uart_tx_init(int index, int function)
{
	   //FAPI_GPIO_SetPinFunction(33, 0x97); //UART0 TX
	   {
#if 0
	      //FREG_GPIO_SetDataOut_DataEnSel(index, 0);
	      {
	    	   ((volatile unsigned*)0xc3000000)[index] = 0;
	      }
#endif
	      //FREG_GPIO_SetDataOut_DataOutSel(index, r16);
	      {
	    	   ((volatile unsigned*)0xc3000000)[index] = function - 0x80;
	      }
	      //FREG_GPIO_SetDataOut_DataEnExchange(index, 0);
	      {
#if 0
	    	   int c = (m_gpio_dataout[a] & ~(1 << 16)) |
	    	      ((b << 16) & (1 << 16));
	    	   m_gpio_dataout[a] = c;
	    	   (GPIO_DataOut)[a] = c;
#endif
	      }
	   }
}

DECLARE_GLOBAL_DATA_PTR;

void board_init_f(ulong dummy)
{
#if 0
	   pl01x_putc(0xc2000000, '+');
//	writel('+', 0xc2000000);

//	   preloader_console_init();

	   printf("hello");

	   pl01x_putc(0xc2000000, '-');
#endif

		struct udevice *dev;
		int ret;

#if 0
	gpio_uart_rx_init(32, MB86H60_GPIO_IN_UART0_DATA);
	gpio_uart_tx_init(33, MB86H60_GPIO_OUT_UART0_DATA);
#endif


#if 0
	ret = uclass_get_device(UCLASS_PINCTRL, 0, &dev);
	printf("pinctrl1 = 0x%x\n", ret);
#endif

#if 0
	preloader_console_init();

	ret = uclass_get_device(UCLASS_PINCTRL, 0, &dev);
	printf("pinctrl2 = 0x%x\n", ret);
#endif

	gd->dm_root = NULL;

	ret = spl_early_init();
	printf("spl_early_init = 0x%x\n", ret);
}

void spl_board_init(void)
{
	struct udevice *dev;
	int ret;
	int node;
	const void *blob = gd->fdt_blob;

#if 0
	gpio_uart_rx_init(32, MB86H60_GPIO_IN_UART0_DATA);
#endif
	gpio_uart_tx_init(33, MB86H60_GPIO_OUT_UART0_DATA);

	preloader_console_init();

	node = fdt_path_offset(blob, "gpio");
	printf("node=%d\n", node);

//	ret = uclass_get_device_by_of_offset(UCLASS_PINCTRL, node, &dev);
	ret = uclass_get_device_by_of_offset(UCLASS_SERIAL, node, &dev);
	printf("uclass_get_device_by_of_offset: ret=%d\n", ret);


#if 0
	ret = uclass_get_device(UCLASS_PINCTRL, 0, &dev);
	printf("pinctrl3 = 0x%x\n", ret);
#endif

#if 0
	ret = spl_early_init();
	printf("spl_early_init = 0x%x\n", ret);
#endif

	ret = uclass_get_device(UCLASS_PINCTRL, 0, &dev);
	printf("pinctrl = 0x%x\n", ret);

}


/*
 * (C) Copyright 2012-2013 Stephen Warren
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <config.h>
#include <asm/global_data.h>
//#include <asm/arch/mb86hxx.h>

DECLARE_GLOBAL_DATA_PTR;

#if 0
flash_info_t	flash_info[CONFIG_SYS_MAX_FLASH_BANKS]; /* info for FLASH chips */
#endif

int dram_init(void)
{
	gd->ram_size = 64*1024*1024;

	return 0;
}

int board_init(void)
{
#if 0
    //gpio configuration
    writel(2+32, MB86HXX_GPIO_BASE+0x200+4*21); //UART0_RXDATA = 21 => GPIO 32 (in)
    writel(23, MB86HXX_GPIO_BASE+4*33); //UART0_TXDATA = 23 => GPIO 33 (out)

    //flash configuration
    writel(2+43,  MB86HXX_GPIO_BASE+0x200+4*2); //SFLASH_DATA_in = 2 => GPIO 43 (in)
    writel(49, MB86HXX_GPIO_BASE+4*44); //SFLASH_HOLD = 49 => GPIO 44 (out)
    writel(50, MB86HXX_GPIO_BASE+4*45); //SFLASH_CS = 50 => GPIO 45 (out)
    writel(51, MB86HXX_GPIO_BASE+4*46); //SFLASH_CLK = 51 => GPIO 46 (out)
    writel(52, MB86HXX_GPIO_BASE+4*47); //SFLASH_DATA_OUT = 52 => GPIO 47 (out)
#endif

	return 0;
}

unsigned long flash_init(void)
{
	return 0x400000; //4MB
}

void ft_board_setup(void *blob, bd_t *bd)
{
}


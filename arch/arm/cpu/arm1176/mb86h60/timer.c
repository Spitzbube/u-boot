/*
 * (C) Copyright 2012 Stephen Warren
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
#include <asm/io.h>
#include <asm/arch/timer.h>

ulong get_timer_us(ulong base)
{
	return 0;
}

ulong get_timer(ulong base)
{
	return 0;
}

unsigned long long get_ticks(void)
{
	return 0;
}

ulong get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}

void __udelay(unsigned long usec)
{
}

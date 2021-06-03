/*
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#if 0
#include <asm/arch/timer.h>
#else
/* Timer */
#define TIMER_ENABLE        0x0
#define TIMER_COUNT_PRE     0x4
#define TIMER_COUNT_LOW     0x8
#define TIMER_COUNT_HI      0xc
#define TIMER_IRQ_RESET     0x18
#define TIMER_COUNT_LOW_IRQ 0x1c
#endif

#define TIMER_BASE			0xCF000000 //MB86HXX_TIMER0_BASE
#define TIMER_PRESCALE		(TIMER_HZ/1000000)
#define TIMER_LOAD_VAL		(~0UL)
#define TIMER_NORMALISED	(1000000/CONFIG_SYS_HZ)


/* Clocks */
#if 1//defined(CONFIG_MACH_FUJITSU_TVSTBSOC_MB86HXX_H60)
#define MB86HXX_CPU_HZ		324000000		/* Core    */
#define MB86HXX_AXI_HZ		162000000		/* AXI/AHB */
#define MB86HXX_AVH_HZ		216000000		/* AV High */
#define MB86HXX_HDMIH_HZ	 74250000		/* HDMI Hi */
#elif defined(CONFIG_MACH_FUJITSU_TVSTBSOC_MB86HXX_H61)
#define MB86HXX_CPU_HZ		396000000		/* Core    */
#define MB86HXX_AXI_HZ		198000000		/* AXI/AHB */
#define MB86HXX_AVH_HZ		198000000		/* AV High */
#define MB86HXX_HDMIH_HZ	148500000		/* HDMI Hi */
#endif

#define MB86HXX_APB_HZ		(MB86HXX_AXI_HZ / 2)	/* APB     */
#define MB86HXX_AVM_HZ		(MB86HXX_AVH_HZ / 4)	/* AV Mid  */
#define MB86HXX_AVL_HZ		(MB86HXX_AVH_HZ / 8)	/* AV Low  */

#define TIMER_HZ			MB86HXX_APB_HZ

static ulong timestamp;
static ulong lastdec;

ulong get_timer_masked (void);

int timer_init(void)
{
	writel(0, 			  	TIMER_BASE + TIMER_ENABLE);
	writel(TIMER_PRESCALE, TIMER_BASE + TIMER_COUNT_PRE);
	writel(TIMER_LOAD_VAL, TIMER_BASE + TIMER_COUNT_LOW);
	writel(0, 			  TIMER_BASE + TIMER_COUNT_HI);

	/* ENDLESS | !IRQ_ENABLE | ENABLE */
	writel(5, 			TIMER_BASE + TIMER_ENABLE);

	/* init the timestamp and lastdec value */
	reset_timer_masked();

	return 0;
}

void reset_timer (void)
{
	reset_timer_masked ();
}

ulong get_timer(ulong base)
{
	return get_timer_masked () - base;
}

void set_timer (ulong t)
{
	timestamp = t;
}

void __udelay(unsigned long usec)
{
	ulong tmo, tmp;

	if(usec >= 1000){		/* if "big" number, spread normalization to seconds */
		tmo = usec / 1000;	/* start to normalize for usec to ticks per sec */
		tmo *= CONFIG_SYS_HZ;		/* find number of "ticks" to wait to achieve target */
		tmo /= 1000;		/* finish normalize. */
	}else{				/* else small number, don't kill it prior to HZ multiply */
		tmo = usec * CONFIG_SYS_HZ;
		tmo /= (1000*1000);
	}

	tmp = get_timer (0);		/* get current timestamp */
	if( (tmo + tmp + 1) < tmp )	/* if setting this fordward will roll time stamp */
		reset_timer_masked ();	/* reset "advancing" timestamp to 0, set lastdec value */
	else
		tmo += tmp;		/* else, set advancing stamp wake up time */

	while (get_timer_masked () < tmo)/* loop till event */
		/*NOP*/;
}

void reset_timer_masked (void)
{
	/* reset time */
	lastdec = readl(TIMER_BASE + TIMER_COUNT_LOW);
							/* capure current decrementer value time */
	timestamp = 0;	       /* start "advancing" time stamp from 0 */
}

ulong get_timer_masked (void)
{
	ulong now;		/* current tick value */

	now = readl(TIMER_BASE + TIMER_COUNT_LOW);

	if (lastdec >= now) {		/* normal mode (non roll) */
		/* normal mode */
		timestamp += lastdec - now; /* move stamp fordward with absoulte diff ticks */
	} else {			/* we have overflow of the count down timer */
		/* nts = ts + ld + (TLV - now)
		 * ts=old stamp, ld=time that passed before passing through -1
		 * (TLV-now) amount of time after passing though -1
		 * nts = new "advancing time stamp"...it could also roll and cause problems.
		 */
		timestamp += (lastdec + TIMER_LOAD_VAL - now);
	}
	lastdec = now;

#if 1
	return (timestamp/TIMER_NORMALISED);
#else
	return timestamp;
#endif
}

void udelay_masked (unsigned long usec)
{
	ulong tmo;
	ulong endtime;
	signed long diff;

	if (usec >= 1000) {		/* if "big" number, spread normalization to seconds */
		tmo = usec / 1000;	/* start to normalize for usec to ticks per sec */
		tmo *= CONFIG_SYS_HZ;		/* find number of "ticks" to wait to achieve target */
		tmo /= 1000;		/* finish normalize. */
	} else {			/* else small number, don't kill it prior to HZ multiply */
		tmo = usec * CONFIG_SYS_HZ;
		tmo /= (1000*1000);
	}

	endtime = get_timer_masked () + tmo;

	do {
		ulong now = get_timer_masked ();
		diff = endtime - now;
	} while (diff >= 0);
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk (void)
{
	ulong tbclk;

	tbclk = CONFIG_SYS_HZ;
	return tbclk;
}


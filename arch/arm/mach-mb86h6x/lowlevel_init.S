/* SPDX-License-Identifier: GPL-2.0 */
/*
 * (C) Copyright 2012 Stephen Warren
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 */

#include <asm-offsets.h>
#include <config.h>
#include <linux/linkage.h>

#ifndef CONFIG_SYS_PHY_UBOOT_BASE
#define CONFIG_SYS_PHY_UBOOT_BASE	CONFIG_SYS_UBOOT_BASE
#endif


.globl lowlevel_init
lowlevel_init:

#ifndef CONFIG_SPL_BUILD
	/* Return to U-Boot via saved link register */
	mov	pc, lr
#else
	/*
	 * flush v4 I/D caches
	 */
	mov	r0, #0
	mcr	p15, 0, r0, c7, c7, 0	/* flush v3/v4 cache */
	mcr	p15, 0, r0, c8, c7, 0	/* flush v4 TLB */

	/*
	 * disable MMU stuff and caches
	 */
	 /*
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00002300	@ clear bits 13, 9:8 (--V- --RS)
	bic	r0, r0, #0x00000087	@ clear bits 7, 2:0 (B--- -CAM)
	orr	r0, r0, #0x00000002	@ set bit 1 (A) Align
	orr	r0, r0, #0x00001000	@ set bit 12 (I) I-Cache
	*/

	mrc p15,0,r0,c1,c0,0 @ read CP15 register 1 into r0
    bic r0,r0,#0x1000    @ disable I-cache
    bic r0,r0,#0x0004    @ disable D-cache
    bic r0,r0,#0x0001    @ disable MMU

	mcr	p15, 0, r0, c1, c0, 0
	nop
	nop

	mov	pc, lr

#endif

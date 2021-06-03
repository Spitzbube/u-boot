/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2013 Broadcom Corporation.
 */

#ifndef __ARCH_MB86H6X_MUSB_H
#define __ARCH_MB86H6X_MUSB_H

struct mb86h60_musb_plat {
	void *base;
	void *usb_mode;
	struct musb_hdrc_platform_data plat;
	struct musb_hdrc_config musb_config;
};

#endif

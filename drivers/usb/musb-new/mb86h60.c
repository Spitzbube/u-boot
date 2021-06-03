// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments da8xx "glue layer"
 *
 * Copyright (c) 2019, by Texas Instruments
 *
 * Based on the DA8xx "glue layer" code.
 * Copyright (c) 2008-2019, MontaVista Software, Inc. <source@mvista.com>
 *
 * DT support
 * Copyright (c) 2016 Petr Kulhavy <petr@barix.com>
 * This file is part of the Inventra Controller Driver for Linux.
 *
 */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <asm/global_data.h>
#include <asm/omap_musb.h>
#include <generic-phy.h>
#include "linux-compat.h"
#include "musb_core.h"
#include "musb_uboot.h"
#include <asm/arch/musb.h>

#define MB86HXX_USB_1BYTE_ACCESS	(0)
#define MB86HXX_USB_2BYTE_ACCESS	(1)
#define MB86HXX_USB_4BYTE_ACCESS	(2)

static u16 mb86h60_musb_readw(struct musb *musb,
				const void __iomem *addr, unsigned offset)
{
	struct udevice *dev = (struct udevice *) musb->controller;
	struct mb86h60_musb_plat *plat = dev_get_plat(dev);
	u16 data;

	offset += (addr - plat->base);

	*((volatile unsigned*)(plat->usb_mode)) = MB86HXX_USB_2BYTE_ACCESS<<1;
	data = ((volatile unsigned*)(plat->base))[offset];
//	printf("readw(0x%x) -> 0x%x\n", offset, data);
	return data;
}

static void mb86h60_musb_writew(struct musb *musb,
			void __iomem *addr, unsigned offset, u16 data)
{
	struct udevice *dev = (struct udevice *) musb->controller;
	struct mb86h60_musb_plat *plat = dev_get_plat(dev);

	offset += (addr - plat->base);
//	printf("writew(0x%x) <- 0x%x\n", offset, data);
	*((volatile unsigned*)(plat->usb_mode)) = MB86HXX_USB_2BYTE_ACCESS<<1;
	((volatile unsigned*)(plat->base))[offset] = data;

}

static u8 mb86h60_musb_readb(struct musb *musb, const void __iomem *addr, unsigned offset)
{
	struct udevice *dev = (struct udevice *) musb->controller;
	struct mb86h60_musb_plat *plat = dev_get_plat(dev);
	u8 data;

	offset += (addr - plat->base);

	*((volatile unsigned*)(plat->usb_mode)) = MB86HXX_USB_1BYTE_ACCESS<<1;
	data = ((volatile unsigned*)(plat->base))[offset];
//	printf("readb(0x%x) -> 0x%x\n", offset, data);
	return data;
}

static void mb86h60_musb_writeb(struct musb *musb, void __iomem *addr, unsigned offset, u8 data)
{
	struct udevice *dev = (struct udevice *) musb->controller;
	struct mb86h60_musb_plat *plat = dev_get_plat(dev);

	offset += (addr - plat->base);

//	printf("writeb(0x%x) <- 0x%x\n", offset, data);
	*((volatile unsigned*)(plat->usb_mode)) = MB86HXX_USB_1BYTE_ACCESS<<1;
	((volatile unsigned*)(plat->base))[offset] = data;

}

static irqreturn_t mb86h60_musb_interrupt(int irq, void *hci)
{
	struct musb		*musb = hci;
	irqreturn_t		ret = IRQ_NONE;

	/* read and flush interrupts */
	musb->int_usb = musb_readb(musb, musb->mregs, MUSB_INTRUSB);
	if (musb->int_usb)
		musb_writeb(musb, musb->mregs, MUSB_INTRUSB, musb->int_usb);

	musb->int_tx = musb_readw(musb, musb->mregs, MUSB_INTRTX);
	if (musb->int_tx)
		musb_writew(musb, musb->mregs, MUSB_INTRTX, musb->int_tx);

	musb->int_rx = musb_readw(musb, musb->mregs, MUSB_INTRRX);
	if (musb->int_rx)
		musb_writew(musb, musb->mregs, MUSB_INTRRX, musb->int_rx);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		ret |= musb_interrupt(musb);

	return ret;
}

static int mb86h60_musb_init(struct musb *musb)
{
	musb->isr = mb86h60_musb_interrupt;

	return 0;
}

static int mb86h60_musb_exit(struct musb *musb)
{
	return 0;
}

/**
 * mb86h60_musb_enable - enable interrupts
 */
static int mb86h60_musb_enable(struct musb *musb)
{
	return 0;
}

/**
 * mb86h60_musb_disable - disable HDRC and flush interrupts
 */
static void mb86h60_musb_disable(struct musb *musb)
{
}

const struct musb_platform_ops mb86h60_ops = {
	.init		= mb86h60_musb_init,
	.exit		= mb86h60_musb_exit,
	.enable		= mb86h60_musb_enable,
	.disable	= mb86h60_musb_disable,
	.musb_readw	= mb86h60_musb_readw,
	.musb_writew	= mb86h60_musb_writew,
	.musb_readb	= mb86h60_musb_readb,
	.musb_writeb	= mb86h60_musb_writeb,
};

static int mb86h60_musb_of_to_plat(struct udevice *dev)
{
	struct mb86h60_musb_plat *plat = dev_get_plat(dev);

	plat->base = dev_remap_addr_name(dev, "usb-base");
	plat->usb_mode = dev_remap_addr_name(dev, "usb-mode");

	plat->musb_config.multipoint = 1;
	plat->musb_config.dyn_fifo = 1;
	plat->musb_config.num_eps = 5;
	plat->musb_config.ram_bits = 10;

	plat->plat.mode = MUSB_HOST;
	plat->plat.config = &plat->musb_config;
	plat->plat.platform_ops = &mb86h60_ops;

	return 0;
}

static int mb86h60_musb_probe(struct udevice *dev)
{
	struct musb_host_data *host = dev_get_priv(dev);
	struct mb86h60_musb_plat *plat = dev_get_plat(dev);
	int ret;

	host->host = musb_init_controller(&plat->plat,
					  (struct device *)dev,
					  plat->base);

	ret = musb_lowlevel_init(host);

	return 0;
}

static int mb86h60_musb_remove(struct udevice *dev)
{
	struct musb_host_data *host = dev_get_priv(dev);

	musb_stop(host->host);

	return 0;
}

static const struct udevice_id mb86h60_musb_ids[] = {
	{ .compatible = "fujitsu,mb86h60-musb" },
	{ }
};

U_BOOT_DRIVER(mb86h60_musb) = {
	.name	= "mb86h60-musb",
	.id		= UCLASS_USB,
	.of_match = mb86h60_musb_ids,
	.of_to_plat = mb86h60_musb_of_to_plat,
	.probe = mb86h60_musb_probe,
	.remove = mb86h60_musb_remove,
	.ops = &musb_usb_ops,
	.plat_auto	= sizeof(struct mb86h60_musb_plat),
	.priv_auto	= sizeof(struct musb_host_data),
};

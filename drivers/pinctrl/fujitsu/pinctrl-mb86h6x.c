// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Thomas Schmidt <spitzbube@mail.bg>
 */

#include <common.h>
#include <config.h>
#include <errno.h>
#include <dm.h>
#include <log.h>
#include <dm/pinctrl.h>
#include <dm/root.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <asm/system.h>
#include <asm/io.h>
#include <dt-bindings/pinctrl/mb86h60.h>
#if 0
#include <asm/gpio.h>
#endif

struct mb86h60_pinctrl_priv {
	u32 *base_reg;
	u32 dataOutMirror[96];
	u32 dataInMirror[67];
};

static const char *mb86h60_pinctrl_dummy_name = "_dummy";

#define MAX_PINS_PER_BANK 16

#define GPIO_OUT_MIN		MB86H60_GPIO_OUT_0
#define GPIO_OUT_MAX		MB86H60_GPIO_OUT_UPI_ADDRX3
#define GPIO_IN_MIN		MB86H60_GPIO_IN_SSP0_DATA
#define GPIO_IN_MAX		MB86H60_GPIO_IN_I2C1_CLK

static void FREG_GPIO_SetDataOut_DataOutSel(struct mb86h60_pinctrl_priv* priv, u32 index, u32 value)
{
	register u32 mask = 0x0000007FUL;
	value = ( value << 0 ) & mask;
	value |= priv->dataOutMirror[index] & ~mask;
	priv->dataOutMirror[index] = value;
	writel(value, &(priv->base_reg[index]));
}

static void FREG_GPIO_SetDataOut_DataEnSel(struct mb86h60_pinctrl_priv* priv, u32 index, u32 value)
{
	register u32 mask = 0x00000F00UL;
	value = ( value << 8 ) & mask;
	value |= priv->dataOutMirror[index] & ~mask;
	priv->dataOutMirror[index] = value;
	writel(value, &(priv->base_reg[index]));
}

static void FREG_GPIO_SetDataIn_DataInSel(struct mb86h60_pinctrl_priv* priv, u32 index, u32 value)
{
	register u32 mask = 0x0000007FUL;
	value = ( value << 0 ) & mask;
	value |= priv->dataInMirror[index] & ~mask;
	priv->dataInMirror[index] = value;
	writel(value, &(priv->base_reg[0x80+index]));
}

static void mb86h60_gpio_set_pin_function(struct udevice *dev, unsigned int func, 
					  unsigned int pin)
{
	struct mb86h60_pinctrl_priv *priv = dev_get_priv(dev);

	if ((func >= GPIO_IN_MIN) && (func <= GPIO_IN_MAX))
	{
		unsigned int in_func = func - MB86H60_GPIO_IN_OFFSET;
        	FREG_GPIO_SetDataOut_DataEnSel( priv, pin, 1 );
	        FREG_GPIO_SetDataIn_DataInSel( priv, in_func, pin+2 );
	}

	if ((func >= GPIO_OUT_MIN ) && (func <= GPIO_OUT_MAX))
	{
		unsigned int out_func = func - MB86H60_GPIO_OUT_OFFSET;
		FREG_GPIO_SetDataOut_DataEnSel( priv, pin, 0 );
		FREG_GPIO_SetDataOut_DataOutSel( priv, pin, out_func );
	}
}

#if 0
static int bcm2835_gpio_get_func_id(struct udevice *dev, unsigned int gpio)
{
	struct mb86h60_pinctrl_priv *priv = dev_get_priv(dev);
	u32 val;

	val = readl(&priv->base_reg[BCM2835_GPIO_FSEL_BANK(gpio)]);

	return (val >> BCM2835_GPIO_FSEL_SHIFT(gpio) & BCM2835_GPIO_FSEL_MASK);
}

#endif

/*
 * mb86h60_pinctrl_set_state: configure pin functions.
 * @dev: the pinctrl device to be configured.
 * @config: the state to be configured.
 * @return: 0 in success
 */
int mb86h60_pinctrl_set_state(struct udevice *dev, struct udevice *config)
{
	u32 func_arr[MAX_PINS_PER_BANK];
	u32 pin_arr[MAX_PINS_PER_BANK];
	int function;
	int i, len, func_count = 0;

	if (!dev_read_prop(config, "fujitsu,function", &len) || !len ||
	    len & 0x3 || dev_read_u32_array(config, "fujitsu,function", func_arr,
						  len / sizeof(u32))) {
		debug("Failed reading function array for pinconfig %s (%d)\n",
		      config->name, len);
		return -EINVAL;
	}

	func_count = len / sizeof(u32);

	if (dev_read_u32_array(config, "fujitsu,pins", pin_arr, func_count))
	{
		debug("Failed reading pins array for pinconfig %s (%d)\n",
		      config->name, len);
		return -EINVAL;
	}

	for (i = 0; i < func_count; i++)
		mb86h60_gpio_set_pin_function(dev, func_arr[i], pin_arr[i]);

	return 0;
}

static int mb86h60_pinctrl_get_gpio_mux(struct udevice *dev, int banknum,
					int index)
{
#if 0
	if (banknum != 0)
		return -EINVAL;

	return bcm2835_gpio_get_func_id(dev, index);
#else
//	pl01x_putc(0xc2000000, 'm');
//	printf("mb86h60_pinctrl_get_gpio_mux\n");

	return 0;
#endif
}

static int mb86h60_pinmux_get(struct udevice *dev, unsigned int selector,
				char *buf, int size)
{
	struct mb86h60_pinctrl_priv *priv = dev_get_priv(dev);
#if 0
	struct meson_pmx_axg_data *pmx_data;
	struct meson_pmx_group *group;
	struct meson_pmx_bank *bank;
	unsigned int offset;
	unsigned int func;
	unsigned int reg;
	int ret, i, j;

	selector += priv->data->pin_base;

	ret = meson_axg_pmx_get_bank(dev, selector, &bank);
	if (ret) {
		snprintf(buf, size, "Unhandled");
		return 0;
	}

	meson_axg_pmx_calc_reg_and_offset(bank, selector, &reg, &offset);

	func = (readl(priv->reg_mux + (reg << 2)) >> offset) & 0xf;

	for (i = 0; i < priv->data->num_groups; i++) {
		group = &priv->data->groups[i];
		pmx_data = (struct meson_pmx_axg_data *)group->data;

		if (pmx_data->func != func)
			continue;

		for (j = 0; j < group->num_pins; j++) {
			if (group->pins[j] == selector) {
				snprintf(buf, size, "%s (%x)",
					 group->name, func);
				return 0;
			}
		}
	}
#endif

	snprintf(buf, size, "Unknown (%x)", selector); //func);

	return 0;
} 

static int mb86h60_pinctrl_get_pins_count(struct udevice *dev)
{
	struct mb86h60_pinctrl_priv *priv = dev_get_priv(dev);
#if 0
	const struct uniphier_pinctrl_pin *pins = priv->socdata->pins;
	int pins_count = priv->socdata->pins_count;

	/*
	 * We do not list all pins in the pin table to save memory footprint.
	 * Report the max pin number + 1 to fake the framework.
	 */
	return pins[pins_count - 1].number + 1;
#else
	return 2;
#endif
}

static const char *mb86h60_pinctrl_get_pin_name(struct udevice *dev,
						 unsigned int selector)
{
	struct mb86h60_pinctrl_priv *priv = dev_get_priv(dev);
#if 0
	const struct uniphier_pinctrl_pin *pins = priv->socdata->pins;
	int pins_count = priv->socdata->pins_count;
	int i;

	for (i = 0; i < pins_count; i++)
		if (pins[i].number == selector)
			return pins[i].name;
#endif

	return mb86h60_pinctrl_dummy_name;
}

static const struct udevice_id mb86h6x_pinctrl_id[] = {
	{.compatible = "fujitsu,mb86h60-gpio"},
	{}
};

int mb86h60_pinctl_of_to_plat(struct udevice *dev)
{
	struct mb86h60_pinctrl_priv *priv;

	priv = dev_get_priv(dev);

	priv->base_reg = dev_read_addr_ptr(dev);
	if (!priv->base_reg) {
		debug("%s: Failed to get base address\n", __func__);
		return -EINVAL;
	}

	return 0;
}

int mb86h60_pinctl_probe(struct udevice *dev)
{
	int ret;
	struct udevice *pdev;

#if 0
	/* Create GPIO device as well */
	ret = device_bind(dev, lists_driver_lookup_name("gpio_bcm2835"),
			  "gpio_bcm2835", NULL, dev_ofnode(dev), &pdev);
	if (ret) {
		/*
		 * While we really want the pinctrl driver to work to make
		 * devices go where they should go, the GPIO controller is
		 * not quite as crucial as it's only rarely used, so don't
		 * fail here.
		 */
		printf("Failed to bind GPIO driver\n");
	}
#endif

	return 0;
}

static struct pinctrl_ops mb86h60_pinctrl_ops = {
	.set_state	= mb86h60_pinctrl_set_state,
	.get_gpio_mux	= mb86h60_pinctrl_get_gpio_mux,
	.get_pins_count = mb86h60_pinctrl_get_pins_count,
	.get_pin_name 	= mb86h60_pinctrl_get_pin_name,
	.get_pin_muxing	= mb86h60_pinmux_get,
};

U_BOOT_DRIVER(pinctrl_mb86h6x) = {
	.name		= "mb86h6x_pinctrl",
	.id		= UCLASS_PINCTRL,
	.of_match	= of_match_ptr(mb86h6x_pinctrl_id),
	.of_to_plat	= mb86h60_pinctl_of_to_plat,
	.priv_auto	= sizeof(struct mb86h60_pinctrl_priv),
	.ops		= &mb86h60_pinctrl_ops,
	.probe		= mb86h60_pinctl_probe,
#if 1//CONFIG_IS_ENABLED(OF_BOARD)
	.flags		= DM_FLAG_PRE_RELOC,
#endif
};


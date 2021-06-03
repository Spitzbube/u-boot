
/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

/* Memory layout */
#define CONFIG_SYS_UBOOT_BASE		CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_INIT_SP_ADDR		(0x24000000 - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_MALLOC_LEN		SZ_4M

/* Environment */
#define CONFIG_SYS_LOAD_ADDR		0x21B00000

/* Console configuration */
#define CONFIG_SYS_CBSIZE		1024
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE +		\
					 sizeof(CONFIG_SYS_PROMPT) + 16)

/* Environment */
#if 0
#define CONFIG_ENV_SIZE			SZ_16K
#define CONFIG_ENV_IS_NOWHERE
#endif
#define CONFIG_ENV_VARS_UBOOT_CONFIG
#define CONFIG_CONSOLE_MUX
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_PREBOOT \
	"if load mmc 0:1 ${loadaddr} /uEnv.txt; then " \
		"env import -t ${loadaddr} ${filesize}; " \
	"fi"

//#define DEBUG

#endif

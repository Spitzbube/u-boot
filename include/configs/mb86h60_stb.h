
#ifndef MB86H60_STB_H_
#define MB86H60_STB_H_

#include <linux/sizes.h>

/* Architecture, CPU, etc.*/
#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_ARM1176
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_SYS_DCACHE_OFF
/*
 * 2835 is a SKU in a series for which the 2708 is the first or primary SoC,
 * so 2708 has historically been used rather than a dedicated 2835 ID.
 */
#define CONFIG_MACH_TYPE		MACH_TYPE_BCM2708

/* Memory layout */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		0x20000000
#define CONFIG_SYS_TEXT_BASE		0x23F00000
#define CONFIG_SYS_UBOOT_BASE		CONFIG_SYS_TEXT_BASE
/*
 * The board really has 256M. However, the VC (VideoCore co-processor) shares
 * the RAM, and uses a configurable portion at the top. We tell U-Boot that a
 * smaller amount of RAM is present in order to avoid stomping on the area
 * the VC uses.
 */
#define CONFIG_SYS_SDRAM_SIZE		SZ_64M

/*
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_SDRAM_BASE + \
					 CONFIG_SYS_SDRAM_SIZE - \
					 GENERATED_GBL_DATA_SIZE)
*/
#define CONFIG_SYS_INIT_SP_ADDR		0x24000000

#define CONFIG_SYS_MALLOC_LEN		SZ_4M
#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_SDRAM_BASE + 0x3800000)
#define CONFIG_LOADADDR			0x22e08000

/* Flash */
#define CONFIG_SYS_NO_FLASH

/* Devices */
/* GPIO */
#if 0
#define CONFIG_BCM2835_GPIO
#endif
/* LCD */
#if 0
#define CONFIG_LCD
#define CONFIG_LCD_DT_SIMPLEFB
#define LCD_BPP				LCD_COLOR16
#endif
/*
 * Prevent allocation of RAM for FB; the real FB address is queried
 * dynamically from the VideoCore co-processor, and comes from RAM
 * not owned by the ARM CPU.
 */
#if 0
#define CONFIG_FB_ADDR			0
#define CONFIG_VIDEO_BCM2835
#define CONFIG_SYS_WHITE_ON_BLACK
#endif

/* SD/MMC configuration */
#if 0
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC
#define CONFIG_SDHCI
#define CONFIG_MMC_SDHCI_IO_ACCESSORS
#define CONFIG_BCM2835_SDHCI
#endif

/* Console UART */
#define CONFIG_PL011_SERIAL
#define CONFIG_PL011_CLOCK		81000000
#define CONFIG_PL01x_PORTS		{ (void *)0xc2000000 }
#define CONFIG_CONS_INDEX		0
#define CONFIG_BAUDRATE			115200

/* Console configuration */
#define CONFIG_SYS_CBSIZE		1024
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE +		\
					 sizeof(CONFIG_SYS_PROMPT) + 16)

/* Environment */
#define CONFIG_ENV_SIZE			SZ_16K
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_VARS_UBOOT_CONFIG
#define CONFIG_SYS_LOAD_ADDR		0x22e08000
#define CONFIG_CONSOLE_MUX
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_PREBOOT \
	"if load mmc 0:1 ${loadaddr} /uEnv.txt; then " \
		"env import -t ${loadaddr} ${filesize}; " \
	"fi"

#define ENV_DEVICE_SETTINGS \
	"stdin=serial,lcd\0" \
	"stdout=serial,lcd\0" \
	"stderr=serial,lcd\0"

#if 0

/*
 * Memory layout for where various images get loaded by boot scripts:
 *
 * scriptaddr can be pretty much anywhere that doesn't conflict with something
 *   else. Put it low in memory to avoid conflicts.
 *
 * pxefile_addr_r can be pretty much anywhere that doesn't conflict with
 *   something else. Put it low in memory to avoid conflicts.
 *
 * kernel_addr_r must be within the first 128M of RAM in order for the
 *   kernel's CONFIG_AUTO_ZRELADDR option to work. Since the kernel will
 *   decompress itself to 0x8000 after the start of RAM, kernel_addr_r
 *   should not overlap that area, or the kernel will have to copy itself
 *   somewhere else before decompression. Similarly, the address of any other
 *   data passed to the kernel shouldn't overlap the start of RAM. Pushing
 *   this up to 16M allows for a sizable kernel to be decompressed below the
 *   compressed load address.
 *
 * fdt_addr_r simply shouldn't overlap anything else. Choosing 32M allows for
 *   the compressed kernel to be up to 16M too.
 *
 * ramdisk_addr_r simply shouldn't overlap anything else. Choosing 33M allows
 *   for the FDT/DTB to be up to 1M, which is hopefully plenty.
 */
#define ENV_MEM_LAYOUT_SETTINGS \
	"scriptaddr=0x00000000\0" \
	"pxefile_addr_r=0x00100000\0" \
	"kernel_addr_r=0x01000000\0" \
	"fdt_addr_r=0x02000000\0" \
	"fdtfile=bcm2835-rpi-b.dtb\0" \
	"ramdisk_addr_r=0x02100000\0" \

#define BOOTCMDS_MMC \
	"mmc_boot=" \
		"setenv devtype mmc; " \
		"if mmc dev ${devnum}; then " \
			"run scan_boot; " \
		"fi\0" \
	"bootcmd_mmc0=setenv devnum 0; run mmc_boot;\0"
#define BOOT_TARGETS_MMC "mmc0"

#define BOOTCMDS_COMMON \
	"rootpart=1\0" \
	\
	"do_script_boot="                                                 \
		"load ${devtype} ${devnum}:${rootpart} "                  \
			"${scriptaddr} ${prefix}${script}; "              \
		"source ${scriptaddr}\0"                                  \
	\
	"script_boot="                                                    \
		"for script in ${boot_scripts}; do "                      \
			"if test -e ${devtype} ${devnum}:${rootpart} "    \
					"${prefix}${script}; then "       \
				"echo Found ${prefix}${script}; "         \
				"run do_script_boot; "                    \
				"echo SCRIPT FAILED: continuing...; "     \
			"fi; "                                            \
		"done\0"                                                  \
	\
	"do_sysboot_boot="                                                \
		"sysboot ${devtype} ${devnum}:${rootpart} any "           \
			"${scriptaddr} ${prefix}extlinux/extlinux.conf\0" \
	\
	"sysboot_boot="                                                   \
		"if test -e ${devtype} ${devnum}:${rootpart} "            \
				"${prefix}extlinux/extlinux.conf; then "  \
			"echo Found ${prefix}extlinux/extlinux.conf; "    \
			"run do_sysboot_boot; "                           \
			"echo SCRIPT FAILED: continuing...; "             \
		"fi\0"                                                    \
	\
	"scan_boot="                                                      \
		"echo Scanning ${devtype} ${devnum}...; "                 \
		"for prefix in ${boot_prefixes}; do "                     \
			"run sysboot_boot; "                              \
			"run script_boot; "                               \
		"done\0"                                                  \
	\
	"boot_targets=" \
		BOOT_TARGETS_MMC " " \
		"\0" \
	\
	"boot_prefixes=/\0" \
	\
	"boot_scripts=boot.scr.uimg\0" \
	\
	BOOTCMDS_MMC

#define CONFIG_BOOTCOMMAND \
	"for target in ${boot_targets}; do run bootcmd_${target}; done"

#define CONFIG_BOOTCOMMAND \
	"for target in ${boot_targets}; do run bootcmd_${target}; done"

#define CONFIG_EXTRA_ENV_SETTINGS \
	ENV_DEVICE_SETTINGS \
	ENV_MEM_LAYOUT_SETTINGS \
	BOOTCMDS_COMMON

#endif

#define CONFIG_BOOTDELAY 2

/* Shell */
#define CONFIG_SYS_MAXARGS		8
#define CONFIG_SYS_PROMPT		"MB86H60> "
#define CONFIG_COMMAND_HISTORY

/* Commands */
#include <config_cmd_default.h>
//#define CONFIG_CMD_GPIO
//#define CONFIG_CMD_MMC
#if 0
#define CONFIG_PARTITION_UUIDS
#define CONFIG_CMD_PART
#endif

/* Device tree support */
//#define CONFIG_OF_BOARD_SETUP

#if 0
/* ATAGs support for bootm/bootz */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_CMDLINE_TAG
#define CONFIG_INITRD_TAG
#endif

#include <config_distro_defaults.h>

/* Some things don't make sense on this HW or yet */
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SAVEENV
#undef CONFIG_CMD_DHCP
#undef CONFIG_CMD_MII
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_PING

#endif /* MB86H60_STB_H_ */

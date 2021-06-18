
#include <common.h>
#include <asm/global_data.h>
#include <asm/armv8/mmu.h>

static struct mm_region rtd129x_mem_map[] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0x80000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0xf0000000UL,
		.phys = 0xf0000000UL,
		.size = 0x10000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = rtd129x_mem_map;

DECLARE_GLOBAL_DATA_PTR;

int print_cpuinfo(void)
{
	return 0;
}

int dram_init(void)
{
	gd->ram_size = 1024*1024;

	return 0;
}

int board_init(void)
{
	return 0;
}


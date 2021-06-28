
#include <common.h>
#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	if (fdtdec_setup_mem_size_base() != 0)
			return -EINVAL;

	return 0;
}

#ifdef CONFIG_OF_BOARD
int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}
#endif

int board_init(void)
{
	printf("ticks=%d\n", get_ticks());

	udelay(1000000);

	printf("ticks=%d\n", get_ticks());

	return 0;
}


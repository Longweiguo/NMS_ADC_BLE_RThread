#include "main.h"
#include "board.h"
#include "rtthread.h"
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "ad7147.h"
#include "am_util.h"
#include "heyos_port.h"
#include "heyos_hal_spi.h"


#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define AM_SRAM_BEGIN    (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define AM_SRAM_BEGIN    (__segment_end("HEAP"))
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define AM_SRAM_BEGIN    ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#else
extern int __bss_end;
#define AM_SRAM_BEGIN    (&__bss_end)
#endif

struct rt_memheap sram_heap;
struct ad714x_touchpad_plat touchpad_config = 
{
    .x_start_stage = 0,
    .x_end_stage = 11,
    .y_start_stage = 0,
    .y_end_stage = 0,
};

struct ad714x_platform_data hw_config = 
{
    .touchpad = &touchpad_config,
	.stage_cfg_reg = 
	{
		{0xFFFE,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},          //CIN0
		{0xFFFB,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},          //CIN1
		{0xFFEF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFFBF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFEFF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFBFF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},
		{0xEFFF,0x1FFF,0x0000,0x2626,1650,1650,1650,1650},

		{0xFFFF,0x1FFE,0x0000,0x2626,1650,1650,1650,1650},         //CIN7
		{0xFFFF,0x1FFB,0x0000,0x2626,1650,1650,1650,1650},
        {0xFFFF,0x1FEF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFFFF,0x1FBF,0x0000,0x2626,1650,1650,1650,1650},
		{0xFFFF,0x1EFF,0x0000,0x2626,1650,1650,1650,1650},
	},
//===================PWR_CONTROL Register = 0x00=======================
//full power mode (normal operation, CDC conversions approximately every 36 ms)
//Number of stages in sequence (N + 1) = 11	, ADC decimation factor = 64
//Interrupt polarity control 0 = active low , enable excitation source to CINx pins
//===================STAGE_CAL_EN Register = 0x01=======================
//STAGE0-STAGE11 calibration enable
//===================AMB_COMP_CTRL0 Register = 0x02=======================
//Fast filter skip control (N + 1) = 11,	
//FP_PROXIMITY_CNT * 16 * time for one conversion sequence in full power mode
	.sys_cfg_reg = {0x02B0,0x0000,0x3BBB,0x0819,0x0832,0x0000,0x0000,0x0000},
	
};

struct ad714x_chip ad714x_config = 
{
    .idx = 0,
	.is_ad7417 = {false,false},
	.read = heyos_hal_spi_read,
	.write = heyos_hal_spi_write,
    .hw = &hw_config,
};

void am_stimer_cmpr0_isr(void)
{
    /* Check the timer interrupt status */
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);

    if (rt_thread_self() != RT_NULL)
    {
        /* enter interrupt */
        rt_interrupt_enter();

        rt_tick_increase();

        /* leave interrupt */
        rt_interrupt_leave();
    }
}

void systick_init(void)
{
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);

    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
}

void* heyos_sram_malloc(const uint32_t size)
{
    void* p = rt_memheap_alloc(&sram_heap, size);
    return p;
}

void heyos_sram_free(const void* ptr)
{
    rt_memheap_free((void*)ptr);
}
void* heyos_sram_realloc(void *rmem, uint32_t newsize)
{
    return rt_memheap_realloc(&sram_heap, rmem, newsize);
}

void heyos_ram_init(void)
{
    /* os menory heap && stack init */
    rt_system_heap_init((void*)AM_SRAM_BEGIN, (void*)AM_SRAM_END);
}

void heyos_board_init(void)
{
	BoardsInit();
    systick_init();
    heyos_ram_init();
	heyos_hal_spim_init(SPI_IDX_AD7174_1);
	ad7147_init(&ad714x_config);
    ad714x_config.idx =1;
	heyos_hal_spim_init(SPI_IDX_AD7174_2);
    ad7147_init(&ad714x_config);
}

void ad714x_thread_run(void *para)
{
	while(1)
	{
		heyos_delay_ms(10);
		ad714x_probe(&ad714x_config,0);
		ad714x_probe(&ad714x_config,1);
	}
}

heyos_thread_t ad714x_thread_ctx;
const heyos_thread_ctx ctx = 
{
	.name = "ad714x_thread",
	.entry = ad714x_thread_run,
	.para = NULL,
	.prio = 2,
	.stack_size = 1 *1024
};

void heyos_application_init(void)
{
    ad714x_thread_ctx = heyos_thread_create(ctx);
    HEYOS_ASSERT(ad714x_thread_ctx);
}


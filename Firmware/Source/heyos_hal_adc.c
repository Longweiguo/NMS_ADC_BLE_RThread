#include <stdint.h>
#include "am_bsp_pins.h"
#include "heyos_hal_adc.h"
#include "am_util_stdio.h"

static void *g_ADCHandle;
uint16_t adc_value;

typedef struct
{
	uint8_t GPIO_Pin;
	am_hal_gpio_pincfg_t GpioCfg;
	am_hal_adc_slot_chan_e Channel;
}sADC_Channel;

const static am_hal_adc_config_t g_sADC_Cfg =
{
	.eClock = AM_HAL_ADC_CLKSEL_HFRC_DIV2,
	.ePolarity = AM_HAL_ADC_TRIGPOL_RISING,
	.eTrigger = AM_HAL_ADC_TRIGSEL_SOFTWARE,
	.eReference = AM_HAL_ADC_REFSEL_INT_1P5,
	.eClockMode = AM_HAL_ADC_CLKMODE_LOW_POWER,
	.ePowerMode = AM_HAL_ADC_LPMODE1,
	.eRepeat = AM_HAL_ADC_REPEATING_SCAN
};

const sADC_Channel ADC_Channel[ADC_INDEX_MAX] =
{
	{
		.GPIO_Pin = AM_BSP_GPIO_BAT_ADC,
		.Channel = AM_HAL_ADC_SLOT_CHSEL_SE4,
		.GpioCfg = {.uFuncSel = AM_HAL_PIN_32_ADCSE4}
	},
};

void adc_init(heyos_adc_inst_t idx)
{
    am_hal_adc_slot_config_t sSlotCfg;

	//Initialize the ADC and get the handle.
    if ( AM_HAL_STATUS_SUCCESS != am_hal_adc_initialize(0, &g_ADCHandle) )
    {
        am_util_stdio_printf("Error - reservation of the ADC instance failed.\n");
    }
    // Power on the ADC.
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_power_control(g_ADCHandle,AM_HAL_SYSCTRL_WAKE,false) )
    {
        am_util_stdio_printf("Error - ADC power on failed.\n");
    }
    // Configure the ADC.
    if ( am_hal_adc_configure(g_ADCHandle, (am_hal_adc_config_t*)&g_sADC_Cfg) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Error - configuring ADC failed.\n");
    }

    sSlotCfg.bEnabled       = true;
    sSlotCfg.bWindowCompare = true;
    sSlotCfg.eChannel       = ADC_Channel[idx].Channel;    // 0
    sSlotCfg.eMeasToAvg     = AM_HAL_ADC_SLOT_AVG_1;        // 0
    sSlotCfg.ePrecisionMode = AM_HAL_ADC_SLOT_14BIT;        // 0

    am_hal_adc_configure_slot(g_ADCHandle, 0, &sSlotCfg);
    am_hal_gpio_pinconfig(ADC_Channel[idx].GPIO_Pin, ADC_Channel[idx].GpioCfg);	
    // Enable the ADC.
    am_hal_adc_enable(g_ADCHandle);
}

void am_adc_isr(void)
{
    uint32_t ui32IntStatus,adc_vol;
    uint32_t ui32NumSamples = 1;
    am_hal_adc_sample_t sSample;

    // Clear timer 3 interrupt.
    am_hal_adc_interrupt_status(g_ADCHandle, &ui32IntStatus, true);
    am_hal_adc_interrupt_clear(g_ADCHandle, ui32IntStatus);

    // Emtpy the FIFO, we'll just look at the last one read.
    while ( AM_HAL_ADC_FIFO_COUNT(ADC->FIFO) )
    {
		ui32NumSamples = 1;
		am_hal_adc_samples_read(g_ADCHandle, true, NULL, &ui32NumSamples, &sSample);

		// Determine which slot it came from?
		if (sSample.ui32Slot == 0)
		{
			adc_value = AM_HAL_ADC_FIFO_SAMPLE(sSample.ui32Sample);
			adc_vol = (30+15)/15*adc_value*ADC_REFER_VOL/ADC_RES_14BIT;
			am_util_stdio_printf("adc_value = %d.adc_vol = %dmV\r\n",adc_value,adc_vol);
		}
    }
}
uint32_t heyos_hal_adc_get(heyos_adc_inst_t idx)
{
    return adc_value;
}

void heyos_hal_adc_init(heyos_adc_inst_t idx)
{
    adc_init(idx);
    // Enable the ADC interrupt in the NVIC.
    NVIC_EnableIRQ(ADC_IRQn);
    am_hal_interrupt_master_enable();

    // Enable the ADC interrupts in the ADC.
    am_hal_adc_interrupt_enable(g_ADCHandle, AM_HAL_ADC_INT_WCINC       |
                                             AM_HAL_ADC_INT_WCEXC       |
                                             AM_HAL_ADC_INT_FIFOOVR2    |
                                             AM_HAL_ADC_INT_FIFOOVR1    |
                                             AM_HAL_ADC_INT_SCNCMP      |
                                             AM_HAL_ADC_INT_CNVCMP);

    // Kick Start Timer 3 with an ADC software trigger in REPEAT used.
    am_hal_adc_sw_trigger(g_ADCHandle);
	am_util_stdio_printf("configuring ADC succ.\r\n");
}

void heyos_hal_adc_deinit(heyos_adc_inst_t idx)
{
    am_hal_adc_disable(g_ADCHandle);
    am_hal_adc_power_control(g_ADCHandle,AM_HAL_SYSCTRL_DEEPSLEEP,true);
    am_hal_gpio_pinconfig(ADC_Channel[idx].GPIO_Pin,g_AM_HAL_GPIO_DISABLE);
    am_hal_adc_deinitialize(g_ADCHandle);
}



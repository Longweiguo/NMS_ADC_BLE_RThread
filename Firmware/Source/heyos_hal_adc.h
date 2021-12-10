/*
* Copyright (c), RY Inc.
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited without the written consent of the copyright owner.
*
*/
#ifndef __HEYOS_HAL_ADC_H__
#define __HEYOS_HAL_ADC_H__

#define ADC_RES_14BIT      (16383)
#define ADC_REFER_VOL      (1500)  //mV
typedef enum
{
    ADC_INDEX_BAT,
    ADC_INDEX_NTC1,
    ADC_INDEX_NTC2,
    ADC_INDEX_USB,
    ADC_INDEX_MAX
}heyos_adc_inst_t;
/*
 * CONSTANTS
 *******************************************************************************
 */


/*
 * TYPES
 *******************************************************************************
 */

/**
 * @brief  Definition for ADC peripheral instance
 */

/*
 * FUNCTIONS
 *******************************************************************************
 */


/**
 * @brief   API to init specified ADC module
 *
 * @param   idx  - The specified ADC instance
 *
 * @return  None
 */
void heyos_hal_adc_init(heyos_adc_inst_t idx);

/**
 * @brief   API to enable specified ADC module
 *
 * @param   idx  - The specified ADC instance
 *
 * @return  None
 */
void heyos_hal_adc_enable(heyos_adc_inst_t idx);

/**
 * @brief   API to disable specified ADC module
 *
 * @param   idx  - The specified ADC instance
 *
 * @return  None
 */
void heyos_hal_adc_disable(void);

void heyos_hal_adc_deInitialize(heyos_adc_inst_t idx);

/**
 * @brief   API to get specified ADC value
 *
 * @param   idx  - The specified ADC instance
 *
 * @return  Value of ADC sampling
 */
unsigned int heyos_hal_adc_get(heyos_adc_inst_t idx);



#endif  /* __RY_HAL_ADC_H__ */

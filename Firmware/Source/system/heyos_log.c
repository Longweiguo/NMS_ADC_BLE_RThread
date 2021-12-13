/*
* Copyright (c), RY Inc.
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited without the written consent of the copyright owner.
*
*/

/*********************************************************************
 * INCLUDES
 */

/* Basic */
#include "heyos_type.h"
#include "heyos_utils.h"
#include "heyos_log.h"

//#include "am_hal_stimer.h"

#if (RT_LOG_ENABLE == TRUE)


/*********************************************************************
 * CONSTANTS
 */ 

#define RT_LOG_ITM_PORT      1

#define clock_time()  am_hal_stimer_counter_get()

/*********************************************************************
 * TYPEDEFS
 */



 
/*********************************************************************
 * LOCAL VARIABLES
 */



/*********************************************************************
 * LOCAL FUNCTIONS
 */

void log_write(int id, int type, u32_t dat)
{
    uint32_t ui32Critical;
    ui32Critical = am_hal_interrupt_master_disable();
    am_hal_itm_stimulus_reg_byte_write(RT_LOG_ITM_PORT, (dat & 0xff));
    am_hal_itm_stimulus_reg_byte_write(RT_LOG_ITM_PORT, ((dat>>8) & 0xff));
    am_hal_itm_stimulus_reg_byte_write(RT_LOG_ITM_PORT, ((dat>>16) & 0xff));
    am_hal_itm_stimulus_reg_byte_write(RT_LOG_ITM_PORT, (type|id) & 0xff);
    am_hal_interrupt_master_set(ui32Critical);
}

void log_task_begin(int id)
{
    log_write(id, RT_LOG_MASK_BEGIN, clock_time());
}

void log_task_end(int id)
{
    log_write(id, RT_LOG_MASK_END, clock_time());
}

void log_event(int id)
{
    log_write(id, RT_LOG_MASK_TGL, clock_time());
}


void log_data(int id, u32_t dat)
{
    log_write(id, RT_LOG_MASK_DAT, dat);
}


#endif /* (RT_LOG_ENABLE == TRUE) */

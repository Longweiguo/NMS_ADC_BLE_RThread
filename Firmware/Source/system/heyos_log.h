/*
* Copyright (c), RY Inc.
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited withouts the written consent of the copyright owner.
*
*/
#pragma once

#include "heyos_type.h"
#include "heyos_log_id.h"
//#include "app_coinfig.h"

/**
 * @brief Definitaion for RT log type.
 */
#define	RT_LOG_MASK_BEGIN		0x40
#define	RT_LOG_MASK_END		    0x00
#define	RT_LOG_MASK_TGL		    0xC0
#define	RT_LOG_MASK_DAT		    0x80

#if (RT_LOG_ENABLE == TRUE)
    void log_write_data(int id, int type, u32 dat);
    void log_task_begin(int id);
    void log_task_end(int id);
    void log_event(int id);
    void log_data(int id, u32 dat);
#else
    #define log_write_data(id, type, dat)
    #define log_task_begin(id)
    #define log_task_end(id)
    #define log_event(id)
    #define log_data(id, dat)
#endif  /* RT_LOG_ENABLE */

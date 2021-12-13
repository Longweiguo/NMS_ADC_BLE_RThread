
#pragma once

// should define the HEYOS_CONFIG macro bsp/project wide. do not change this file to fit project config
#ifdef HEYOS_CONFIG
#include HEYOS_STRINGIZE(HEYOS_CONFIG)
#endif

#ifndef HEYOS_KERNEL_OBJ_NAME_LEN
#define HEYOS_KERNEL_OBJ_NAME_LEN 20
#endif

#ifndef HEYOS_FILE_PATH_MAX_LEN
#define HEYOS_FILE_PATH_MAX_LEN 256
#endif


#define HEYOS_DEBUG                                1
#define HEYOS_RELEASE                              2

#ifndef HEYOS_BUILD
#define HEYOS_BUILD                                HEYOS_DEBUG
#endif

#if HEYOS_BUILD == HEYOS_DEBUG
#define HEYOS_BUILD_DEBUG
#endif

#if HEYOS_BUILD == HEYOS_RELEASE
#define HEYOS_BUILD_RELEASE
#endif

#if defined(HEYOS_BUILD_DEBUG) && defined(HEYOS_BUILD_RELEASE)
#error conflict build env
#endif

#ifndef HEYOS_THREAD_USAGE_DBG_ENABLE
    #if defined(HEYOS_BUILD_DEBUG)
        #define HEYOS_THREAD_USAGE_DBG_ENABLE           1
    #else
        #define HEYOS_THREAD_USAGE_DBG_ENABLE           0
    #endif
#endif

#ifndef RT_MEMHEAP_USE_TRACE
    #if defined(HEYOS_BUILD_DEBUG)
        #define RT_MEMHEAP_USE_TRACE           1
    #else
        #define RT_MEMHEAP_USE_TRACE           0
    #endif
#endif


#ifndef RT_SEGGER_RTT_KEEP_CONSOLE
    // 保留segger_rtt在rt-thread中的serial dev debug 建议保留-->用于串口ymodem提高稳定性
    #if defined(HEYOS_BUILD_DEBUG)
        #define RT_SEGGER_RTT_KEEP_CONSOLE           1
    #else
        #define RT_SEGGER_RTT_KEEP_CONSOLE           0
    #endif
#endif


#ifndef HEYOS_USE_TAG_TRACE
    // 适用于单线程的时间占用打点分析
    #if defined(HEYOS_BUILD_DEBUG)
        #define HEYOS_USE_TAG_TRACE           0
    #else
        #define HEYOS_USE_TAG_TRACE           0
    #endif
#endif


#ifndef HEYOS_USE_SYSVIEW_TRACE
    // 适用于segger system_view, 分析系统调度情况
    #if defined(HEYOS_BUILD_DEBUG)
        #define HEYOS_USE_SYSVIEW_TRACE           0
    #else
        #define HEYOS_USE_SYSVIEW_TRACE           0
    #endif
#endif


#if HEYOS_USE_SYSVIEW_TRACE
#define PKG_USING_SYSTEMVIEW
#define SYSVIEW_APP_NAME            "heyos-trace"
#define SEGGER_SYSVIEW_CORE         SEGGER_SYSVIEW_CORE_CM3
#define SYSVIEW_RAM_BASE            (0x10000000)
#define SYSVIEW_DEVICE_NAME         "Cortex-M4"
#define SYSVIEW_SYSDESC0            "I#39=SysTick"
#define SYSVIEW_TIMESTAMP_FREQ      (96000000u)
#define SYSVIEW_CPU_FREQ            (96000000u)
#endif

#include "board_ram.h"

#define RT_KERNEL_MALLOC(sz)            heyos_sram_malloc(sz)
#define RT_KERNEL_FREE(ptr)             heyos_sram_free(ptr)
#define RT_KERNEL_REALLOC(ptr, size)    heyos_sram_realloc(ptr, size)

//#define HEYOS_MEDULE_IOT

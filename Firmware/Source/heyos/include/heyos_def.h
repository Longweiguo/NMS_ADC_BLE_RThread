
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "heyos_type.h"


/**
 *  #define HEYOS_CONFIG_H heyos_rum_config.h
 * 
 *  #include HEYOS_STRINGIZE(HEYOS_CONFIG_H) --> #include "heyos_rum_config.h"
 */
#define HEYOS_STRINGIZE(x) HEYOS_STRINGIZE2(x)
#define HEYOS_STRINGIZE2(x) #x


#ifdef ARCH_CPU_64BIT
typedef signed long                     int64_t;     /**< 64bit integer type */
typedef unsigned long                   uint64_t;    /**< 64bit unsigned integer type */
#else
#if !defined (__GNUC__)                /* GNU GCC Compiler */
    typedef signed long long                int64_t;     /**< 64bit integer type */
    typedef unsigned long long              uint64_t;    /**< 64bit unsigned integer type */
#endif
#endif

/* boolean type definitions */
#define HEYOS_TRUE                         1               /**< boolean true  */
#define HEYOS_FALSE                        0               /**< boolean fails */

#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#define __CLANG_ARM
#endif

typedef void (*init_call)(void);
/* Compiler Related Definitions */
#if defined(__CC_ARM) || defined(__CLANG_ARM)           /* ARM Compiler */
    #include <stdarg.h>
    #define SECTION(x)                  __attribute__((section(x)))
    #define HEYOS_UNUSED                __attribute__((unused))
    #define HEYOS_USED                  __attribute__((used))
    #define ALIGN(n)                    __attribute__((aligned(n)))

    #define HEYOS_WEAK                  __attribute__((weak))
    #define reyos_inline                static __inline

#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    #include <stdarg.h>
    #define SECTION(x)                  @ x
    #define HEYOS_UNUSED
    #define HEYOS_USED                  __root
    #define PRAGMA(x)                   _Pragma(#x)
    #define ALIGN(n)                    PRAGMA(data_alignment=n)
    #define HEYOS_WEAK                  __weak
    #define heyos_inline                static inline


#elif defined (__GNUC__)                /* GNU GCC Compiler */
    #ifdef USING_NEWLIB
        #include <stdarg.h>
    #endif

//    extern init_call START_DEVICES     __asm("section$start$__DATA$__devsection");
//    extern init_call END_DEVICES       __asm("section$end$__DATA$__devsection");
    #if defined (__clang__)

        #define SECTION(x)                  __attribute__((section("__DATA,"x)))
        #define HEYOS_UNUSED                __attribute__((unused))
        #define HEYOS_USED                  __attribute__((used))
        #define ALIGN(n)                    __attribute__((aligned(n)))

    #else
        /* the version of GNU GCC must be greater than 4.x */

        typedef __builtin_va_list           __gnuc_va_list;
        typedef __gnuc_va_list              va_list;
        #define va_start(v,l)               __builtin_va_start(v,l)
        #define va_end(v)                   __builtin_va_end(v)
        #define va_arg(v,l)                 __builtin_va_arg(v,l)

        #define SECTION(x)                  __attribute__((section(x)))
        #define HEYOS_UNUSED                __attribute__((unused))
        #define HEYOS_USED                  __attribute__((used))
        #define ALIGN(n)                    __attribute__((aligned(n)))
        #define HEYOS_WEAK                  __attribute__((weak))
        #define heyos_inline                   static __inline
    #endif
#elif defined (_MSC_VER)
    #define HEYOS_WEAK
#else
    #error not supported tool chain
#endif

#ifdef _MSC_VER
    #define INIT_SCTION(func, comp)
#else
    #define INIT_SCTION(func, comp)            \
    HEYOS_USED const init_call __init_##func         SECTION(comp) = func
#endif


#define HEYOS_NAME_MAX                     16


#ifdef __cplusplus
}
#endif

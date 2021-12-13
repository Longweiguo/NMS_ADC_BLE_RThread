/*
* Copyright (c), RY Inc.
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited without the written consent of the copyright owner.
*
*/
#pragma once

#include "heyos.h"
#include "board.h"
#include "heyos_type.h"
#include "rtthread.h"

typedef enum {
    PRINT_LEVEL_DEBUG = 0,
    PRINT_LEVEL_INFO,
    PRINT_LEVEL_WARNING,
    PRINT_LEVEL_ERROR,
    PRINT_LEVEL_FATAL,
} print_level_t;

extern print_level_t print_level;

char* heyos_log_print_time(void);

#ifndef LOG_LEVEL_RATE
#define LOG_LEVEL_RATE            1
#endif


#ifndef APPCONFIG_DEBUG_ENABLE
    #if defined(HEYOS_BUILD_DEBUG)
        #define APPCONFIG_DEBUG_ENABLE  1
    #else
        #define APPCONFIG_DEBUG_ENABLE  0
    #endif
#endif

extern void heyos_raw_log_info_save(const char* fmt, ...);
extern void heyos_error_info_save(const char* fmt, ...);
extern void heyos_error_info_fatal_save(const char* p_str, int length);
// extern void ctx_info_save(char* fmt, ...);

#if APPCONFIG_DEBUG_ENABLE
    #define LOG_DEBUG_RAW(_fmt_, ...)   do{rt_kprintf(_fmt_, ##__VA_ARGS__);}while(0)
    #define LOG_DEBUG(_fmt_, ...)   do{rt_kprintf("%s[D] "_fmt_, heyos_log_print_time(), ##__VA_ARGS__);}while(0)
    #define LOG_INFO(_fmt_, ...)    do{rt_kprintf("%s[I] "_fmt_, heyos_log_print_time(), ##__VA_ARGS__);heyos_error_info_save("%s[I] "_fmt_, heyos_log_print_time(), ##__VA_ARGS__);}while(0)
    #define LOG_INFO_APPEND(_fmt_, ...) do{rt_kprintf(_fmt_, ##__VA_ARGS__);heyos_error_info_save(_fmt_, ##__VA_ARGS__);}while(0)
    #define LOG_ERROR(_fmt_, ...)   do{rt_kprintf("%s[E] "_fmt_, heyos_log_print_time(), ##__VA_ARGS__);heyos_error_info_save("%s[E] "_fmt_, heyos_log_print_time(), ##__VA_ARGS__);}while(0)
    #define LOG_WARN(_fmt_, ...)        do{rt_kprintf("%s[W] "_fmt_, heyos_log_print_time(), ##__VA_ARGS__);}while(0)
    #define LOG_FATAL(p_str, length)    do{heyos_error_info_fatal_save(p_str, length);}while(0)
    #if RAWLOG_SAMPLE_ENABLE
        #define LOG_RAW(_fmt_, ...)     do{heyos_raw_log_info_save(_fmt_, ##__VA_ARGS__);}while(0)
    #else
        #define LOG_RAW(...)
    #endif
#else
    #define LOG_DEBUG(_fmt_, ...)
    #define LOG_DEBUG_RAW(_fmt_, ...)
    #define LOG_FATAL(_fmt_, ...)
    #define LOG_WARN(_fmt_, ...)
    #if RAWLOG_SAMPLE_ENABLE
        #define LOG_INFO(...)
        #define LOG_INFO_APPEND(...)
        #define LOG_ERROR(...)
        #define LOG_RAW(_fmt_, ...)         do{heyos_raw_log_info_save(_fmt_, ##__VA_ARGS__);}while(0)
    #else
        #define LOG_INFO(_fmt_, ...)        do{heyos_error_info_save("%s[I] "_fmt_"\n", heyos_log_print_time(), ##__VA_ARGS__);}while(0)
        #define LOG_INFO_APPEND(_fmt_, ...) do{heyos_error_info_save(_fmt_"\n", ##__VA_ARGS__);}while(0)
        #define LOG_ERROR(_fmt_, ...)       do{heyos_error_info_save("%s[E] "_fmt_"\n", heyos_log_print_time(), ##__VA_ARGS__);}while(0)
    #endif
#endif /* APPCONFIG_DEBUG_ENABLE */


#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif

// error info str

#define ERR_STR_MALLOC_FAIL         "malloc fail"
#define ERR_STR_ENCODE_FAIL         "encode fail"
#define ERR_STR_DECODE_FAIL         "decode fail"

#define ERR_STR_QUEUE_SEND_FAIL     "Queue send error"
#define ERR_STR_FILE_OPEN           "file open fail"
#define ERR_STR_FILE_WRITE          "file write fail"
#define ERR_STR_FILE_READ           "file read fail"
#define ERR_STR_FILE_SEEK           "file seek fail"
#define ERR_STR_FILE_CLOSE          "file close fail"


/**
 * @brief   Data dump utility API
 *
 * @param   None
 *
 * @return  None
 */
void heyos_data_dump(uint8_t* data, int len, char split);
void heyos_sp_dump(uint32_t sizeBytes);


/**
 * @brief   Utility API to check if all data is empty: 0xFF
 *
 * @param   data - Data to check
 * @param   len - Len of data
 *
 * @return  None
 */
bool is_empty(u8* data, int len);

void     reverse_mac(uint8_t* mac);

int      str2hex(char *s, int len, uint8_t* h);
int      heyos_utc_parse(uint32_t ts, uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday, uint8_t* hour, uint8_t* min, uint8_t* sec);

heyos_ret_t heyos_aes_cbc_encrypt(uint8_t* key, uint8_t* iv, int len, uint8_t* input, uint8_t* output);
heyos_ret_t heyos_aes_cbc_decrypt(uint8_t* key, uint8_t* iv, int len, uint8_t* input, uint8_t* output);


// MD5
void heyos_md5(uint8_t* input, int input_len, uint8_t* output);

// MD5 in stream
#if 0
heyos_ret_t heyos_md5_init(void);
heyos_ret_t heyos_md5_update(heyos_md5_handle_t* md5_ctx, uint8_t* input, int input_len);
heyos_ret_t heyos_md5_finish(uint8_t *output);
void heyos_md5_reset(void);
#endif

typedef void heyos_md5_handle_t;
heyos_md5_handle_t* heyos_md5_init(void);
heyos_ret_t heyos_md5_update(heyos_md5_handle_t* md5_ctx, uint8_t* input, int input_len);
heyos_ret_t heyos_md5_finish(heyos_md5_handle_t** md5_ctx, uint8_t *output);
void heyos_md5_reset(heyos_md5_handle_t** handle);

/**
 * @brief Update utc to hardware system
 *
 * @return uint32_t
 */
void heyos_timestamp_update(uint32_t utc);

/**
 * @brief get local stamp from rtc
 *
 * @return uint32_t
 */
uint32_t heyos_timestamp_local_now(void);

/**
 * @brief get utc currently
 *
 * @return uint32_t
 */
uint32_t heyos_utc_now(void);

/**
 * @brief 判断utc是否在当天的有效范围内
 *
 * @param target_utc - 待校验的utc时间戳
 *
 * @return true: 在当天有效范围内
 * @return false: 不在当天有效范围内
 */
bool heyos_check_utc_is_within_today(uint32_t target_utc);

/**
 * @brief utc关机后，时间未同步之前的时间是无效的
 *
 * @return 时间是无效的 or 不是明显无效的
 */
bool heyos_utc_time_is_invalid(void);

/**
 * @brief get utc parse string
 *      - careful:
 *          do not call two times in one print statement
 *          in the same print statement, should cache the first result, and then print
 */
char* heyos_parse_utc_to_string(uint32_t utc);

int heyos_utc_parse(uint32_t ts, uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday, uint8_t* hour, uint8_t* min, uint8_t* sec);
int heyos_utc_parse_without_timezone(uint32_t ts, uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday, uint8_t* hour, uint8_t* min, uint8_t* sec);

uint32_t heyos_rand(void);
void heyos_block_ms(uint32_t ms);
void heyos_block_us(uint32_t us);
signed long heyos_divround(signed long A,signed long B);

#if HEYOS_USE_TAG_TRACE
// 适用于单线程的打点分析
void heyos_trace_tag_reset(void);
void heyos_trace_tag_add(char* tag);
void heyos_trace_tag_dump(void);
#else
#define heyos_trace_tag_reset(...)
#define heyos_trace_tag_add(...)
#define heyos_trace_tag_dump(...)
#endif

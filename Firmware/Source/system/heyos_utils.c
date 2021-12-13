// /*
// * Copyright (c), ry Inc.
// *
// * All rights are reserved. Reproduction in whole or in part is
// * prohibited without the written consent of the copyright owner.
// *
// */

// /*********************************************************************
//  * INCLUDES
//  */

// /* Basic */
#include "heyos_type.h"
#include "heyos_utils.h"
#include <time.h>
#include "heyos_hal_inc.h"

#include <stdio.h>
#include <stdlib.h>

#include <heyos.h>
#include "motor/ss_motor.h"
#include "psram_no_init.h"

#define FREE_PTR(ptr)   \
do{\
    heyos_free(ptr);\
    ptr = NULL;\
}while(0)

HEYOS_WEAK int32_t get_device_time_deviation(void){return 0;}
HEYOS_WEAK int system_time_zone_deviation_get(void){return 0;}

/**
 * @brief   Data dump utility API
 *
 * @param   None
 *
 * @return  None
 */
void heyos_data_dump(uint8_t* data, int len, char split)
{
#if defined(HEYOS_BUILD_DEBUG)
    for (int i = 0; i < len; i++) {
        LOG_DEBUG_RAW("%02x", data[i]);
        if(i < len - 1){
            LOG_DEBUG_RAW("%c", split);
        }
    }
    LOG_DEBUG_RAW("\r\n");
#endif
}

// static inline uint32_t __get_PSP(void)
// {
//   register uint32_t __regProcessStackPointer  __asm("psp");
//   return(__regProcessStackPointer);
// }

// static inline uint32_t __get_MSP(void)
// {
//   register uint32_t __regMainStackPointer     __asm("msp");
//   return(__regMainStackPointer);
// }

// static inline uint32_t __get_IPSR(void)
// {
//   register uint32_t __regIPSR          __asm("ipsr");
//   return(__regIPSR);
// }


// void heyos_sp_dump(uint32_t sizeBytes)
// {
//     uint32_t msp = __get_MSP();
//     uint32_t psp = __get_PSP();
//     uint32_t ispr = __get_IPSR();
//     const uint32_t ram_begin = 0x10000000;
//     const uint32_t ram_end = 0x10040000;
//     uint8_t sp_type[4] = {"msp\0"};
//     uint32_t* start_addr = NULL;
//     if(sizeBytes%4 != 0) {
//         return;
//     }

//     if(ispr == 0) {
//         start_addr = (uint32_t*)(psp);
//         sp_type[0] = 'p';
//     } else {
//         start_addr = (uint32_t*)(msp);
//     }

//     if(((uint32_t)start_addr + sizeBytes) < ram_end) {
// #if (RY_BUILD == RY_DEBUG)
//         ry_board_printf("[SP_DUMP] %s:%08X, len:%d\n", sp_type, start_addr,sizeBytes);
//         for(int i=0;i<sizeBytes/4;i++) {
//             ry_board_printf("%08X ", start_addr[i]);
//         }
//         ry_board_printf("\r\n");
// #else
//         LOG_INFO("[SP_DUMP] %s:%08X, [5]:%08X,[6]:%08X,[7]:%08X,[8]:%08X\n",
//             sp_type,
//             start_addr,
//             start_addr[5],
//             start_addr[6],
//             start_addr[7],
//             start_addr[8]
//             );
// #endif
//     }
// }


// /**
//  * @brief   Utility API to check if all data is empty: 0xFF
//  *
//  * @param   data - Data to check
//  * @param   len - Len of data
//  *
//  * @return  None
//  */
// bool is_empty(u8_t* data, int len)
// {
//     int i;
//     for (i = 0; i < len; i++) {
//         if (data[i] != EMPTY) {
//             return FALSE;
//         }
//     }
//     return TRUE;
// }

static uint64_t get_utc_tick_from_rtc_without_timezone(void)
{
    static uint64_t __last_timestamp = 1;
    static uint32_t __last_rt_tick = 1;
    uint32_t tick = heyos_tick_get();
    if(__last_rt_tick == tick){
        return __last_timestamp;
    }
    __last_rt_tick = tick;
    heyos_hal_rtc_time_t time = {0};
    heyos_hal_rtc_get(&time);
    if(time.year >= 2000){
        time.year -= 2000;
    }
    /*
    int tm_sec;   // seconds after the minute - [0, 60] including leap second
    int tm_min;   // minutes after the hour - [0, 59]
    int tm_hour;  // hours since midnight - [0, 23]
    int tm_mday;  // day of the month - [1, 31]
    int tm_mon;   // months since January - ~~[0, 11]~~
    int tm_year;  // years since 1900
    int tm_wday;  // days since Sunday - [0, 6]
    int tm_yday;  // days since January 1 - [0, 365]
    int tm_isdst; // daylight savings time flag
    */
    struct tm now;
    now.tm_year = time.year + 100;
    now.tm_mon = time.month - 1;
    now.tm_mday = time.dayOfMonth;
    now.tm_isdst = false;
    now.tm_wday = time.dayOfWeek;
    now.tm_hour = time.hour;
    now.tm_min = time.min;
    now.tm_sec = time.sec;
    __last_timestamp = mktime(&now);
    // #if defined(NO_UI)
    // uint32_t raw_utc = __last_timestamp;
    // LOG_DEBUG("[utc] %d-%02d-%02d %02d:%02d:%02d->%u\n",
    //     now.tm_year,
    //     now.tm_mon,
    //     now.tm_mday,
    //     now.tm_hour,
    //     now.tm_min,
    //     now.tm_sec,
    //     raw_utc
    //     );
    // #endif
    return __last_timestamp;
}

uint32_t heyos_timestamp_local_now(void)
{
    uint32_t stamp = get_utc_tick_from_rtc_without_timezone();
    return stamp;
}

uint32_t heyos_utc_now(void)
{
    uint32_t stamp = get_utc_tick_from_rtc_without_timezone();
    stamp -= system_time_zone_deviation_get();
    //stamp -= get_device_time_deviation();
    return stamp;
}

bool heyos_utc_time_is_invalid(void)
{
    bool invalid = (heyos_utc_now() < 1601481600);    // utc < 2020-10-01
    return invalid;
}

void dump_utc_now(void)
{
    uint32_t utc = heyos_utc_now();
    LOG_DEBUG("[utc] %d 0x%X\n", utc, utc);
    LOG_DEBUG("[zone_deviation] %d 0x%X\n", system_time_zone_deviation_get(), system_time_zone_deviation_get());
    LOG_DEBUG("[local_now] %d 0x%X\n", heyos_timestamp_local_now(), heyos_timestamp_local_now());
}
MSH_CMD_EXPORT(dump_utc_now, dump_utc_now);

/**
 * @brief 判断utc是否在当天的有效范围内
 *
 * @param target_utc - 待校验的utc时间戳
 *
 * @return true: 在当天有效范围内
 * @return false: 不在当天有效范围内
 */
bool heyos_check_utc_is_within_today(uint32_t target_utc)
{
    bool within_today = false;
    const int32_t k_begin_offset = 0;
    const int32_t k_end_offset = 86399;

    //uint32_t local_target_stamp = target_utc + get_device_time_deviation();
    uint32_t local_target_stamp = target_utc + system_time_zone_deviation_get();
    uint32_t local_stamp_now = heyos_timestamp_local_now();
    uint32_t local_begin_of_day = local_stamp_now - (local_stamp_now % 86400);

    /*
     * 基于当前时区时间进行判断
     */
    uint32_t today_timestamp_start = local_begin_of_day + k_begin_offset;
    uint32_t today_timestamp_end = local_begin_of_day + k_end_offset;
    if ((local_target_stamp >= today_timestamp_start) && (local_target_stamp <= today_timestamp_end)){
        within_today = true;
    }

    // LOG_DEBUG("[step] timestamp:%d,valid:%d,start:%d end:%d\n", \
        target_utc, within_today, today_timestamp_start, today_timestamp_end);

    return within_today;
}

HEYOS_WEAK void ryos_shutdown_prepare(void)
{

}


//---------------This part should be remove from heyos_utils.c--------------

extern heyos_ret_t heyos_charger_wdt_disable(void);
extern void idle_page_refresh_cancel(void);
extern void system_profile_do_update(void);
extern void usr_profile_do_update(void);
extern void alarm_to_storage(void);
extern void service_health_storage(void);
extern int heyos_filesystem_teardown(void);
extern void poweroff_stop_thread(void);
extern void service_workthread_suspend(void);
extern void ry_error_log_force_save(void);
extern heyos_ret_t heyos_charger_in_shipping_mode(void);
extern heyos_ret_t heyos_power_poweroff(void);

void heyos_sys_reboot(void)
{
    heyos_charger_wdt_disable();
    idle_page_refresh_cancel();
    system_profile_do_update();
    usr_profile_do_update();
    alarm_to_storage();
    service_health_storage();

    poweroff_stop_thread();

    heyos_filesystem_teardown();
    no_init_hardfault_set(false);
    is_reboot_soft(true);
    heyos_hal_mcu_reset();
}

void heyos_sys_shutdown(void)
{
    idle_page_refresh_cancel();
    poweroff_stop_thread();
    service_workthread_suspend();
    system_profile_do_update();
    usr_profile_do_update();
    alarm_to_storage();
    service_health_storage();
    // Save last 4K log from PSRAM to Filesystem.
    ry_error_log_force_save();
    heyos_delay_ms(100);
    heyos_filesystem_teardown();
    //    heyos_delay_ms(100);
    is_reboot_soft(true);
    heyos_thread_t home_thread = heyos_find_thread("home_button");
    if(home_thread) {
        heyos_thread_resume(home_thread);
    }
    heyos_power_poweroff();
}

void heyos_sys_shipping_mode(void)
{
    if(!get_dev_power_status())
    {
        idle_page_refresh_cancel();
        poweroff_stop_thread();
        system_profile_do_update();
        usr_profile_do_update();
        heyos_filesystem_teardown();
    }
    am_hal_interrupt_master_disable();
    //    heyos_delay_ms(500);
    is_reboot_soft(true);
    heyos_charger_in_shipping_mode();
    while(1);
}

//----------------------------------------------------------------------



// /**
//  * @brief   Utility API print the time in LOG.
//  *
//  * @param   None
//  *
//  * @return  None
//  */

#if defined(HEYOS_BUILD_DEBUG)
char* heyos_log_print_time(void)
{
    static char print_time_buffer[64] = "";
    heyos_hal_rtc_time_t now_time;
    heyos_hal_rtc_get(&now_time);

    uint32_t len = rt_snprintf(print_time_buffer, 64, "%04d-%02d-%02d %02d:%02d:%02d %d t:%d",
                               now_time.year, now_time.month, now_time.dayOfMonth,
                               now_time.hour, now_time.min, now_time.sec,
                               heyos_calc_ms(heyos_hal_clock_time()), heyos_tick_get());

//    if(len < 30) {
//        print_time_buffer[len] = '\0';
//    }
    return print_time_buffer;
//	ry_board_printf();
}
#else
char* heyos_log_print_time(void)
{
    static char release_print_time_buffer[32];
    heyos_hal_rtc_time_t now_time;
    heyos_hal_rtc_get(&now_time);
    heyos_hal_rtc_time_t* p_rtc_now = &now_time;
    //uint32_t utc = heyos_hal_rtc_get();

//    uint32_t y = p_rtc_now->year;
    uint32_t mon = p_rtc_now->month;
    uint32_t d = p_rtc_now->dayOfMonth;
    uint32_t h = p_rtc_now->hour;
    uint32_t min = p_rtc_now->min;
    uint32_t s = p_rtc_now->sec;
    uint32_t t = heyos_tick_get();
    uint32_t len = rt_snprintf(release_print_time_buffer, 32, "[%d-%d %d:%d:%d %d] ", mon, d, h, min, s, t);
    if(len < 30) {
        release_print_time_buffer[len] = '\0';
    }

    return release_print_time_buffer;
}
#endif

char* heyos_parse_utc_to_string(uint32_t utc)
{
    static char utc_time_buffer[32];
    uint16_t y;
    uint8_t  mon, d, hour, min, s, weekday;

    heyos_utc_parse(utc, &y, &mon, &d, &weekday, &hour, &min, &s);
    heyos_memset(utc_time_buffer, 0, 32);
    uint32_t len = rt_snprintf(utc_time_buffer, 32, "%04d-%02d-%02d %02d:%02d:%02d", y, mon, d, hour, min, s);

    return utc_time_buffer;
}

void heyos_timestamp_update(uint32_t utc)
{
    uint16_t y;
    uint8_t mon, d, h, min, s, weekday;
    heyos_hal_rtc_time_t temp_time = {0};
    heyos_utc_parse(utc, &y, &mon, &d, &weekday, &h, &min, &s);

    temp_time.hour       = h;
    temp_time.min        = min;
    temp_time.sec        = s;
    temp_time.dayOfWeek  = (heyos_hal_rtc_week_t)weekday;
    temp_time.dayOfMonth = d;
    temp_time.month      = mon;
    temp_time.year       = y;

    heyos_hal_rtc_set(&temp_time);
}


int heyos_utc_parse(uint32_t ts, uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    //uint32_t time = ts + get_device_time_deviation();
    uint32_t time = ts + system_time_zone_deviation_get();

    struct tm *ptm = localtime(&time);
    if(ptm == NULL){
        LOG_ERROR("[util] parse utc err:%d\n", ts);
    }
    if(year != NULL){
        *year = ptm->tm_year + 1900;
    }
    if(month != NULL){
        *month = ptm->tm_mon + 1;
    }
    if(day != NULL){
        *day = ptm->tm_mday;
    }
    if(weekday != NULL){
        *weekday = ptm->tm_wday;
    }
    if(hour != NULL){
        *hour = ptm->tm_hour;
    }
    if(min != NULL){
        *min = ptm->tm_min;
    }
    if(sec != NULL){
        *sec = ptm->tm_sec;
    }
    return 0;
}

int heyos_utc_parse_without_timezone(uint32_t ts, uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    //uint32_t time = ts + get_device_time_deviation();
    uint32_t time = ts;// + system_time_zone_deviation_get();

    struct tm *ptm = localtime(&time);
    if(ptm == NULL){
        LOG_ERROR("[util] parse utc err:%d\n", ts);
    }
    if(year != NULL){
        *year = ptm->tm_year + 1900;
    }
    if(month != NULL){
        *month = ptm->tm_mon + 1;
    }
    if(day != NULL){
        *day = ptm->tm_mday;
    }
    if(weekday != NULL){
        *weekday = ptm->tm_wday;
    }
    if(hour != NULL){
        *hour = ptm->tm_hour;
    }
    if(min != NULL){
        *min = ptm->tm_min;
    }
    if(sec != NULL){
        *sec = ptm->tm_sec;
    }
    return 0;
}

uint64_t heyos_stamp_ms_u64(uint64_t *last_time)
{
    uint32_t u32_last_time_low = (uint32_t)*last_time;
    uint32_t u32_last_time_high = (uint32_t)(*last_time >> 32);
    uint32_t now = heyos_clock_time();
    if (u32_last_time_low > now) {
        // 增加判断，防止其它如重入导致的时间变小的问题
        if ((u32_last_time_low - now) >= 0x80000000) {
            u32_last_time_high++;
            LOG_DEBUG("[stamp] %d->%d overflow\n", u32_last_time_low, now);
        }
    }
    u32_last_time_low = now;
    *last_time = (((uint64_t)u32_last_time_high) << 32) + (uint64_t)u32_last_time_low;
    uint64_t ms_low = heyos_calc_ms(now);
    uint64_t ms_high = ((uint64_t)u32_last_time_high<<(32-15))*1000;  // ((u32_last_time_high << 32) / 32768) * 1000
    return ms_high + ms_low;
}
// awtk weak
int64_t get_time_ms64(void)
{
    static uint64_t last_time = 0;
    return heyos_stamp_ms_u64(&last_time);
}
// awtk timer 专门使用的函数，防止重入导致的问题
uint64_t get_time_ms64_for_timer(void)
{
    static uint64_t last_time = 0;
    return heyos_stamp_ms_u64(&last_time);
}

int str2hex(char *src, int len, uint8_t* dst)
{
    uint8_t* s_dst;
    s_dst = dst;
    for (int i = 0; i < len; i+=2) {
        //sscanf(&src[i], "%02X", (char*)dst);
        sscanf(&src[i], "%02s", (char*)dst);
        dst++;
    }

    return (dst - s_dst);
}



#include "mbedtls/aes.h"
heyos_ret_t heyos_aes_cbc_encrypt(u8_t* key, u8_t* iv, int len, u8_t* input, u8_t* output)
{
   int cnt;
   int last_len;
   int ret;
   mbedtls_aes_context *ctx = (mbedtls_aes_context*)heyos_malloc(sizeof(mbedtls_aes_context));
   if (ctx == NULL) {
       LOG_ERROR("[heyos_aes_cbc_encrypt] No mem. %d \r\n", ret);
       return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_NO_MEM);
   }

   mbedtls_aes_init( ctx );
   ret = mbedtls_aes_setkey_enc( ctx, key, 128 );
   if (ret < 0) {
       heyos_free(ctx);
       LOG_ERROR("[heyos_aes_cbc_encrypt] AES Set key Error. %d \r\n", ret);
       return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_ENCRYPTION);
   }

   last_len = len%16;
   cnt = last_len ? (len>>4)+1 : (len>>4);


   for (int i = 0; i < cnt; i++) {

       ret = mbedtls_aes_crypt_cbc( ctx, MBEDTLS_AES_ENCRYPT, 16, iv, input + i*16, output + i*16 );
       if (ret < 0) {
            heyos_free(ctx);
            LOG_ERROR("AES Crypt Error. %d \r\n", ret);
            return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_ENCRYPTION);;
       }

       heyos_data_dump(output + i*16, 16, ' ');
   }

   heyos_free(ctx);
   return HEYOS_SUCC;
}


heyos_ret_t heyos_aes_cbc_decrypt(u8_t* key, u8_t* iv, int len, u8_t* input, u8_t* output)
{
   int cnt;
   int last_len;
   int ret;
   mbedtls_aes_context *ctx = (mbedtls_aes_context*)heyos_malloc(sizeof(mbedtls_aes_context));
   if (ctx == NULL) {
       LOG_ERROR("[heyos_aes_cbc_decrypt] No mem. %d \r\n", ret);
       return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_NO_MEM);
   }

   mbedtls_aes_init( ctx );
   ret = mbedtls_aes_setkey_dec( ctx, key, 128 );
   if (ret < 0) {
       LOG_ERROR("[heyos_aes_cbc_decrypt] AES Set key Error. %d \r\n", ret);
       heyos_free(ctx);
       return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_ENCRYPTION);
   }

   last_len = len%16;
   cnt = last_len ? (len>>4)+1 : (len>>4);


   for (int i = 0; i < cnt; i++) {

       ret = mbedtls_aes_crypt_cbc( ctx, MBEDTLS_AES_DECRYPT, 16, iv, input + i*16, output + i*16 );
       if (ret < 0) {
           LOG_ERROR("AES Crypt Error. %d \r\n", ret);
           heyos_free(ctx);
           return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_DECRYPTION);;
       }

       heyos_data_dump(output + i*16, 16, ' ');
   }

   FREE_PTR(ctx);
   return HEYOS_SUCC;
}


#include "mbedtls/md5.h"
#include "heyos_fs.h"
#include "string.h"
void heyos_md5(uint8_t* input, int input_len, uint8_t* output)
{
    mbedtls_md5_ret(input, input_len, output);
}

void ck_md5(int argc, char** argv)
{
    char* fn = argv[1];
    int file_len = 0;
    int read_len = 0;
    int fd;
    LOG_DEBUG("check %s md5:\n", fn);

    heyos_md5_handle_t* md5_t = heyos_md5_init();
    if(md5_t) {
        uint8_t* src_data = heyos_psram_malloc(1024 * 2);
        file_len = heyos_file_size(fn);
        fd = open(fn, O_RDONLY, 0);
        int len = 0;
        while(read_len < file_len) {
            memset(src_data, 0, 1024 * 2);
            len = (file_len - read_len) > (1024 * 2) ? 1024 * 2 : (file_len - read_len);
            read(fd, src_data, len);
            read_len += len;

            heyos_md5_update(md5_t, src_data, len);
        }
        uint8_t src_md5[16];
        heyos_md5_finish(&md5_t, src_md5);
        LOG_DEBUG(" %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
             src_md5[0],src_md5[1],src_md5[2],src_md5[3],src_md5[4],src_md5[5],src_md5[6],src_md5[7],
             src_md5[8],src_md5[9],src_md5[10],src_md5[11],src_md5[12],src_md5[13],src_md5[14],src_md5[15]);
        heyos_free(src_data);
    }
}
MSH_CMD_EXPORT(ck_md5, check file md5 code);

// MD5 for stream
#if 0
mbedtls_md5_context *heyos_md5_ctx_v = NULL;
heyos_ret_t heyos_md5_init(void)
{
    int ret = HEYOS_SUCC;

    if (heyos_md5_ctx_v) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_BUSY);
    }

    heyos_md5_ctx_v = heyos_malloc(sizeof(mbedtls_md5_context));
    if (heyos_md5_ctx_v == NULL) {
        LOG_ERROR("[%s] No mem. \n", __FUNCTION__);
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_NO_MEM);
    }

    if ((ret = mbedtls_md5_starts_ret(heyos_md5_ctx_v)) != HEYOS_SUCC) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INIT_FAIL);
    }

    return ret;
}

heyos_ret_t heyos_md5_update(uint8_t* input, int input_len)
{
    int ret = HEYOS_SUCC;

    if (heyos_md5_ctx_v == NULL) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }

    if ((ret = mbedtls_md5_update_ret(heyos_md5_ctx_v, input, input_len)) != HEYOS_SUCC) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }

    return ret;
}

heyos_ret_t heyos_md5_finish(uint8_t *output)
{
    int ret = HEYOS_SUCC;

    if (heyos_md5_ctx_v == NULL) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }

    if ((ret = mbedtls_md5_finish_ret(heyos_md5_ctx_v, output)) != HEYOS_SUCC) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }

    heyos_free(heyos_md5_ctx_v);
    heyos_md5_ctx_v = NULL;

    return ret;
}

void heyos_md5_reset(void)
{
    if (heyos_md5_ctx_v) {
        heyos_free(heyos_md5_ctx_v);
    }
    heyos_md5_ctx_v = NULL;
}
#endif


// MD5 for stream


heyos_md5_handle_t* heyos_md5_init(void)
{
    int ret = 0;
    heyos_md5_handle_t *p_ctx = (heyos_md5_handle_t*)heyos_malloc(sizeof(mbedtls_md5_context));

    if (p_ctx == NULL) {
        LOG_ERROR("[%s] No mem. \n", __FUNCTION__);
        return NULL;
    }

    if ((ret = mbedtls_md5_starts_ret(p_ctx)) != HEYOS_SUCC) {
        heyos_free(p_ctx);
        return NULL;
    }
    (void)ret;

    return p_ctx;
}

heyos_ret_t heyos_md5_update(heyos_md5_handle_t* handle, uint8_t* input, int input_len)
{
    int ret = HEYOS_SUCC;

    if (handle == NULL) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }

    if ((ret = mbedtls_md5_update_ret(handle, input, input_len)) != HEYOS_SUCC) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }

    return ret;
}

heyos_ret_t heyos_md5_finish(heyos_md5_handle_t** handle, uint8_t *output)
{
    int ret = HEYOS_SUCC;

    if (*handle == NULL) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }

    if ((ret = mbedtls_md5_finish_ret((heyos_md5_handle_t*)*handle, output)) != HEYOS_SUCC) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_AES, HEYOS_ERR_INVALID_PARA);
    }
    heyos_free(*handle);
    *handle = NULL;
    return ret;
}

void heyos_md5_reset(heyos_md5_handle_t** handle)
{
    if (*handle) {
        heyos_free(*handle);
        *handle = NULL;
    }
}


//total 32bit random
uint32_t heyos_rand(void)
{
    static size_t nextRand = 1;
    size_t multiplier = ( size_t ) heyos_hal_clock_time(), increment = ( size_t ) 1;

    /* Utility function to generate a pseudo random number. */
    nextRand = ( multiplier * nextRand ) + increment;
    return( ( nextRand >> 16 ) & ( ( size_t ) 0x7fff ) ) | ((heyos_hal_clock_time()&0x00000FFF)<<16);
}

/*
void heyos_block_ms(u32_t ms)
{
     volatile int32_t n;

     while (ms > 0) {
         for (n = 4800; n > 0; n--)
             {__asm("nop");
         }
         ms--;
     }
}
*/

void heyos_block_ms(uint32_t ms)
{
    heyos_hal_delay_ms(ms);
}

void heyos_block_us(u32_t us)
{
    heyos_hal_delay_us(us);
}

// Returns the round value of A/B
signed long heyos_divround(signed long A,signed long B)
{
	if (A<0)
		return (A-B/2)/B;
	else
		return (A+B/2)/B;
}

#if HEYOS_USE_TAG_TRACE

#ifndef HEYOS_TAG_TRACE_CNT
#define HEYOS_TAG_TRACE_CNT 8192
#endif

// static uint32_t trace_stamp_points[HEYOS_TAG_TRACE_CNT];
// static char* trace_tag_points[HEYOS_TAG_TRACE_CNT];
static uint32_t* trace_stamp_points = NULL;
static char** trace_tag_points = NULL;

uint32_t trace_tag_idx = HEYOS_TAG_TRACE_CNT;

void heyos_trace_tag_reset(void)
{
    trace_tag_idx = 0;
    if(trace_stamp_points == NULL){
        trace_stamp_points = heyos_psram_malloc(sizeof(uint32_t)*HEYOS_TAG_TRACE_CNT);
        trace_tag_points = heyos_psram_malloc(sizeof(char*)*HEYOS_TAG_TRACE_CNT);
    }
}
void heyos_trace_tag_add(char* tag)
{
    if(trace_tag_idx >= HEYOS_TAG_TRACE_CNT){
        return;
    }
    trace_tag_points[trace_tag_idx] = tag;
    trace_stamp_points[trace_tag_idx] = heyos_clock_time();
    trace_tag_idx++;
}
void heyos_trace_tag_dump(void)
{
    uint32_t begin = trace_stamp_points[0];
    for(int i=0;i<trace_tag_idx;i++){
        uint32_t delta = trace_stamp_points[i] - begin;
        uint32_t us_100 = delta;//(delta * 10000)>>15;
        LOG_DEBUG_RAW("%5d,%s\n", us_100, trace_tag_points[i]);
    }
}
#endif

/*
 * test
 * -----------------------------------------------------------------------------
 */
void force_set_unix_stamp(int argc, char** argv)
{
    uint32_t stamp = 0;

    if (argc == 2){
        stamp = atoi(argv[1]);
        // sscanf(argv[1], "%d", &stamp);
        LOG_DEBUG("[dip] intput unix stamp:%d\r\n", stamp);

        if ((stamp >= 1546300800) //
            &&(stamp < 2177452800)  //2039-1-1
            ){
            LOG_DEBUG("[dip] force stamp begin\r\n");
            heyos_timestamp_update(stamp);
            heyos_delay_tick(2);
            LOG_DEBUG("[dip] force stamp done\r\n");
        }

        {
            uint32_t utc = heyos_utc_now();
            uint32_t stamp = heyos_timestamp_local_now();
            uint16_t y;
            uint8_t m,d,h,min,s;
            heyos_utc_parse(utc, &y, &m, &d, NULL, &h, &min, &s);
            LOG_DEBUG("parse %d,%d %d-%d-%d %d:%d:%d\n", utc,stamp, y, m, d, h, min, s);
        }

        extern void alarm_refresh(void);
        alarm_refresh();
    }
}
MSH_CMD_EXPORT(force_set_unix_stamp, force_set_unix_stamp);

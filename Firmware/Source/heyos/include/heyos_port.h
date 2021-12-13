/*
* Copyright (c), RY Inc.
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited without the written consent of the copyright owner.
*
*/
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <heyos_def.h>

typedef void* heyos_void_p;
/* os entry */
void heyos_init(void);
void heyos_start(void);
void heyos_kernel_init(void);
/* platform related */
bool heyos_is_inside_isr(void);
uint32_t heyos_irq_disable(void);
void heyos_irq_enable(uint32_t irq);
void heyos_enter_critical(void);
void heyos_exit_critical(void);
uint32_t heyos_tick_get(void);
uint32_t heyos_get_utc_now(void);


#if 0
/* clock time */
uint32_t heyos_clock_time(void);
uint32_t heyos_calc_ms(uint32_t clock);

/* memory */
void* heyos_malloc(const uint32_t size);
void heyos_free(const void* p_data);
void* heyos_sram_malloc(const uint32_t size);
void heyos_sram_free(const void* p_data);
void* heyos_realloc(const void* p_data, const uint32_t size);
void* heyos_calloc(uint32_t count, uint32_t size);
uint32_t heyos_heap_get_free_bytes(void);
void heyos_memcpy(void* dst, const void* src, uint32_t size);
int heyos_memcmp(const void* src1, const void* src2, uint32_t size);
void heyos_memset(void* dst, const uint8_t value, uint32_t size);
void heyos_heap_free_bytes(int* sram_free_size, int* psram_free_size);

/* string */
int heyos_strncmp(const char *cs, const char *ct, int count);
int heyos_strcmp(const char *cs, const char *ct);
int heyos_strlen(const char *src);
int heyos_strnlen(const char *s, int maxlen);
char* heyos_strncpy(char *dst, const char *src, int maxlen);
#endif

/* thread */
typedef heyos_void_p heyos_thread_t;
typedef void (*heyos_thread_entry_t)(void* p_context);
typedef struct
{
  void* para;
  const char* name;
  const heyos_thread_entry_t entry;
  const uint32_t stack_size;
  const uint32_t prio;
}heyos_thread_ctx;

heyos_thread_t heyos_thread_create(heyos_thread_ctx ctx);
heyos_ret_t heyos_thread_delete(const heyos_thread_t thread);
heyos_thread_t heyos_thread_self(void);
heyos_ret_t heyos_thread_suspend(const heyos_thread_t thread);
heyos_ret_t heyos_thread_resume(const heyos_thread_t thread);
heyos_thread_t heyos_find_thread(char* name);

/* delay */
heyos_ret_t heyos_delay_ms(uint32_t ms);
heyos_ret_t heyos_delay_tick(uint32_t tick);
uint32_t heyos_tick_from_ms(uint32_t ms);
uint32_t heyos_tick_per_second(void);


#if 0
/* timer */
typedef heyos_base_t heyos_timer_t;
typedef void (*heyos_timer_handler_t)(void* p_context);
heyos_timer_t heyos_timer_create(const char* name, heyos_timer_handler_t handler, void* para, uint32_t timeout_ms, bool is_repeat);
heyos_ret_t heyos_timer_delete(const heyos_timer_t timer);
heyos_ret_t heyos_timer_start(const heyos_timer_t timer);
heyos_ret_t heyos_timer_stop(const heyos_timer_t timer);
heyos_ret_t heyos_timer_set_timeout_ms(const heyos_timer_t timer, const uint32_t timeout_ms);
heyos_ret_t heyos_timer_set_timeout_tick(const heyos_timer_t timer, const uint32_t timeout_tick);
heyos_ret_t heyos_timer_set_handler(const heyos_timer_t timer, heyos_timer_handler_t handler);
heyos_ret_t heyos_timer_set_para(const heyos_timer_t timer, void* para);
bool heyos_timer_is_running(const heyos_timer_t timer); // invalid -> false

/* semaphore */
typedef heyos_base_t heyos_sem_t;
heyos_sem_t heyos_sem_create(const char* name, const uint32_t init_value);
heyos_ret_t heyos_sem_delete(const heyos_sem_t sem);
heyos_ret_t heyos_sem_wait(const heyos_sem_t sem);    // forever
heyos_ret_t heyos_sem_wait_timeout_ms(const heyos_sem_t sem, const uint32_t ms);
heyos_ret_t heyos_sem_post(const heyos_sem_t sem);

/* mutex */
typedef heyos_base_t heyos_mutex_t;
heyos_mutex_t heyos_mutex_create(const char* name); //default released
heyos_ret_t heyos_mutex_delete(const heyos_mutex_t mutex);
heyos_ret_t heyos_mutex_take(const heyos_mutex_t mutex);
heyos_ret_t heyos_mutex_take_timeout_ms(const heyos_mutex_t mutex, const uint32_t ms);
heyos_ret_t heyos_mutex_release(const heyos_mutex_t mutex);

/* queue */
typedef heyos_base_t heyos_queue_t;
heyos_queue_t heyos_queue_create(const char* name, const uint32_t item_size, const uint32_t item_max_cnt);
heyos_ret_t heyos_queue_delete(const heyos_queue_t queue);
heyos_ret_t heyos_queue_recv(const heyos_queue_t queue, void* p_data);
heyos_ret_t heyos_queue_recv_timeout_ms(const heyos_queue_t queue, void* p_data, uint32_t ms);
heyos_ret_t heyos_queue_send(const heyos_queue_t queue, const void* p_data);

/* event */
typedef heyos_base_t heyos_event_t;
typedef enum{
    HEYOS_EVENT_FLAG_AND = (1<<0),
    HEYOS_EVENT_FLAG_OR = (1<<1),
    HEYOS_EVENT_FLAG_CLEAR = (1<<2),
}heyos_event_flag_t;

heyos_event_t heyos_event_create(const char* name);
heyos_ret_t heyos_event_delete(const heyos_event_t event);
heyos_ret_t heyos_event_send_bits(const heyos_event_t event, uint32_t bits);
heyos_ret_t heyos_event_clear_bits(const heyos_event_t event, uint32_t bits);
heyos_ret_t heyos_event_recv(const heyos_event_t event, uint32_t bits, uint32_t flag, uint32_t* recved_bits);
heyos_ret_t heyos_event_recv_timeout_ms(const heyos_event_t event, uint32_t bits, uint32_t flag, uint32_t* recved_bits, uint32_t timeout_ms);

/* mailbox */
typedef heyos_base_t heyos_mailbox_t;
heyos_mailbox_t heyos_mailbox_create(const char* name, uint32_t size);
heyos_ret_t heyos_mailbox_delete(const heyos_mailbox_t mailbox);
heyos_ret_t heyos_mailbox_send(const heyos_mailbox_t mailbox, const uint32_t value);
heyos_ret_t heyos_mailbox_recv(const heyos_mailbox_t mailbox, uint32_t* p_value);

/* rwlock */
typedef struct _rw_lock _rw_lock_t;
typedef heyos_ret_t(*rw_lock_cb_t)(struct _rw_lock* lock, uint32_t wait_time);
struct _rw_lock{
    heyos_mutex_t  reader_mutex;   /** Mutex for reader mutual exclusion */
    heyos_sem_t    rw_lock;        /** Lock which when held by reader, writer cannot enter critical section */
    rw_lock_cb_t   reader_cb;      /** Function being called when first reader gets the lock */
    uint32_t       reader_count;   /** Counter to maintain number of readers in critical section */
};
typedef _rw_lock_t* heyos_rw_lock_t;

heyos_rw_lock_t heyos_rwlock_create(const char* name);
heyos_rw_lock_t heyos_rwlock_create_with_cb(const char* name, rw_lock_cb_t r_fn);
heyos_ret_t heyos_rwlock_read_lock(heyos_rw_lock_t lock, uint32_t wait_time);
heyos_ret_t heyos_rwlock_read_unlock(heyos_rw_lock_t lock);
heyos_ret_t heyos_rwlock_write_lock(heyos_rw_lock_t lock, uint32_t wait_time);
heyos_ret_t heyos_rwlock_write_unlock(heyos_rw_lock_t lock);
heyos_ret_t heyos_rw_lock_delete(heyos_rw_lock_t lock);
#endif


/* assert */
// compile-time assert
#define HEYOS_STATIC_ASSERT(x, msg) \
  enum { static_assertion_failed_ ## msg = 1 / (!!(x)) }

void heyos_assert_handler(const char *ex, const char *func, uint32_t line);
// runtime assert
#define HEYOS_ASSERT(x)   if (!(x))  {heyos_assert_handler(#x, __FUNCTION__, __LINE__);}


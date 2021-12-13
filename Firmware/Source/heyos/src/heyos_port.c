#include <heyos_port.h>
#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include "main.h"

void heyos_kernel_init(void)
{
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
    /* show version */
    rt_show_version();

    /* init timer system */
    rt_system_timer_init();

    /* init scheduler system */
    rt_system_scheduler_init();

    /* init timer thread */
    rt_system_timer_thread_init();

    /* init idle thread */
    rt_thread_idle_init();
}

void heyos_init(void)
{
    heyos_board_init();
    heyos_kernel_init();
	heyos_application_init();
}

void heyos_start(void)
{
	int irq = 0xFFFFFFFF;
	
    heyos_irq_enable(irq);
    rt_system_scheduler_start();
}

#if defined(__CLANG_ARM)
/* platform related */
/**
  \brief   Get IPSR Register
  \details Returns the content of the IPSR Register.
  \return               IPSR Register value
 */
static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
#endif

bool heyos_is_inside_isr(void)
{
#if defined(__CLANG_ARM)
    uint32_t ulCurrentInterrupt = __get_IPSR();
    bool xReturn;

    if( ulCurrentInterrupt == 0 ) {
        xReturn = false;
    } else {
        xReturn = true;
    }

    return xReturn;
#else

    uint32_t ulCurrentInterrupt;
    bool xReturn;

    /* Obtain the number of the currently executing interrupt. */
    __asm {
        mrs ulCurrentInterrupt, ipsr
    }

    if( ulCurrentInterrupt == 0 ) {
        xReturn = false;
    } else {
        xReturn = true;
    }

    return xReturn;
#endif
}

uint32_t heyos_irq_disable(void)
{
    return rt_hw_interrupt_disable();
}
void heyos_irq_enable(uint32_t irq)
{
    rt_hw_interrupt_enable(irq);
}

void heyos_enter_critical(void)
{
    rt_enter_critical();
}

void heyos_exit_critical(void)
{
    rt_exit_critical();
}

uint32_t heyos_tick_get(void)
{
    return rt_tick_get();
}

uint32_t heyos_tick_per_second(void)
{
    return RT_TICK_PER_SECOND;
}



#if 0
/* clock time */

/**
 * @brief   API to get current CPU time
 *
 * @param   None
 *
 * @return  The current cpu time in 1/32768 s
 */
uint32_t heyos_clock_time(void)
{
    return heyos_hal_clock_time();
}


/**
 * @brief   API to transfer the clock time to millsecond.
 *
 * @param   clock - The sepcified clock tick
 *
 * @return  The millsecond
 */
uint32_t heyos_calc_ms(uint32_t clock)
{
    return heyos_hal_calc_ms(clock);
}

/* memory */
HEYOS_WEAK void* heyos_sram_malloc(const uint32_t size)
{
    return rt_malloc(size);
}
HEYOS_WEAK void heyos_sram_free(const void* p_data)
{
    rt_free((void*)p_data);
}
HEYOS_WEAK void* heyos_malloc(const uint32_t size)
{
    return rt_malloc(size);
}
HEYOS_WEAK void heyos_free(const void* p_data)
{
    rt_free((void*)p_data);
}
HEYOS_WEAK void* heyos_realloc(const void* p_data, const uint32_t size)
{
    return rt_realloc((void*)p_data, size);
}
HEYOS_WEAK void* heyos_calloc(uint32_t count, uint32_t size)
{
    size *= count;
    void* p = heyos_malloc(size);
    if(p!= NULL){
        heyos_memset(p, 0, size);
    }
    return p;
}

uint32_t heyos_heap_get_free_bytes(void)
{
    uint32_t total_size, free_size, max_used;
    rt_memory_info(&total_size,
        &free_size,
        &max_used);
    return (total_size-free_size);
}
HEYOS_WEAK void heyos_memcpy(void* dst, const void* src, uint32_t size)
{
    rt_memcpy(dst, src, size);
}
int heyos_memcmp(const void* src1, const void* src2, uint32_t size)
{
    return rt_memcmp(src1, src2, size);
}
void heyos_memset(void* dst, const uint8_t value, uint32_t size)
{
    rt_memset(dst, value, size);
}

/* string */
int heyos_strncmp(const char *cs, const char *ct, int count)
{
    return (int)rt_strncmp(cs, ct, count);
}
int heyos_strcmp(const char *cs, const char *ct)
{
    return (int)rt_strcmp(cs, ct);
}
int heyos_strlen(const char *src)
{
    return (int)rt_strlen(src);
}
int heyos_strnlen(const char *s, int maxlen)
{
    return (int)rt_strnlen(s, maxlen);
}
char* heyos_strncpy(char *dst, const char *src, int maxlen)
{
    dst = rt_strncpy(dst, src, maxlen);
    dst[maxlen - 1] = '\0';
    return dst;
}
#endif

/* thread */
heyos_thread_t heyos_thread_create(heyos_thread_ctx ctx)
{
    rt_thread_t thread = NULL;
    thread = rt_thread_create(ctx.name, ctx.entry, ctx.para, ctx.stack_size, ctx.prio, 10);
    if(thread != NULL){
        rt_thread_startup(thread);
    }
    return thread;
}

heyos_ret_t heyos_thread_delete(const heyos_thread_t thread)
{
    heyos_ret_t ret = 0;
    rt_thread_t t = (rt_thread_t)thread;
    rt_err_t rt_ret = rt_thread_delete(t);
    ret = (rt_ret == RT_EOK)? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_OS, HEYOS_ERR_THREAD_CREATION);
    return ret;
}

heyos_thread_t heyos_thread_self(void)
{
    return rt_thread_self();
}

heyos_ret_t heyos_thread_suspend(const heyos_thread_t thread)
{
    heyos_ret_t ret = 0;
    rt_thread_t t = (rt_thread_t)thread;
    rt_err_t rt_ret = rt_thread_suspend(t);
    ret = (rt_ret == RT_EOK)? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_OS, HEYOS_ERR_THREAD_CREATION);
    return ret;
}

heyos_ret_t heyos_thread_resume(const heyos_thread_t thread)
{
    heyos_ret_t ret = 0;
    rt_thread_t t = (rt_thread_t)thread;
    rt_err_t rt_ret = rt_thread_resume(t);
    ret = (rt_ret == RT_EOK)? HEYOS_SUCC : HEYOS_SET_STS_VAL(HEYOS_CID_OS, HEYOS_ERR_THREAD_CREATION);
    return ret;
}

heyos_thread_t heyos_find_thread(char* name)
{
    return rt_thread_find(name);
}

/* delay */
heyos_ret_t heyos_delay_ms(uint32_t ms)
{
    return rt_thread_mdelay(ms);
}
heyos_ret_t heyos_delay_tick(uint32_t tick)
{
    return rt_thread_delay(tick);
}
uint32_t heyos_tick_from_ms(uint32_t ms)
{
    if(ms == 0){
        return 0;
    } else {
        return rt_tick_from_millisecond(ms);
    }
}

void heyos_assert_handler(const char *ex, const char *func, uint32_t line)
{
    rt_assert_handler(ex, func, line);
}
#if 0
/* timer */
heyos_timer_t heyos_timer_create(const char* name, heyos_timer_handler_t handler, void* para, uint32_t timeout_ms, bool is_repeat)
{
    rt_timer_t timer = NULL;
    rt_flag_t flag = is_repeat?RT_TIMER_FLAG_PERIODIC:RT_TIMER_FLAG_ONE_SHOT;
    flag |= RT_TIMER_FLAG_SOFT_TIMER;
    HEYOS_ASSERT(timeout_ms < (1000 * 60 * 60 * 24 * 31)); // 1 month, if assert here ,you must be a bug
    timer = rt_timer_create(name, handler, para, heyos_tick_from_ms(timeout_ms), flag);
    return timer;
}
heyos_ret_t heyos_timer_delete(const heyos_timer_t timer)
{
    rt_timer_t t = (rt_timer_t)timer;
    if(t == NULL){
        return HEYOS_ERR_INVALID_PARA;
    }
    rt_err_t rt_ret = rt_timer_delete(t);
    return rt_ret;
}
heyos_ret_t heyos_timer_start(const heyos_timer_t timer)
{
    rt_timer_t t = (rt_timer_t)timer;
    if(t == NULL){
        return HEYOS_ERR_INVALID_PARA;
    }
    rt_err_t rt_ret = rt_timer_start(t);
    return rt_ret;
}
heyos_ret_t heyos_timer_stop(const heyos_timer_t timer)
{
    rt_timer_t t = (rt_timer_t)timer;
    if(t == NULL){
        return HEYOS_ERR_INVALID_PARA;
    }
    rt_err_t rt_ret = rt_timer_stop(t);
    return rt_ret;
}
heyos_ret_t heyos_timer_set_timeout_ms(const heyos_timer_t timer, const uint32_t timeout_ms)
{
    //    rt_timer_t t = (rt_timer_t)timer;
    HEYOS_ASSERT(timeout_ms < (1000 * 60 * 60 * 24 * 31)); // 1 month, if assert here ,you must be a bug
    uint32_t tick = heyos_tick_from_ms(timeout_ms);
    return heyos_timer_set_timeout_tick(timer, tick);
}

heyos_ret_t heyos_timer_set_timeout_tick(const heyos_timer_t timer, const uint32_t timeout_tick)
{
    rt_timer_t t = (rt_timer_t)timer;
    if(t == NULL){
        return HEYOS_ERR_INVALID_PARA;
    }
    rt_err_t rt_ret = 0;
    if(timeout_tick != t->init_tick){
        rt_tick_t tick = timeout_tick;
        heyos_enter_critical();
        rt_ret = rt_timer_control(t, RT_TIMER_CTRL_SET_TIME, (void*)&tick);
        heyos_exit_critical();
    }
    return rt_ret;
}
heyos_ret_t heyos_timer_set_handler(const heyos_timer_t timer, heyos_timer_handler_t handler)
{
    rt_timer_t t = (rt_timer_t)timer;
    rt_err_t rt_ret = 0;
    if(handler != t->timeout_func){
        heyos_enter_critical();
        t->timeout_func = handler;
        heyos_exit_critical();
    }
    return rt_ret;
}
heyos_ret_t heyos_timer_set_para(const heyos_timer_t timer, void* para)
{
    rt_timer_t t = (rt_timer_t)timer;
    rt_err_t rt_ret = 0;
    if(para != t->parameter){
        heyos_enter_critical();
        t->parameter = para;
        heyos_exit_critical();
    }
    return rt_ret;
}
bool heyos_timer_is_running(const heyos_timer_t timer)
{
    rt_timer_t t = (rt_timer_t)timer;
    bool is_running = false;
    if(t == NULL){
        is_running = false;
        goto exit;
    }
    if(rt_object_get_type(&t->parent) != RT_Object_Class_Timer){
        is_running = false;
        goto exit;
    }
    is_running = ((t->parent.flag & RT_TIMER_FLAG_ACTIVATED) != 0);
exit:
    return is_running;
}

/* semaphore */
heyos_sem_t heyos_sem_create(const char* name, const uint32_t init_value)
{
    rt_sem_t sem = rt_sem_create(name, init_value, RT_IPC_FLAG_FIFO);
    return sem;
}
heyos_ret_t heyos_sem_delete(const heyos_sem_t sem)
{
    rt_err_t rt_ret;
    rt_ret = rt_sem_delete(sem);
    return rt_ret;
}
heyos_ret_t heyos_sem_wait(const heyos_sem_t sem)
{
    rt_err_t rt_ret;
    heyos_ret_t ret = HEYOS_SUCC;
    rt_ret = rt_sem_take(sem, RT_WAITING_FOREVER);
    if(rt_ret == -RT_ETIMEOUT){
        ret = HEYOS_ERR_TIMEOUT;
    } else if(ret != RT_EOK){
        ret = HEYOS_ERR_INVALID_STATE;
    }
    return ret;
}
heyos_ret_t heyos_sem_wait_timeout_ms(const heyos_sem_t sem, const uint32_t ms)
{
    rt_err_t rt_ret;
    heyos_ret_t ret = HEYOS_SUCC;
    uint32_t tick = heyos_tick_from_ms(ms);
    rt_ret = rt_sem_take(sem, tick);
    if(rt_ret == -RT_ETIMEOUT){
        ret = HEYOS_ERR_TIMEOUT;
    } else if(ret != RT_EOK){
        ret = HEYOS_ERR_INVALID_STATE;
    }
    return ret;
}
heyos_ret_t heyos_sem_post(const heyos_sem_t sem)
{
    rt_err_t rt_ret;
    heyos_ret_t ret = HEYOS_SUCC;
    if(sem == NULL){
        return HEYOS_ERR_INVALID_PARA;
    }
    rt_ret = rt_sem_release(sem);
    if(rt_ret != RT_EOK){
        ret = HEYOS_ERR_INVALID_STATE;
    }
    return ret;
}

/* mutex */
heyos_mutex_t heyos_mutex_create(const char* name)
{
    rt_mutex_t mutex = rt_mutex_create(name, RT_IPC_FLAG_FIFO);
    return mutex;
}
heyos_ret_t heyos_mutex_delete(const heyos_mutex_t mutex)
{
    heyos_ret_t ret = 0;
    rt_mutex_delete(mutex);
    return ret;
}
heyos_ret_t heyos_mutex_take(const heyos_mutex_t mutex)
{
    heyos_ret_t ret = 0;
    rt_mutex_take(mutex, RT_WAITING_FOREVER);
    return ret;
}
heyos_ret_t heyos_mutex_take_timeout_ms(const heyos_mutex_t mutex, const uint32_t ms)
{
    heyos_ret_t ret = 0;
    uint32_t tick = heyos_tick_from_ms(ms);
    ret = rt_mutex_take(mutex, tick);
    return ret;
}
heyos_ret_t heyos_mutex_release(const heyos_mutex_t mutex)
{
    heyos_ret_t ret = 0;
    rt_mutex_release(mutex);
    return ret;
}

/* queue */
heyos_queue_t heyos_queue_create(const char* name, const uint32_t item_size, const uint32_t item_max_cnt)
{
    rt_mq_t q = rt_mq_create(name, item_size, item_max_cnt, RT_IPC_FLAG_FIFO);
    return (heyos_queue_t)q;
}
heyos_ret_t heyos_queue_delete(const heyos_queue_t queue)
{
    rt_err_t rt_ret = RT_EOK;
    rt_ret = rt_mq_delete(queue);
    return rt_ret;
}
heyos_ret_t heyos_queue_recv(const heyos_queue_t queue, void* p_data)
{
    heyos_ret_t ret = HEYOS_SUCC;
    rt_err_t rt_ret = RT_EOK;
    rt_mq_t q = (rt_mq_t)queue;
    rt_ret = rt_mq_recv(q, p_data,  q->msg_size, RT_WAITING_FOREVER);
    if(rt_ret == -RT_ETIMEOUT){
        ret = HEYOS_ERR_TIMEOUT;
    } else if(ret != RT_EOK){
        ret = HEYOS_ERR_INVALID_STATE;
    }
    return ret;
}
heyos_ret_t heyos_queue_recv_timeout_ms(const heyos_queue_t queue, void* p_data, uint32_t ms)
{
    uint32_t tick = heyos_tick_from_ms(ms);
    heyos_ret_t ret = HEYOS_SUCC;
    rt_err_t rt_ret = RT_EOK;
    rt_mq_t q = (rt_mq_t)queue;
    rt_ret = rt_mq_recv(q, p_data, q->msg_size, tick);
    if(rt_ret == -RT_ETIMEOUT){
        ret = HEYOS_ERR_TIMEOUT;
    } else if(ret != RT_EOK){
        ret = HEYOS_ERR_INVALID_STATE;
    }
    return ret;
}
heyos_ret_t heyos_queue_send(const heyos_queue_t queue, const void* p_data)
{
    heyos_ret_t ret = HEYOS_SUCC;
    rt_err_t rt_ret = RT_EOK;
    rt_mq_t q = (rt_mq_t)queue;
    rt_ret = rt_mq_send(q, (void*)p_data, q->msg_size);
    if(rt_ret == -RT_EFULL){
        ret = HEYOS_ERR_QUEUE_FULL;
    } else if(ret != RT_EOK){
        ret = HEYOS_ERR_INVALID_STATE;
    }
    return ret;
}

/* event */
heyos_event_t heyos_event_create(const char* name)
{
    return rt_event_create(name, RT_IPC_FLAG_FIFO);
}
heyos_ret_t heyos_event_delete(const heyos_event_t event)
{
    return rt_event_delete(event);
}
heyos_ret_t heyos_event_send_bits(const heyos_event_t event, uint32_t bits)
{
    return rt_event_send(event, bits);
}

heyos_ret_t heyos_event_clear_bits(const heyos_event_t event, uint32_t bits)
{
    heyos_ret_t ret = HEYOS_SUCC;
    rt_err_t rt_ret;
    rt_ret = rt_event_recv(event, bits, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, 0, NULL);
    if(rt_ret != RT_EOK){
        ret = HEYOS_ERR_INVALID_STATE;
    }
    return ret;
}
static heyos_ret_t heyos_event_recv_timeout_tick(const heyos_event_t event, uint32_t bits, uint32_t flag, uint32_t* recved_bits, uint32_t timeout_tick)
{
    heyos_ret_t status = HEYOS_SUCC;
    rt_err_t rt_ret = RT_EOK;
    rt_ret = rt_event_recv((rt_event_t)event, bits, flag, timeout_tick, recved_bits);
    if(rt_ret == -RT_ETIMEOUT){
        status = HEYOS_ERR_TIMEOUT;
    } else if(rt_ret != RT_EOK){
        status = HEYOS_ERR_INVALID_STATE;
    }
    return status;
}
heyos_ret_t heyos_event_recv(const heyos_event_t event, uint32_t bits, uint32_t flag, uint32_t* recved_bits)
{
    //    heyos_ret_t status = 0;
    //    rt_err_t rt_ret = RT_EOK;

    return heyos_event_recv_timeout_tick(event, bits, flag, recved_bits, RT_WAITING_FOREVER);
}
heyos_ret_t heyos_event_recv_timeout_ms(const heyos_event_t event, uint32_t bits, uint32_t flag, uint32_t* recved_bits, uint32_t timeout_ms)
{
    //    heyos_ret_t status = HEYOS_SUCC;
    //    rt_err_t rt_ret = RT_EOK;
    uint32_t tick = heyos_tick_from_ms(timeout_ms);
    return heyos_event_recv_timeout_tick(event, bits, flag, recved_bits, tick);
}

/* mailbox */
heyos_mailbox_t heyos_mailbox_create(const char* name, uint32_t size);
heyos_ret_t heyos_mailbox_delete(const heyos_mailbox_t mailbox);
heyos_ret_t heyos_mailbox_send(const heyos_mailbox_t mailbox, const uint32_t value);
heyos_ret_t heyos_mailbox_recv(const heyos_mailbox_t mailbox, uint32_t* p_value);


/* rwlock */
heyos_rw_lock_t heyos_rwlock_create(const char* name)
{
    return heyos_rwlock_create_with_cb(name, NULL);
}

heyos_rw_lock_t heyos_rwlock_create_with_cb(const char* name, rw_lock_cb_t r_fn)
{
    char name_str[HEYOS_KERNEL_OBJ_NAME_LEN] = {0};

    if (rt_strlen(name)>(HEYOS_KERNEL_OBJ_NAME_LEN-2)) {
        return NULL;
    }

    heyos_rw_lock_t p_new_rwlock = (heyos_rw_lock_t)heyos_malloc(sizeof(_rw_lock_t));
    if (!p_new_rwlock) {
        return NULL;
    }
    heyos_memset(p_new_rwlock, 0 ,sizeof(_rw_lock_t));

    rt_snprintf(name_str, HEYOS_KERNEL_OBJ_NAME_LEN, "%s_m", name);
    p_new_rwlock->reader_mutex = heyos_mutex_create(name_str);
    if (!p_new_rwlock->reader_mutex) {
        goto error2;
    }

    heyos_memset(name_str, 0, HEYOS_KERNEL_OBJ_NAME_LEN);
    rt_snprintf(name_str, HEYOS_KERNEL_OBJ_NAME_LEN, "%s_s", name);
    p_new_rwlock->rw_lock = heyos_sem_create(name_str, 1);
    if (!p_new_rwlock->rw_lock) {
        goto error1;
    }

    p_new_rwlock->reader_count = 0;
    p_new_rwlock->reader_cb = r_fn;
    return p_new_rwlock;

error1:
    heyos_mutex_delete(p_new_rwlock->reader_mutex);
error2:
    heyos_free((void*)p_new_rwlock);
    return NULL;
}

heyos_ret_t heyos_rwlock_read_lock(heyos_rw_lock_t lock, uint32_t wait_time)
{
    heyos_ret_t ret = HEYOS_SUCC;
    ret = heyos_mutex_take(lock->reader_mutex);
    if (ret != HEYOS_SUCC) {
        return ret;
    }

    lock->reader_count++;
    if (lock->reader_count == 1) {
        if (lock->reader_cb) {
            ret = lock->reader_cb(lock, wait_time);
            if (ret != HEYOS_SUCC) {
                lock->reader_count--;
                goto exit;
            }
        } else {
            /* If it is the first reader and
             * if writer is not active, reader will get access.
             * else reader will block.
             */
            if (HEYOS_WAIT_FOREVER == wait_time) {
                ret = heyos_sem_wait(lock->rw_lock);
            } else {
                ret = heyos_sem_wait_timeout_ms(lock->rw_lock, wait_time);
            }
            if (ret != HEYOS_SUCC) {
                lock->reader_count--;
                goto exit;
            }

        }
    }

exit:
    heyos_mutex_release(lock->reader_mutex);
    return ret;
}


heyos_ret_t heyos_rwlock_read_unlock(heyos_rw_lock_t lock)
{
    heyos_ret_t ret = HEYOS_SUCC;
    if (lock->reader_count == 0) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_OS, HEYOS_ERR_INVALID_PARA);
    }

    ret = heyos_mutex_take(lock->reader_mutex);
    if (ret != HEYOS_SUCC) {
        return ret;
    }

    lock->reader_count--;
    if (lock->reader_count == 0) {
        /* This is last reader so give a chance to writer now */
        ret = heyos_sem_post(lock->rw_lock);
    }

    heyos_mutex_release(lock->reader_mutex);
    return ret;
}

heyos_ret_t heyos_rwlock_write_lock(heyos_rw_lock_t lock, uint32_t wait_time)
{
    heyos_ret_t ret = HEYOS_SUCC;
    ret = heyos_mutex_take(lock->reader_mutex);
    if (ret != HEYOS_SUCC) {
        return ret;
    }

    if (HEYOS_WAIT_FOREVER == wait_time) {
        ret = heyos_sem_wait(lock->rw_lock);
    } else {
        ret = heyos_sem_wait_timeout_ms(lock->rw_lock, wait_time);
    }

    heyos_mutex_release(lock->reader_mutex);
    return ret;
}


heyos_ret_t heyos_rwlock_write_unlock(heyos_rw_lock_t lock)
{
    return heyos_sem_post(lock->rw_lock);
}

heyos_ret_t heyos_rw_lock_delete(heyos_rw_lock_t lock)
{
    if (lock == NULL) {
        return HEYOS_SET_STS_VAL(HEYOS_CID_OS, HEYOS_ERR_INVALID_PARA);
    }

    lock->reader_cb = NULL;
    heyos_sem_delete(lock->rw_lock);
    heyos_mutex_delete(lock->reader_mutex);
    lock->reader_count = 0;

    heyos_free((void*)lock);
    return HEYOS_SUCC;
}
#endif


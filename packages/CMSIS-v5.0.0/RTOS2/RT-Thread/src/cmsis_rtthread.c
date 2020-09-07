/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-04-12     misonyo      the first version
 * 2019-04-04     misonyo      fix some bugs
 */

#include <cmsis_os2.h>
#include "cmsis_rtthread.h"
#include "board.h"
#include "os_tick.h"
#include <rthw.h>

/// Kernel Information
#define API_VERSION         20010002   ///< API version (2.1.2)
///< RT-Thread Kernel version
#define RT_KERNEL_VERSION            (((rt_uint32_t)RT_VERSION * 10000000UL) | \
                                   ((rt_uint32_t)RT_SUBVERSION *    10000UL) | \
                                   ((rt_uint32_t)RT_REVISION *        1UL))
#define KERNEL_Id     "RT-Thread"  ///< Kernel identification string

#define DEFAULT_STACK_SIZE 1024
#define DEFAULT_TICK 5
#define WAITING_THREAD_FLAGS 0x08
#define MALLOC_CB 0x10
#define MALLOC_STACK 0x04
//#define MALLOC_MEM 0x02

extern void rt_thread_exit(void);

static struct rt_event cmsisi_rtt_event;

static osKernelState_t kernel_state = osKernelInactive;
volatile static rt_uint32_t clear_flag = 0;

static void thread_cleanup(rt_thread_t thread)
{
    thread_cb_t *thread_cb;
    thread_cb = (thread_cb_t *)(thread->user_data);

    /* clear cleanup function */
    thread->cleanup = RT_NULL;
    if (thread_cb->flags & osThreadJoinable)
    {
        rt_sem_release(thread_cb->joinable_sem);
    }
    else
    {
        if (thread_cb->flags & MALLOC_STACK)
            rt_free(thread_cb->thread.stack_addr);

        if (thread_cb->flags & MALLOC_CB)
            rt_free(thread_cb);
    }
}

/// Initialize the RTOS Kernel.
/// \return status code that indicates the execution status of the function.
osStatus_t osKernelInitialize(void)
{
    kernel_state = osKernelReady;
    rt_event_init(&cmsisi_rtt_event, "os_event", RT_IPC_FLAG_PRIO);
	
	return osOK;
}

/// Get the current RTOS Kernel state.
/// \return current RTOS Kernel state.
osKernelState_t osKernelGetState(void)
{
    return kernel_state;
}

/// Start the RTOS Kernel scheduler.
/// \return status code that indicates the execution status of the function.
osStatus_t osKernelStart(void)
{
    osStatus_t state;

    if (osKernelReady == kernel_state)
    {
        kernel_state = osKernelRunning;

        state = osOK;
    }
    else
    {
        state = osError;
    }

    return state;
}

/// Get the RTOS kernel tick frequency.
/// \return frequency of the kernel tick.
uint32_t osKernelGetTickFreq(void)
{
    return RT_TICK_PER_SECOND;
}


//  ==== Thread Management Functions ====

/// Create a thread and add it to Active Threads.
/// \param[in]     func          thread function.
/// \param[in]     argument      pointer that is passed to the thread function as start argument.
/// \param[in]     attr          thread attributes; NULL: default values.
/// \return thread ID for reference by other functions or NULL in case of error.
osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr)
{
    void *stack;
    rt_uint8_t rtt_prio;
    rt_uint32_t stack_size;
    thread_cb_t *thread_cb;
    char name[RT_NAME_MAX];
    static rt_uint16_t thread_number = 1U;
    /* Check parameters */
    if (RT_NULL == func)
    {
        return RT_NULL;
    }

    if ((RT_NULL == attr) || (RT_NULL == attr->cb_mem))
    {
        thread_cb = rt_malloc(sizeof(thread_cb_t));
        if (RT_NULL == thread_cb)
            return RT_NULL;
        rt_memset(thread_cb, 0, sizeof(thread_cb_t));
        thread_cb->flags |= MALLOC_CB;
    }
    else
    {
        if (attr->cb_size >= sizeof(thread_cb_t))
        {
            thread_cb = attr->cb_mem;
            thread_cb->flags = 0;
        }
        else
            return RT_NULL;
    }

    if ((RT_NULL == attr) || (osPriorityNone == attr->priority))
    {
        thread_cb->prio = osPriorityNormal;
    }
    else
    {
        if ((attr->priority < osPriorityIdle) || (attr->priority > osPriorityISR))
            return RT_NULL;

        thread_cb->prio = attr->priority;
    }
    if ((RT_NULL == attr) || (0U == attr->stack_size))
        stack_size = DEFAULT_STACK_SIZE;
    else
        stack_size = attr->stack_size;

    if ((RT_NULL == attr) || (RT_NULL == attr->stack_mem))
    {
        stack = rt_malloc(stack_size);
        if (RT_NULL == stack)
        {
            if (thread_cb->flags & MALLOC_CB)
                rt_free(thread_cb);
            return RT_NULL;
        }
        thread_cb->flags |= MALLOC_STACK;
    }
    else
    {
        stack = (void *)(attr->stack_mem);
    }

    if ((RT_NULL != attr) && (0 != attr->attr_bits))
        thread_cb->flags |= attr->attr_bits;

    if ((RT_NULL == attr) || (RT_NULL == attr->name))
    {
        rt_snprintf(name, sizeof(name), "th%02d", thread_number++);
        rtt_prio = 0x05; 
    }
    else
    {
        rt_snprintf(name, sizeof(name), "%s", attr->name);
        rtt_prio = 0x03;
    }

    rt_thread_init(&(thread_cb->thread), name, func, argument, stack, stack_size, rtt_prio, DEFAULT_TICK);

    if (thread_cb->flags & osThreadJoinable)
    {
        thread_cb->joinable_sem = rt_sem_create(name, 0, RT_IPC_FLAG_FIFO);
        if (RT_NULL == thread_cb->joinable_sem)
        {
            if (thread_cb->flags & MALLOC_CB)
                rt_free(thread_cb);
            if (thread_cb->flags & MALLOC_STACK)
                rt_free(stack);

            return RT_NULL;
        }
    }
    else
        thread_cb->joinable_sem = RT_NULL;

    thread_cb->thread.cleanup = thread_cleanup;
    thread_cb->thread.user_data = (rt_uint32_t)thread_cb;

    rt_thread_startup(&(thread_cb->thread));

    return thread_cb;
}

/// Wait for specified thread to terminate.
/// \param[in]     thread_id     thread ID obtained by \ref osThreadNew or \ref osThreadGetId.
/// \return status code that indicates the execution status of the function.
osStatus_t osThreadJoin(osThreadId_t thread_id)
{
    rt_err_t result;
    thread_cb_t *thread_cb = (thread_cb_t *)thread_id;

    /* Check parameters */
    if (RT_NULL == thread_cb)
    {
        return osErrorParameter;
    }

    if (((&thread_cb->thread) == rt_thread_self()) ||
            (0 == (thread_cb->flags & osThreadJoinable)))
    {
        /* join self or join a detached thread*/
        return osErrorResource;
    }

    result = rt_sem_take(thread_cb->joinable_sem, RT_WAITING_FOREVER);
    if (RT_EOK == result)
    {
        /* release resource */
        if (thread_cb->flags & osThreadJoinable)
            rt_sem_delete(thread_cb->joinable_sem);

        if (thread_cb->flags & MALLOC_STACK)
            rt_free(thread_cb->thread.stack_addr);

        if (thread_cb->flags & MALLOC_CB)
            rt_free(thread_cb);
    }
    else
        return osError;

    return osOK;
}

/// Terminate execution of a thread.
/// \param[in]     thread_id     thread ID obtained by \ref osThreadNew or \ref osThreadGetId.
/// \return status code that indicates the execution status of the function.
osStatus_t osThreadTerminate(osThreadId_t thread_id)
{
    thread_cb_t *thread_cb;

    thread_cb = (thread_cb_t *)(rt_thread_self()->user_data);

    /* Check parameters */
    if (RT_NULL == thread_cb)
    {
        return osErrorParameter;
    }

    rt_thread_detach(&(thread_cb->thread));
    rt_schedule();

    return osOK;
}

//  ==== Thread Flags Functions ====

/// Set the specified Thread Flags of a thread.
/// \param[in]     thread_id     thread ID obtained by \ref osThreadNew or \ref osThreadGetId.
/// \param[in]     flags         specifies the flags of the thread that shall be set.
/// \return thread flags after setting or error code if highest bit set.
uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags)
{
    register rt_base_t status;
    register rt_ubase_t level;
    rt_bool_t need_schedule = RT_FALSE;
    thread_cb_t *thread_cb;
    rt_uint32_t return_value;
    
    thread_cb = (thread_cb_t *)(thread_id);
    
    /* Check parameters */
    if ((RT_NULL == thread_cb) || (rt_object_get_type((rt_object_t)(&thread_cb->thread)) != RT_Object_Class_Thread))
    {
        return osFlagsErrorParameter;
    }

    level = rt_hw_interrupt_disable();

    thread_cb->flag_set |= flags;
    return_value = thread_cb->flag_set;

    /* Check if Thread is waiting for Thread Flags */
    if (thread_cb->thread.event_info & WAITING_THREAD_FLAGS)
    {
        status = -RT_ERROR;
        if (thread_cb->thread.event_info & osFlagsWaitAll)
        {
            if ((thread_cb->thread.event_set & thread_cb->flag_set) == thread_cb->thread.event_set)
            {
                /* received an AND event */
                status = RT_EOK;
            }
        }
        else
        {
            if (thread_cb->thread.event_set & thread_cb->flag_set)
            {
                /* save recieved event set */
                thread_cb->thread.event_set &= thread_cb->flag_set;
                /* received an OR event */
                status = RT_EOK;
            }
        }

        /* condition is satisfied, resume thread */
        if (RT_EOK == status)
        {
            thread_cb->thread.event_info &= ~WAITING_THREAD_FLAGS;
            /* clear event */
            if (!(thread_cb->thread.event_info & osFlagsNoClear))
                thread_cb->flag_set &= ~thread_cb->thread.event_set;

            /* resume thread, and thread list breaks out */
            rt_thread_resume(&(thread_cb->thread));
            need_schedule = RT_TRUE;
        }
    }

    rt_hw_interrupt_enable(level);

    if (need_schedule == RT_TRUE)
        rt_schedule();

//    rt_kprintf("setflag : 0x%x\n", return_value);
    return return_value;
}

/// Wait for one or more Thread Flags of the current running thread to become signaled.
/// \param[in]     flags         specifies the flags to wait for.
/// \param[in]     options       specifies flags options (osFlagsXxxx).
/// \param[in]     timeout       \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out.
/// \return thread flags before clearing or error code if highest bit set.
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    rt_uint32_t return_value;
    register rt_ubase_t level;
    register rt_base_t status = -RT_ERROR;
    rt_thread_t thread = rt_thread_self();
    thread_cb_t *thread_cb;

    /* Check parameters */
    if (RT_NULL == thread)
    {
        return osFlagsErrorParameter;
    }

    thread->error = RT_EOK;
    thread_cb = (thread_cb_t *)(thread->user_data);

    level = rt_hw_interrupt_disable();

    if (options & osFlagsWaitAll)
    {
        if ((thread_cb->flag_set & flags) == flags)
            status = RT_EOK;
    }
    else
    {
        if (thread_cb->flag_set & flags)
            status = RT_EOK;
    }

    if (RT_EOK == status)
    {
        return_value = thread_cb->flag_set & flags;
        if (!(options & osFlagsNoClear))
            thread_cb->flag_set &= ~flags;
    }
    else if (0U == timeout)
    {
        rt_hw_interrupt_enable(level);
        return osFlagsErrorResource;
    }
    else
    {
        thread->event_set = flags;
        thread->event_info = options | WAITING_THREAD_FLAGS;
        rt_thread_suspend(thread);
        /* if there is a waiting timeout, active thread timer */
        if ((timeout > 0U) && (timeout != osWaitForever))
        {
            /* reset the timeout of thread timer and start it */
            rt_timer_control(&(thread->thread_timer),
                             RT_TIMER_CTRL_SET_TIME,
                             &timeout);
            rt_timer_start(&(thread->thread_timer));
        }

        rt_hw_interrupt_enable(level);
        rt_schedule();

        if (thread->error != RT_EOK)
        {
            return thread->error;
        }

        level = rt_hw_interrupt_disable();
        return_value = thread->event_set;
    }

//    rt_kprintf("waitflag : 0x%x\n", return_value);
    rt_hw_interrupt_enable(level);

    return return_value;
}

//  ==== Generic Wait Functions ====

/// Wait for Timeout (Time Delay).
/// \param[in]     ticks         \ref CMSIS_RTOS_TimeOutValue "time ticks" value
/// \return status code that indicates the execution status of the function.
osStatus_t osDelay(uint32_t ticks)
{
    rt_thread_delay(ticks);

    return osOK;
}

//  ==== Timer Management Functions ====

/// Create and Initialize a timer.
/// \param[in]     func          start address of a timer call back function.
/// \param[in]     type          osTimerOnce for one-shot or osTimerPeriodic for periodic behavior.
/// \param[in]     argument      argument to the timer call back function.
/// \param[in]     attr          timer attributes; NULL: default values.
/// \return timer ID for reference by other functions or NULL in case of error.

osTimerId_t osTimerNew(osTimerFunc_t func, osTimerType_t type, void *argument, const osTimerAttr_t *attr)
{
    timer_cb_t *timer_cb;
    char name[RT_NAME_MAX];
    static rt_uint16_t timer_number = 0U;
    rt_uint8_t flag = RT_TIMER_FLAG_SOFT_TIMER;

    /* Check parameters */
    if ((RT_NULL == func) || ((type != osTimerOnce) && (type != osTimerPeriodic)))
    {
        return RT_NULL;
    }

    /* RT-Thread object's name can't be NULL */
    if ((RT_NULL == attr) || (RT_NULL == attr->name))
        rt_snprintf(name, sizeof(name), "timer%02d", timer_number++);
    else
        rt_snprintf(name, sizeof(name), "%s", attr->name);

    if ((RT_NULL == attr) || (RT_NULL == attr->cb_mem))
    {
        timer_cb = rt_malloc(sizeof(timer_cb_t));
        /* Check parameters */
        RT_ASSERT(timer_cb != RT_NULL);

        rt_memset(timer_cb, 0, sizeof(timer_cb_t));
        timer_cb->flags |= MALLOC_CB;
    }
    else
    {
        if (attr->cb_size >= sizeof(timer_cb_t))
        {
            timer_cb = attr->cb_mem;
            timer_cb->flags = 0;
        }
        else
            return RT_NULL;
    }

    if (osTimerPeriodic == type)
    {
        flag |= RT_TIMER_FLAG_PERIODIC;
    }

    rt_timer_init(&(timer_cb->timer), name, func, argument, 0, flag);

    return timer_cb;
}

/// Start or restart a timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerNew.
/// \param[in]     ticks         \ref CMSIS_RTOS_TimeOutValue "time ticks" value of the timer.
/// \return status code that indicates the execution status of the function.
osStatus_t osTimerStart(osTimerId_t timer_id, uint32_t ticks)
{
    rt_err_t result;
    timer_cb_t *timer_cb = RT_NULL;
    timer_cb = (timer_cb_t *)timer_id;
   
    /* Check parameters */
    RT_ASSERT(timer_cb != RT_NULL);

    if ((RT_NULL == timer_cb) || (ticks == 0))
    {
        return osErrorParameter;
    }

    rt_timer_control(&(timer_cb->timer), RT_TIMER_CTRL_SET_TIME, &ticks);

    result = rt_timer_start(&(timer_cb->timer));
    if (RT_EOK == result)
        return osOK;
    else
        return osError;
}

/// Stop a timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerNew.
/// \return status code that indicates the execution status of the function.
osStatus_t osTimerStop(osTimerId_t timer_id)
{
    rt_err_t result = RT_EOK;
    timer_cb_t *timer_cb = RT_NULL;
    timer_cb = (timer_cb_t *)timer_id;
    
    /* Check parameters */
    RT_ASSERT(timer_cb != RT_NULL);
    
    /* Check parameters */
    if (RT_NULL == timer_cb)
    {
        return osErrorParameter;
    }

    result = rt_timer_stop(&(timer_cb->timer));

    if (RT_EOK == result)
        return osOK;
    else
        return osError;
}

/// Check if a timer is running.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerNew.
/// \return 0 not running, 1 running.
uint32_t osTimerIsRunning(osTimerId_t timer_id)
{
    timer_cb_t *timer_cb = RT_NULL;
    timer_cb = (timer_cb_t *)timer_id;
    
    /* Check parameters */
    RT_ASSERT(timer_cb != RT_NULL);
    
    /* Check parameters */
    if ((RT_NULL == timer_cb) || (rt_object_get_type(&timer_cb->timer.parent) != RT_Object_Class_Timer))
    {
        return 0U;
    }

    if ((timer_cb->timer.parent.flag & RT_TIMER_FLAG_ACTIVATED) == 1u)
    {
        return 1;
    }
    else
        return 0U;
}

/// Delete a timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerNew.
/// \return status code that indicates the execution status of the function.
osStatus_t osTimerDelete(osTimerId_t timer_id)
{
    timer_cb_t *timer_cb = RT_NULL;
    timer_cb = (timer_cb_t *)timer_id;

    /* Check parameters */
    RT_ASSERT(timer_cb != RT_NULL);

    rt_timer_detach(&(timer_cb->timer));

    if (timer_cb->flags & MALLOC_CB)
    {
        rt_free(timer_cb);
    }
    
    return osOK;
}

#ifdef RT_USING_SEMAPHORE
osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr)
{
    char name[RT_NAME_MAX];
    sem_cb_t *sem_cb = RT_NULL;
    static rt_uint16_t semaphore_number = 0U;

    /* Check parameters */
    if ((0U == max_count) || (initial_count > max_count))
    {
        return RT_NULL;
    }

    if ((RT_NULL == attr) || (RT_NULL == attr->name))
    {
        rt_snprintf(name, sizeof(name), "sem%02d", semaphore_number++);
    }
    else
    {
        rt_snprintf(name, sizeof(name), "%s", attr->name);
    }
    
    if ((RT_NULL == attr) || (RT_NULL == attr->cb_mem))
    {
        sem_cb = rt_malloc(sizeof(sem_cb_t));
        
        /* Check parameters */
        RT_ASSERT(sem_cb != RT_NULL);
        
        rt_memset(sem_cb, 0, sizeof(sem_cb_t));
        sem_cb->flags |= MALLOC_CB;
    }
    else
    {
        if (attr->cb_size >= sizeof(sem_cb_t))
        {
            sem_cb = attr->cb_mem;
            sem_cb->flags = 0;
        }
        else
        {
            return RT_NULL;
        }
    }

    rt_sem_init(&(sem_cb->sem), name, initial_count, RT_IPC_FLAG_FIFO);

    return sem_cb;
}

/// Acquire a Semaphore token or timeout if no tokens are available.
/// \param[in]     semaphore_id  semaphore ID obtained by \ref osSemaphoreNew.
/// \param[in]     timeout       \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out.
/// \return status code that indicates the execution status of the function.
osStatus_t osSemaphoreAcquire(osSemaphoreId_t semaphore_id, uint32_t timeout)
{
    rt_err_t result;
    sem_cb_t *sem_cb = (sem_cb_t *)semaphore_id;

    /* Check parameters */
    RT_ASSERT(sem_cb != RT_NULL);

    result = rt_sem_take(&(sem_cb->sem), timeout);

    if (RT_EOK == result)
        return osOK;
    else if (-RT_ETIMEOUT == result)
    {
        if (0U == timeout)
            return osErrorResource;
        else
            return osErrorTimeout;
    }
    else
        return osError;
}

/// Release a Semaphore token that was acquired by \ref osSemaphoreAcquire.
/// \param[in]     semaphore_id  semaphore ID obtained by \ref osSemaphoreNew.
/// \return status code that indicates the execution status of the function.
osStatus_t osSemaphoreRelease(osSemaphoreId_t semaphore_id)
{
    rt_err_t result;
    sem_cb_t *sem_cb = (sem_cb_t *)semaphore_id;

    /* Check parameters */
    RT_ASSERT(sem_cb != RT_NULL);

    result = rt_sem_release(&(sem_cb->sem));

    if (RT_EOK == result)
        return osOK;
    else
        return osError;
}

/// Delete a Semaphore object.
/// \param[in]     semaphore_id  semaphore ID obtained by \ref osSemaphoreNew.
/// \return status code that indicates the execution status of the function.
osStatus_t osSemaphoreDelete(osSemaphoreId_t semaphore_id)
{
    sem_cb_t *sem_cb = (sem_cb_t *)semaphore_id;

    /* Check parameters */
    RT_ASSERT(sem_cb != RT_NULL);

    rt_sem_detach(&(sem_cb->sem));

    if (sem_cb->flags & MALLOC_CB)
        rt_free(sem_cb);

    return osOK;
}

#endif

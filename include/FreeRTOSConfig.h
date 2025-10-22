#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* Scheduler and tick settings */
#define configUSE_PREEMPTION            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE         0
#define configCPU_CLOCK_HZ              (100000000UL) /* Dummy */
#define configTICK_RATE_HZ              (1000)        /* 1ms tick */
#define configMAX_PRIORITIES            (5)
#define configMINIMAL_STACK_SIZE        (128)
#define configTOTAL_HEAP_SIZE           (20240)
#define configMAX_TASK_NAME_LEN         (16)
#define configUSE_16_BIT_TICKS          0
#define configIDLE_SHOULD_YIELD         1

/* Hooks */
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0

/* Synchronization */
#define configUSE_MUTEXES               1
#define configUSE_RECURSIVE_MUTEXES     1
#define configUSE_COUNTING_SEMAPHORES   1
#define configUSE_QUEUE_SETS            1
#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       (3)
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    (256)
#define configCHECK_FOR_STACK_OVERFLOW  1
#define configGENERATE_RUN_TIME_STATS 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1
#define configUSE_TRACE_FACILITY 1

#define INCLUDE_uxTaskGetSystemState 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
/* Optional functions */
#define INCLUDE_vTaskPrioritySet        1
#define INCLUDE_uxTaskPriorityGet       1
#define INCLUDE_vTaskDelete             1
#define INCLUDE_vTaskCleanUpResources   0
#define INCLUDE_vTaskSuspend            1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelay              1

/* POSIX port will define portYIELD() itself */
#define portNOP()                       asm volatile("nop")

#endif /* FREERTOS_CONFIG_H */


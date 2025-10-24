#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "health.h"
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

QueueHandle_t gameQueue = NULL;
QueueHandle_t accelQueue = NULL;
QueueHandle_t oscQueue = NULL;
QueueHandle_t gestureQueue = NULL;

TaskHandle_t netTaskHandle = NULL;
TaskHandle_t senderTaskHandle = NULL;
TaskHandle_t oscHandle = NULL;
TaskHandle_t gameHandle = NULL;


//gestureState gesture;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // This function is called if a stack overflow is detected
    printf("Stack overflow in task: %s\n", pcTaskName);

    // Stop execution to prevent undefined behavior
    while (1)
        ; // or while(1); in embedded systems
}

void vConfigureTimerForRunTimeStats(void)
{
    // nothing special needed here
}

uint32_t ulGetRunTimeCounterValue(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec * 1000000ul) + (ts.tv_nsec / 1000)); // microseconds
}
void MonitorTask(void *params)
{
    char stats[512];
    for (;;)
    {
        vTaskGetRunTimeStats(stats);
        printf("\n%s\n", stats);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void vApplicationIdleHook(void) {
    struct timespec ts = {0, 1000000}; // 1 ms
    nanosleep(&ts, NULL);
}
int main()
{
    accelQueue = xQueueCreate(10, sizeof(SensorBuffer *));
    gameQueue = xQueueCreate(10, sizeof(SensorSample));
    oscQueue = xQueueCreate(10, sizeof(oscSample));
    gestureQueue = xQueueCreate(1, sizeof(gestureState*));

    xTaskCreate(netprocess, "Receiver", 4096, NULL, configMAX_PRIORITIES-1, &netTaskHandle);
    xTaskCreate(detectorTask, "Detector", 5048, NULL, configMAX_PRIORITIES-2, &senderTaskHandle);
    xTaskCreate(gameTask, "Game processor", 1024, NULL, configMAX_PRIORITIES-3, &gameHandle);
    xTaskCreate(oscTask, "OSC client", 1024, NULL, configMAX_PRIORITIES-3, &oscHandle);

    xTaskCreate(MonitorTask, "Monitor", 1024, NULL, 1, NULL); // monitor task

    vTaskStartScheduler();

    // Should never reach here
    return 0;
}

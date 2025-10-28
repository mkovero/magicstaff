#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <time.h>
#include <stdio.h>

QueueHandle_t gameQueue = NULL;
QueueHandle_t accelQueue = NULL;
QueueHandle_t oscQueue = NULL;
QueueHandle_t oscEventQueue = NULL;
QueueHandle_t gestureQueue = NULL;
QueueHandle_t freeQueue = NULL;  // Holds pointers to free buffers
QueueHandle_t readyQueue = NULL; // Holds pointers to filled buffers
TaskHandle_t udpRXHandle = NULL;
TaskHandle_t jsonHandle = NULL;
TaskHandle_t detectorHandle = NULL;
TaskHandle_t oscHandle = NULL;
TaskHandle_t oscEventHandle = NULL;

static uint8_t ucHeap[configTOTAL_HEAP_SIZE];

HeapRegion_t xHeapRegions[] = {
    {ucHeap, sizeof(ucHeap)}, // first (and only) region
    {NULL, 0}                 // terminator
};

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
    TaskHandle_t tasks[] = {udpRXHandle, jsonHandle, detectorHandle, oscHandle};
    const char *taskNames[] = {"udpRX", "json", "detector", "OSC"};

    for (;;)
    {
        vTaskGetRunTimeStats(stats);
        printf("\n%s\n", stats);

        printf("=== FreeRTOS Health Report ===\n");
        printf("Task Name       StackHighWater (words)\n");
        printf("-------------------------------------\n");

        //   UBaseType_t water = uxTaskGetStackHighWaterMark(oscHandle);
        // printf("water: %lu", (unsigned long)water);

        for (int i = 0; i < 3; i++)
        {
            UBaseType_t water = uxTaskGetStackHighWaterMark(tasks[i]);
            printf("%-15s %lu\n", taskNames[i], (unsigned)water);
        } /*
         size_t free_bytes = xPortGetFreeHeapSize();
         size_t min_ever_free = xPortGetMinimumEverFreeHeapSize();
         printf("Free heap: %u bytes, minimum ever free: %u bytes\n",
                (unsigned)free_bytes, (unsigned)min_ever_free);*/
        if (oscQueue != NULL)
        {
            printf("Queue oscQueue length: %u / %u\n",
                   (unsigned int)uxQueueMessagesWaiting(oscQueue),
                   (unsigned int)uxQueueSpacesAvailable(oscQueue));
        }
        if (freeQueue != NULL)
        {
            printf("Queue freeQueue length: %u / %u\n",
                   (unsigned int)uxQueueMessagesWaiting(freeQueue),
                   (unsigned int)uxQueueSpacesAvailable(freeQueue));
        }
        static unsigned int peak = 0;
        if (readyQueue != NULL)
        {
            unsigned int waiting =  (unsigned int)uxQueueMessagesWaiting(readyQueue);
            unsigned int avail = (unsigned int)uxQueueSpacesAvailable(readyQueue);
            if (waiting > peak) { peak = waiting; }
            printf("Queue readyQueue length: %u / %u with peak %u\n",
                   waiting,
                   avail, peak);
        }
        if (accelQueue != NULL)
        {
            printf("Queue accelQueue length: %u / %u\n",
                   (unsigned int)uxQueueMessagesWaiting(accelQueue),
                   (unsigned int)uxQueueSpacesAvailable(accelQueue));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void vApplicationIdleHook(void)
{
    struct timespec ts = {0, 1000000}; // 1 ms
    nanosleep(&ts, NULL);
}

int main()
{
    accelQueue = xQueueCreate(10, sizeof(SensorBuffer *));
    oscQueue = xQueueCreate(10, sizeof(oscSample));
    oscEventQueue = xQueueCreate(64, sizeof(OSC_Event));
    gestureQueue = xQueueCreate(1, sizeof(gestureState *));
    freeQueue = xQueueCreate(POOL_SIZE, sizeof(UdpPacket *));
    readyQueue = xQueueCreate(POOL_SIZE, sizeof(UdpPacket *));

    xTaskCreate(udpRX, "Receiver", 6048, NULL, configMAX_PRIORITIES - 2, &udpRXHandle);
    xTaskCreate(jsonTask, "json parser", 1024, NULL, configMAX_PRIORITIES - 1, &jsonHandle);
    xTaskCreate(detectorTask, "Detector", 1024, NULL, configMAX_PRIORITIES - 4, &detectorHandle);
    xTaskCreate(oscTask, "OSC client", 1024, NULL, configMAX_PRIORITIES - 2, &oscHandle);
    xTaskCreate(oscEvent, "OSC event", 1024, NULL, configMAX_PRIORITIES - 2, &oscEventHandle);

    // xTaskCreate(MonitorTask, "Monitor", 1024, NULL, configMAX_PRIORITIES-1, NULL); // monitor task

    vTaskStartScheduler();

    // Should never reach here
    return 0;
}

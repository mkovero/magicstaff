#include "health.h"
#include <stdio.h>

extern QueueHandle_t sensorQueue; // add any other queues here
extern TaskHandle_t netTaskHandle;
extern TaskHandle_t senderTaskHandle;
void FreeRTOS_HealthReport(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    TaskHandle_t tasks[] = { netTaskHandle, senderTaskHandle };
    const char *taskNames[] = { "NetProcess", "Sender" };

    printf("=== FreeRTOS Health Report ===\n");
    printf("Task Name       StackHighWater (words)\n");
    printf("-------------------------------------\n");

    for (int i = 0; i < 2; i++) {
        UBaseType_t water = uxTaskGetStackHighWaterMark(tasks[i]);
        printf("%-15s %lu\n", taskNames[i], (unsigned long)water);
    }

    // Queue example
    if (sensorQueue != NULL) {
        printf("Queue sensorQueue length: %u / %u\n",
               (unsigned int)uxQueueMessagesWaiting(sensorQueue),
               (unsigned int)uxQueueSpacesAvailable(sensorQueue));
    } else {
        printf("Queue sensorQueue not created yet!\n");
    }

    printf("=== End of Report ===\n");
}

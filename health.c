#include "health.h"
#include <stdio.h>

extern QueueHandle_t accelQueue; // add any other queues here
extern QueueHandle_t gameQueue; // add any other queues here
extern QueueHandle_t oscQueue;
extern TaskHandle_t netTaskHandle;
extern TaskHandle_t senderTaskHandle;
extern TaskHandle_t gameHandle;
extern TaskHandle_t oscHandle;

void FreeRTOS_HealthReport(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    TaskHandle_t tasks[] = { netTaskHandle, senderTaskHandle,gameHandle, oscHandle };
    const char *taskNames[] = { "NetProcess", "Sender", "Processor", "OSC" };

    printf("=== FreeRTOS Health Report ===\n");
    printf("Task Name       StackHighWater (words)\n");
    printf("-------------------------------------\n");

    for (int i = 0; i < 4; i++) {
        UBaseType_t water = uxTaskGetStackHighWaterMark(tasks[i]);
        printf("%-15s %lu\n", taskNames[i], (unsigned long)water);
    }

    // Queue example
    if (accelQueue != NULL) {
        printf("Queue accelQueue length: %u / %u\n",
               (unsigned int)uxQueueMessagesWaiting(accelQueue),
               (unsigned int)uxQueueSpacesAvailable(accelQueue));
    } else {
        printf("Queue accelQueue not created yet!\n");
    }
        if (gameQueue != NULL) {
        printf("Queue gameQueue length: %u / %u\n",
               (unsigned int)uxQueueMessagesWaiting(gameQueue),
               (unsigned int)uxQueueSpacesAvailable(gameQueue));
    } else {
        printf("Queue gameQueue not created yet!\n");
    }
        if (oscQueue != NULL) {
        printf("Queue oscQueue length: %u / %u\n",
               (unsigned int)uxQueueMessagesWaiting(oscQueue),
               (unsigned int)uxQueueSpacesAvailable(oscQueue));
    } else {
        printf("Queue oscQueue not created yet!\n");
    }
    printf("=== End of Report ===\n");
}

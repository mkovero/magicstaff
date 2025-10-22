#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "health.h"
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

QueueHandle_t sensorQueue = NULL;

TaskHandle_t netTaskHandle = NULL;
TaskHandle_t senderTaskHandle = NULL;


void classiferTask(void *pv)
{
    SensorSample sensor;
    while (1)
    {
        while (sensorQueue == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (xQueueReceive(sensorQueue, &sensor, portMAX_DELAY))
        {
            printf("Stored sample: type: %d, timestamp=%ld, values=[", sensor.type, sensor.timestamp);
            for (int i = 0; i < VECTOR_SIZE; i++)
            {
                printf("%.6f", sensor.values[i]);
                if (i < VECTOR_SIZE - 1)
                    printf(", ");
            }
            printf("]\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // This function is called if a stack overflow is detected
    printf("Stack overflow in task: %s\n", pcTaskName);

    // Stop execution to prevent undefined behavior
    while (1)
        ; // or while(1); in embedded systems
}
void MonitorTask(void *params)
{
    for (;;)
    {
        FreeRTOS_HealthReport();
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2s interval
    }
}
int main()
{
    sensorQueue = xQueueCreate(10, sizeof(SensorBuffer *));

    xTaskCreate(netprocess, "Receiver", 4096, NULL, 1, &netTaskHandle);
    xTaskCreate(detectorTask, "Detector", 5048, NULL, 1, &senderTaskHandle);
    // xTaskCreate(classiferTask, "Classifier", 1024, NULL, 3, NULL);
    //xTaskCreate(MonitorTask, "Monitor", 512, NULL, 1, NULL); // monitor task

    vTaskStartScheduler();

    // Should never reach here
    return 0;
}

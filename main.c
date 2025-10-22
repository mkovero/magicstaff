#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "health.h"
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <tinyosc.h>
#include <arpa/inet.h>
#include <string.h>
#include <time.h>

QueueHandle_t gameQueue = NULL;
QueueHandle_t accelQueue = NULL;
QueueHandle_t oscQueue = NULL;

TaskHandle_t netTaskHandle = NULL;
TaskHandle_t senderTaskHandle = NULL;
TaskHandle_t oscHandle = NULL;
TaskHandle_t gameHandle = NULL;

static inline float fmaxf_safe(float a, float b) { return (a > b) ? a : b; }
static inline float fminf_safe(float a, float b) { return (a < b) ? a : b; }
static uint64_t oscSent;
uint8_t item = 0;
bool locked = false;

uint8_t scale_to_8bit(float value, float rng, bool use_abs)
{
    float v;
    float safe_rng = fmaxf_safe(rng, 1e-9f); // avoid divide by zero
    float scaled;

    if (use_abs)
    {
        v = fabsf(value);
        v = fminf_safe(fmaxf_safe(v, 0.0f), rng); // clamp to [0, rng]
        scaled = 255.0f * (v / safe_rng);         // scale 0–rng → 0–255
    }
    else
    {
        v = fminf_safe(fmaxf_safe(value, -rng), rng);      // clamp to [-rng, rng]
        scaled = (v + rng) * (255.0f / (2.0f * safe_rng)); // scale -rng–rng → 0–255
    }

    // Clamp final result just in case of rounding
    if (scaled < 0.0f)
        scaled = 0.0f;
    if (scaled > 255.0f)
        scaled = 255.0f;

    return (uint8_t)lroundf(scaled);
}

colorSample gyroToRGB(float *values)
{
    float *v = values;
    colorSample color;

    for (int i = 0; i < VECTOR_SIZE; i++)
    {
        color.r = scale_to_8bit(values[0], 1, true);
        color.g = scale_to_8bit(values[1], 1, true);
        color.b = scale_to_8bit(values[2], 1, true);
    }
    return color;
}
void gameTask(void *pv)
{
    printf("Game processor Started\n");
    while (gameQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (oscQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    };
    while (1)
    {
        SensorSample game;
        oscSample osc;

        if (xQueueReceive(gameQueue, &game, portMAX_DELAY))
        {
            osc.color = gyroToRGB(game.values);
            osc.type = GAMERGB;
            // printf("Game sample: timestamp=%ld, values=[%.2f, %.2f, %.2f]->[%d, %d, %d]\n", game.timestamp, game.values[0], game.values[1], game.values[2], color.r, color.g, color.b);

            if (xQueueSend(oscQueue, &osc, portMAX_DELAY) != pdPASS)
            {
                printf("Game->OSC send failed\n");
            }
        }
        //  vTaskDelay(pdMS_TO_TICKS(20));
    }
}
void oscTask(void *pv)
{
    printf("OSC Started\n");
    while (oscQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // UDP socket setup
    int sockfd;
    struct sockaddr_in server_addr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("socket");
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(7700);                       // target port
    server_addr.sin_addr.s_addr = inet_addr("192.168.9.131"); // target IP

    while (1)
    {
        oscSample osc;

        if (xQueueReceive(oscQueue, &osc, portMAX_DELAY))
        {
            switch (osc.type)
            {
            case GAMERGB:
                if (!locked)
                {
                    char buffer[64];
                    tosc_bundle bundle;
                    tosc_writeBundle(&bundle, TINYOSC_TIMETAG_IMMEDIATELY, buffer, sizeof(buffer));
                    switch (item)
                    {

                    case 0:
                        tosc_writeNextMessage(&bundle, "/r", "i", osc.color.r);
                        tosc_writeNextMessage(&bundle, "/g", "i", osc.color.g);
                        tosc_writeNextMessage(&bundle, "/b", "i", osc.color.b);
                        break;
                    case 1:
                        tosc_writeNextMessage(&bundle, "/r2", "i", osc.color.r);
                        tosc_writeNextMessage(&bundle, "/g2", "i", osc.color.g);
                        tosc_writeNextMessage(&bundle, "/b2", "i", osc.color.b);
                        break;
                    default:
                        break;
                    }
                    int msg_size = tosc_getBundleLength(&bundle);
                    // printf("Message size is: %d",msg_size);
                    // tosc_printOscBuffer(buffer, msg_size);
                    //  Send OSC message via UDP
                    if (sendto(sockfd, buffer, msg_size, 0,
                               (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                    {
                        perror("sendto");
                    }
                    oscSent++;
                }
                break;
            case CTRLLEFT:
                char bufferL[16];
                int msg_size = tosc_writeMessage(bufferL, sizeof(bufferL), "/left", "i", 1);
                if (sendto(sockfd, bufferL, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }
                printf("L1\n");

                vTaskDelay(pdMS_TO_TICKS(100));
                msg_size = tosc_writeMessage(bufferL, sizeof(bufferL), "/left", "i", 0);
                if (sendto(sockfd, bufferL, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }
                printf("L0\n");

                break;
            case CTRLRIGHT:
                char bufferR[16];
                msg_size = tosc_writeMessage(bufferR, sizeof(bufferR), "/right", "i", 1);
                if (sendto(sockfd, bufferR, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }
                printf("R1\n");

                vTaskDelay(pdMS_TO_TICKS(100));
                msg_size = tosc_writeMessage(bufferL, sizeof(bufferL), "/right", "i", 0);
                if (sendto(sockfd, bufferL, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }
                printf("R0\n");

                break;
            default:
                break;
            }
        }
        //  vTaskDelay(pdMS_TO_TICKS(20));
    }
    close(sockfd);
}
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

    xTaskCreate(netprocess, "Receiver", 4096, NULL, configMAX_PRIORITIES-1, &netTaskHandle);
    xTaskCreate(detectorTask, "Detector", 5048, NULL, configMAX_PRIORITIES-2, &senderTaskHandle);
    xTaskCreate(gameTask, "Game processor", 1024, NULL, configMAX_PRIORITIES-3, &gameHandle);
    xTaskCreate(oscTask, "OSC client", 1024, NULL, configMAX_PRIORITIES-3, &oscHandle);

   // xTaskCreate(MonitorTask, "Monitor", 1024, NULL, 1, NULL); // monitor task

    vTaskStartScheduler();

    // Should never reach here
    return 0;
}

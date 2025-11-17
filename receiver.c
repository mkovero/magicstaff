#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdint.h>
#include "jsmn.h"
#include "receiver.h"
#include "datatypes.h"
#include <errno.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <time.h>
#include <game.h>

extern QueueHandle_t accelQueue;
extern QueueHandle_t oscQueue;
extern QueueHandle_t freeQueue;
extern QueueHandle_t readyQueue;
extern QueueHandle_t gestureQueue;

extern gestureState gesture;
// Parse JSON and store sample
void jsonTask(void *pvParameters)
{
    SensorBuffer *a = pvPortMalloc(sizeof(SensorBuffer));
    a->head = 0;
    while (accelQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    };
    xQueueSend(accelQueue, &a, portMAX_DELAY);
    //  xQueuePeek(accelQueue, &a, portMAX_DELAY);
    while (readyQueue == NULL)
    {
        vTaskDelay(1000);
    }
    UdpPacket *pkt;

        static gestureState gesture = {
        .lastGesture = 0,
        .gestureCooldown = 500,
        .locked = false,
        .reallyLocked = false,
        .item = 0,
        .still_time = 200,
    };
    gestureState *ptr = &gesture;
    xQueueSend(gestureQueue, &ptr, portMAX_DELAY);


    printf("Json parser started\n");

    while (1)
    {
        xQueueReceive(readyQueue, &pkt, portMAX_DELAY);
        const char *json = (const char *)pkt->buf;
        jsmn_parser parser;
        jsmntok_t tokens[MAX_TOKENS];
        jsmn_init(&parser);
        int ret = jsmn_parse(&parser, json, pkt->len, tokens, MAX_TOKENS);
        if (ret < 0)
        {
            printf("Failed to parse JSON: %d\n", ret);
            return;
        }
        if (ret < 1 || tokens[0].type != JSMN_OBJECT)
        {
            printf("Expected JSON object\n");
            return;
        }

        char type[64] = {0};
        int64_t timestamp = 0;
        float values[VECTOR_SIZE] = {0};

        // Iterate over top-level keys
        for (int i = 1; i < ret; i++)
        {
            jsmntok_t *key = &tokens[i];
            if (key->type != JSMN_STRING)
                continue;

            char key_str[64];
            json_token_to_str(json, key, key_str, sizeof(key_str));

            jsmntok_t *val = &tokens[i + 1]; // value is next token

            if (strcmp(key_str, "type") == 0)
            {
                json_token_to_str(json, val, type, sizeof(type));
            }
            else if (strcmp(key_str, "timestamp") == 0)
            {
                char ts_str[32];
                json_token_to_str(json, val, ts_str, sizeof(ts_str));
                timestamp = atoll(ts_str);
            }
            else if (strcmp(key_str, "values") == 0 && val->type == JSMN_ARRAY)
            {
                int count = val->size < VECTOR_SIZE ? val->size : VECTOR_SIZE;
                for (int j = 0; j < count; j++)
                {
                    jsmntok_t *numtok = &tokens[i + 2 + j]; // value tokens follow the array token
                    char numbuf[32];
                    json_token_to_str(json, numtok, numbuf, sizeof(numbuf));
                    values[j] = atof(numbuf);
                }
            }

            i++; // skip value token
            if (val->type == JSMN_ARRAY || val->type == JSMN_OBJECT)
            {
                i += val->size; // skip nested tokens
            }
        }
        // Dispatch to buffer
        if (strcmp(type, "android.sensor.game_rotation_vector") == 0)
        {
            oscSample osc;
            colorSample color;
 //    printf("Received %.2f/%.2f/%.2f ", values[0], values[1], values[2]);
 
            color.r = scale_to_8bit(values[0], 0.4, true);
            color.g = scale_to_8bit(values[1], 0.8, true);
            color.b = scale_to_8bit(values[2], 0.6, true);
            osc.color = color;
            osc.type = GAMERGB;
            osc.delay = 200;

          //      printf("converted to %d/%d/%d\n", color.r, color.g, color.b);

            if (oscQueue != NULL)
            {
                xQueueSend(oscQueue, &osc, portMAX_DELAY);
            }
            else
            {
                printf("oscQueue is NULL, send failed\n");
            }
        }
        else if (strcmp(type, "android.sensor.linear_acceleration") == 0)
        {
            processSample(values[0], values[1], values[2]);
           /* SensorSample *s = &a->samples[a->head];

            s->received_ms = get_current_ms();
            s->timestamp = timestamp / 1000000; // convert ns → ms
            s->type = LINEAR_ACCEL;

            memcpy(s->values, values, sizeof(s->values));

            a->head = (a->head + 1) % BUFFER_SIZE;

            // Send buffer to detector task
            if (accelQueue != NULL)
            {
                xQueueSend(accelQueue, &a, portMAX_DELAY);
            }
            else
            {
                printf("accelQueue is NULL, send failed\n");
            }*/
        }
        else
        {
            printf("Unknown sensor type: %s\n", type);
            return;
        }
        xQueueSend(freeQueue, &pkt, 0);
    }
}
void init_rxBuffers(UdpPacket *packetPool)
{
    while (freeQueue == NULL)
    {
        vTaskDelay(1000);
    }

    for (int i = 0; i < POOL_SIZE; i++)
    {
        UdpPacket *p = &packetPool[i];

        // Zero out the buffer
        memset(p->buf, 0, BUF_SIZE);
        p->len = 0;

        // Send pointer to the free queue
        xQueueSend(freeQueue, &p, portMAX_DELAY);
    }
}
int init_udpserver(uint16_t port)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("socket failed");
        return -1;
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY; // bind to all interfaces
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        perror("bind failed");
        close(sockfd);
        return -1;
    }
    else
    {
        printf("Created UDP listener for port %d\n", port);
    }

    return sockfd;
}

void udpRX(void *pvParameters)
{
    UdpPacket packetPool[POOL_SIZE];
    init_rxBuffers(packetPool);
    int sockfd = init_udpserver(PORT);

    UdpPacket *pkt;
    printf("UDP RX task started\n");

    while (1)
    {
        xQueueReceive(freeQueue, &pkt, portMAX_DELAY);
        socklen_t addr_len = sizeof(pkt->addr);
        struct sockaddr_in client_addr;
        ssize_t n;

        while (1)
        { // Inner retry loop for transient errors
            n = recvfrom(sockfd, pkt->buf, BUF_SIZE, 0,
                         (struct sockaddr *)&client_addr, &addr_len);

            if (n < 0)
            {
                int err = errno;
                if (err == EINTR)
                {
                    continue; // Retry immediately on interrupt
                }
                else if (err == EAGAIN || err == EWOULDBLOCK)
                {
                    vTaskDelay(pdMS_TO_TICKS(1)); // Minimal yield if no data (rare for blocking socket)
                    continue;
                }
                else
                {
                    printf("recvfrom failed: errno=%d\n", err);
                    // Fatal error: release buffer and break to get a new one
                    xQueueSend(freeQueue, &pkt, portMAX_DELAY);
                    break;
                }
            }
            else
            {
                // Success path (n >= 0)
                pkt->len = (size_t)n;
                pkt->addr = client_addr;
                if (pkt->len < BUF_SIZE)
                {
                    pkt->buf[pkt->len] = '\0';
                }
                xQueueSend(readyQueue, &pkt, portMAX_DELAY);
                break; // Exit inner loop to get next free buffer
            }
        }
    }

    close(sockfd);
}

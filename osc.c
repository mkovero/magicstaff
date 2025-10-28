#include <tinyosc.h>
#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

extern QueueHandle_t oscQueue;
extern QueueHandle_t gestureQueue;
extern QueueHandle_t oscEventQueue;

// Compute 16-bit CRC/X.25 (poly 0x1021, init 0xFFFF, refin=true, refout=true, xorout=0xFFFF)
uint16_t qlc_crc16_x25(const char *path)
{
    const uint8_t *data = (const uint8_t *)path;
    uint16_t crc = 0xFFFF;
    while (*data)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return (uint16_t)(~crc);
}

void osc_send(oscFixture *fixture)
{
    smallUdpPacket txOscBundle;
    memset(&txOscBundle, 0, sizeof(txOscBundle));
    txOscBundle.addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    txOscBundle.addr.sin_port = htons(7700);
    txOscBundle.addr.sin_family = AF_INET;

    txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf), fixture->pathR, "i", fixture->color.r);
    udp_send(&txOscBundle, fixture->sockfd);
    txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf), fixture->pathG, "i", fixture->color.g);
    udp_send(&txOscBundle, fixture->sockfd);
    txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf), fixture->pathB, "i", fixture->color.b);
    udp_send(&txOscBundle, fixture->sockfd);
}
void schedule_osc_event(int64_t delay_us, event_cb_t cb, oscFixture *fixture)
{
    OSC_Event evt;
    evt.target_us = get_current_us() + delay_us;
    evt.callback = cb;
    evt.data = fixture;
    xQueueSend(oscEventQueue, &evt, portMAX_DELAY);
}

void light_off_cb(oscFixture *fixture)
{
    fixture->color = fixture->oldColor;
    osc_send(fixture);
}

void light_on_cb(oscFixture *fixture)
{
    fixture->oldColor = fixture->color;
    fixture->color.r = fixture->onvalue;
    fixture->color.g = fixture->onvalue;
    fixture->color.b = fixture->onvalue;

    osc_send(fixture);
    schedule_osc_event(200000, light_off_cb, fixture); // off after 200 ms
}

void oscTask(void *pv)
{
    printf("OSC Started\n");
    uint64_t oscSent = 0;

    while (oscQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // UDP socket setup
    int sockfd;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("socket");
    }

    while (gestureQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    };
    gestureState *gesture;
    xQueuePeek(gestureQueue, &gesture, portMAX_DELAY);
#define FIXTURE_COUNT 4
    oscFixture fixture[FIXTURE_COUNT];
    fixture[0].pathR = "/r";
    fixture[0].pathG = "/g";
    fixture[0].pathB = "/b";
    fixture[1].pathR = "/r2";
    fixture[1].pathG = "/g2";
    fixture[1].pathB = "/b2";
    fixture[2].pathR = "/r3";
    fixture[2].pathG = "/g3";
    fixture[2].pathB = "/b3";
    fixture[3].pathR = "/r4";
    fixture[3].pathG = "/g4";
    fixture[3].pathB = "/b4";
    for (int x = 0; x < FIXTURE_COUNT; x++)
    {

        printf("Fixture %d path %s %d\n", x, fixture[x].pathR, qlc_crc16_x25(fixture[x].pathR) + 1);
        printf("Fixture %d path %s %d\n", x, fixture[x].pathG, qlc_crc16_x25(fixture[x].pathG) + 1);
        printf("Fixture %d path %s %d\n", x, fixture[x].pathB, qlc_crc16_x25(fixture[x].pathB) + 1);

        fixture[x].onvalue = 255;
        fixture[x].offvalue = 0;
        fixture[x].sockfd = sockfd;
        fixture[x].color.r = 0;
        fixture[x].color.g = 0;
        fixture[x].color.b = 0;
        fixture[x].oldColor.r = 0;
        fixture[x].oldColor.g = 0;
        fixture[x].oldColor.b = 0;
    }
    while (1)
    {
        oscSample osc;
        uint32_t msg_size = 0;
        if (xQueueReceive(oscQueue, &osc, portMAX_DELAY))
        {
            uint8_t item = atomic_load(&gesture->item); // read

            // printf("Received OSC control with type %d delay %d and item %d\n",osc.type, osc.delay, osc.item);
            switch (osc.type)
            {
                oscData data;
            case GAMERGB:
                if (!atomic_load(&gesture->locked))
                {
                    if (!atomic_load(&gesture->active))
                    {
                        fixture[item].color = osc.color;
                        osc_send(&fixture[item]);
                    }
                }
                break;
            case CTRLLEFT:
                light_on_cb(&fixture[item]);
                break;
            case CTRLRIGHT:
                light_on_cb(&fixture[item]);
                break;
            case CTRLBOTH:

                for (int x = 0; x < FIXTURE_COUNT; x++)
                {
                    light_on_cb(&fixture[x]);
                }

                break;
            default:
                break;
            }
            oscSent++;
        }
        //  vTaskDelay(pdMS_TO_TICKS(20));
    }
    close(sockfd);
}

void oscEvent(void *pvParameters)
{
    OSC_Event evt;
    for (;;)
    {
        if (xQueueReceive(oscEventQueue, &evt, portMAX_DELAY))
        {
            int64_t now_us = get_current_us(); // CLOCK_MONOTONIC_RAW
            if (evt.target_us <= now_us)
            {
                evt.callback(evt.data);
            }
            else
            {
                // Not yet time, put it back at the front of the queue
                xQueueSendToFront(oscEventQueue, &evt, portMAX_DELAY);
            }
        }
    }
}
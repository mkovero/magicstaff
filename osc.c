#include <tinyosc.h>
#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <game.h>

extern QueueHandle_t oscQueue;
extern QueueHandle_t vectorQueue;
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

void osc_send(oscFixture fixture)
{
    smallUdpPacket txOscBundle;
    memset(&txOscBundle, 0, sizeof(txOscBundle));
    txOscBundle.addr.sin_addr.s_addr = inet_addr("192.168.9.131");
    txOscBundle.addr.sin_port = htons(7700);
    txOscBundle.addr.sin_family = AF_INET;
    tosc_bundle bundle;

    tosc_writeBundle(&bundle, TINYOSC_TIMETAG_IMMEDIATELY, txOscBundle.buf, sizeof(txOscBundle.buf));
    tosc_writeNextMessage(&bundle, fixture.pathR, "i", fixture.color.r);
    tosc_writeNextMessage(&bundle, fixture.pathG, "i", fixture.color.g);
    tosc_writeNextMessage(&bundle, fixture.pathB, "i", fixture.color.b);
    txOscBundle.len = tosc_getBundleLength(&bundle); // msg_size is reported to be 1
    // printf("Asked to send %d/%d/%d\n", fixture.color.r, fixture.color.g, fixture.color.b);
    udp_send(&txOscBundle, fixture.sockfd);
}
void schedule_osc_event(int64_t delay_us, event_cb_t cb, oscFixture fixture)
{
    OSC_Event evt;
    evt.target_us = get_current_us() + delay_us;
    evt.callback = cb;
    evt.data = fixture;
    xQueueSend(oscEventQueue, &evt, portMAX_DELAY);
}

void light_off_cb(oscFixture fixture)
{
    fixture.color = fixture.oldColor;
    osc_send(fixture);
}

void light_on_cb(oscFixture fixture)
{
    fixture.oldColor = fixture.color;
    fixture.color.r = fixture.onvalue;
    fixture.color.g = fixture.onvalue;
    fixture.color.b = fixture.onvalue;

    osc_send(fixture);
    schedule_osc_event(300000, light_off_cb, fixture); // off after 200 ms
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
    while (xQueuePeek(gestureQueue, &gesture, portMAX_DELAY) != pdPASS)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        printf(",\n");
    }
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
        fixture[x].quat[0] = 0.0f;
        fixture[x].quat[1] = 0.0f;
        fixture[x].quat[2] = 0.0f;
        fixture[x].virtualQuat = false;
        fixture[x].oscInitialized = false;
    }
    while (1)
    {
        oscSample osc;
        uint32_t msg_size = 0;
        colorSample color;
        //    printf("Received %.2f/%.2f/%.2f ", values[0], values[1], values[2]);
        SensorSample v;
        atomic_uintmax_t item = atomic_load(&gesture->item); // read

        if (xQueueReceive(vectorQueue, &v, 0))
        {
            if (!atomic_load(&gesture->locked))
            {
                if (!fixture[item].oscInitialized)
                {
                    memcpy(fixture[item].quat, v.values, sizeof(fixture[item].quat));
                    fixture[item].oscInitialized = true;
                }
                float x_dist = sqrt(pow(v.values[0] - fixture[item].quat[0], 2));
                float y_dist = sqrt(pow(v.values[1] - fixture[item].quat[1], 2));
                float z_dist = sqrt(pow(v.values[2] - fixture[item].quat[2], 2));

                float vx = v.values[0];
                float vy = v.values[1];
                float vz = v.values[2];
                //            printf("[1 origin/virt %.2f/%.2f || %.2f/%.2f || %.2f/%.2f\n", v.values[0], vx, v.values[1], vy, v.values[2], vz);

                if (vx >= 1.0f)
                    vx = 1.0f;
                if (vy >= 1.0f)
                    vy = 1.0f;
                if (vz >= 1.0f)
                    vz = 1.0f;

                if (vx <= -1.0f)
                    vx = -1.0f;
                if (vy <= -1.0f)
                    vy = -1.0f;
                if (vz <= -1.0f)
                    vz = -1.0f;
                //  printf("[1 origin/virt %.2f/%.2f || %.2f/%.2f || %.2f/%.2f\n", v.values[0], vx, v.values[1], vy, v.values[2], vz);
                fixture[item].color.r = scale_to_8bit(vx, 0.4, true);
                fixture[item].color.g = scale_to_8bit(vy, 0.8, true);
                fixture[item].color.b = scale_to_8bit(vz, 0.6, true);
                //   printf("[2 scaled/dist %d/%.2f || %d/%.2f || %d/%.2f\n", fixture[item].color.r, x_dist, fixture[item].color.g, y_dist, fixture[item].color.b, z_dist);

                osc_send(fixture[item]);
                //  if (fixture[item].virtualQuat)
                //    fixture[item].virtualQuat = false;
            }
            else
            {
                if (!fixture[item].virtualQuat)
                {
                    memcpy(fixture[item].quat, v.values, sizeof(fixture[item].quat));
                    fixture[item].virtualQuat = true;
                }
            }
        }
        if (xQueueReceive(oscQueue, &osc, 0))
        {

            // printf("Received OSC control with type %d delay %d and item %d\n",osc.type, osc.delay, osc.item);
            switch (osc.type)
            {
                oscData data;
            case GAMERGB:

                break;
            case CTRLLEFT:
                light_on_cb(fixture[item]);
                break;
            case CTRLRIGHT:
                light_on_cb(fixture[item]);
                break;
            case CTRLBOTH:

                for (int x = 0; x < FIXTURE_COUNT; x++)
                {
                    light_on_cb(fixture[x]);
                }

                break;
            default:
                break;
            }
            oscSent++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
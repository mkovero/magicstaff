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

void osc_send(int sockfd, const char*addr, uint8_t value) {
            smallUdpPacket txOscBundle;
            memset(&txOscBundle, 0, sizeof(txOscBundle));
            txOscBundle.addr.sin_addr.s_addr = inet_addr("192.168.9.131");
            txOscBundle.addr.sin_port = htons(7700);
            txOscBundle.addr.sin_family = AF_INET;
                           
            txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf),addr, "i",  value);
            udp_send(&txOscBundle,sockfd);
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
    }
    while (1)
    {
        oscSample osc;
        uint32_t msg_size = 0;
        if (xQueueReceive(oscQueue, &osc, portMAX_DELAY))
        {


            // printf("Received OSC control with type %d delay %d and item %d\n",osc.type, osc.delay, osc.item);
            switch (osc.type)
            {
            case GAMERGB:
                if (!gesture->locked)
                {
                    osc_send(sockfd, fixture[gesture->item].pathR, osc.color.r );
                    osc_send(sockfd, fixture[gesture->item].pathG, osc.color.g );
                    osc_send(sockfd, fixture[gesture->item].pathB, osc.color.b );
                    fixture[gesture->item].color = osc.color;
                }
                break;
            case CTRLLEFT:
                    osc_send(sockfd, fixture[gesture->item].pathR, fixture[gesture->item].onvalue );
                    osc_send(sockfd, fixture[gesture->item].pathG, fixture[gesture->item].onvalue );
                    osc_send(sockfd, fixture[gesture->item].pathB, fixture[gesture->item].onvalue );

                vTaskDelay(pdMS_TO_TICKS(osc.delay));
                    osc_send(sockfd, fixture[gesture->item].pathR, fixture[gesture->item].color.r );
                    osc_send(sockfd, fixture[gesture->item].pathG, fixture[gesture->item].color.g );
                    osc_send(sockfd, fixture[gesture->item].pathB, fixture[gesture->item].color.b );
                break;
            case CTRLRIGHT:
                    osc_send(sockfd, fixture[gesture->item].pathR, fixture[gesture->item].onvalue );
                    osc_send(sockfd, fixture[gesture->item].pathG, fixture[gesture->item].onvalue );
                    osc_send(sockfd, fixture[gesture->item].pathB, fixture[gesture->item].onvalue );

                vTaskDelay(pdMS_TO_TICKS(osc.delay));
                    osc_send(sockfd, fixture[gesture->item].pathR, fixture[gesture->item].color.r );
                    osc_send(sockfd, fixture[gesture->item].pathG, fixture[gesture->item].color.g );
                    osc_send(sockfd, fixture[gesture->item].pathB, fixture[gesture->item].color.b );

                break;
            case CTRLBOTH:

                for (int x = 0; x < FIXTURE_COUNT; x++)
                {
                    osc_send(sockfd, fixture[x].pathR, fixture[x].onvalue );
                    osc_send(sockfd, fixture[x].pathG, fixture[x].onvalue );
                    osc_send(sockfd, fixture[x].pathB, fixture[x].onvalue );
                }


                vTaskDelay(pdMS_TO_TICKS(osc.delay));
                for (int x = 0; x < FIXTURE_COUNT; x++)
                {
                    osc_send(sockfd, fixture[x].pathR, fixture[x].color.r );
                    osc_send(sockfd, fixture[x].pathG, fixture[x].color.g );
                    osc_send(sockfd, fixture[x].pathB, fixture[x].color.b );
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
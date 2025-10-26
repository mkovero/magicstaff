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

    UdpPacket txOscBundle;
    memset(&txOscBundle, 0, sizeof(txOscBundle));
    txOscBundle.addr.sin_addr.s_addr = inet_addr("192.168.9.131");
    txOscBundle.addr.sin_port = htons(7700);
    txOscBundle.addr.sin_family = AF_INET;

    while (gestureQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    };
    gestureState *gesture;
    xQueuePeek(gestureQueue, &gesture, portMAX_DELAY);

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
                    tosc_bundle bundle;

                    tosc_writeBundle(&bundle, TINYOSC_TIMETAG_IMMEDIATELY, txOscBundle.buf, sizeof(txOscBundle.buf));
                    switch (gesture->item)
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
                        printf("OSC item did not hit %d\n", gesture->item);
                        break;
                    }

                    txOscBundle.len = tosc_getBundleLength(&bundle); // msg_size is reported to be 16
                    udp_send(&txOscBundle, sockfd);
                }
                break;
            case CTRLLEFT:
                txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf), "/left", "i", 1);
                udp_send(&txOscBundle, sockfd);

                vTaskDelay(pdMS_TO_TICKS(osc.delay));
                txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf), "/left", "i", 0);
                udp_send(&txOscBundle, sockfd);

                break;
            case CTRLRIGHT:
                txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf), "/right", "i", 1);
                udp_send(&txOscBundle, sockfd);

                vTaskDelay(pdMS_TO_TICKS(osc.delay));
                txOscBundle.len = tosc_writeMessage(txOscBundle.buf, sizeof(txOscBundle.buf), "/right", "i", 0);
                udp_send(&txOscBundle, sockfd);

                break;
            case CTRLBOTH:
                tosc_bundle bundleBoth;
                tosc_writeBundle(&bundleBoth, TINYOSC_TIMETAG_IMMEDIATELY, txOscBundle.buf, sizeof(txOscBundle.buf));
                tosc_writeNextMessage(&bundleBoth, "/right", "i", 1);
                tosc_writeNextMessage(&bundleBoth, "/left", "i", 1);
                txOscBundle.len = tosc_getBundleLength(&bundleBoth);
                udp_send(&txOscBundle, sockfd);

                vTaskDelay(pdMS_TO_TICKS(osc.delay));
                tosc_writeBundle(&bundleBoth, TINYOSC_TIMETAG_IMMEDIATELY, txOscBundle.buf, sizeof(txOscBundle.buf));
                tosc_writeNextMessage(&bundleBoth, "/right", "i", 0);
                tosc_writeNextMessage(&bundleBoth, "/left", "i", 0);
                txOscBundle.len = tosc_getBundleLength(&bundleBoth);
                udp_send(&txOscBundle, sockfd);

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
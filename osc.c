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
extern uint64_t oscSent;
extern gestureState gesture;
extern uint8_t item;

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
                if (!gesture.locked)
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

                vTaskDelay(pdMS_TO_TICKS(200));
                msg_size = tosc_writeMessage(bufferL, sizeof(bufferL), "/left", "i", 0);
                if (sendto(sockfd, bufferL, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }

                break;
            case CTRLRIGHT:
                char bufferR[16];
                msg_size = tosc_writeMessage(bufferR, sizeof(bufferR), "/right", "i", 1);
                if (sendto(sockfd, bufferR, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }

                vTaskDelay(pdMS_TO_TICKS(200));
                msg_size = tosc_writeMessage(bufferL, sizeof(bufferL), "/right", "i", 0);
                if (sendto(sockfd, bufferL, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }

                break;
            case CTRLBOTH:
                char bufferB[64];
                tosc_bundle bundleBoth;
                tosc_writeBundle(&bundleBoth, TINYOSC_TIMETAG_IMMEDIATELY, bufferB, sizeof(bufferB));
                tosc_writeNextMessage(&bundleBoth, "/right", "i", 1);
                tosc_writeNextMessage(&bundleBoth, "/left", "i", 1);
                msg_size = tosc_getBundleLength(&bundleBoth);
                if (sendto(sockfd, bufferB, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }

                vTaskDelay(pdMS_TO_TICKS(200));
                tosc_writeBundle(&bundleBoth, TINYOSC_TIMETAG_IMMEDIATELY, bufferB, sizeof(bufferB));
                tosc_writeNextMessage(&bundleBoth, "/right", "i", 0);
                tosc_writeNextMessage(&bundleBoth, "/left", "i", 0);
                msg_size = tosc_getBundleLength(&bundleBoth);
                if (sendto(sockfd, bufferB, msg_size, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                {
                    perror("sendto");
                }

                break;
            default:
                break;
            }
        }
        //  vTaskDelay(pdMS_TO_TICKS(20));
    }
    close(sockfd);
}
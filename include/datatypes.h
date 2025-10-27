#ifndef DATATYPES_H
#define DATATYPES_H
#include <stdint.h>
#include <tinyosc.h>
#include <FreeRTOS.h>
#include <arpa/inet.h>

#define BUFFER_SIZE_BYTES 1024
#define MAX_TOKENS 128

#define SAMPLE_RATE 50
#define BUFFER_SECONDS 2
#define BUFFER_SIZE (SAMPLE_RATE * BUFFER_SECONDS)
#define VECTOR_SIZE 3

#define GAME_ROTATION 0
#define LINEAR_ACCEL 1
#define UNKNOWN 255

#define POOL_SIZE 12
#define BUF_SIZE 512

#define HISTORY_SIZE 64 // history of previous samples for sliding detection

typedef struct
{
    uint8_t buf[BUF_SIZE];
    size_t len;
    struct sockaddr_in addr;
} UdpPacket;

typedef enum
{
    GAMERGB,
    CTRLLEFT,
    CTRLRIGHT,
    CTRLBOTH
} oscType;

typedef struct
{
    struct timespec ts;
    float ax[BUFFER_SIZE];
    float ay[BUFFER_SIZE];
    float az[BUFFER_SIZE];
    float mag[BUFFER_SIZE];
    int above[BUFFER_SIZE];

} trackingStats;

typedef struct
{
    TickType_t lastGesture;
    TickType_t gestureCooldown;
    bool locked;
    bool reallyLocked;
    uint8_t item;
    bool active;
    int start_sample;
    int above_count;
    float ax_history[HISTORY_SIZE];
    float ay_history[HISTORY_SIZE];
    float az_history[HISTORY_SIZE];
    int history_count;
    int total_samples;
} gestureState;

typedef struct
{
    double yaw;
    double pitch;
    double roll;
} positionSample;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} colorSample;
typedef struct
{
    colorSample color;
    oscType type;
    uint32_t delay;
} oscSample;

typedef struct
{
    uint8_t type;
    int64_t timestamp;
    int64_t received_ms; // processing timestamp in ms
    float values[VECTOR_SIZE];
} SensorSample;

typedef struct
{
    SensorSample samples[BUFFER_SIZE];
    int head; // next write position
} SensorBuffer;

static inline void udp_send(UdpPacket *pkt, int sockfd)
{
    sendto(sockfd, pkt->buf, pkt->len, 0,
           (struct sockaddr *)&pkt->addr, sizeof(pkt->addr));
}

void udpRX(void *pvParameters);
void detectorTask(void *params);
int64_t get_current_ms(void);
void gameTask(void *pv);
void oscTask(void *pv);
void jsonTask(void *pv);

#endif
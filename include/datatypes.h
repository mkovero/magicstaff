#ifndef DATATYPES_H
#define DATATYPES_H
#include <stdint.h>
#include <tinyosc.h>
#include <FreeRTOS.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdatomic.h>

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

typedef struct
{
    uint8_t buf[64];
    size_t len;
    struct sockaddr_in addr;
} smallUdpPacket;

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
    atomic_bool locked;
    bool reallyLocked;
    atomic_uintmax_t item;
    atomic_bool active;
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
    int sockfd;
    const char *addr;
    uint8_t value;
    uint8_t offvalue;
} oscData;
typedef struct
{
    const char *pathR;
    const char *pathG;
    const char *pathB;
    uint8_t onvalue;
    uint8_t offvalue;
    colorSample color;
    colorSample oldColor;
    int sockfd;
} oscFixture;

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

typedef void (*event_cb_t)(oscFixture*);

typedef struct
{
    int64_t target_us; // execution time in microseconds (monotonic)
    event_cb_t callback;
    oscFixture *data; // optional argument
} OSC_Event;

static inline void udp_send(smallUdpPacket *pkt, int sockfd)
{
    sendto(sockfd, pkt->buf, pkt->len, 0,
           (struct sockaddr *)&pkt->addr, sizeof(pkt->addr));
}

static inline int64_t get_current_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts); // Use monotonic time for relative measurements
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}
static inline int64_t get_current_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts); // high-resolution monotonic clock
    return ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}

void udpRX(void *pvParameters);
void detectorTask(void *params);
int64_t get_current_ms(void);
void gameTask(void *pv);
void oscTask(void *pv);
void jsonTask(void *pv);
void oscEvent(void *pvParameters);
#endif
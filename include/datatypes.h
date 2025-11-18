#ifndef DATATYPES_H
#define DATATYPES_H
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

typedef enum
{
    RIGHT,
    LEFT,
    UP,
    DOWN,
    UPRIGHT,
    UPLEFT,
    DOWNRIGHT,
    DOWNLEFT,
    MIXED,
    WEAK,
    SHAKE,
    PUSH,
    PULL,
    NOTKNOWN
} Direction;

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
    uint8_t type;
    int64_t timestamp;
    int64_t received_ms; // processing timestamp in ms
    float values[VECTOR_SIZE];
} SensorSample;
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
    SensorSample data;
    float mag;
    int above;
} historySample;

typedef struct
{
    TickType_t lastGesture;
    TickType_t gestureCooldown;
    atomic_bool locked;
    bool reallyLocked;
    atomic_uintmax_t item;
    atomic_bool holding;
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
    float quat[3];
    bool virtualQuat;
    bool oscInitialized;

} oscFixture;

typedef struct
{
    SensorSample samples[BUFFER_SIZE];
    int head; // next write position
} SensorBuffer;

typedef void (*event_cb_t)(oscFixture);

typedef struct
{
    int64_t target_us; // execution time in microseconds (monotonic)
    event_cb_t callback;
    oscFixture data; // optional argument
} OSC_Event;

typedef struct
{
    float ax_sum, ay_sum, az_sum;
    float ax_min, ay_min, az_min;
    float ax_max, ay_max, az_max;
    float ax_variance, ay_variance, az_variance;
    float ax_mean, ay_mean, az_mean;
    float ax_diff, ay_diff, az_diff;
    float ax_range, ay_range, az_range;
    float x_mag, y_mag, z_mag;
    float x_std, y_std, z_std;
    float x_dom, y_dom, z_dom;
    Direction direction;
} SampleResult;

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
void sampler(void *pvParameters);
static SensorSample template[64] = {
    {0, 0, 0, {0.515934f, 0.434529f, -1.315498f}},
    {0, 0, 0, {2.275398f, 1.309419f, -1.881936f}},
    {0, 0, 0, {4.065137f, 1.620382f, -0.378214f}},
    {0, 0, 0, {5.008082f, 2.250175f, 0.403848f}},
    {0, 0, 0, {3.116725f, 0.026841f, 3.365610f}},
    {0, 0, 0, {1.205442f, -1.484193f, 4.813375f}},
    {0, 0, 0, {-1.232596f, -2.454959f, 5.622409f}},
    {0, 0, 0, {-2.407148f, -2.430534f, 3.850601f}},
    {0, 0, 0, {-2.189280f, -2.148539f, 2.857198f}},
    {0, 0, 0, {-3.210461f, -3.281310f, 1.660294f}},
    {0, 0, 0, {-2.735851f, -2.984207f, 1.457501f}},
    {0, 0, 0, {-3.318381f, -3.224852f, 0.848485f}},
    {0, 0, 0, {-2.198433f, -3.019356f, -0.387088f}},
    {0, 0, 0, {-1.700021f, -2.513060f, -1.249652f}},
    {0, 0, 0, {-1.448377f, -2.160467f, -1.079592f}},
    {0, 0, 0, {-1.774167f, -1.860167f, -1.925772f}},
    {0, 0, 0, {-0.839747f, -1.565743f, -2.987540f}},
    {0, 0, 0, {-1.545378f, -1.208486f, -3.254928f}},
    {0, 0, 0, {-0.054077f, -0.748615f, -3.257933f}},
    {0, 0, 0, {-0.470731f, -0.885120f, -3.208192f}},
    {0, 0, 0, {0.258980f, -0.753250f, -3.737431f}},
    {0, 0, 0, {0.515944f, -0.929309f, -2.696878f}},
    {0, 0, 0, {1.734420f, -0.172833f, -2.836617f}},
    {0, 0, 0, {0.605573f, -1.172851f, -2.748393f}},
    {0, 0, 0, {2.251215f, -0.528906f, -2.511288f}},
    {0, 0, 0, {2.627738f, -0.854336f, -0.990566f}},
    {0, 0, 0, {2.447414f, -0.643223f, -1.057773f}},
    {0, 0, 0, {3.076159f, -0.116641f, -1.542778f}},
    {0, 0, 0, {2.288935f, -0.587274f, -0.652847f}},
    {0, 0, 0, {2.116817f, -0.740490f, -0.045181f}},
    {0, 0, 0, {2.845815f, -0.165648f, 0.966131f}},
    {0, 0, 0, {2.912344f, -0.550055f, 1.301203f}},
    {0, 0, 0, {1.791911f, -0.948022f, 2.262531f}},
    {0, 0, 0, {1.673456f, -0.385793f, 1.175851f}},
    {0, 0, 0, {1.615522f, -0.311060f, 2.316550f}},
    {0, 0, 0, {1.473367f, -0.025644f, 1.610903f}},
    {0, 0, 0, {0.631738f, 0.028205f, 1.851729f}},
    {0, 0, 0, {0.167027f, -0.130717f, 1.955631f}},
    {0, 0, 0, {-0.949701f, -0.643201f, 1.720969f}},
    {0, 0, 0, {-1.911155f, -0.914961f, 0.877097f}},
    {0, 0, 0, {-3.137415f, -0.875119f, 0.393210f}},
    {0, 0, 0, {-2.546921f, -0.840634f, 0.391338f}},
    {0, 0, 0, {-2.398175f, -1.034975f, -0.371794f}},
    {0, 0, 0, {-0.759208f, -0.187378f, -0.483179f}},
    {0, 0, 0, {-0.434099f, 0.190816f, -0.309746f}},
    {0, 0, 0, {-0.041207f, 0.239041f, 0.619967f}},
    {0, 0, 0, {-0.253278f, -0.174419f, 0.442494f}},
    {0, 0, 0, {0.093348f, -0.182827f, 0.208012f}},
    {0, 0, 0, {-0.041418f, 0.000219f, -0.284677f}},
    {0, 0, 0, {0.233788f, 0.101001f, -0.321692f}},
    {0, 0, 0, {-0.118567f, -0.178250f, 0.240465f}},
    {0, 0, 0, {1.112714f, 0.308724f, -0.095137f}},
    {0, 0, 0, {0.003449f, -0.092840f, -0.042301f}},
    {0, 0, 0, {0.415914f, 0.242938f, 0.003417f}},
    {0, 0, 0, {-0.374961f, -0.277178f, 0.139076f}},
    {0, 0, 0, {0.221531f, 0.005158f, -0.103955f}},
    {0, 0, 0, {-0.298042f, -0.152861f, -0.259699f}},
    {0, 0, 0, {0.384979f, 0.009128f, 0.290164f}},
    {0, 0, 0, {-0.140100f, 0.048520f, -0.378643f}},
    {0, 0, 0, {0.294494f, 0.163830f, 0.161612f}},
    {0, 0, 0, {-0.114326f, -0.065895f, -0.063911f}},
    {0, 0, 0, {0.050609f, 0.052857f, 0.015669f}},
    {0, 0, 0, {-0.269161f, -0.129941f, 0.074886f}},
    {1, 73149881, 7250305, {0.233899f, 0.199894f, 0.017919f}}};

#endif
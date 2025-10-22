#ifndef DATATYPES_H
#define DATATYPES_H
#include <stdint.h>
#include <tinyosc.h>
#define BUFFER_SIZE_BYTES 1024
#define MAX_TOKENS 128

#define SAMPLE_RATE 50
#define BUFFER_SECONDS 2
#define BUFFER_SIZE (SAMPLE_RATE * BUFFER_SECONDS)
#define VECTOR_SIZE 3

#define GAME_ROTATION 0
#define LINEAR_ACCEL 1
#define UNKNOWN 255

typedef enum
{
GAMERGB,
CTRLLEFT,
CTRLRIGHT
} oscType;


typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} colorSample;
typedef struct {
    colorSample color;
    oscType type;
} oscSample;

typedef struct {
    uint8_t type;
    int64_t timestamp;
    int64_t received_ms; // processing timestamp in ms
    float values[VECTOR_SIZE];
} SensorSample;

typedef struct {
    SensorSample samples[BUFFER_SIZE];
    int head; // next write position
} SensorBuffer;

void netprocess(void *pvParameters);
void detectorTask(void *params);
int64_t get_current_ms(void);
void gameTask(void *pv);
void oscTask(void *pv);


#endif
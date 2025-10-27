// Three buffers for different types
#ifndef DETECTOR_H
#define DETECTOR_H

typedef struct
{
    float alpha;
    float prev_input;
    float prev_output;
} HighpassFilter;

void HighpassFilter_Init(HighpassFilter *f, float cutoff, float fs)
{
    float RC = 1.0f / (2.0f * 3.1415926f * cutoff);
    float dt = 1.0f / fs;
    f->alpha = RC / (RC + dt);
    f->prev_input = 0;
    f->prev_output = 0;
}

float HighpassFilter_Update(HighpassFilter *f, float input)
{
    float out = f->alpha * (f->prev_output + input - f->prev_input);
    f->prev_input = input;
    f->prev_output = out;
    return out;
}

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
    NOTKNOWN
} Direction;

#define MAXITEMS 1 // 0..1..
#define CUTOFF 0.2f     // Hz
#define THRESH 4.0f     // g units (filter out gentle movements)
#define MIN_LEN 3       // min consecutive samples for gesture (reduced for faster detection)

#endif
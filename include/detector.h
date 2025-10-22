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




#define CUTOFF 0.1f     // Hz
#define THRESH 1.5f     // g units (filter out gentle movements)
#define MIN_LEN 3       // min consecutive samples for gesture (reduced for faster detection)
#define HISTORY_SIZE 64 // history of previous samples for sliding detection

#endif
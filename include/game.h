#ifndef GAME_H
#define GAME_H
#include <datatypes.h>
static inline float fmaxf_safe(float a, float b) { return (a > b) ? a : b; }
static inline float fminf_safe(float a, float b) { return (a < b) ? a : b; }
#define PI 3.14159265358979323846
double degrees(double radians)
{
    return radians * (180.0 / M_PI);
}

static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s <= 0.0f)
    {
        long lv = lroundf(v * 255.0f);
        lv = (lv < 0) ? 0 : (lv > 255) ? 255
                                       : lv;
        *r = *g = *b = (uint8_t)lv;
        return;
    }
    h = fmodf(h, 360.0f);
    if (h < 0.0f)
        h += 360.0f;
    h /= 60.0f;
    int i = (int)floorf(h);
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    long lr, lg, lb;
    switch (i)
    {
    case 0:
        lr = lroundf(v * 255);
        lr = (lr < 0) ? 0 : (lr > 255) ? 255
                                       : lr;
        lg = lroundf(t * 255);
        lg = (lg < 0) ? 0 : (lg > 255) ? 255
                                       : lg;
        lb = lroundf(p * 255);
        lb = (lb < 0) ? 0 : (lb > 255) ? 255
                                       : lb;
        break;
    case 1:
        lr = lroundf(q * 255);
        lr = (lr < 0) ? 0 : (lr > 255) ? 255
                                       : lr;
        lg = lroundf(v * 255);
        lg = (lg < 0) ? 0 : (lg > 255) ? 255
                                       : lg;
        lb = lroundf(p * 255);
        lb = (lb < 0) ? 0 : (lb > 255) ? 255
                                       : lb;
        break;
    case 2:
        lr = lroundf(p * 255);
        lr = (lr < 0) ? 0 : (lr > 255) ? 255
                                       : lr;
        lg = lroundf(v * 255);
        lg = (lg < 0) ? 0 : (lg > 255) ? 255
                                       : lg;
        lb = lroundf(t * 255);
        lb = (lb < 0) ? 0 : (lb > 255) ? 255
                                       : lb;
        break;
    case 3:
        lr = lroundf(p * 255);
        lr = (lr < 0) ? 0 : (lr > 255) ? 255
                                       : lr;
        lg = lroundf(q * 255);
        lg = (lg < 0) ? 0 : (lg > 255) ? 255
                                       : lg;
        lb = lroundf(v * 255);
        lb = (lb < 0) ? 0 : (lb > 255) ? 255
                                       : lb;
        break;
    case 4:
        lr = lroundf(t * 255);
        lr = (lr < 0) ? 0 : (lr > 255) ? 255
                                       : lr;
        lg = lroundf(p * 255);
        lg = (lg < 0) ? 0 : (lg > 255) ? 255
                                       : lg;
        lb = lroundf(v * 255);
        lb = (lb < 0) ? 0 : (lb > 255) ? 255
                                       : lb;
        break;
    case 5:
        lr = lroundf(v * 255);
        lr = (lr < 0) ? 0 : (lr > 255) ? 255
                                       : lr;
        lg = lroundf(p * 255);
        lg = (lg < 0) ? 0 : (lg > 255) ? 255
                                       : lg;
        lb = lroundf(q * 255);
        lb = (lb < 0) ? 0 : (lb > 255) ? 255
                                       : lb;
        break;
    }
    *r = (uint8_t)lr;
    *g = (uint8_t)lg;
    *b = (uint8_t)lb;
}

positionSample quaternion_to_euler_ypr(float x, float y, float z, float w)
{
    // Returns yaw (Z), pitch (Y), roll (X) in radians
    // Formulas from standard quaternion to Tait-Bryan Z-Y-X
    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    double pitch = 0.0;
    // # pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (abs(sinp) >= 1)
    {
        pitch = copysign(PI / 2.0, sinp);
    }
    else
    {
        pitch = asin(sinp);
    }
    // # roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(sinr_cosp, cosr_cosp);
    positionSample pos = {.yaw = yaw, .pitch = pitch, .roll = roll};

    return pos;
}

int map_yaw_circular_deg(float yaw_deg, float offset_deg)
{
    float a = fmodf(yaw_deg - offset_deg, 360.0f);
    if (a < 0.0f)
        a += 360.0f;
    return (int)roundf(a * (255.0f / 360.0f));
}

float minimal_angle_diff_deg(float a, float b)
{
    float d = fmodf(a - b + 180.0f, 360.0f);
    if (d < 0)
        d += 360.0f;   // ensure it's in [0, 360)
    return d - 180.0f; // now in (-180, 180]
}

#endif
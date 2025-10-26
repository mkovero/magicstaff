#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
extern QueueHandle_t gameQueue;
extern QueueHandle_t oscQueue;
static inline float fmaxf_safe(float a, float b) { return (a > b) ? a : b; }
static inline float fminf_safe(float a, float b) { return (a < b) ? a : b; }
#define PI 3.14159265358979323846

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
    if (d < 0) d += 360.0f;  // ensure it's in [0, 360)
    return d - 180.0f;        // now in (-180, 180]

}
uint8_t scale_to_8bit(float value, float rng, bool use_abs)
{
    float v;
    float safe_rng = fmaxf_safe(rng, 1e-9f); // avoid divide by zero
    float scaled;
    if (use_abs)
    {
        v = fabsf(value);
        v = fminf_safe(fmaxf_safe(v, 0.0f), rng); // clamp to [0, rng]
        scaled = 255.0f * (v / safe_rng);         // scale 0–rng → 0–255
    }
    else
    {
        v = fminf_safe(fmaxf_safe(value, -rng), rng);      // clamp to [-rng, rng]
        scaled = (v + rng) * (255.0f / (2.0f * safe_rng)); // scale -rng–rng → 0–255
    }

    // Clamp final result just in case of rounding
    if (scaled < 0.0f)
        scaled = 0.0f;
    if (scaled > 255.0f)
        scaled = 255.0f;

    return (uint8_t)lroundf(scaled);
}
double degrees(double radians)
{
    return radians * (180.0 / M_PI);
}

colorSample gyroColor(float values[3])
{
    positionSample pos;
    colorSample color;
    float qx = values[0];
    float qy = values[1];
    float qz = values[2];
    float w2 = fmax(0.0f, 1.0f - (qx * qx + qy * qy + qz * qz));
    float qw = sqrtf(w2);
    //pos = quaternion_to_euler_ypr(qx, qy, qz, qw);
    //        printf("Position after quaternion: %.2f, %.2f, %.2f\n", pos.yaw, pos.pitch, pos.roll);
    pos.yaw = qx;
    pos.pitch = qy;
    pos.roll = qz;
    float yaw_deg = degrees(pos.yaw);
    float pitch_deg = degrees(pos.pitch);
    float roll_deg = degrees(pos.roll);
    float grv_ema = 0.9f;
    //printf("Gyro converted: %.2f(x) -> %.2f, %.2f(y) -> %.2f, %.2f(z) -> %.2f, 0(w) -> %.2f \n", qx, yaw_deg, qy, pitch_deg, qz, roll_deg, qw);
    static float prev_yaw_deg = 69.69f;
    static float prev_pitch_deg = 69.69f;
    static float prev_roll_deg = 69.69f;
    static bool yaw_init = false;
    static bool pitch_init = false;
    static bool roll_init = false;
    const float yaw_offset = 0.0f;
    const float yaw_deadband = 0.0f;
    const float yaw_range = 60.0f;
    const float pitch_offset = 0.0f;
    const float pitch_deadband = 0.0f;
    const float pitch_range = 90.0f;
    const float roll_offset = 0.0f;
    const float roll_deadband = 0.0f;
    const float roll_range = 90.0f;
    // New: Adjustable yaw calibration

    if (grv_ema > 0.0f)
    {
        if (!yaw_init)
        {
            prev_yaw_deg = yaw_deg;
            yaw_init = true;
        }
        else
        {
            float yaw_delta = yaw_deg - prev_yaw_deg; // Use full delta, not minimal, to allow unwinding
            prev_yaw_deg += grv_ema * yaw_delta;
        }
        yaw_deg = prev_yaw_deg;
        if (!pitch_init)
        {
            prev_pitch_deg = pitch_deg;
            pitch_init = true;
        }
        else
        {
            prev_pitch_deg = prev_pitch_deg + grv_ema * (pitch_deg - prev_pitch_deg);
        }
        pitch_deg = prev_pitch_deg;
        if (!roll_init)
        {
            prev_roll_deg = roll_deg;
            roll_init = true;
        }
        else
        {
            prev_roll_deg = prev_roll_deg + grv_ema * (roll_deg - prev_roll_deg);
        }
        roll_deg = prev_roll_deg;
    }
    const bool distance = true;
    const bool circular = false;

    float dist = 0.0;
    float rdist = 0.0;
    float pdist = 0.0;

    float hue = 0.0;
    if (distance)
    {
        dist = fabs(minimal_angle_diff_deg(yaw_deg, yaw_offset));
        if (dist < yaw_deadband)
        {
            dist = 0.0;
        }
        pdist = fabs(minimal_angle_diff_deg(pitch_deg, pitch_offset));
        if (pdist < pitch_deadband)
        {
            pdist = 0.0;
        }
        rdist = fabs(minimal_angle_diff_deg(roll_deg, roll_offset));
        if (rdist < roll_deadband)
        {
            rdist = 0.0;
        }
    }

    pdist = fminf_safe(pdist, pitch_range); // Cap at range max
    rdist = fminf_safe(rdist, roll_range);  // Cap at range max
 /*   if (circular)
    {
        float a = fmodf(yaw_deg - yaw_offset, 360.0f);
        if (a < 0.0f)
            a += 360.0f;
        hue = a; // Direct 0-360 for full rotation cycle
    }
    else
    {
        dist = fminf_safe(dist, yaw_range);
        hue = dist * (360.0f / yaw_range); // Optional: Non-circular, scale deviation to 0-360
    }*/

    // Always vibrant mode
    /* float sat = 1.0f;
      float combined_max = fmaxf_safe(pdist / pitch_range, rdist / roll_range); // Now <=1.0
      float val = combined_max;
      hsv_to_rgb(hue, sat, val, &color.r, &color.g, &color.b);
      printf("Asked for hue:%.2f sat:%.2f val:%.2f and got colors %d/%d/%d", hue, sat, val, color.r, color.g, color.b);*/
    color.r = scale_to_8bit(dist, 60.0, true);
    color.g = scale_to_8bit(pdist, 90.0, true);
    color.b = scale_to_8bit(rdist, 90.0, true);

    return color;
}

void gameTask(void *pv)
{
    printf("Game processor Started\n");
    while (gameQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (oscQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    };
    while (1)
    {
        SensorSample game;
        oscSample osc;
        if (xQueueReceive(gameQueue, &game, portMAX_DELAY))
        {
            osc.color = gyroColor(game.values);
            osc.type = GAMERGB;
            osc.delay = 200;
          //  printf("Game sample: timestamp=%ld, values=[%.2f, %.2f, %.2f]->[%d, %d, %d]\n", game.timestamp, game.values[0], game.values[1], game.values[2], osc.color.r, osc.color.g, osc.color.b);

            if (xQueueSend(oscQueue, &osc, portMAX_DELAY) != pdPASS)
            {
                printf("Game->OSC send failed\n");
            }
        }
        //  vTaskDelay(pdMS_TO_TICKS(20));
    }
}

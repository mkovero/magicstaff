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
    return (int)roundf(a * (255.0f / 360.0f));
}

float minimal_angle_diff_deg(float a, float b)
{
    // Signed minimal difference a-b in (-180, 180]
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return d;
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
    pos = quaternion_to_euler_ypr(qx, qy, qz, qw);
    float yaw_deg = degrees(pos.yaw);
    float pitch_deg = degrees(pos.pitch);
    float roll_deg = degrees(pos.roll);
    float grv_ema = 0.8;
    printf("Gyro converted: %.2f(x) -> %.2f, %.2f(y) -> %.2f, %.2f(z) -> %.2f, 0(w) -> %.2f \n", qx, yaw_deg, qy, pitch_deg, qz, roll_deg, qw);
    static float prev_yaw_deg = 69.69f;
    static float prev_pitch_deg = 69.69f;
    static float prev_roll_deg = 69.69f;

    if (grv_ema > 0.0)
    {
        if (prev_yaw_deg == 69.69)
        {
            prev_yaw_deg = yaw_deg;
        }
        else
        {
            float yaw_delta = minimal_angle_diff_deg(yaw_deg, prev_yaw_deg);
            prev_yaw_deg = prev_yaw_deg + grv_ema + yaw_delta;
        }
        yaw_deg = prev_yaw_deg;
        if (prev_pitch_deg == 69.69)
        {
            prev_pitch_deg = pitch_deg;
        }
        else
        {
            prev_pitch_deg = prev_pitch_deg + grv_ema * (pitch_deg - prev_pitch_deg);
        }
        pitch_deg = prev_pitch_deg;
        if (prev_roll_deg == 69.69)
        {
            prev_roll_deg = roll_deg;
        }
        else
        {
            prev_roll_deg = prev_roll_deg + grv_ema * (roll_deg - prev_roll_deg);
        }
        roll_deg = prev_roll_deg;
    }
    const bool distance = true;
    const bool circular = true;
    const float yaw_offset = 0.0f;
    const float yaw_deadband = 2.0f;
    const float yaw_range = 60.0f;
    const float pitch_offset = 0.0f;
    const float pitch_deadband = 2.0f;
    const float pitch_range = 60.0f;
    const float roll_offset = 0.0f;
    const float roll_deadband = 2.0f;
    const float roll_range = 60.0f;

    if (distance) {
        float dist = fabs(minimal_angle_diff_deg(yaw_deg, yaw_offset));
        if (dist < yaw_deadband) {
            dist = 0.0;
        }
        float pdist = fabs(minimal_angle_diff_deg(pitch_deg, pitch_offset));
        if (pdist < pitch_deadband) {
            pdist = 0.0;
        }
        float rdist = fabs(minimal_angle_diff_deg(roll_deg, roll_offset));
        if (rdist < roll_deadband) {
            rdist = 0.0;
        }
        color.g = scale_to_8bit(pitch_deg, pitch_range, true);
        color.b = scale_to_8bit(roll_deg,roll_range, true);
    } 
    if (circular) {
         color.r = map_yaw_circular_deg(yaw_deg, yaw_offset);
    } else {
        color.r = scale_to_8bit(yaw_deg, yaw_range, true);
    }

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
             printf("Game sample: timestamp=%ld, values=[%.2f, %.2f, %.2f]->[%d, %d, %d]\n", game.timestamp, game.values[0], game.values[1], game.values[2], osc.color.r, osc.color.g, osc.color.b);

            if (xQueueSend(oscQueue, &osc, portMAX_DELAY) != pdPASS)
            {
                printf("Game->OSC send failed\n");
            }
        }
        //  vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <game.h>
extern QueueHandle_t gameQueue;
extern QueueHandle_t oscQueue;



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

colorSample gyroColor(float values[3])
{
    positionSample pos;
    colorSample color;
    float qx = values[0];
    float qy = values[1];
    float qz = values[2];
    float w2 = fmax(0.0f, 1.0f - (qx * qx + qy * qy + qz * qz));
    float qw = sqrtf(w2);
    // pos = quaternion_to_euler_ypr(qx, qy, qz, qw);
    //         printf("Position after quaternion: %.2f, %.2f, %.2f\n", pos.yaw, pos.pitch, pos.roll);
    pos.yaw = qx;
    pos.pitch = qy;
    pos.roll = qz;
    float yaw_deg = degrees(pos.yaw);
    float pitch_deg = degrees(pos.pitch);
    float roll_deg = degrees(pos.roll);
    float grv_ema = 0.9f;
    // printf("Gyro converted: %.2f(x) -> %.2f, %.2f(y) -> %.2f, %.2f(z) -> %.2f, 0(w) -> %.2f \n", qx, yaw_deg, qy, pitch_deg, qz, roll_deg, qw);
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
    bool simpleMode = true;
    while (1)
    {
        SensorSample game;
        oscSample osc;
        if (xQueueReceive(gameQueue, &game, portMAX_DELAY))
        {
            colorSample color;
            if (!simpleMode)
            {

                osc.color = gyroColor(game.values);
            }
            else
            {
                color.r = scale_to_8bit(game.values[0], 0.6, true);
                color.g = scale_to_8bit(game.values[1], 0.6, true);
                color.b = scale_to_8bit(game.values[2], 0.6, true);

                osc.color = color;
            }
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

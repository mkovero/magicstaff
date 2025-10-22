#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <math.h>
#include <stdio.h>
extern QueueHandle_t gameQueue;
extern QueueHandle_t oscQueue;
static inline float fmaxf_safe(float a, float b) { return (a > b) ? a : b; }
static inline float fminf_safe(float a, float b) { return (a < b) ? a : b; }
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

colorSample gyroToRGB(float *values)
{
    float *v = values;
    colorSample color;

    for (int i = 0; i < VECTOR_SIZE; i++)
    {
        color.r = scale_to_8bit(values[0], 1, true);
        color.g = scale_to_8bit(values[1], 1, true);
        color.b = scale_to_8bit(values[2], 1, true);
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
            osc.color = gyroToRGB(game.values);
            osc.type = GAMERGB;
            // printf("Game sample: timestamp=%ld, values=[%.2f, %.2f, %.2f]->[%d, %d, %d]\n", game.timestamp, game.values[0], game.values[1], game.values[2], color.r, color.g, color.b);

            if (xQueueSend(oscQueue, &osc, portMAX_DELAY) != pdPASS)
            {
                printf("Game->OSC send failed\n");
            }
        }
        //  vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "detector.h"
#include <time.h>
#include <game.h>
extern QueueHandle_t accelQueue;
extern QueueHandle_t oscQueue;
extern QueueHandle_t gestureQueue;

extern gestureState gesture;

// #define MAX_RECORD_GESTURE 256

static SampleResult classify_gesture(SensorSample *history, int start, int end)
{
    SampleResult result = {
        .ax_sum = 0.0f,
        .ay_sum = 0.0f,
        .az_sum = 0.0f,
        .ax_variance = 0.0f,
        .ay_variance = 0.0f,
        .az_variance = 0.0f,
        .ax_min = history[start].values[0],
        .ax_max = history[start].values[0],
        .ay_min = history[start].values[1],
        .ay_max = history[start].values[1],
        .az_min = history[start].values[2],
        .az_max = history[start].values[2],

    };

    if (end <= start)
    {
        result.direction = NOTKNOWN;
        return result;
    }

    int count = end - start;
    if (count < 3)
    {
        result.direction = NOTKNOWN;
        return result;
    }

    // First pass: calculate mean, min, max
    for (int i = start; i < end; i++)
    {
        result.ax_sum += history[i].values[0];
        result.ay_sum += history[i].values[1];
        result.az_sum += history[i].values[2];

        if (history[i].values[0] < result.ax_min)
            result.ax_min = history[i].values[0];
        if (history[i].values[0] > result.ax_max)
            result.ax_max = history[i].values[0];
        if (history[i].values[1] < result.ay_min)
            result.ay_min = history[i].values[1];
        if (history[i].values[1] > result.ay_max)
            result.ay_max = history[i].values[1];
        if (history[i].values[2] < result.az_min)
            result.az_min = history[i].values[2];
        if (history[i].values[2] > result.az_max)
            result.az_max = history[i].values[2];
    }

    result.ax_mean = result.ax_sum / count;
    result.ay_mean = result.ay_sum / count;
    result.az_mean = result.az_sum / count;

    // Second pass: calculate variance
    for (int i = start; i < end; i++)
    {
        result.ax_diff = history[i].values[0] - result.ax_mean;
        result.ay_diff = history[i].values[1] - result.ay_mean;
        result.az_diff = history[i].values[2] - result.az_mean;

        result.ax_variance += result.ax_diff * result.ax_diff;
        result.ay_variance += result.ay_diff * result.ay_diff;
        result.az_variance += result.az_diff * result.az_diff;
    }
    result.ax_variance /= count;
    result.ay_variance /= count;
    result.az_variance /= count;

    // Calculate ranges and magnitudes
    result.ax_range = result.ax_max - result.ax_min;
    result.ay_range = result.ay_max - result.ay_min;
    result.az_range = result.az_max - result.az_min;

    result.x_mag = fabsf(result.ax_mean);
    result.y_mag = fabsf(result.ay_mean);
    result.z_mag = fabsf(result.az_mean);

    result.x_std = sqrtf(result.ax_variance);
    result.y_std = sqrtf(result.ay_variance);
    result.z_std = sqrtf(result.az_variance);

    // Improved classification logic (verbose analysis disabled for cleaner output)
    // printf("Gesture analysis: result.ax_range=%.3f, result.ay_range=%.3f, x_mag=%.3f, y_mag=%.3f, x_std=%.3f, y_std=%.3f\n",
    //        result.ax_range, result.ay_range, result.x_mag, result.y_mag, x_stddev, y_stddev);

    // Determine dominant axis based on both range and magnitude
    result.x_dom = result.ax_range * result.x_mag;
    result.y_dom = result.ay_range * result.y_mag;
    result.z_dom = result.az_range * result.z_mag;

    // Minimum threshold to avoid noise
    const float MIN_THRESHOLD = 0.5f;
    if (result.x_dom < MIN_THRESHOLD && result.y_dom < MIN_THRESHOLD)
    {
        result.direction = WEAK;
        return result;
    }

    const float SHAKE_THRESHOLD = 250.0f;
    static uint8_t shakeCount = 0;
    static int64_t lastShake = 0;
    const int64_t shakeLifetime = 1000;
    if ((get_current_ms() - lastShake) > shakeLifetime)
    {
        shakeCount = 0;
    }

    if (result.x_dom > SHAKE_THRESHOLD || result.y_dom > SHAKE_THRESHOLD)
    {
        shakeCount++;
        // printf("Shake detected (%d) %.2f/%.2f\n", shakeCount, result.x_dom, result.y_dom);
        lastShake = get_current_ms();
    }
    if (shakeCount > 3)
    {
        //    printf("Shake triggered\n");
        shakeCount = 0;
        result.direction = SHAKE;
        return result;
    }
    // Classify based on dominant axis and direction
    if (result.x_dom > result.y_dom * 1.2f)
    { // X-axis dominant (with some tolerance)
        if (result.ax_mean > 0)
        {
            result.direction = RIGHT;
            return result;
        }
        else
        {
            result.direction = LEFT;
            return result;
        }
    }
    else if (result.y_dom > result.x_dom * 1.2f)
    { // Y-axis dominant
        if (result.ay_mean > 0)
        {
            result.direction = UP;
            return result;
        }
        else
        {
            result.direction = DOWN;
            return result;
        }
    }
    /* else
     {
         // Mixed or diagonal gesture
         if (result.ax_mean > 0 && ay_mean > 0)
             return UPRIGHT;
         else if (result.ax_mean < 0 && ay_mean > 0)
             return UPLEFT;
         else if (result.ax_mean > 0 && ay_mean < 0)
             return DOWNRIGHT;
         else if (result.ax_mean < 0 && ay_mean < 0)
             return DOWNLEFT;
         else
             return MIXED;
     }*/
}

void gestureReact()
{
    TickType_t currentTime = xTaskGetTickCount();
    static uint8_t item = 0;

    gestureState *gesture;
    xQueuePeek(gestureQueue, &gesture, portMAX_DELAY);
    oscSample osc;
    osc.delay = 200;
    SampleResult result = classify_gesture(gesture->history, 0, gesture->history_count);

    // Gesture is still active and long enough - classify it
    switch (result.direction)
    {
    case LEFT:
        if ((item >= 0) && atomic_load(&gesture->locked) && ((currentTime - gesture->lastGesture) > gesture->gestureCooldown) && !gesture->reallyLocked)
        {
            if (item > 0)
            {
                item--;
            }
            atomic_store(&gesture->item, item); // write
            osc.type = CTRLLEFT;
        }
        break;
    case RIGHT:
        if ((item <= MAXITEMS) && atomic_load(&gesture->locked) && ((currentTime - gesture->lastGesture) > gesture->gestureCooldown) && !gesture->reallyLocked)
        {
            if (item < MAXITEMS)
            {
                item++;
            }
            atomic_store(&gesture->item, item); // write
            osc.type = CTRLRIGHT;
        }
        break;
    case DOWN:
        if (!gesture->reallyLocked && ((currentTime - gesture->lastGesture) > gesture->gestureCooldown))
        {
            if (atomic_load(&gesture->locked))
            {
                atomic_store(&gesture->locked, false);
                osc.type = CTRLBOTH;
                printf("Control unlocked\n");
            }
        }
        break;
    case UP:
        if (!gesture->reallyLocked && ((currentTime - gesture->lastGesture) > gesture->gestureCooldown))
        {
            if (!atomic_load(&gesture->locked))
            {
                atomic_store(&gesture->locked, true);
                osc.type = CTRLBOTH;
                printf("Control locked\n");
            }
        }
        break;
    case SHAKE:
        if (atomic_load(&gesture->locked) && !gesture->reallyLocked)
        {
            gesture->reallyLocked = true;
            osc.type = CTRLBOTH;
            printf("Control really locked\n");
        }
        else if (gesture->reallyLocked)
        {
            gesture->reallyLocked = false;
            osc.type = CTRLBOTH;
            printf("Control really unlocked\n");
        }

        break;
    default:
        break;
    }
    gesture->lastGesture = xTaskGetTickCount();
    xQueueSend(oscQueue, &osc, portMAX_DELAY);
}

void detectorTask(void *params)
{
    SensorBuffer *buf;
    HighpassFilter fx, fy, fz;

    while (gestureQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    };

    double fs = 50.0;

    // Gesture tracking state (persistent across processing cycles)

    historySample tracking[BUFFER_SIZE];

    static gestureState gesture = {
        .lastGesture = 0,
        .gestureCooldown = 500,
        .locked = false,
        .reallyLocked = false,
        .item = 0,
        .still_time = 500,
    };
    gestureState *ptr = &gesture;
    xQueueSend(gestureQueue, &ptr, portMAX_DELAY);

    for (;;)
    {
        if (xQueueReceive(accelQueue, &buf, portMAX_DELAY) == pdPASS)
        {
            int64_t now_ms = get_current_ms();

            if (buf->head >= 2)
            {
                // Estimate sampling frequency from timestamp differences
                double dt_sum = 0;
                int valid_samples = 0;
                for (int i = 1; i < buf->head; i++)
                {
                    int64_t dt_ms = (buf->samples[i].timestamp - buf->samples[i - 1].timestamp) / 1000000;
                    if (dt_ms > 0 && dt_ms < 1000)
                    {                             // reasonable sample interval (1ms to 1s)
                        dt_sum += dt_ms / 1000.0; // convert to seconds
                        valid_samples++;
                    }
                }
                if (valid_samples > 0)
                {
                    fs = valid_samples / dt_sum;
                }
                printf("Estimated sampling frequency: %.2f Hz (from %d valid intervals)\n", fs, valid_samples);
            }

            // Initialize filters
            HighpassFilter_Init(&fx, CUTOFF, fs);
            HighpassFilter_Init(&fy, CUTOFF, fs);
            HighpassFilter_Init(&fz, CUTOFF, fs);

            // Filter samples & compute magnitude
            for (int i = 0; i < buf->head; i++)
            {
                tracking[i].data.values[0] = HighpassFilter_Update(&fx, buf->samples[i].values[0]);
                tracking[i].data.values[1] = HighpassFilter_Update(&fy, buf->samples[i].values[1]);
                tracking[i].data.values[2] = HighpassFilter_Update(&fz, buf->samples[i].values[2]);

                tracking[i].mag = sqrtf(tracking[i].data.values[0] * tracking[i].data.values[0] + tracking[i].data.values[1] * tracking[i].data.values[1] + tracking[i].data.values[2] * tracking[i].data.values[2]);
                tracking[i].above = (tracking[i].mag > THRESH) ? 1 : 0;

                if (tracking[i].above)
                {
                    gesture.still_start = 0;
                    if (!atomic_load(&gesture.active))
                    {
                        // Gesture started
                        atomic_store(&gesture.active, true);
                        gesture.start_sample = gesture.total_samples + i;
                        gesture.above_count = 1;
                        gesture.history_count = 0;
                        printf("🎬 Gesture started at sample %ld (mag=%.3f)\n",
                               gesture.total_samples + i, tracking[i].mag);
                    }
                    else
                    {
                        // Gesture continuing
                        gesture.above_count++;
                        // printf("🎬 Gesture continuing at sample %ld (count=%d, mag=%.3f)\n",
                        //      total_samples + i, gesture_above_count, mag[i]);
                    }

                    // Store this sample in history (rolling buffer)
                    if (gesture.history_count < HISTORY_SIZE)
                    {
                        gesture.history[gesture.history_count] = tracking[i].data;
                        gesture.history_count++;
                    }
                }
                else
                {
                    if (!gesture.still_start)
                    {
                        gesture.still_start = now_ms;
                    }
                    if (atomic_load(&gesture.active) && gesture.above_count >= MIN_LEN && ((now_ms - gesture.still_start) > gesture.still_time))
                    {
                        gestureReact();
                        resample(gesture.history, gesture.history_count, gesture.normalized, HISTORY_SIZE);
                        float distance = dtw_distance(gesture.normalized, 64, template, 64);
                        if (distance < 1.2)
                        {
                            printf("Pretty sure its a circle\n");
                        }
                        printf("Distance is %.2f count %d\n", distance, gesture.history_count);
                        atomic_store(&gesture.active, false);
                        gesture.above_count = 0;
                        gesture.history_count = 0;
                    }
                }
            }

            gesture.total_samples += buf->head;

            // Check if there's an ongoing gesture that needs to be completed

            // Reset buffer for next batch (this is a circular buffer, so we reset head)
            buf->head = 0;
        }
    }
}

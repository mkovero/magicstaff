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
extern QueueHandle_t accelQueue;
extern QueueHandle_t oscQueue;
extern QueueHandle_t gestureQueue;

extern gestureState gesture;

static Direction classify_gesture(float *ax, float *ay, int start, int end)
{
    if (end <= start)
        return NOTKNOWN;

    int count = end - start;
    if (count < 3) // Need at least 3 samples for meaningful classification
        return NOTKNOWN;

    // Calculate statistics for better classification
    float ax_sum = 0.0f, ay_sum = 0.0f;
    float ax_min = ax[start], ax_max = ax[start];
    float ay_min = ay[start], ay_max = ay[start];
    float ax_variance = 0.0f, ay_variance = 0.0f;

    // First pass: calculate mean, min, max
    for (int i = start; i < end; i++)
    {
        ax_sum += ax[i];
        ay_sum += ay[i];

        if (ax[i] < ax_min)
            ax_min = ax[i];
        if (ax[i] > ax_max)
            ax_max = ax[i];
        if (ay[i] < ay_min)
            ay_min = ay[i];
        if (ay[i] > ay_max)
            ay_max = ay[i];
    }

    float ax_mean = ax_sum / count;
    float ay_mean = ay_sum / count;

    // Second pass: calculate variance
    for (int i = start; i < end; i++)
    {
        float ax_diff = ax[i] - ax_mean;
        float ay_diff = ay[i] - ay_mean;
        ax_variance += ax_diff * ax_diff;
        ay_variance += ay_diff * ay_diff;
    }
    ax_variance /= count;
    ay_variance /= count;

    // Calculate ranges and magnitudes
    float x_range = ax_max - ax_min;
    float y_range = ay_max - ay_min;
    float x_magnitude = fabsf(ax_mean);
    float y_magnitude = fabsf(ay_mean);
    float x_stddev = sqrtf(ax_variance);
    float y_stddev = sqrtf(ay_variance);

    // Improved classification logic (verbose analysis disabled for cleaner output)
    // printf("Gesture analysis: x_range=%.3f, y_range=%.3f, x_mag=%.3f, y_mag=%.3f, x_std=%.3f, y_std=%.3f\n",
    //        x_range, y_range, x_magnitude, y_magnitude, x_stddev, y_stddev);

    // Determine dominant axis based on both range and magnitude
    float x_dominance = x_range * x_magnitude;
    float y_dominance = y_range * y_magnitude;

    // Minimum threshold to avoid noise
    const float MIN_THRESHOLD = 0.5f;
    if (x_dominance < MIN_THRESHOLD && y_dominance < MIN_THRESHOLD)
        return WEAK;

    const float SHAKE_THRESHOLD = 250.0f;
    static uint8_t shakeCount = 0;
    static int64_t lastShake = 0;
    const int64_t shakeLifetime = 1000;
    if ((get_current_ms() - lastShake) > shakeLifetime)
    {
        shakeCount = 0;
    }

    if (x_dominance > SHAKE_THRESHOLD || y_dominance > SHAKE_THRESHOLD)
    {
        shakeCount++;
        // printf("Shake detected (%d) %.2f/%.2f\n", shakeCount, x_dominance, y_dominance);
        lastShake = get_current_ms();
    }
    if (shakeCount > 3)
    {
        //    printf("Shake triggered\n");
        shakeCount = 0;
        return SHAKE;
    }
    // Classify based on dominant axis and direction
    if (x_dominance > y_dominance * 1.2f)
    { // X-axis dominant (with some tolerance)
        if (ax_mean > 0)
            return RIGHT;
        else
            return LEFT;
    }
    else if (y_dominance > x_dominance * 1.2f)
    { // Y-axis dominant
        if (ay_mean > 0)
            return UP;
        else
            return DOWN;
    }
    else
    {
        // Mixed or diagonal gesture
        if (ax_mean > 0 && ay_mean > 0)
            return UPRIGHT;
        else if (ax_mean < 0 && ay_mean > 0)
            return UPLEFT;
        else if (ax_mean > 0 && ay_mean < 0)
            return DOWNRIGHT;
        else if (ax_mean < 0 && ay_mean < 0)
            return DOWNLEFT;
        else
            return MIXED;
    }
}


void gestureReact()
{
    TickType_t currentTime = xTaskGetTickCount();
    static uint8_t item = 0;
    gestureState *gesture;
    xQueuePeek(gestureQueue, &gesture, portMAX_DELAY);
    oscSample osc;
    osc.delay = 200;
    Direction direction = classify_gesture(gesture->ax_history, gesture->ay_history, 0, gesture->history_count);
    // Gesture is still active and long enough - classify it
    switch (direction)
    {
    case LEFT:
        if ((item >= 0) && atomic_load(&gesture->locked) && ((currentTime - gesture->lastGesture) > gesture->gestureCooldown) && !gesture->reallyLocked)
        {
            if (item > 0)
            {
                item--;
            }
            atomic_store(&gesture->item, item);   // write
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
            atomic_store(&gesture->item, item);   // write
            osc.type = CTRLRIGHT;
        }
        break;
    case DOWN:
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
            else
            {
                atomic_store(&gesture->locked, false);
                osc.type = CTRLBOTH;
                printf("Control unlocked\n");
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

    trackingStats tracking;

    static gestureState gesture = {
        .lastGesture = 0,
        .gestureCooldown = 500,
        .locked = false,
        .reallyLocked = false,
        .item = 0,
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
                //   printf("Estimated sampling frequency: %.2f Hz (from %d valid intervals)\n", fs, valid_samples);
            }

            // Initialize filters
            HighpassFilter_Init(&fx, CUTOFF, fs);
            HighpassFilter_Init(&fy, CUTOFF, fs);
            HighpassFilter_Init(&fz, CUTOFF, fs);

            // Filter samples & compute magnitude
            for (int i = 0; i < buf->head; i++)
            {
                tracking.ax[i] = HighpassFilter_Update(&fx, buf->samples[i].values[0]);
                tracking.ay[i] = HighpassFilter_Update(&fy, buf->samples[i].values[1]);
                tracking.az[i] = HighpassFilter_Update(&fz, buf->samples[i].values[2]);

                tracking.mag[i] = sqrtf(tracking.ax[i] * tracking.ax[i] + tracking.ay[i] * tracking.ay[i] + tracking.az[i] * tracking.az[i]);
                tracking.above[i] = (tracking.mag[i] > THRESH) ? 1 : 0;

                // Debug: show magnitude values for first few samples (only when debugging)
                // if (i < 3) {
                //     printf("Sample %d: ax=%.3f, ay=%.3f, az=%.3f, mag=%.3f, above=%d (threshold=%.3f)\n",
                //            i, ax[i], ay[i], az[i], mag[i], above[i], THRESH);
                // }
            }

            // Simple gesture detection for each sample above threshold

            // Process samples in the buffer
            int samples_to_process = buf->head;
            // Processing batch silently
            // Sliding window gesture detection across processing cycles
            for (int i = 0; i < samples_to_process; i++)
            {
                if (tracking.above[i])
                {
                    if (!atomic_load(&gesture.active))
                    {
                        // Gesture started
                        atomic_store(&gesture.active, true);
                        gesture.start_sample = gesture.total_samples + i;
                        gesture.above_count = 1;
                        gesture.history_count = 0;
                        // printf("🎬 Gesture started at sample %ld (mag=%.3f)\n",
                        //      total_samples + i, mag[i]);
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
                        gesture.ax_history[gesture.history_count] = tracking.ax[i];
                        gesture.ay_history[gesture.history_count] = tracking.ay[i];
                        gesture.history_count++;
                    }
                }
                else
                {
                    if (atomic_load(&gesture.active))
                    {
                        if (gesture.above_count >= MIN_LEN)
                        {
                            gestureReact();
                        }
                        atomic_store(&gesture.active, false);
                        gesture.above_count = 0;
                        gesture.history_count = 0;
                    }
                }
            }

            gesture.total_samples += samples_to_process;

            // Check if there's an ongoing gesture that needs to be completed
            if (atomic_load(&gesture.active) && gesture.above_count >= MIN_LEN)
            {
                gestureReact();
                atomic_store(&gesture.active, false);
                gesture.above_count = 0;
                gesture.history_count = 0;
            }

            // Reset buffer for next batch (this is a circular buffer, so we reset head)
            buf->head = 0;

            // Periodically show magnitude statistics for debugging
            if (gesture.total_samples % 50 == 0 && gesture.total_samples > 0)
            {
                float max_mag = 0.0f;
                int above_count = 0;
                for (int i = 0; i < samples_to_process; i++)
                {
                    if (tracking.mag[i] > max_mag)
                        max_mag = tracking.mag[i];
                    if (tracking.above[i])
                        above_count++;
                }
                // printf("📊 Stats: max_mag=%.3f, above_threshold=%d/%d, threshold=%.3f\n",
                //      max_mag, above_count, samples_to_process, THRESH);
            }
        }
    }
}

#include <datatypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "health.h"
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "detector.h"
#include <time.h>
extern QueueHandle_t accelQueue;
extern QueueHandle_t oscQueue;

static int64_t offset_ms = 0; // estimated host - sensor offset
static bool first_sample = true;
static const double alpha = 0.01;              // EMA smoothing factor for offset tracking
static const double beta = 0.1;                // EMA smoothing factor for latency estimation
static int64_t estimated_processing_delay = 0; // estimated processing delay in ms

// Gesture tracking state (persistent across processing cycles)
static bool gesture_active = false;
static int gesture_start_sample = 0;
static int gesture_above_count = 0;
static float gesture_ax_history[HISTORY_SIZE];
static float gesture_ay_history[HISTORY_SIZE];
static int gesture_history_count = 0;
extern uint8_t item;
extern gestureState gesture;

static Direction classify_single_sample(float ax_val, float ay_val)
{
    // Simple classification based on single sample values
    float x_magnitude = fabsf(ax_val);
    float y_magnitude = fabsf(ay_val);

    // Minimum threshold to avoid noise
    const float MIN_THRESHOLD = 0.5f;
    if (x_magnitude < MIN_THRESHOLD && y_magnitude < MIN_THRESHOLD)
        return WEAK;

    // Determine dominant axis and direction
    if (x_magnitude > y_magnitude * 1.2f)
    { // X-axis dominant
        if (ax_val > 0)
            return RIGHT;
        else
            return LEFT;
    }
    else if (y_magnitude > x_magnitude * 1.2f)
    { // Y-axis dominant
        if (ay_val > 0)
            return UP;
        else
            return DOWN;
    }
    else
    {
        // Mixed or diagonal gesture
        if (ax_val > 0 && ay_val > 0)
            return UPRIGHT;
        else if (ax_val < 0 && ay_val > 0)
            return UPLEFT;
        else if (ax_val > 0 && ay_val < 0)
            return DOWNRIGHT;
        else if (ax_val < 0 && ay_val < 0)
            return DOWNLEFT;
        else
            return MIXED;
    }
}

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

    const float SHAKE_THRESHOLD = 300.0f;
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
        //printf("Shake detected (%d) %.2f/%.2f\n", shakeCount, x_dominance, y_dominance);
        lastShake = get_current_ms();
    }
    if (shakeCount > 2)
    {
        printf("Shake triggered\n");
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

int64_t get_current_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // Use monotonic time for relative measurements
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}
void update_latency(int64_t now_ms, int64_t received_ms)
{
    int64_t latency = now_ms - received_ms;

    if (first_sample && (latency > 0))
    {
        offset_ms = latency; // initialize EMA with first sample
        first_sample = false;
    }
    else
    {
        offset_ms = (int64_t)((1.0 - alpha) * offset_ms + alpha * latency);
    }

    int64_t compensated_latency = latency - offset_ms;
    // printf("Latency (EMA): now=%ld, recv=%ld, lat=%ld offset=%ld, comp=%ld ms\n", now_ms, received_ms, latency, offset_ms, compensated_latency);
}
int64_t compute_latency(int64_t sensor_timestamp_ms)
{
    int64_t host_now = get_current_ms();

    // Since sensor timestamps appear to be relative (not epoch-based),
    // we'll focus on measuring processing delay from when the sample was received
    // to when it's being processed in the detector

    // For now, we'll use a simple approach: measure the time since the sample was received
    // This gives us the actual processing delay through the system

    // The latency is the time from when we received the sample to now
    // This is already tracked in the received_ms field, so we can use that

    // For sensor timestamp analysis, we'll just track the relative timing
    if (first_sample)
    {
        offset_ms = 0; // Reset offset since we're not doing clock sync
        estimated_processing_delay = 0;
        first_sample = false;
        printf("First sample: starting relative timing analysis (host=%ld, sensor=%ld)\n", host_now, sensor_timestamp_ms);
    }

    // The actual processing delay will be calculated in the main loop
    // where we have access to the received_ms timestamp

    printf("Compute: now=%ld, sensor_ts=%ld, processing_delay=%ld ms\n",
           host_now, sensor_timestamp_ms, estimated_processing_delay);

    return estimated_processing_delay;
}

void gestureReact()
{
    TickType_t currentTime = xTaskGetTickCount();

    oscSample osc;
    Direction direction = classify_gesture(gesture_ax_history, gesture_ay_history, 0, gesture_history_count);
    // Gesture is still active and long enough - classify it
    static bool reallyLocked = false;
    switch (direction)
    {
    case LEFT:
        if ((item > 0) && gesture.locked && ((currentTime - gesture.lastGesture) > gesture.gestureCooldown) && !reallyLocked)
        {
            item--;
            osc.type = CTRLLEFT;
            xQueueSend(oscQueue, &osc, portMAX_DELAY);
        }
        break;
    case RIGHT:
        if ((item < MAXITEMS) && gesture.locked && ((currentTime - gesture.lastGesture) > gesture.gestureCooldown) && !reallyLocked)
        {
            item++;
            osc.type = CTRLRIGHT;
            xQueueSend(oscQueue, &osc, portMAX_DELAY);
        }
        break;
    case DOWN:
        break;
    case UP:
        if (!reallyLocked && ((currentTime - gesture.lastGesture) > gesture.gestureCooldown))
        {
            if (!gesture.locked)
            {
                osc.type = CTRLBOTH;
                xQueueSend(oscQueue, &osc, portMAX_DELAY);
                gesture.locked = true;
                printf("Control locked\n");
            }
            else
            {
                osc.type = CTRLBOTH;
                xQueueSend(oscQueue, &osc, portMAX_DELAY);
                gesture.locked = false;
                printf("Control unlocked\n");
            }
        }
        break;
    case SHAKE:
        if (gesture.locked && !reallyLocked)
        {
            osc.type = CTRLBOTH;
            xQueueSend(oscQueue, &osc, portMAX_DELAY);
            reallyLocked = true;
            printf("Control really locked\n");
        }
        else if (reallyLocked)
        {
            osc.type = CTRLBOTH;
            xQueueSend(oscQueue, &osc, portMAX_DELAY);
            reallyLocked = false;
            printf("Control really unlocked\n");
        }

        break;
    default:
        break;
    }
    gesture.lastGesture = xTaskGetTickCount();
    //   printf("🎯 GESTURE COMPLETE (end of batch): %d (duration=%d samples)\n",
    //        direction, gesture_above_count);
}
void detectorTask(void *params)
{
    SensorBuffer *buf;
    HighpassFilter fx, fy, fz;
    double fs = 50.0;

    // Sliding window state
    static int prev_above_count = 0;
    static int gesture_start_abs = 0;
    static bool gesture_reported = false;
    static int total_samples = 0;
    static int last_index = 0;
    int64_t min_latency = INT64_MAX;
    int64_t max_latency = 0;
    float avglat = 0.0;

    struct timespec ts;
    float ax[BUFFER_SIZE];
    float ay[BUFFER_SIZE];
    float az[BUFFER_SIZE];
    float mag[BUFFER_SIZE];
    int above[BUFFER_SIZE];
    gesture.lastGesture = 0;
    gesture.gestureCooldown = 500;

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
                    int64_t dt_ms = buf->samples[i].timestamp - buf->samples[i - 1].timestamp;
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
                ax[i] = HighpassFilter_Update(&fx, buf->samples[i].values[0]);
                ay[i] = HighpassFilter_Update(&fy, buf->samples[i].values[1]);
                az[i] = HighpassFilter_Update(&fz, buf->samples[i].values[2]);

                mag[i] = sqrtf(ax[i] * ax[i] + ay[i] * ay[i] + az[i] * az[i]);
                above[i] = (mag[i] > THRESH) ? 1 : 0;

                // Debug: show magnitude values for first few samples (only when debugging)
                // if (i < 3) {
                //     printf("Sample %d: ax=%.3f, ay=%.3f, az=%.3f, mag=%.3f, above=%d (threshold=%.3f)\n",
                //            i, ax[i], ay[i], az[i], mag[i], above[i], THRESH);
                // }
            }

            // Simple gesture detection for each sample above threshold
            int64_t received_ms = 0;
            int64_t latency = 0;

            int64_t sum_latency = 0;
            int64_t count = 0;
            int64_t samplets = 0;
            int64_t curReceived = 0;

            int64_t prevReceived = 0;
            for (int i = last_index; i < buf->head; i++)
            {
                samplets = buf->samples[i].timestamp;
                // Processing sample silently

                // Calculate actual processing delay from when sample was received to now
                now_ms = get_current_ms();
                received_ms = buf->samples[i].received_ms;
                latency = now_ms - received_ms;

                // Update processing delay estimation
                if (latency >= 0 && latency < 1000)
                { // reasonable processing delay (< 1s)
                    estimated_processing_delay = (int64_t)(beta * latency + (1.0 - beta) * estimated_processing_delay);
                }

                // printf("Processing delay: %ld ms (received at %ld, processing at %ld)\n",
                //      latency, received_ms, now_ms);

                if (latency < min_latency)
                    min_latency = latency;
                if (latency > max_latency)
                    max_latency = latency;
                sum_latency += latency;
                count++;
            }

            // Update latency tracking for consecutive samples
            if (buf->head >= 2)
            {
                curReceived = buf->samples[buf->head - 1].timestamp;
                prevReceived = buf->samples[buf->head - 2].timestamp;
                update_latency(curReceived, prevReceived);
            }
            last_index = buf->head;

            // Calculate average latency
            if (count > 0)
            {
                avglat = sum_latency / (float)count;
                //  printf("Processing delay stats: min=%ld, max=%ld, avg=%.2f, estimated=%.2f ms (from %ld samples)\n",
                //       min_latency, max_latency, avglat, (float)estimated_processing_delay, count);
            }

            // Process samples in the buffer
            int samples_to_process = buf->head;
            // Processing batch silently
            // Sliding window gesture detection across processing cycles
            for (int i = 0; i < samples_to_process; i++)
            {
                if (above[i])
                {
                    if (!gesture_active)
                    {
                        // Gesture started
                        gesture_active = true;
                        gesture_start_sample = total_samples + i;
                        gesture_above_count = 1;
                        gesture_history_count = 0;
                        // printf("🎬 Gesture started at sample %ld (mag=%.3f)\n",
                        //      total_samples + i, mag[i]);
                    }
                    else
                    {
                        // Gesture continuing
                        gesture_above_count++;
                        // printf("🎬 Gesture continuing at sample %ld (count=%d, mag=%.3f)\n",
                        //      total_samples + i, gesture_above_count, mag[i]);
                    }

                    // Store this sample in history (rolling buffer)
                    if (gesture_history_count < HISTORY_SIZE)
                    {
                        gesture_ax_history[gesture_history_count] = ax[i];
                        gesture_ay_history[gesture_history_count] = ay[i];
                        gesture_history_count++;
                    }
                }
                else
                {
                    if (gesture_active)
                    {
                        if (gesture_above_count >= MIN_LEN)
                        {
                            gestureReact();
                        }
                        gesture_active = false;
                        gesture_above_count = 0;
                        gesture_history_count = 0;
                    }
                }
            }

            total_samples += samples_to_process;

            // Check if there's an ongoing gesture that needs to be completed
            if (gesture_active && gesture_above_count >= MIN_LEN)
            {
                gestureReact();
                gesture_active = false;
                gesture_above_count = 0;
                gesture_history_count = 0;
            }

            // Reset buffer for next batch (this is a circular buffer, so we reset head)
            buf->head = 0;

            // Periodically show magnitude statistics for debugging
            if (total_samples % 50 == 0 && total_samples > 0)
            {
                float max_mag = 0.0f;
                int above_count = 0;
                for (int i = 0; i < samples_to_process; i++)
                {
                    if (mag[i] > max_mag)
                        max_mag = mag[i];
                    if (above[i])
                        above_count++;
                }
                // printf("📊 Stats: max_mag=%.3f, above_threshold=%d/%d, threshold=%.3f\n",
                //      max_mag, above_count, samples_to_process, THRESH);
            }
        }
    }
}

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
extern QueueHandle_t oscQueue;
extern QueueHandle_t accelQueue;
extern QueueHandle_t gestureQueue;

// do virtual quaternion, center yaw on every lock

void gestureReact(Direction direction)
{
    gestureState *gesture;
    TickType_t currentTime = xTaskGetTickCount();
    static atomic_uintmax_t item = 0;
    if (xQueuePeek(gestureQueue, &gesture, portMAX_DELAY))
    {
        oscSample osc;
        osc.delay = 200;
        // Direction direction = classify_gesture(gesture->history, 0, gesture->history_count);
        switch (direction)
        {
        case LEFT:
            if (atomic_load(&gesture->locked) && ((currentTime - gesture->lastGesture) > gesture->gestureCooldown) && !gesture->reallyLocked)
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
        case PULL:
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
        case PUSH:
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
}

// Counters per axis
float counterX = 0.0f;
float counterY = 0.0f;
float counterZ = 0.0f;

// Thresholds
const float T_down = 0.5f;     // Below this, decay occurs
const float T_trigger = 60.0f; // Trigger threshold
const float decay_rate = 5.0f; // How fast counters decay
const float alpha = 0.5f;
int64_t lastTrigger = 0;
int64_t triggerHold = 1 * 1000 * 1000;

void sampler(void *pvParameters)
{
    printf("Sampler started\n");
    static gestureState gesture = {
        .lastGesture = 0,
        .gestureCooldown = 500,
        .locked = false,
        .reallyLocked = false,
        .item = 0,
        .still_time = 200,
    };
    gestureState *ptr = &gesture;
    atomic_store(&gesture.item, 0); // write

    while (gestureQueue == NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    xQueueSend(gestureQueue, &ptr, portMAX_DELAY);
            SensorSample a;

   // SensorSample a;
    while (1)
    {
        if ((xQueueReceive(accelQueue, &a, portMAX_DELAY)) == pdPASS)
        {
            float x = a.values[0];
            float y = a.values[1];
            float z = a.values[2];
            // printf("%.2f/%.2f/%.2f\n", x, y, z);
            int64_t current = get_current_us();
            Direction direction;
            // X axis
            if (fabs(x) > T_down)
                counterX += x; // add signed value
            else
                counterX += alpha * (0.0f - counterX); // decay toward zero

            // Y axis
            if (fabs(y) > T_down)
                counterY += y;
            else
                counterY += alpha * (0.0f - counterY);

            // Z axis
            if (fabs(z) > T_down)
                counterZ += z;
            else
                counterZ += alpha * (0.0f - counterZ);

            if ((fabs(counterX) >= T_trigger) || (fabs(counterY) >= T_trigger) || (fabs(counterZ) >= T_trigger))
            {
                if ((current - lastTrigger) > triggerHold)
                {
                    lastTrigger = get_current_us();
                    // Trigger detection
                    if (counterX >= T_trigger)
                    {
                        direction = RIGHT;
                        counterX = 0;
                    }
                    else if (counterX <= -T_trigger)
                    {
                        direction = LEFT;
                        counterX = 0;
                    }

                    if (counterY >= T_trigger)
                    {
                        direction = PUSH;
                        counterY = 0;
                    }
                    else if (counterY <= -T_trigger)
                    {
                        direction = PULL;
                        counterY = 0;
                    }

                    if (counterZ >= T_trigger)
                    {
                        direction = UP;
                        counterZ = 0;
                    }
                    else if (counterZ <= -T_trigger)
                    {
                        direction = DOWN;
                        counterZ = 0;
                    }
                    gestureReact(direction);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
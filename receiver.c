#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdint.h>
#include "jsmn.h"
#include "receiver.h"
#include "datatypes.h"
#include <errno.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <time.h>

extern QueueHandle_t sensorQueue;


// Add sample to buffer
void add_sample(SensorBuffer *buf, uint8_t type, int64_t timestamp, float values[VECTOR_SIZE]) {
    SensorSample *s = &buf->samples[buf->head];
    s->received_ms = get_current_ms();
    
    // Determine timestamp format and convert appropriately
    // Android sensor timestamps are typically nanoseconds since epoch
    if (timestamp > 1000000000000000LL) { // > 1e15 - definitely nanoseconds
        // Nanoseconds since epoch - convert to milliseconds since epoch
        s->timestamp = timestamp / 1000000;
    } else if (timestamp > 1000000000000LL) { // > 1e12 - likely nanoseconds
        // Nanoseconds since epoch - convert to milliseconds since epoch
        s->timestamp = timestamp / 1000000;
    } else if (timestamp > 1000000000LL) { // > 1e9 - milliseconds since epoch
        // Already in milliseconds since epoch
        s->timestamp = timestamp;
    } else {
        // Assume relative milliseconds
        s->timestamp = timestamp;
    }
    
    s->type = type;
    for (int i = 0; i < VECTOR_SIZE; i++)
        s->values[i] = values[i];
    buf->head = (buf->head + 1) % BUFFER_SIZE;
    // Sample added silently
}

// Utility: extract JSON token as string
void json_token_to_str(const char *json, jsmntok_t *tok, char *buf, size_t buflen) {
    int len = tok->end - tok->start;
    if ((size_t)len >= buflen) len = buflen - 1;
    memcpy(buf, json + tok->start, len);
    buf[len] = '\0';
}

// Parse JSON and store sample
void handle_json(const char *json, int length, SensorBuffer *sensor) {
    SensorBuffer *s = sensor;
    
    
    jsmn_parser parser;
    jsmntok_t tokens[MAX_TOKENS];
    jsmn_init(&parser);
    int ret = jsmn_parse(&parser, json, length, tokens, MAX_TOKENS);
    if (ret < 0) {
        printf("Failed to parse JSON: %d\n", ret);
        return;
    }
    if (ret < 1 || tokens[0].type != JSMN_OBJECT) {
        printf("Expected JSON object\n");
        return;
    }

    char type[64] = {0};
    int64_t timestamp = 0;
    float values[VECTOR_SIZE] = {0};

    // Iterate over top-level keys
    for (int i = 1; i < ret; i++) {
        jsmntok_t *key = &tokens[i];
        if (key->type != JSMN_STRING) continue;

        char key_str[64];
        json_token_to_str(json, key, key_str, sizeof(key_str));

        jsmntok_t *val = &tokens[i + 1]; // value is next token

        if (strcmp(key_str, "type") == 0) {
            json_token_to_str(json, val, type, sizeof(type));
        } else if (strcmp(key_str, "timestamp") == 0) {
            char ts_str[32];
            json_token_to_str(json, val, ts_str, sizeof(ts_str));
            timestamp = atoll(ts_str);
        } else if (strcmp(key_str, "values") == 0 && val->type == JSMN_ARRAY) {
            int count = val->size < VECTOR_SIZE ? val->size : VECTOR_SIZE;
            for (int j = 0; j < count; j++) {
                jsmntok_t *numtok = &tokens[i + 2 + j]; // value tokens follow the array token
                char numbuf[32];
                json_token_to_str(json, numtok, numbuf, sizeof(numbuf));
                values[j] = atof(numbuf);
            }
        }

        i++; // skip value token
        if (val->type == JSMN_ARRAY || val->type == JSMN_OBJECT) {
            i += val->size; // skip nested tokens
        }
    }

    // Dispatch to buffer
    if (strcmp(type, "android.sensor.game_rotation_vector") == 0) {
        add_sample(s, GAME_ROTATION, timestamp, values);
    } else if (strcmp(type, "android.sensor.linear_acceleration") == 0) {
        add_sample(s, LINEAR_ACCEL, timestamp, values);
    } else {
        printf("Unknown sensor type: %s\n", type);
        return;
    }

    // Send buffer to detector task
    if (sensorQueue != NULL) { 
        xQueueSend(sensorQueue, &s, portMAX_DELAY); 
    } else { 
        printf("sensorQueue is NULL, send failed\n");
    }
}

void netprocess(void *pvParameters) {
    printf("netprocess task.."); 
    SensorBuffer *sensor = pvPortMalloc(sizeof(SensorBuffer));
    sensor->head = 0;
    while (sensorQueue == NULL) {vTaskDelay(pdMS_TO_TICKS(1000));};
    xQueueSend(sensorQueue, &sensor, portMAX_DELAY);

    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    char buffer[BUFFER_SIZE_BYTES];
    socklen_t addr_len = sizeof(client_addr);

    // Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Listening on UDP port %d...%zu..%zu\n", PORT,sizeof(SensorBuffer),sizeof(SensorSample));

    while (1) {
        retry_recv:
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE_BYTES - 1, 0,
                         (struct sockaddr *)&client_addr, &addr_len);
        if (n < 0) {
            if (errno == EINTR) goto retry_recv;  // interrupted, try again
            perror("recvfrom failed");
            //vTaskDelay(pdMS_TO_TICKS(100));       // avoid busy-loop
            continue;
        }
        buffer[n] = '\0';
        handle_json(buffer, n, sensor);
         //   vTaskDelay(pdMS_TO_TICKS(100));       // avoid busy-loop

    }

    close(sockfd);
}


#ifndef RECEIVER_H
#define RECEIVER_H
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#define PORT 8686
// Utility: extract JSON token as string
void json_token_to_str(const char *json, jsmntok_t *tok, char *buf, size_t buflen)
{
    int len = tok->end - tok->start;
    if ((size_t)len >= buflen)
        len = buflen - 1;
    memcpy(buf, json + tok->start, len);
    buf[len] = '\0';
}
// One sample is 32bytes
// One buffer is 3208bytes.

#endif
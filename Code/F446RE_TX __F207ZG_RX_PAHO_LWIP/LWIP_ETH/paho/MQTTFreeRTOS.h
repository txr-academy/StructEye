/*
 * MQTTFreeRTOS.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Platform adaptation layer for Paho MQTTClient-C on FreeRTOS + LwIP netconn.
 *
 * Paho's MQTTClient.c includes this file via:
 *     #include "MQTTFreeRTOS.h"
 * (set MQTTCLIENT_PLATFORM_HEADER in your build or rename accordingly)
 *
 * Provides:
 *   - Timer  — backed by FreeRTOS xTaskGetTickCount()
 *   - Network — backed by LwIP netconn (no sockets required)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef MQTT_FREERTOS_H_
#define MQTT_FREERTOS_H_

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════════
 *  Timer — Paho uses this for keepalive / command timeouts
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct Timer {
    TickType_t xTicksToWait;   /* countdown duration              */
    TimeOut_t  xTimeOut;       /* snapshot taken at TimerCountdownMS */
} Timer;

static inline void TimerInit(Timer *timer)
{
    timer->xTicksToWait = 0;
    memset(&timer->xTimeOut, 0, sizeof(timer->xTimeOut));
}

static inline char TimerIsExpired(Timer *timer)
{
    return (xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait) == pdTRUE);
}

static inline void TimerCountdownMS(Timer *timer, unsigned int ms)
{
    timer->xTicksToWait = pdMS_TO_TICKS(ms);
    vTaskSetTimeOutState(&timer->xTimeOut);
}

static inline void TimerCountdown(Timer *timer, unsigned int seconds)
{
    TimerCountdownMS(timer, seconds * 1000);
}

static inline int TimerLeftMS(Timer *timer)
{
    /* If already expired → 0 */
    if (xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait) == pdTRUE)
        return 0;
    /* Otherwise return remaining ticks converted to ms */
    return (int)(timer->xTicksToWait * portTICK_PERIOD_MS);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Network — thin wrapper, actual read/write supplied by mqtt_app.c
 *
 *  Paho expects:
 *    int  mqttread (Network*, unsigned char*, int len, int timeout_ms);
 *    int  mqttwrite(Network*, unsigned char*, int len, int timeout_ms);
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct Network {
    int (*mqttread) (struct Network *, unsigned char *, int, int);
    int (*mqttwrite)(struct Network *, unsigned char *, int, int);
} Network;

/* NetworkInit / NetworkConnect are *not* needed by Paho's MQTTClient.c —
   we set mqttread/mqttwrite manually in mqtt_app.c before calling
   MQTTClientInit(). */

#endif /* MQTT_FREERTOS_H_ */

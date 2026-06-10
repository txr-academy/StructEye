#ifndef MQTT_TRANSPORT_H
#define MQTT_TRANSPORT_H

#include "main.h"

typedef struct Timer {
    uint32_t end_time;
} Timer;

typedef struct Network Network;
struct Network {
    int (*mqttread)(Network*, unsigned char*, int, int);
    int (*mqttwrite)(Network*, unsigned char*, int, int);
    int (*connect)(Network*, char*, int);
    void (*disconnect)(Network*);
};

/* Fill a Network struct with LwIP netconn transport functions */
void MQTT_NetworkInit(Network *n, const char *broker_ip, uint16_t port);

#endif

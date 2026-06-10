#include "mqtt_transport.h"
#include "lwip/api.h"
#include <string.h>
#include <stdio.h>

/* One persistent connection — reconnected if lost */
static struct netconn *mqtt_conn = NULL;
static char           s_broker_ip[32];
static uint16_t       s_broker_port;

/* ── LwIP netconn read — Paho calls this ── */
static int lwip_read(Network *n, unsigned char *buf, int len, int timeout_ms)
{
    (void)n;
    if (mqtt_conn == NULL) return -1;

#if LWIP_SO_RCVTIMEO
    netconn_set_recvtimeout(mqtt_conn, (uint32_t)timeout_ms);
#endif

    struct netbuf *nb = NULL;
    err_t err = netconn_recv(mqtt_conn, &nb);
    if (err != ERR_OK || nb == NULL) return 0;

    void   *data;
    uint16_t avail;
    netbuf_data(nb, &data, &avail);

    int copy = (avail < (uint16_t)len) ? avail : len;
    memcpy(buf, data, (size_t)copy);
    netbuf_delete(nb);
    return copy;
}

/* ── LwIP netconn write — Paho calls this ── */
static int lwip_write(Network *n, unsigned char *buf, int len, int timeout_ms)
{
    (void)n; (void)timeout_ms;
    if (mqtt_conn == NULL) return -1;

    err_t err = netconn_write(mqtt_conn, buf, (size_t)len, NETCONN_COPY);
    return (err == ERR_OK) ? len : -1;
}

/* ── Connect or reconnect ── */
static int lwip_connect(Network *n, char *ip, int port)
{
    (void)n;
    if (mqtt_conn != NULL) {
        netconn_close(mqtt_conn);
        netconn_delete(mqtt_conn);
        mqtt_conn = NULL;
    }

    mqtt_conn = netconn_new(NETCONN_TCP);
    if (mqtt_conn == NULL) return -1;

    ip_addr_t dest;
    if (!ip4addr_aton(ip, &dest)) return -1;

    err_t err = netconn_connect(mqtt_conn, &dest, (uint16_t)port);
    if (err != ERR_OK) {
        netconn_delete(mqtt_conn);
        mqtt_conn = NULL;
        return -1;
    }
    printf("[MQTT] TCP connected to %s:%d\r\n", ip, port);
    return 0;
}

/* ── Disconnect ── */
static void lwip_disconnect(Network *n)
{
    (void)n;
    if (mqtt_conn) {
        netconn_close(mqtt_conn);
        netconn_delete(mqtt_conn);
        mqtt_conn = NULL;
    }
}

/* ── Public init ── */
void MQTT_NetworkInit(Network *n, const char *broker_ip, uint16_t port)
{
    strncpy(s_broker_ip, broker_ip, sizeof(s_broker_ip) - 1);
    s_broker_port = port;

    n->mqttread    = lwip_read;
    n->mqttwrite   = lwip_write;
    n->connect     = lwip_connect;
    n->disconnect  = lwip_disconnect;
}

/* ── Timer Implementations ── */
void TimerInit(Timer* timer) {
    timer->end_time = 0;
}
char TimerIsExpired(Timer* timer) {
    return (HAL_GetTick() >= timer->end_time) ? 1 : 0;
}
void TimerCountdownMS(Timer* timer, unsigned int ms) {
    timer->end_time = HAL_GetTick() + ms;
}
void TimerCountdown(Timer* timer, unsigned int seconds) {
    timer->end_time = HAL_GetTick() + (seconds * 1000);
}
int TimerLeftMS(Timer* timer) {
    int32_t left = (int32_t)(timer->end_time - HAL_GetTick());
    return (left < 0) ? 0 : left;
}

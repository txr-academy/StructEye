/*
 * tcp_hello.c
 * ─────────────────────────────────────────────────────────────────────────────
 * Minimal TCP "Hello World" sender for STM32F207ZG + LwIP (netconn API).
 *
 * Flow:
 *   1. Wait for the Ethernet link to come up.
 *   2. Connect to the Python TCP server on the PC.
 *   3. Send "Hello World\n" every 2 seconds.
 *   4. If the connection drops, reconnect automatically.
 *
 * Uses the LwIP netconn API (LWIP_NETCONN=1) — blocking, FreeRTOS-friendly.
 * No sockets needed (LWIP_SOCKET can stay 0).
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "tcp_hello.h"

/* LwIP */
#include "lwip/netif.h"
#include "lwip/api.h"        /* netconn API */
#include "lwip/ip_addr.h"

/* HAL / RTOS */
#include "cmsis_os.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

/* ── gnetif is declared in lwip.c ──────────────────────────────────────── */
extern struct netif gnetif;

/* ══════════════════════════════════════════════════════════════════════════
 *  TCP_Hello_Task  — called from defaultTask after MX_LWIP_Init()
 * ══════════════════════════════════════════════════════════════════════════ */
void TCP_Hello_Task(void *argument)
{
    (void)argument;

    /* ── 1. Wait for Ethernet link ───────────────────────────────────── */
    printf("[TCP] Waiting for Ethernet link...\r\n");

    uint32_t t0 = osKernelGetTickCount();
    while (!netif_is_link_up(&gnetif)) {
        osDelay(200);
        if ((osKernelGetTickCount() - t0) > 15000U) {
            printf("[TCP] ERROR: No link after 15 s — check cable!\r\n");
            for (;;) {
                HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);   /* blink red */
                osDelay(300);
            }
        }
    }

    printf("[TCP] Link UP!  STM32 IP : %s\r\n",
           ip4addr_ntoa(netif_ip4_addr(&gnetif)));

    uint32_t msg_count = 0;

    /* ── 2. Outer loop: (re)connect + send ───────────────────────────── */
    for (;;)
    {
        /* ── 2a. Create TCP connection ───────────────────────────────── */
        printf("[TCP] Connecting to %s:%d ...\r\n", PC_IP_ADDR, PC_TCP_PORT);

        struct netconn *conn = netconn_new(NETCONN_TCP);
        if (conn == NULL) {
            printf("[TCP] netconn_new failed!\r\n");
            osDelay(3000);
            continue;
        }

        ip_addr_t dest;
        memset(&dest, 0, sizeof(dest));
        if (!ipaddr_aton(PC_IP_ADDR, &dest)) {
            printf("[TCP] FATAL: Invalid PC IP address!\r\n");
            osDelay(3000);
            continue;
        }

        printf("[TCP] netconn_new OK, calling netconn_connect to %s ...\r\n", PC_IP_ADDR);
        
        err_t err = netconn_connect(conn, &dest, PC_TCP_PORT);
        
        printf("[TCP] netconn_connect returned %d\r\n", (int)err);

        if (err != ERR_OK) {
            printf("[TCP] Connect FAILED err=%d — retry in 3 s\r\n", (int)err);
            netconn_delete(conn);
            osDelay(3000);
            continue;
        }

        printf("[TCP] *** CONNECTED to PC! ***\r\n");

        /* ── 2b. Send "Hello World" every 2 seconds ─────────────────── */
        for (;;)
        {
            msg_count++;

            const char *msg = "Hello World\n";
            err = netconn_write(conn, msg, strlen(msg), NETCONN_COPY);

            if (err != ERR_OK) {
                printf("[TCP] Send FAILED err=%d — reconnecting\r\n", (int)err);
                break;   /* break inner loop → reconnect */
            }

            printf("[TCP] Sent #%lu: Hello World\r\n",
                   (unsigned long)msg_count);

            /* Toggle green LED on each successful send */
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

            osDelay(2000);   /* wait 2 seconds */
        }

        /* ── Clean up before reconnect ───────────────────────────────── */
        netconn_close(conn);
        netconn_delete(conn);
        osDelay(2000);
    }
}

/*
 * tcp_server.c
 *
 *  TCP Server for StructEye Gateway
 *  Listens on port 5000, accepts one client, and pushes
 *  LoRa telemetry + heartbeats to the connected PC.
 */
#include "tcp_server.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/debug.h"
#include <stdio.h>
#include <string.h>

/* ---- private state ---- */
static struct tcp_pcb *server_pcb;
static struct tcp_pcb *client_pcb = NULL;

/* ---- forward declarations ---- */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  tcp_server_error(void *arg, err_t err);

/**
 * @brief  Initialises the TCP server (call once after MX_LWIP_Init + delay).
 *         Must be called from a FreeRTOS thread (not from an ISR).
 */
void TCP_Server_Init(void)
{
    /* Lock the LwIP core so raw API calls are safe from our user-thread */
    LOCK_TCPIP_CORE();

    server_pcb = tcp_new();
    if (server_pcb != NULL)
    {
        err_t err = tcp_bind(server_pcb, IP_ADDR_ANY, 5000);
        if (err == ERR_OK)
        {
            server_pcb = tcp_listen(server_pcb);
            tcp_accept(server_pcb, tcp_server_accept);
            printf("[ETH] TCP Server Started on Port 5000\r\n");
            printf("[ETH] Waiting for client connection...\r\n");
        }
        else
        {
            memp_free(MEMP_TCP_PCB, server_pcb);
            printf("[ETH] ERROR: TCP Bind Failed (err=%d)\r\n", err);
        }
    }
    else
    {
        printf("[ETH] ERROR: tcp_new() failed — out of memory\r\n");
    }

    UNLOCK_TCPIP_CORE();
}

/**
 * @brief  Called by LwIP (from tcpip thread) when a client connects.
 */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    /* Register callbacks for this connection */
    tcp_recv(newpcb, tcp_server_recv);
    tcp_err(newpcb, tcp_server_error);

    /* Save the client pcb so we can push data to it later */
    client_pcb = newpcb;

    /* Print the client's IP address */
    printf("[ETH] TCP Client Connected from %lu.%lu.%lu.%lu:%u\r\n",
           (newpcb->remote_ip.addr >>  0) & 0xFF,
           (newpcb->remote_ip.addr >>  8) & 0xFF,
           (newpcb->remote_ip.addr >> 16) & 0xFF,
           (newpcb->remote_ip.addr >> 24) & 0xFF,
           newpcb->remote_port);

    return ERR_OK;
}

/**
 * @brief  Called by LwIP when data arrives from the client or client disconnects.
 */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        /* Client closed the connection */
        printf("[ETH] TCP Client Disconnected\r\n");
        if (client_pcb == tpcb) {
            client_pcb = NULL;
        }
        tcp_close(tpcb);
        return ERR_OK;
    }

    if (err != ERR_OK)
    {
        pbuf_free(p);
        return err;
    }

    /* Acknowledge reception to update the TCP window */
    tcp_recved(tpcb, p->tot_len);

    /* We are a push-only server; discard any incoming data from the PC */
    pbuf_free(p);

    return ERR_OK;
}

/**
 * @brief  Called by LwIP on a fatal connection error.
 */
static void tcp_server_error(void *arg, err_t err)
{
    printf("[ETH] TCP Connection Error: %d\r\n", err);
    /* LwIP has already freed the pcb; just clear our pointer */
    client_pcb = NULL;
}

/**
 * @brief  Query whether a TCP client is currently connected.
 * @retval 1 if connected, 0 if not
 */
uint8_t TCP_Server_IsClientConnected(void)
{
    return (client_pcb != NULL) ? 1 : 0;
}

/**
 * @brief  Push data to the connected TCP client.
 * @param  data  pointer to the payload
 * @param  len   number of bytes
 * @retval 1 if data was sent, 0 if no client is connected or write failed
 */
uint8_t TCP_Server_Send(const char *data, uint16_t len)
{
    uint8_t sent = 0;

    /* Lock the core – we are calling raw API from a user thread */
    LOCK_TCPIP_CORE();

    if (client_pcb != NULL)
    {
        err_t write_err = tcp_write(client_pcb, data, len, TCP_WRITE_FLAG_COPY);
        if (write_err == ERR_OK)
        {
            tcp_output(client_pcb);
            sent = 1;
        }
        else
        {
            printf("[ETH] TCP Write failed: %d\r\n", write_err);
        }
    }

    UNLOCK_TCPIP_CORE();
    return sent;
}

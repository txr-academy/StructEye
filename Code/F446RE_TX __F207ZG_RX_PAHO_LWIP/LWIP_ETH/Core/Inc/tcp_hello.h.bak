/*
 * tcp_hello.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Minimal TCP "Hello World" sender for STM32F207ZG + LwIP (netconn API).
 *
 * The STM32 connects to a Python TCP server on the PC and sends
 * "Hello World" every 2 seconds.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef INC_TCP_HELLO_H_
#define INC_TCP_HELLO_H_

/* ── Network settings ─── change these to match your setup ────────────── */
#define PC_IP_ADDR    "192.168.7.10"   /* Your Windows PC IP              */
#define PC_TCP_PORT   7777             /* Port the Python script listens  */

/* ── Called from defaultTask after MX_LWIP_Init() — never returns ──── */
void TCP_Hello_Task(void *argument);

#endif /* INC_TCP_HELLO_H_ */

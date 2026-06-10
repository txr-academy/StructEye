#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void    TCP_Server_Init(void);
uint8_t TCP_Server_Send(const char *data, uint16_t len);
uint8_t TCP_Server_IsClientConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* TCP_SERVER_H */

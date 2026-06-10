#ifndef LORA_H
#define LORA_H

#include "main.h"

/* SX1278 Registers */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_STAT           0x18
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_SYMB_TIMEOUT_LSB     0x1F
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

/* Pin Definitions — uses CubeMX-generated names from main.h */
#define LORA_NSS_PORT            LORA_NSS_GPIO_Port
#define LORA_NSS_PIN             LORA_NSS_Pin
#define LORA_RST_PORT            LORA_RST_GPIO_Port
#define LORA_RST_PIN             LORA_RST_Pin

/* Operating modes */
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RXCONTINUOUS        0x05

/* Function prototypes */
void    SX1278_WriteRegister(uint8_t reg, uint8_t data);
uint8_t SX1278_ReadRegister(uint8_t reg);
void    SX1278_Init(void);
void    SX1278_StartRX(void);
void    SX1278_RestartRX(void);

#endif /* LORA_H */

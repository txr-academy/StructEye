#ifndef LORA_H
#define LORA_H

#include "main.h"

// LoRa SX1278 Registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_PA_RAMP              0x0A
#define REG_OCP                  0x0B
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_RSSI_VALUE           0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2A
#define REG_RSSI_WIDEBAND        0x2C
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3B
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

// Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#pragma pack(push, 1)
typedef struct {
    uint8_t preamble;       // B0: 0xAA
    uint8_t master_id;      // B1: 0x00
    uint8_t device_id;      // B2: 0x01 to 0x04
    uint16_t seq_num;       // B3-B4: 0-65535
    uint8_t msg_type;       // B5: 0x01/0x02
    uint8_t payload_len;    // B6: 0x0C (12)
    
    int16_t accel_x;        // B7-B8
    int16_t accel_y;        // B9-B10
    int16_t accel_z;        // B11-B12
    
    int16_t gyro_x;         // B13-B14
    int16_t gyro_y;         // B15-B16
    int16_t gyro_z;         // B17-B18
    
    uint8_t alert_flags;    // B19
    uint8_t node_status;    // B20
    uint16_t crc16;         // B21-B22
} StructEyeFrame;
#pragma pack(pop)

typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef*      NSS_port;
    uint16_t           NSS_pin;
    GPIO_TypeDef*      RST_port;
    uint16_t           RST_pin;
    GPIO_TypeDef*      DIO0_port;
    uint16_t           DIO0_pin;
} LoRa;

int LoRa_Init(LoRa* _LoRa);
void LoRa_Transmit(LoRa* _LoRa, uint8_t* data, uint8_t length);
int LoRa_Receive(LoRa* _LoRa, uint8_t* data, uint8_t length);
void LoRa_WriteRegister(LoRa* _LoRa, uint8_t address, uint8_t value);
uint8_t LoRa_ReadRegister(LoRa* _LoRa, uint8_t address);

// Configuration functions
void LoRa_SetFrequency(LoRa* _LoRa, long frequency);
void LoRa_SetSpreadingFactor(LoRa* _LoRa, int sf);
void LoRa_SetSignalBandwidth(LoRa* _LoRa, long sbw);
void LoRa_SetSyncWord(LoRa* _LoRa, int sw);

int LoRa_GetRSSI(LoRa* _LoRa);
float LoRa_EstimateDistance(int rssi);

#endif // LORA_H

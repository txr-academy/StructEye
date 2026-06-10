#include "lora.h"

extern SPI_HandleTypeDef hspi1;

void SX1278_WriteRegister(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    uint8_t tx[2] = {reg | 0x80, data};
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

uint8_t SX1278_ReadRegister(uint8_t reg) {
    uint8_t tx[2] = {reg & 0x7F, 0x00};
    uint8_t rx[2] = {0x00, 0x00};
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    return rx[1];
}

void SX1278_Init(void) {
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

    SX1278_WriteRegister(REG_OP_MODE, MODE_SLEEP);
    HAL_Delay(10);

    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    HAL_Delay(10);

    SX1278_WriteRegister(REG_FRF_MSB, 0x6C);
    SX1278_WriteRegister(REG_FRF_MID, 0x40);
    SX1278_WriteRegister(REG_FRF_LSB, 0x00);

    SX1278_WriteRegister(REG_MODEM_CONFIG_1, 0x82);
    SX1278_WriteRegister(REG_MODEM_CONFIG_2, 0x74);
    SX1278_WriteRegister(REG_PA_CONFIG, 0x80);
    SX1278_WriteRegister(REG_SYNC_WORD, 0x12);
    SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x80);
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x00);
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(10);
}

void SX1278_StartRX(void) {
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_ADDR_PTR, 0x00);
    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);
    SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x00);
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RXCONTINUOUS);
}

void SX1278_RestartRX(void) {
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(1);
    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_ADDR_PTR, 0x00);
    SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x00);
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RXCONTINUOUS);
}

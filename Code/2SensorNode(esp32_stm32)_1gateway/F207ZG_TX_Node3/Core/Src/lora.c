#include "lora.h"

extern SPI_HandleTypeDef hspi1;

void SX1278_WriteRegister(uint8_t reg, uint8_t data) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS LOW
  uint8_t tx[2] = {reg | 0x80, data};
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // CS HIGH
}

uint8_t SX1278_ReadRegister(uint8_t reg) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS LOW
  uint8_t tx = reg & 0x7F;
  uint8_t rx = 0;
  HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // CS HIGH
  return rx;
}

void SX1278_Init(void) {
  // Hardware Reset (PB5)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(10);

  // Switch to sleep mode
  SX1278_WriteRegister(REG_OP_MODE, MODE_SLEEP);
  HAL_Delay(10);
  // Enable LoRa mode
  SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  HAL_Delay(10);

  // Set frequency 433 MHz (433M / 61.035 = 7094272 = 0x6C4000)
  SX1278_WriteRegister(REG_FRF_MSB, 0x6C);
  SX1278_WriteRegister(REG_FRF_MID, 0x40);
  SX1278_WriteRegister(REG_FRF_LSB, 0x00);

  // Modem config 1: BW 125kHz, CR 4/5, Implicit Header OFF
  SX1278_WriteRegister(REG_MODEM_CONFIG_1, 0x72);
  // Modem config 2: SF7, CRC Enable
  SX1278_WriteRegister(REG_MODEM_CONFIG_2, 0x74);

  // PA Config: 17dBm
  SX1278_WriteRegister(REG_PA_CONFIG, 0x8F);

  // Sync word: 0x12
  SX1278_WriteRegister(REG_SYNC_WORD, 0x12);

  // Set to Standby Mode initially
  SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  HAL_Delay(10);
}

void SX1278_Transmit(uint8_t *data, uint8_t length) {
  // Set to Standby
  SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  
  // Map DIO0 to TxDone (01 for TxDone)
  SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x40);

  // Set FIFO ptr to TX base addr
  SX1278_WriteRegister(REG_FIFO_ADDR_PTR, 0x80); // TX Base
  SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x80);

  // Write payload
  for (uint8_t i = 0; i < length; i++) {
    SX1278_WriteRegister(REG_FIFO, data[i]);
  }
  
  // Set payload length
  SX1278_WriteRegister(REG_PAYLOAD_LENGTH, length);

  // Start TX
  SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

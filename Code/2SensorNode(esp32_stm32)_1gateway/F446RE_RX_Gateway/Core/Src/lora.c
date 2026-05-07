#include "lora.h"

extern SPI_HandleTypeDef hspi1;

void SX1278_WriteRegister(uint8_t reg, uint8_t data) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  uint8_t tx[2] = {reg | 0x80, data};
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t SX1278_ReadRegister(uint8_t reg) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  uint8_t tx = reg & 0x7F;
  uint8_t rx = 0;
  HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  return rx;
}

void SX1278_Init(void) {
  // Hardware Reset
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
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

  // Map DIO0 to RxDone (00 for RxDone in continuous mode)
  SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x00);

  // Set to Standby Mode initially
  SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  HAL_Delay(10);
}

void SX1278_StartRX(void) {
  // Switch to Continuous RX mode
  SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

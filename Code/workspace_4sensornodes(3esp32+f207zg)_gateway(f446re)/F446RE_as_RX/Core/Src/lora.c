#include "lora.h"

extern SPI_HandleTypeDef hspi1;

// ─── SPI Register Write ───────────────────────────────────────────────────────
void SX1278_WriteRegister(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET); // CS LOW
    uint8_t tx[2] = {reg | 0x80, data};
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);   // CS HIGH
}

// ─── SPI Register Read ────────────────────────────────────────────────────────
uint8_t SX1278_ReadRegister(uint8_t reg) {
    uint8_t tx[2] = {reg & 0x7F, 0x00};
    uint8_t rx[2] = {0x00, 0x00};
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);   // CS HIGH
    return rx[1];
}

// ─── Initialization ───────────────────────────────────────────────────────────
void SX1278_Init(void) {
    // Hardware Reset
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

    // Step 1: Enter Sleep mode first (required before enabling LoRa bit)
    SX1278_WriteRegister(REG_OP_MODE, MODE_SLEEP);
    HAL_Delay(10);

    // Step 2: Enable LoRa mode (bit 7) while in sleep
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    HAL_Delay(10);

    // Step 3: Set frequency — 433 MHz
    // 433000000 / 61.035 = 7094272 = 0x6C4000
    SX1278_WriteRegister(REG_FRF_MSB, 0x6C);
    SX1278_WriteRegister(REG_FRF_MID, 0x40);
    SX1278_WriteRegister(REG_FRF_LSB, 0x00);

    // Step 4: Modem Config 1
    // 0x72 = BW 125kHz (0111) | CR 4/5 (001) | Explicit Header (0)
    SX1278_WriteRegister(REG_MODEM_CONFIG_1, 0x72);

    // Step 5: Modem Config 2
    // 0x74 = SF7 (0111) | TxContinuous OFF | CRC ON (1) | SymbTimeout MSB=00
    SX1278_WriteRegister(REG_MODEM_CONFIG_2, 0x74);

    // Step 6: PA Config — low power for bench testing (prevents LNA saturation)
    // 0x80 = PA_BOOST off, min power ~2 dBm
    SX1278_WriteRegister(REG_PA_CONFIG, 0x80);

    // Step 7: Sync word — must match ALL TX nodes (ESP32 + F207ZG)
    SX1278_WriteRegister(REG_SYNC_WORD, 0x12);

    // Step 8: Set FIFO base addresses
    SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x80);
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

    // Step 9: Map DIO0 → RxDone (bits [7:6] = 00)
    // This is CRITICAL — without this DIO0 never fires for received packets
    SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x00);

    // Step 10: Go to Standby
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(10);
}

// ─── Start Continuous Receive Mode ───────────────────────────────────────────
void SX1278_StartRX(void) {
    // Reset FIFO pointers
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_ADDR_PTR,     0x00);

    // Clear ALL IRQ flags before starting
    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);

    // Map DIO0 → RxDone (confirm again here)
    SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x00);

    // Enter continuous RX mode
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RXCONTINUOUS);
}

// ─── Restart RX After Receiving a Packet ─────────────────────────────────────
// Must be called after EVERY packet (valid or invalid) to prevent FIFO buildup
// (the 228-byte wrong length bug is caused by missing this reset)
void SX1278_RestartRX(void) {
    // Step 1: Go to standby first
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(1);

    // Step 2: Clear ALL IRQ flags
    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);

    // Step 3: Reset FIFO base address and pointer — THIS FIXES THE 228-BYTE BUG
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_ADDR_PTR,     0x00);

    // Step 4: Map DIO0 → RxDone again (stays configured)
    SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x00);

    // Step 5: Re-enter continuous RX
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RXCONTINUOUS);
}

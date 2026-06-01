#include "lora.h"

extern SPI_HandleTypeDef hspi2;

/* -----------------------------------------------------------------------
 * Low-level SPI register access
 * ----------------------------------------------------------------------- */

void SX1278_WriteRegister(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg | 0x80, data};
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 2, 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

uint8_t SX1278_ReadRegister(uint8_t reg) {
    uint8_t tx = reg & 0x7F;
    uint8_t rx = 0;
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &tx, 1, 1000);
    HAL_SPI_Receive(&hspi2, &rx, 1, 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
    return rx;
}

/* Write multiple bytes to FIFO in a single CS-asserted burst (fast + safe) */
static void SX1278_WriteFIFO(uint8_t *data, uint8_t length) {
    uint8_t cmd = REG_FIFO | 0x80;
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 1000);
    HAL_SPI_Transmit(&hspi2, data, length, 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

/* -----------------------------------------------------------------------
 * Init — returns 1 on success, 0 if SPI/chip not responding
 * ----------------------------------------------------------------------- */

uint8_t SX1278_Init(void) {

    /* 1. Hardware reset: pull LOW 10ms, release, wait 100ms for chip ready */
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);   /* CS idle high */
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);  /* SX1278 needs ~5ms min; 100ms is safe for all modules */

    /* 2. Verify SPI is working — REG_VERSION should return 0x12 */
    uint8_t version = SX1278_ReadRegister(REG_VERSION);
    if (version != 0x12) {
        /* SPI not reaching chip — wrong wiring, GPIO AF not set, or CS issue */
        return 0;
    }

    /* 3. Enter FSK sleep first (required before switching to LoRa mode) */
    SX1278_WriteRegister(REG_OP_MODE, MODE_SLEEP);
    HAL_Delay(15);

    /* 4. Switch to LoRa mode — MUST be done while in sleep mode */
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    HAL_Delay(15);

    /* 5. Verify LoRa mode took effect */
    uint8_t mode = SX1278_ReadRegister(REG_OP_MODE);
    if ((mode & 0x80) == 0) {
        /* LoRa bit not set — chip did not accept the mode change */
        return 0;
    }

    /* 6. Set frequency: 433 MHz
     *    f = Fstep x FRF,  Fstep = 32MHz/2^19 = 61.03515625 Hz
     *    FRF = 433000000 / 61.03515625 = 7094272 = 0x6C4000          */
    SX1278_WriteRegister(REG_FRF_MSB, 0x6C);
    SX1278_WriteRegister(REG_FRF_MID, 0x40);
    SX1278_WriteRegister(REG_FRF_LSB, 0x00);

    /* 7. Modem config 1: BW=250kHz, CR=4/5, Explicit header mode
     *    Bits[7:4]=1000 (250kHz), Bits[3:1]=001 (CR4/5), Bit0=0 (explicit)
     *    = 0x82                                                         */
    SX1278_WriteRegister(REG_MODEM_CONFIG_1, 0x82);

    /* 8. Modem config 2: SF=7, TX continuous=0, CRC on, RxTimeout MSB=0
     *    Bits[7:4]=0111 (SF7), Bit3=0, Bit2=1 (CRC on), Bits[1:0]=00
     *    = 0x74                                                         */
    SX1278_WriteRegister(REG_MODEM_CONFIG_2, 0x74);

    /* 9. Symbol timeout LSB */
    SX1278_WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x64);

    /* 10. PA config: PA_BOOST pin, lowered power to prevent Brown-Out Resets */
    SX1278_WriteRegister(REG_PA_CONFIG, 0x82);

    /* 11. Sync word: 0x12 (private network — must match gateway + all nodes) */
    SX1278_WriteRegister(REG_SYNC_WORD, 0x12);

    /* 12. FIFO base addresses — both TX and RX at 0x00 */
    SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

    /* 13. DIO0 mapped to TxDone */
    SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x40);

    /* 14. Enter standby — ready to TX */
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(10);

    return 1;
}

/* -----------------------------------------------------------------------
 * Transmit — blocks until TxDone flag or 3s timeout
 * ----------------------------------------------------------------------- */

void SX1278_Transmit(uint8_t *data, uint8_t length) {

    /* 1. Standby before touching FIFO */
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(10);

    /* 2. Clear all IRQ flags */
    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);

    /* 3. Point FIFO address pointer to TX base */
    SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_ADDR_PTR,     0x00);

    /* 4. Burst-write entire payload in one SPI transaction */
    SX1278_WriteFIFO(data, length);

    /* 5. Set payload length */
    SX1278_WriteRegister(REG_PAYLOAD_LENGTH, length);

    /* 6. Start TX */
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    /* 7. Wait for TxDone flag (bit 3 of REG_IRQ_FLAGS), 3s timeout */
    uint32_t t_start = HAL_GetTick();
    while (1) {
        uint8_t irq = SX1278_ReadRegister(REG_IRQ_FLAGS);
        if (irq & 0x08) break;                          /* TxDone set      */
        if ((HAL_GetTick() - t_start) > 3000) break;    /* 3s safety bail  */
    }

    /* 8. Clear IRQ flags and return to standby */
    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}
/* -----------------------------------------------------------------------
 * Channel Activity Detection (CAD) — Listen-Before-Talk
 * Returns: 1 = channel busy (activity detected), 0 = channel free
 * ----------------------------------------------------------------------- */

uint8_t SX1278_PerformCAD(void) {
    /* 1. Enter standby before switching modes */
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(1);

    /* 2. Enter RXCONTINUOUS mode to measure raw RF energy (RSSI) */
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | 0x05); // 0x05 = RXCONTINUOUS
    HAL_Delay(2); // Wait 2ms for RSSI to stabilize

    /* 3. Read REG_RSSI_VALUE (0x1B) */
    uint8_t rssi_raw = SX1278_ReadRegister(0x1B);

    /* 4. Return to standby */
    SX1278_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    /* 5. For 433MHz, actual RSSI in dBm = rssi_raw - 157.
          Noise floor is ~ -110 dBm (rssi_raw = 47).
          If rssi_raw > 70 (approx > -87 dBm), another node is actively transmitting. */
    if (rssi_raw > 70) {
        return 1; // Channel is busy
    }
    return 0; // Channel is free
}

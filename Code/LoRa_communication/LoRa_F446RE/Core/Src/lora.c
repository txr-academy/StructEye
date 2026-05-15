#include "lora.h"
#include <math.h>

// Helper functions for SPI read/write
void LoRa_WriteRegister(LoRa* _LoRa, uint8_t address, uint8_t value) {
    uint8_t tx[2] = { address | 0x80, value };
    HAL_GPIO_WritePin(_LoRa->NSS_port, _LoRa->NSS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hspi, tx, 2, 100);
    HAL_GPIO_WritePin(_LoRa->NSS_port, _LoRa->NSS_pin, GPIO_PIN_SET);
}

uint8_t LoRa_ReadRegister(LoRa* _LoRa, uint8_t address) {
    uint8_t tx = address & 0x7F;
    uint8_t rx = 0;
    HAL_GPIO_WritePin(_LoRa->NSS_port, _LoRa->NSS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hspi, &tx, 1, 100);
    HAL_SPI_Receive(_LoRa->hspi, &rx, 1, 100);
    HAL_GPIO_WritePin(_LoRa->NSS_port, _LoRa->NSS_pin, GPIO_PIN_SET);
    return rx;
}

int LoRa_Init(LoRa* _LoRa) {
    // Reset the module
    if (_LoRa->RST_port != NULL) {
        HAL_GPIO_WritePin(_LoRa->RST_port, _LoRa->RST_pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(_LoRa->RST_port, _LoRa->RST_pin, GPIO_PIN_SET);
        HAL_Delay(10);
    }

    // Check version
    uint8_t version = LoRa_ReadRegister(_LoRa, REG_VERSION);
    if (version != 0x12) {
        return 0; // Failed
    }

    // Sleep mode
    LoRa_WriteRegister(_LoRa, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    HAL_Delay(10);

    // Set frequency to 433 MHz
    LoRa_SetFrequency(_LoRa, 433E6);

    // Set base addresses
    LoRa_WriteRegister(_LoRa, REG_FIFO_TX_BASE_ADDR, 0);
    LoRa_WriteRegister(_LoRa, REG_FIFO_RX_BASE_ADDR, 0);

    // Setup LNA
    LoRa_WriteRegister(_LoRa, REG_LNA, LoRa_ReadRegister(_LoRa, REG_LNA) | 0x03);

    // Set auto AGC
    LoRa_WriteRegister(_LoRa, REG_MODEM_CONFIG_3, 0x04);

    // Set output power to 17 dBm
    LoRa_WriteRegister(_LoRa, REG_PA_CONFIG, PA_BOOST | 0x0F);

    // Put in standby mode
    LoRa_WriteRegister(_LoRa, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    return 1; // Success
}

void LoRa_Transmit(LoRa* _LoRa, uint8_t* data, uint8_t length) {
    // Put in standby mode
    LoRa_WriteRegister(_LoRa, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    // Set FIFO pointer to TX base address
    LoRa_WriteRegister(_LoRa, REG_FIFO_ADDR_PTR, 0);

    // Write payload length
    LoRa_WriteRegister(_LoRa, REG_PAYLOAD_LENGTH, length);

    // Write data to FIFO
    for (int i = 0; i < length; i++) {
        LoRa_WriteRegister(_LoRa, REG_FIFO, data[i]);
    }

    // Enable TX done interrupt and clear all interrupts
    LoRa_WriteRegister(_LoRa, REG_DIO_MAPPING_1, 0x40); // DIO0 = TxDone
    LoRa_WriteRegister(_LoRa, REG_IRQ_FLAGS, 0xFF);

    // Switch to TX mode
    LoRa_WriteRegister(_LoRa, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    // Wait for TX done (poll DIO0 or IRQ_FLAGS)
    // Here we poll IRQ_FLAGS just in case DIO0 isn't connected
    uint32_t startTick = HAL_GetTick();
    while ((LoRa_ReadRegister(_LoRa, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        if (HAL_GetTick() - startTick > 1000) {
            // Timeout after 1000ms
            break;
        }
    }

    // Clear IRQ
    LoRa_WriteRegister(_LoRa, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

// Check for received data and return number of bytes (0 if none)
// Ensure you are in RX mode before calling this, or handle switching to RX mode
int LoRa_Receive(LoRa* _LoRa, uint8_t* data, uint8_t length) {
    uint8_t irqFlags = LoRa_ReadRegister(_LoRa, REG_IRQ_FLAGS);

    // If RX_DONE
    if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
        // Check for payload error
        if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
            uint8_t rxLength = LoRa_ReadRegister(_LoRa, REG_RX_NB_BYTES);
            uint8_t currentAddr = LoRa_ReadRegister(_LoRa, REG_FIFO_RX_CURRENT_ADDR);
            LoRa_WriteRegister(_LoRa, REG_FIFO_ADDR_PTR, currentAddr);

            for (int i = 0; i < rxLength; i++) {
                data[i] = LoRa_ReadRegister(_LoRa, REG_FIFO);
            }
            // Clear IRQ
            LoRa_WriteRegister(_LoRa, REG_IRQ_FLAGS, irqFlags);
            
            // Re-enter RX mode
            LoRa_WriteRegister(_LoRa, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
            
            return rxLength;
        }
        // Clear IRQ
        LoRa_WriteRegister(_LoRa, REG_IRQ_FLAGS, irqFlags);
    }
    
    // Switch back to RX continuous mode just in case
    uint8_t opMode = LoRa_ReadRegister(_LoRa, REG_OP_MODE);
    if ((opMode & MODE_RX_CONTINUOUS) != MODE_RX_CONTINUOUS) {
        LoRa_WriteRegister(_LoRa, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    }

    return 0;
}

void LoRa_SetFrequency(LoRa* _LoRa, long frequency) {
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    LoRa_WriteRegister(_LoRa, REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_WriteRegister(_LoRa, REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_WriteRegister(_LoRa, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa_SetSpreadingFactor(LoRa* _LoRa, int sf) {
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;

    if (sf == 6) {
        LoRa_WriteRegister(_LoRa, REG_DETECTION_OPTIMIZE, 0xc5);
        LoRa_WriteRegister(_LoRa, REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        LoRa_WriteRegister(_LoRa, REG_DETECTION_OPTIMIZE, 0xc3);
        LoRa_WriteRegister(_LoRa, REG_DETECTION_THRESHOLD, 0x0a);
    }

    LoRa_WriteRegister(_LoRa, REG_MODEM_CONFIG_2, (LoRa_ReadRegister(_LoRa, REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

int LoRa_GetRSSI(LoRa* _LoRa) {
    // Read the RSSI value from the packet RSSI register
    uint8_t rssi_val = LoRa_ReadRegister(_LoRa, REG_PKT_RSSI_VALUE);
    // For low frequency port (like 433MHz), the formula is -164 + RSSI_value
    return -164 + rssi_val;
}

float LoRa_EstimateDistance(int rssi) {
    // A: RSSI at 1 meter distance (you will need to calibrate this value for your specific environment/antenna)
    // A is usually between -60 to -70 for 433MHz LoRa modules. Let's use -65 as a rough default.
    float A = -65.0f; 
    
    // n: Path loss exponent (2.0 for free space, 2.7 to 3.5 for urban/indoor). Let's use 2.5 for general mixed environment.
    float n = 2.5f; 

    // Distance formula: d = 10 ^ ((A - RSSI) / (10 * n))
    float distance = powf(10.0f, ((A - (float)rssi) / (10.0f * n)));
    return distance;
}

/* ==========================================================================
 * StructEye — Gateway TEST (receives dummy frames from F446RE test node)
 * Board   : STM32F446ZE (Nucleo-144)
 * UART    : USART3 @ 115200 — PuTTY output
 * SPI     : SPI1   — LoRa SX1278
 * ========================================================================== */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>

/* --------------------------------------------------------------------------
 * Frame constants (must match sensor node exactly)
 * -------------------------------------------------------------------------- */
#define PREAMBLE           0xAA
#define MSG_TYPE_RESPONSE  0x02
#define ALERT_ACCEL        (1 << 0)
#define ALERT_GYRO         (1 << 1)
#define ALERT_SENSOR_ERR   (1 << 2)
#define STATUS_OK          0x00
#define STATUS_FAULT       0x01
#define FRAME_SIZE         23
#define LORA_FREQUENCY     433000000UL

/* --------------------------------------------------------------------------
 * Frame structure
 * -------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t  preamble;
    uint8_t  master_id;
    uint8_t  device_id;
    uint16_t seq_number;
    uint8_t  msg_type;
    uint8_t  payload_len;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    uint8_t  alert_flags;
    uint8_t  node_status;
    uint16_t crc16;
} LoRa_Frame;

/* --------------------------------------------------------------------------
 * Peripheral handles
 * -------------------------------------------------------------------------- */
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart3;

/* --------------------------------------------------------------------------
 * Stats counters
 * -------------------------------------------------------------------------- */
static uint32_t g_total_rx    = 0;
static uint32_t g_crc_errors  = 0;
static uint32_t g_fault_count = 0;

/* --------------------------------------------------------------------------
 * Node name lookup
 * -------------------------------------------------------------------------- */
static const char *node_name(uint8_t id) {
    switch (id) {
        case 0x01: return "Sensor Node 1 — Pillar A";
        case 0x02: return "Sensor Node 2 — Pillar B";
        case 0x03: return "Sensor Node 3 — Floor Beam";
        case 0x04: return "Sensor Node 4 — Roof Slab";
        default:   return "Unknown Node";
    }
}

/* --------------------------------------------------------------------------
 * UART helper
 * -------------------------------------------------------------------------- */
static void uart_print(const char *msg) {
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 500);
}
static void uart_printf(const char *fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_print(buf);
}

/* --------------------------------------------------------------------------
 * CRC-16
 * -------------------------------------------------------------------------- */
static uint16_t crc16_calc(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

/* ==========================================================================
 * LoRa SX1278
 * ========================================================================== */
static void lora_nss_low(void)  { HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET); }
static void lora_nss_high(void) { HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);   }

static void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg | 0x80, val };
    lora_nss_low();
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    lora_nss_high();
}
static uint8_t lora_read_reg(uint8_t reg) {
    uint8_t tx = reg & 0x7F, rx = 0;
    lora_nss_low();
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
    HAL_SPI_Receive(&hspi1, &rx, 1, 100);
    lora_nss_high();
    return rx;
}
static void lora_read_fifo(uint8_t *buf, uint8_t len) {
    uint8_t cmd = 0x00;
    lora_nss_low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, buf, len, 500);
    lora_nss_high();
}

static uint8_t LoRa_Init(void) {
    /* Hardware Reset */
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    lora_write_reg(0x01, 0x00); HAL_Delay(10);
    lora_write_reg(0x01, 0x80); HAL_Delay(10);
    if (lora_read_reg(0x42) != 0x12) return 0;
    uint32_t frf = (uint32_t)((double)LORA_FREQUENCY / 61.03515625);
    lora_write_reg(0x06, (frf >> 16) & 0xFF);
    lora_write_reg(0x07, (frf >>  8) & 0xFF);
    lora_write_reg(0x08, (frf >>  0) & 0xFF);
    lora_write_reg(0x09, 0x8F);
    lora_write_reg(0x1D, 0x72);
    lora_write_reg(0x1E, 0x74);
    lora_write_reg(0x26, 0x04);
    lora_write_reg(0x39, 0x12);  /* Must match TX sync word                */
    lora_write_reg(0x0F, 0x00);
    lora_write_reg(0x0D, 0x00);
    lora_write_reg(0x12, 0xFF);
    lora_write_reg(0x01, 0x85);  /* Continuous RX mode                     */
    HAL_Delay(10);
    return 1;
}

static uint8_t LoRa_PacketReady(void) {
    return (lora_read_reg(0x12) & 0x40) ? 1 : 0;
}

static uint8_t LoRa_ReadPacket(uint8_t *buf) {
    uint8_t irq = lora_read_reg(0x12);
    lora_write_reg(0x12, 0xFF);
    if (irq & 0x20) {
        uart_print("[WARN] LoRa radio CRC error\r\n");
        lora_write_reg(0x0D, 0x00);
        lora_write_reg(0x01, 0x85);
        return 0;
    }
    uint8_t len     = lora_read_reg(0x13);
    uint8_t rx_addr = lora_read_reg(0x10);
    lora_write_reg(0x0D, rx_addr);
    lora_read_fifo(buf, len);
    lora_write_reg(0x0D, 0x00);
    lora_write_reg(0x01, 0x85);
    return len;
}

static int LoRa_GetRSSI(void) {
    return (int)lora_read_reg(0x1A) - 164;
}

/* ==========================================================================
 * Frame parser — full structured display
 * ========================================================================== */
static void parse_and_display(uint8_t *raw, uint8_t len) {
    g_total_rx++;

    /* ---- length check ---- */
    if (len != FRAME_SIZE) {
        uart_printf("[WARN] Wrong length: got %d, expected %d\r\n",
                    len, FRAME_SIZE);
        return;
    }

    LoRa_Frame *f = (LoRa_Frame *)raw;

    /* ---- preamble check ---- */
    if (f->preamble != PREAMBLE) {
        uart_printf("[WARN] Bad preamble: 0x%02X\r\n", f->preamble);
        return;
    }

    /* ---- CRC check ---- */
    uint16_t rx_crc   = __builtin_bswap16(f->crc16);
    uint16_t calc_crc = crc16_calc(raw, FRAME_SIZE - 2);
    uint8_t  crc_ok   = (rx_crc == calc_crc);
    if (!crc_ok) g_crc_errors++;

    /* ---- sequence number (big-endian) ---- */
    uint16_t seq = __builtin_bswap16(f->seq_number);

    /* ---- RSSI ---- */
    int rssi = LoRa_GetRSSI();

    /* ---- alert flag description ---- */
    char flag_str[64] = "";
    if      (f->alert_flags == 0x00)            strcpy(flag_str, "NONE");
    else {
        if (f->alert_flags & ALERT_ACCEL)        strcat(flag_str, "ACCEL ");
        if (f->alert_flags & ALERT_GYRO)         strcat(flag_str, "GYRO ");
        if (f->alert_flags & ALERT_SENSOR_ERR)   strcat(flag_str, "SENSOR_ERR ");
    }

    /* ---- result verdict ---- */
    const char *verdict;
    if (!crc_ok) {
        verdict        = "!! CRC ERROR !!";
    } else if (f->node_status == STATUS_FAULT) {
        g_fault_count++;
        if ((f->alert_flags & ALERT_ACCEL) && (f->alert_flags & ALERT_GYRO)) {
            verdict        = "!! DANGER !!";
        } else if (f->alert_flags & ALERT_ACCEL) {
            verdict        = "!! WARNING !!";
        } else if (f->alert_flags & ALERT_GYRO) {
            verdict        = "!! WARNING !!";
        } else {
            verdict        = "!! FAULT !!";
        }
    } else {
        verdict        = "NORMAL";
    }

    /* ---- raw hex dump ---- */
    char hex_line[128] = "";
    char tmp[8];
    for (int i = 0; i < FRAME_SIZE; i++) {
        snprintf(tmp, sizeof(tmp), "%02X ", raw[i]);
        strcat(hex_line, tmp);
        if (i == 11) strcat(hex_line, "\r\n                    ");
    }

    /* ================================================================
     * MAIN DISPLAY — Compact Single Line Output
     * ============================================================== */
    uart_printf(
        "[RX 0x%02X] SEQ:%-4d | A(%6d,%6d,%6d) | G(%6d,%6d,%6d) | STATUS: %-14s | RSSI:%4d | CRC:%s\r\n",
        f->device_id, seq,
        (int)f->ax, (int)f->ay, (int)f->az,
        (int)f->gx, (int)f->gy, (int)f->gz,
        verdict,
        rssi,
        crc_ok ? "OK" : "FAIL"
    );

}

/* ==========================================================================
 * main()
 * ========================================================================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART3_UART_Init();

    uart_print("========================================\r\n");
    uart_print("  StructEye Gateway — Master 0x00\r\n");
    uart_print("========================================\r\n");

    if (!LoRa_Init()) {
        uart_print("[INIT] LoRa SX1278 FAILED\r\n");
        while (1);
    }
    uart_print("[INIT] LoRa SX1278 OK\r\n");
    uart_printf("[INIT] Listening on %lu MHz...\r\n", LORA_FREQUENCY / 1000000);
    uart_print("[INIT] Gateway ready.\r\n");

    uint8_t rx_buf[64];

    while (1) {
        if (LoRa_PacketReady()) {
            uint8_t len = LoRa_ReadPacket(rx_buf);
            if (len > 0) parse_and_display(rx_buf, len);
        }
        HAL_Delay(5);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET); // NSS high
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET); // RST high

  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);
}

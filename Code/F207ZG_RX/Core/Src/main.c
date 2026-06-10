/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   StructEye Gateway — STM32F207ZG LoRa RX
 *          Receives from 4 sensor nodes and forwards via Paho MQTT.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "lora.h"
#include <stdio.h>
#include <string.h>
#include "lwip/api.h"

#include "MQTTClient.h"
#include "mqtt_transport.h"

#define MQTT_BROKER_IP   "192.168.1.100"
#define MQTT_BROKER_PORT 1883
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
volatile uint8_t lora_rx_flag = 0;
MQTTClient mqttClient;
Network network;
unsigned char sendbuf[512];
unsigned char readbuf[512];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
uint16_t crc16(const uint8_t *data, uint16_t len);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ── USART3 debug (Nucleo virtual COM) ─────────────────────────────────── */
static void USART3_DebugInit(void) {
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  const uint32_t baud = 115200U;
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  USART3->BRR  = (pclk + (baud / 2U)) / baud;
  USART3->CR1  = USART_CR1_TE | USART_CR1_RE;
  USART3->CR2  = 0;
  USART3->CR3  = 0;
  USART3->CR1 |= USART_CR1_UE;
}

static void USART3_WriteBlocking(const uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    while ((USART3->SR & USART_SR_TXE) == 0) {}
    USART3->DR = data[i];
  }
  while ((USART3->SR & USART_SR_TC) == 0) {}
}

int _write(int file, char *ptr, int len) {
  (void)file;
  USART3_WriteBlocking((const uint8_t *)ptr, (uint16_t)len);
  return len;
}

int __io_putchar(int ch) {
  uint8_t c = (uint8_t)ch;
  USART3_WriteBlocking(&c, 1);
  return ch;
}

/* ── CRC16 ──────────────────────────────────────────────────────────────── */
uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

/* ── LoRa DIO0 interrupt ────────────────────────────────────────────────── */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == LORA_DIO0_Pin) {
    lora_rx_flag = 1;
  }
}

/* USER CODE END 0 */

/* ── main ───────────────────────────────────────────────────────────────── */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  USART3_DebugInit();
  setvbuf(stdout, NULL, _IONBF, 0);

  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  printf("\r\n============================================================\r\n");
  printf("  StructEye Gateway - STM32F207ZG LoRa Receiver (Paho MQTT)\r\n");
  printf("  MQTT Broker -> %s:%u\r\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
  printf("============================================================\r\n");

  SX1278_Init();
  uint8_t loraVersion = SX1278_ReadRegister(REG_VERSION);
  printf("LoRa Version: 0x%02X %s\r\n", loraVersion,
         (loraVersion == 0x12) ? "(OK)" : "(ERROR!)");

  SX1278_StartRX();
  printf("Listening for LoRa packets...\r\n\r\n");
  /* USER CODE END 2 */

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osKernelStart();

  while (1) { osDelay(100); }
}

/* ── FreeRTOS default task ───────────────────────────────────────────────── */
void StartDefaultTask(void const * argument) {
  extern struct netif gnetif;

  /* Init LwIP inside the task (required for FreeRTOS+LwIP) */
  MX_LWIP_Init();

  /* Wait for link to come up */
  osDelay(3000);
  printf("netif link: %s\r\n", netif_is_link_up(&gnetif) ? "UP" : "DOWN");
  printf("netif up:   %s\r\n", netif_is_up(&gnetif)      ? "UP" : "DOWN");

  /* USER CODE BEGIN 5 */
  
  /* --- PAHO MQTT INITIALIZATION --- */
  MQTT_NetworkInit(&network, MQTT_BROKER_IP, MQTT_BROKER_PORT);
  MQTTClientInit(&mqttClient, &network, 5000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

  printf("[MQTT] Connecting to broker %s...\r\n", MQTT_BROKER_IP);
  int rc = network.connect(&network, MQTT_BROKER_IP, MQTT_BROKER_PORT);
  if (rc == 0) {
      MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
      data.MQTTVersion = 3;
      data.clientID.cstring = "STM32_Gateway";
      
      rc = MQTTConnect(&mqttClient, &data);
      if (rc == MQTT_SUCCESS) {
          printf("[MQTT] Connected successfully to Mosquitto!\r\n");
      } else {
          printf("[MQTT] MQTTConnect failed with code %d\r\n", rc);
      }
  } else {
      printf("[MQTT] TCP connection failed\r\n");
  }

  uint8_t  rx_buffer[256];
  memset(rx_buffer, 0, sizeof(rx_buffer));
  uint32_t total_rx      = 0;
  uint32_t valid_rx      = 0;

  for (;;) {
    
    /* Keep MQTT connection alive or try to reconnect */
    if (mqttClient.isconnected) {
        MQTTYield(&mqttClient, 10);
    } else {
        static uint32_t last_reconnect_attempt = 0;
        if (HAL_GetTick() - last_reconnect_attempt > 5000) {
            last_reconnect_attempt = HAL_GetTick();
            printf("[MQTT] Attempting to reconnect...\r\n");
            network.disconnect(&network);
            int rc = network.connect(&network, MQTT_BROKER_IP, MQTT_BROKER_PORT);
            if (rc == 0) {
                MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
                data.MQTTVersion = 3;
                data.clientID.cstring = "STM32_Gateway";
                rc = MQTTConnect(&mqttClient, &data);
                if (rc == MQTT_SUCCESS) {
                    printf("[MQTT] Reconnected successfully to Mosquitto!\r\n");
                }
            }
        }
    }

    /* ── LoRa packet received ────────────────────────────────────────── */
    if (lora_rx_flag) {
      lora_rx_flag = 0;

      uint8_t irqFlags = SX1278_ReadRegister(REG_IRQ_FLAGS);
      SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);
      total_rx++;

      /* Check RxDone flag (bit 6) */
      if (!(irqFlags & 0x40)) {
        SX1278_RestartRX();
        continue;
      }

      /* Check packet length */
      uint8_t len = SX1278_ReadRegister(REG_RX_NB_BYTES);
      if (len != 23) {
        printf("[DROP] Wrong length: %u\r\n", len);
        SX1278_RestartRX();
        continue;
      }

      /* Read FIFO */
      uint8_t fifoAddr = SX1278_ReadRegister(REG_FIFO_RX_CURRENT_ADDR);
      SX1278_WriteRegister(REG_FIFO_ADDR_PTR, fifoAddr);

      HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
      uint8_t reg_addr = REG_FIFO & 0x7F;
      HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
      HAL_SPI_Receive(&hspi1, rx_buffer, len, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

      /* Check start byte */
      if (rx_buffer[0] != 0xAA) {
        printf("[DROP] Bad start: 0x%02X\r\n", rx_buffer[0]);
        SX1278_RestartRX();
        continue;
      }

      /* Check CRC */
      uint16_t rx_crc   = (uint16_t)((rx_buffer[21] << 8) | rx_buffer[22]);
      uint16_t calc_crc = crc16(rx_buffer, 21);
      if (rx_crc != calc_crc) {
        printf("[DROP] CRC: 0x%04X vs 0x%04X\r\n", rx_crc, calc_crc);
        SX1278_RestartRX();
        continue;
      }

      /* Parse packet */
      valid_rx++;
      uint8_t  node_id = rx_buffer[1];
      uint8_t  seq     = rx_buffer[2];
      int16_t  ax      = (int16_t)((rx_buffer[3]  << 8) | rx_buffer[4]);
      int16_t  ay      = (int16_t)((rx_buffer[5]  << 8) | rx_buffer[6]);
      int16_t  az      = (int16_t)((rx_buffer[7]  << 8) | rx_buffer[8]);
      int16_t  gx      = (int16_t)((rx_buffer[9]  << 8) | rx_buffer[10]);
      int16_t  gy      = (int16_t)((rx_buffer[11] << 8) | rx_buffer[12]);
      int16_t  gz      = (int16_t)((rx_buffer[13] << 8) | rx_buffer[14]);
      uint16_t vib     = (uint16_t)((rx_buffer[15] << 8) | rx_buffer[16]);
      int16_t  tilt    = (int16_t)((rx_buffer[17] << 8) | rx_buffer[18]);
      uint8_t  status  = rx_buffer[19];
      uint8_t  alert   = rx_buffer[20];

      SX1278_RestartRX();

      /* Print to serial */
      const char *alert_str = (alert == 0) ? "OK" :
                              (alert == 1) ? "WARN" :
                              (alert == 2) ? "CRITICAL" : "UNKNOWN";

      printf("\r\n--- Packet Received ---\r\n");
      printf("Node: %u | Seq: %u | RX: %lu Valid: %lu\r\n",
             node_id, seq, total_rx, valid_rx);
      printf("Accel: X=%.2f Y=%.2f Z=%.2f m/s2\r\n",
             ax/100.0f, ay/100.0f, az/100.0f);
      printf("Gyro:  X=%.2f Y=%.2f Z=%.2f deg/s\r\n",
             gx/100.0f, gy/100.0f, gz/100.0f);
      printf("Vib: %.2f m/s2 | Tilt: %.2f deg\r\n",
             vib/100.0f, tilt/100.0f);
      printf("Status: 0x%02X | Alert: %u [%s]\r\n",
             status, alert, alert_str);

      /* Publish via MQTT */
      if (mqttClient.isconnected) {
          char payload[256];
          int n = snprintf(payload, sizeof(payload),
              "{\"node\":%u, \"seq\":%u, \"ax\":%.2f, \"ay\":%.2f, \"az\":%.2f, "
              "\"gx\":%.2f, \"gy\":%.2f, \"gz\":%.2f, \"vib\":%.2f, \"tilt\":%.2f, "
              "\"alert\":%u}",
              node_id, seq, ax/100.0f, ay/100.0f, az/100.0f,
              gx/100.0f, gy/100.0f, gz/100.0f, vib/100.0f, tilt/100.0f, alert);

          MQTTMessage message;
          message.qos = QOS0;
          message.retained = 0;
          message.payload = payload;
          message.payloadlen = n;

          char topic[64];
          snprintf(topic, sizeof(topic), "struceye/node/%u/data", node_id);

          int pub_rc = MQTTPublish(&mqttClient, topic, &message);
          if (pub_rc == MQTT_SUCCESS) {
              printf("[MQTT] Successfully Published to %s\r\n", topic);
          } else {
              printf("[MQTT] Publish failed! code = %d\r\n", pub_rc);
          }
      } else {
          printf("[MQTT] Cannot publish, client disconnected.\r\n");
      }
    }

    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* ── HAL / system callbacks ─────────────────────────────────────────────── */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) HAL_IncTick();
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 13;
  RCC_OscInitStruct.PLL.PLLN            = 195;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void) {
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_GPIO_WritePin(LORA_RST_GPIO_Port,        LORA_RST_Pin,        GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port,        LORA_NSS_Pin,        GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port,             LD2_Pin,             GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LORA_RST_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RST_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LORA_NSS_Pin;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

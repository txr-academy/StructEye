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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include <stdio.h>
#include <string.h>

#include "MQTTClient.h"

/* IMPORTANT: Update this to your actual PC IP address! */
#define MQTT_BROKER_IP "192.168.1.69"
#define MQTT_BROKER_PORT 1883
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */
extern struct netif gnetif;
volatile uint8_t lora_rx_flag = 0;
MQTTClient mqttClient;
Network network;
unsigned char sendbuf[512];
unsigned char readbuf[512];

/* Used for LwIP Netconn transport instead of missing mqtt_transport.h */
static struct netconn *s_conn = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
uint16_t crc16(const uint8_t *data, uint16_t len);

/* --- Network transport functions for LwIP Netconn --- */
static int transport_read(Network *n, unsigned char *buf, int len,
                          int timeout_ms) {
  (void)n;
  if (s_conn == NULL)
    return -1;
  netconn_set_recvtimeout(s_conn, (uint32_t)(timeout_ms < 1 ? 1 : timeout_ms));
  struct netbuf *nb = NULL;
  err_t err = netconn_recv(s_conn, &nb);
  if (err == ERR_TIMEOUT)
    return 0;
  if (err != ERR_OK || nb == NULL)
    return -1;
  void *data = NULL;
  uint16_t avail = 0;
  netbuf_data(nb, &data, &avail);
  int copy = ((int)avail < len) ? (int)avail : len;
  memcpy(buf, data, (size_t)copy);
  netbuf_delete(nb);
  return copy;
}

static int transport_write(Network *n, unsigned char *buf, int len,
                           int timeout_ms) {
  (void)n;
  (void)timeout_ms;
  if (s_conn == NULL)
    return -1;
  err_t err = netconn_write(s_conn, buf, (size_t)len, NETCONN_COPY);
  return (err == ERR_OK) ? len : -1;
}

static int transport_connect(const char *ip_str, uint16_t port) {
  if (s_conn != NULL) {
    netconn_close(s_conn);
    netconn_delete(s_conn);
    s_conn = NULL;
  }
  s_conn = netconn_new(NETCONN_TCP);
  if (s_conn == NULL)
    return -1;
  ip_addr_t dest;
  memset(&dest, 0, sizeof(dest));
  if (!ipaddr_aton(ip_str, &dest)) {
    netconn_delete(s_conn);
    s_conn = NULL;
    return -1;
  }
  err_t err = netconn_connect(s_conn, &dest, port);
  if (err != ERR_OK) {
    netconn_delete(s_conn);
    s_conn = NULL;
    return -1;
  }
  return 0;
}

static void transport_disconnect(void) {
  if (s_conn != NULL) {
    netconn_close(s_conn);
    netconn_delete(s_conn);
    s_conn = NULL;
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── USART3 debug (Nucleo virtual COM) ─────────────────────────────────── */
static void USART3_DebugInit(void) {
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  const uint32_t baud = 115200U;
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  USART3->BRR = (pclk + (baud / 2U)) / baud;
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE;
  USART3->CR2 = 0;
  USART3->CR3 = 0;
  USART3->CR1 |= USART_CR1_UE;
}

static void USART3_WriteBlocking(const uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    while ((USART3->SR & USART_SR_TXE) == 0) {
    }
    USART3->DR = data[i];
  }
  while ((USART3->SR & USART_SR_TC) == 0) {
  }
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
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
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

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  USART3_DebugInit();
  setvbuf(stdout, NULL, _IONBF, 0);

  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  printf(
      "\r\n============================================================\r\n");
  printf("  StructEye Gateway - STM32F207ZG LoRa Receiver (Paho MQTT)\r\n");
  printf("  MQTT Broker -> %s:%u\r\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
  printf("============================================================\r\n");

  SX1278_Init();
  uint8_t loraVersion = SX1278_ReadRegister(REG_VERSION);
  printf("LoRa Version: 0x%02X %s\r\n", loraVersion,
         (loraVersion == 0x12) ? "(OK)" : "(ERROR!)");

  // Debug: Dump Registers to verify SPI writes
  printf("--- SX1278 REG DUMP ---\r\n");
  printf("OP_MODE: 0x%02X\r\n", SX1278_ReadRegister(REG_OP_MODE));
  printf("FRF: %02X %02X %02X\r\n", SX1278_ReadRegister(REG_FRF_MSB),
         SX1278_ReadRegister(REG_FRF_MID), SX1278_ReadRegister(REG_FRF_LSB));
  printf("MODEM_1: 0x%02X\r\n", SX1278_ReadRegister(REG_MODEM_CONFIG_1));
  printf("MODEM_2: 0x%02X\r\n", SX1278_ReadRegister(REG_MODEM_CONFIG_2));
  printf("MODEM_3: 0x%02X\r\n", SX1278_ReadRegister(REG_MODEM_CONFIG_3));
  printf("SYNC_WORD: 0x%02X\r\n", SX1278_ReadRegister(REG_SYNC_WORD));
  printf("-----------------------\r\n");

  SX1278_StartRX();
  printf("Listening for LoRa packets...\r\n\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL) {
    printf("[FATAL] FreeRTOS could not create StartDefaultTask! (Out of "
           "Heap)\r\n");
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RST_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* --- PAHO MQTT INITIALIZATION --- */
  network.mqttread = transport_read;
  network.mqttwrite = transport_write;
  MQTTClientInit(&mqttClient, &network, 5000, sendbuf, sizeof(sendbuf), readbuf,
                 sizeof(readbuf));

  uint8_t rx_buffer[256];
  memset(rx_buffer, 0, sizeof(rx_buffer));
  uint32_t total_rx = 0;
  uint32_t valid_rx = 0;

  uint8_t eth_link_was_up = 0;
  uint8_t ip_was_assigned = 0;
  uint32_t last_reconnect_attempt = 0;

  for (;;) {

    /* 1. Check Ethernet Link */
    uint8_t link_is_up = netif_is_link_up(&gnetif);
    if (link_is_up && !eth_link_was_up) {
      printf("[ETH] Link is UP!\r\n");
      eth_link_was_up = 1;
    } else if (!link_is_up && eth_link_was_up) {
      printf("[ETH] Link went DOWN!\r\n");
      eth_link_was_up = 0;
      ip_was_assigned = 0;
    }

    /* 2. Check DHCP IP Assignment */
    if (link_is_up) {
      uint32_t current_ip = gnetif.ip_addr.addr;
      if (current_ip != 0 && !ip_was_assigned) {
        printf("[DHCP] Assigned IP: %lu.%lu.%lu.%lu\r\n", (current_ip & 0xff),
               ((current_ip >> 8) & 0xff), ((current_ip >> 16) & 0xff),
               (current_ip >> 24));
        ip_was_assigned = 1;
        last_reconnect_attempt =
            HAL_GetTick() - 5000; /* Force immediate connect */
      }
    }

    /* 3. Handle MQTT Connection (only if IP assigned) */
    if (ip_was_assigned) {
      if (mqttClient.isconnected) {
        MQTTYield(&mqttClient, 10);
      } else {
        if (HAL_GetTick() - last_reconnect_attempt > 5000) {
          last_reconnect_attempt = HAL_GetTick();
          printf("[MQTT] Attempting to connect to %s...\r\n", MQTT_BROKER_IP);
          transport_disconnect();
          int rc = transport_connect(MQTT_BROKER_IP, MQTT_BROKER_PORT);
          if (rc == 0) {
            MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
            data.MQTTVersion = 3;
            data.clientID.cstring = "STM32_Gateway";
            rc = MQTTConnect(&mqttClient, &data);
            if (rc == MQTT_SUCCESS) {
              printf("[MQTT] Connected successfully to Mosquitto!\r\n");
            } else {
              printf("[MQTT] Connect failed with code %d\r\n", rc);
              transport_disconnect();
            }
          } else {
            printf("[MQTT] TCP connection failed.\r\n");
          }
        }
      }
    }

    /* 4. LoRa packet received ────────────────────────────────────────── */

    // Debug: Poll IRQ flags to see if hardware received it but interrupt didn't
    // fire
    uint8_t irq_flags = SX1278_ReadRegister(REG_IRQ_FLAGS);
    if ((irq_flags & 0x40) != 0) {
      if (!lora_rx_flag) {
        printf("[DEBUG] RxDone flag is set in hardware, but DIO0 interrupt DID "
               "NOT FIRE! Check your PD15 wire.\r\n");
        lora_rx_flag = 1; // trigger it anyway
      }
    }

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
      uint16_t rx_crc = (uint16_t)((rx_buffer[21] << 8) | rx_buffer[22]);
      uint16_t calc_crc = crc16(rx_buffer, 21);
      if (rx_crc != calc_crc) {
        printf("[DROP] CRC: 0x%04X vs 0x%04X\r\n", rx_crc, calc_crc);
        SX1278_RestartRX();
        continue;
      }

      /* Parse packet */
      valid_rx++;
      uint8_t node_id = rx_buffer[1];
      uint8_t seq = rx_buffer[2];
      int16_t ax = (int16_t)((rx_buffer[3] << 8) | rx_buffer[4]);
      int16_t ay = (int16_t)((rx_buffer[5] << 8) | rx_buffer[6]);
      int16_t az = (int16_t)((rx_buffer[7] << 8) | rx_buffer[8]);
      int16_t gx = (int16_t)((rx_buffer[9] << 8) | rx_buffer[10]);
      int16_t gy = (int16_t)((rx_buffer[11] << 8) | rx_buffer[12]);
      int16_t gz = (int16_t)((rx_buffer[13] << 8) | rx_buffer[14]);
      uint16_t vib = (uint16_t)((rx_buffer[15] << 8) | rx_buffer[16]);
      int16_t tilt = (int16_t)((rx_buffer[17] << 8) | rx_buffer[18]);
      uint8_t status = rx_buffer[19];
      uint8_t alert = rx_buffer[20];

      SX1278_RestartRX();

      /* Print to serial */
      const char *alert_str = (alert == 0)   ? "OK"
                              : (alert == 1) ? "WARN"
                              : (alert == 2) ? "CRITICAL"
                                             : "UNKNOWN";

#define F_SIGN(x) ((x) < 0 ? "-" : "")
#define F_INT(x) (abs(x) / 100)
#define F_FRAC(x) (abs(x) % 100)

      printf("\r\n--- Packet Received ---\r\n");
      printf("Node: %u | Seq: %u | RX: %lu Valid: %lu\r\n", node_id, seq,
             total_rx, valid_rx);
      printf("Accel: X=%s%d.%02d Y=%s%d.%02d Z=%s%d.%02d m/s2\r\n", F_SIGN(ax),
             F_INT(ax), F_FRAC(ax), F_SIGN(ay), F_INT(ay), F_FRAC(ay),
             F_SIGN(az), F_INT(az), F_FRAC(az));
      printf("Gyro:  X=%s%d.%02d Y=%s%d.%02d Z=%s%d.%02d deg/s\r\n", F_SIGN(gx),
             F_INT(gx), F_FRAC(gx), F_SIGN(gy), F_INT(gy), F_FRAC(gy),
             F_SIGN(gz), F_INT(gz), F_FRAC(gz));
      printf("Vib: %d.%02d m/s2 | Tilt: %s%d.%02d deg\r\n", (vib / 100),
             (vib % 100), F_SIGN(tilt), F_INT(tilt), F_FRAC(tilt));
      printf("Status: 0x%02X | Alert: %u [%s]\r\n", status, alert, alert_str);

      /* Publish via MQTT */
      if (mqttClient.isconnected) {
        char payload[256];
        int n = snprintf(
            payload, sizeof(payload),
            "{\"node\":%u, \"seq\":%u, \"ax\":%s%d.%02d, \"ay\":%s%d.%02d, "
            "\"az\":%s%d.%02d, "
            "\"gx\":%s%d.%02d, \"gy\":%s%d.%02d, \"gz\":%s%d.%02d, "
            "\"vib\":%d.%02d, \"tilt\":%s%d.%02d, "
            "\"alert\":%u}",
            node_id, seq, F_SIGN(ax), F_INT(ax), F_FRAC(ax), F_SIGN(ay),
            F_INT(ay), F_FRAC(ay), F_SIGN(az), F_INT(az), F_FRAC(az),
            F_SIGN(gx), F_INT(gx), F_FRAC(gx), F_SIGN(gy), F_INT(gy),
            F_FRAC(gy), F_SIGN(gz), F_INT(gz), F_FRAC(gz), (vib / 100),
            (vib % 100), F_SIGN(tilt), F_INT(tilt), F_FRAC(tilt), alert);

        MQTTMessage message;
        message.qos = QOS0;
        message.retained = 0;
        message.payload = payload;
        message.payloadlen = n;

        char topic[64];
        snprintf(topic, sizeof(topic), "structeye/node/%u/data", node_id);

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

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
       // ne

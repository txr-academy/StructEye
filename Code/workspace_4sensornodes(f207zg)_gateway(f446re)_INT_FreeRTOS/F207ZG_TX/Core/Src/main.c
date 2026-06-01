/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : StructEye Node 4 — STM32F207ZG FreeRTOS LoRa Transmitter
 *
 *                   Architecture:
 *                     Task 1 — Sensor Acquisition   (EXTI + periodic poll)
 *                     Task 2 — Threshold Detection  (vibration/tilt analysis)
 *                     Task 3 — LoRa Communication   (CAD + TX)
 *                     Task 4 — System Monitor        (health telemetry)
 *
 *                   Inter-task communication via FreeRTOS Queues.
 *                   Motion interrupt (MPU6050 INT on PC0/EXTI0) wakes
 *                   Task 1 via vTaskNotifyGiveFromISR for sub-5ms latency.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lora.h"
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* --- Data structure passed from Task 1 → Task 2 via xSensorQueue --- */
typedef struct {
  float accelX;  /* m/s² */
  float accelY;
  float accelZ;
  float gyroX;   /* deg/s */
  float gyroY;
  float gyroZ;
  uint8_t sensor_ok; /* 1 = valid read, 0 = I2C error / all-zeros */
} SensorData_t;

/* --- 23-byte TX frame passed from Task 2 → Task 3 via xTxQueue --- */
typedef struct {
  uint8_t frame[23];
} TxPacket_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NODE_ID               0x04   /* Node 4 = F207ZG */

/* Heartbeat & Alert timing (in RTOS ticks, 1 tick = 1ms) */
#define HEARTBEAT_PERIOD_MS   30000  /* 30 seconds normal heartbeat */
#define ALERT_PERIOD_MS       100    /* 100ms rapid-fire during alert */
#define SENSOR_POLL_MS        50     /* 50ms sensor polling interval */

/* Threshold values (Dynamic shaking above calibrated rest baseline) */
#define VIB_WARN_HIGH         1.2f   /* m/s² — dynamic shake warning */
#define VIB_CRIT_HIGH         3.0f   /* m/s² — severe dynamic shaking */
#define TILT_WARN_DEG         10.0f  /* degrees */
#define TILT_CRIT_DEG         15.0f  /* degrees */

/* CAD retry configuration */
#define CAD_MAX_RETRIES       15
#define CAD_BACKOFF_MIN_MS    20
#define CAD_BACKOFF_MAX_MS    80

/* Queue sizes */
#define SENSOR_QUEUE_SIZE     10
#define TX_QUEUE_SIZE         5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

/* htim7 is defined in stm32f2xx_hal_timebase_tim.c as the HAL timebase */
extern TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* --- FreeRTOS Handles --- */
TaskHandle_t xSensorTaskHandle  = NULL;
TaskHandle_t xProcessTaskHandle = NULL;
TaskHandle_t xLoRaTaskHandle    = NULL;
TaskHandle_t xMonitorTaskHandle = NULL;

QueueHandle_t xSensorQueue = NULL;   /* SensorData_t: Task1 → Task2 */
QueueHandle_t xTxQueue     = NULL;   /* TxPacket_t:   Task2 → Task3 */

/* --- Shared state --- */
MPU6050_t MPU6050;
volatile uint8_t mpu_motion_detected = 0;  /* Set by EXTI callback */

/* --- PRNG for CAD backoff --- */
static uint32_t rng_state = 0x12345678;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void User_Periph_Init(void);
uint16_t crc16(const uint8_t *data, uint16_t len);

/* FreeRTOS Task Functions */
void vSensorAcquisitionTask(void *pvParameters);
void vThresholdDetectionTask(void *pvParameters);
void vLoRaCommunicationTask(void *pvParameters);
void vSystemMonitorTask(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -----------------------------------------------------------------------
 * CRC-16/CCITT — used for packet integrity
 * ----------------------------------------------------------------------- */
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

/* -----------------------------------------------------------------------
 * Simple LCG PRNG for randomized CAD backoff
 * ----------------------------------------------------------------------- */
static uint32_t get_random_ms(uint32_t min_ms, uint32_t max_ms) {
  rng_state = rng_state * 1664525U + 1013904223U;
  return min_ms + (rng_state % (max_ms - min_ms + 1));
}

/* -----------------------------------------------------------------------
 * EXTI Callback — called by HAL when MPU6050 motion interrupt fires
 *
 * This is the critical low-latency path:
 *   MPU6050 INT pin → EXTI0 ISR → HAL_GPIO_EXTI_Callback →
 *   vTaskNotifyGiveFromISR → portYIELD_FROM_ISR → Task 1 wakes instantly
 * ----------------------------------------------------------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU_INT_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Notify the Sensor Acquisition Task that motion was detected */
    if (xSensorTaskHandle != NULL) {
      vTaskNotifyGiveFromISR(xSensorTaskHandle, &xHigherPriorityTaskWoken);
    }

    /* Force an immediate context switch if the sensor task has
     * higher priority than whatever was running when the ISR fired */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/* ========================================================================
 * TASK 1: SENSOR ACQUISITION
 * ========================================================================
 * - Blocks waiting for EXTI notification (motion interrupt) OR
 *   times out after SENSOR_POLL_MS (50ms) for periodic heartbeat readings.
 * - Reads MPU6050 via I2C burst read (14 bytes, ~1.4ms at 100kHz).
 * - Clears MPU6050 interrupt status register (0x3A).
 * - Scales raw values to engineering units (m/s², deg/s).
 * - Pushes SensorData_t to xSensorQueue for Task 2.
 * ======================================================================== */
void vSensorAcquisitionTask(void *pvParameters) {
  (void)pvParameters;

  SensorData_t sensorData;

  for (;;) {
    /* Block until notified by EXTI ISR or timeout after SENSOR_POLL_MS.
     * ulTaskNotifyTake returns the notification count:
     *   > 0 means interrupt woke us (motion detected)
     *   = 0 means we timed out (periodic poll for heartbeat data) */
    uint32_t notification = ulTaskNotifyTake(pdTRUE,
                                             pdMS_TO_TICKS(SENSOR_POLL_MS));

    if (notification > 0) {
      /* Motion interrupt fired — log it */
      const char *intMsg = ">>> HARDWARE INTERRUPT: Motion Detected! <<<\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)intMsg, strlen(intMsg), 50);
    }

    /* Read all 6 axes from MPU6050 in a single I2C burst */
    MPU6050_Read_All(&hi2c1, &MPU6050);

    /* Scale raw values to engineering units:
     *   Accel: ±2g range, sensitivity = 16384 LSB/g, convert to m/s²
     *   Gyro:  ±250°/s range, sensitivity = 131 LSB/°/s             */
    sensorData.accelX = (MPU6050.Accel_X / 16384.0f) * 9.81f;
    sensorData.accelY = (MPU6050.Accel_Y / 16384.0f) * 9.81f;
    sensorData.accelZ = (MPU6050.Accel_Z / 16384.0f) * 9.81f;
    sensorData.gyroX  = MPU6050.Gyro_X / 131.0f;
    sensorData.gyroY  = MPU6050.Gyro_Y / 131.0f;
    sensorData.gyroZ  = MPU6050.Gyro_Z / 131.0f;

    /* Detect sensor fault (all zeros = I2C failure or sensor disconnected) */
    sensorData.sensor_ok = !(MPU6050.Accel_X == 0 &&
                             MPU6050.Accel_Y == 0 &&
                             MPU6050.Accel_Z == 0);

    /* Clear MPU6050 interrupt status register to allow next interrupt */
    uint8_t int_status;
    HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3A, 1, &int_status, 1, 100);

    /* Push to processing queue — non-blocking, drop oldest if full */
    if (xQueueSend(xSensorQueue, &sensorData, 0) != pdPASS) {
      /* Queue full — receive and discard the oldest, then re-send */
      SensorData_t discard;
      xQueueReceive(xSensorQueue, &discard, 0);
      xQueueSend(xSensorQueue, &sensorData, 0);
    }
  }
}

/* ========================================================================
 * TASK 2: PROCESSING & THRESHOLD DETECTION
 * ========================================================================
 * - Blocks waiting on xSensorQueue (zero CPU usage while idle).
 * - Computes RMS vibration magnitude and tilt angle.
 * - Runs alert state machine:
 *     Normal mode → sends 1 packet per HEARTBEAT_PERIOD_MS (30s)
 *     Alert mode  → sends 1 packet per ALERT_PERIOD_MS (100ms)
 * - Constructs the 23-byte StructEye frame with CRC-16.
 * - Pushes TxPacket_t to xTxQueue for Task 3.
 * ======================================================================== */
void vThresholdDetectionTask(void *pvParameters) {
  (void)pvParameters;

  SensorData_t rxData;
  TxPacket_t   txPacket;
  uint8_t      seq_num = 0;
  TickType_t   lastTxTick = 0;
  uint8_t      alert_active = 0; /* 0 = normal, 1 = alert mode */
  char uartMsg[128];

  float base_gravity = 9.81f;
  uint16_t cal_samples = 0;
  float cal_sum = 0.0f;
  uint8_t cooldown_counter = 0;

  for (;;) {
    /* Block indefinitely waiting for sensor data from Task 1 */
    if (xQueueReceive(xSensorQueue, &rxData, portMAX_DELAY) != pdPASS) {
      continue;
    }

    /* Compute raw acceleration magnitude */
    float accel_mag = sqrtf(rxData.accelX * rxData.accelX +
                            rxData.accelY * rxData.accelY +
                            rxData.accelZ * rxData.accelZ);

    /* Self-calibration over first 50 samples at startup */
    if (cal_samples < 50) {
      cal_sum += accel_mag;
      cal_samples++;
      if (cal_samples == 50) {
        base_gravity = cal_sum / 50.0f;
        snprintf(uartMsg, sizeof(uartMsg), "[CAL] Auto-calibration done. Base Gravity = %.3f m/s2\r\n", base_gravity);
        HAL_UART_Transmit(&huart3, (uint8_t *)uartMsg, strlen(uartMsg), 100);
      }
      /* Skip processing until calibrated */
      continue;
    }

    /* Compute true dynamic vibration (shaking above baseline gravity) */
    float vibration = fabsf(accel_mag - base_gravity);

    float maxAxis = fabsf(rxData.accelX);
    if (fabsf(rxData.accelY) > maxAxis) maxAxis = fabsf(rxData.accelY);
    if (fabsf(rxData.accelZ) > maxAxis) maxAxis = fabsf(rxData.accelZ);

    float tilt = 0.0f;
    if (accel_mag > 0.1f) {
      tilt = acosf(maxAxis / accel_mag) * 180.0f / 3.14159265f;
    }

    /* --- Status flags & alert level --- */
    uint8_t status_flags = 0;
    uint8_t alert_level  = 0;

    /* Vibration thresholds */
    if (vibration > VIB_CRIT_HIGH)
      status_flags |= (1 << 0);  /* CRITICAL vibration */
    else if (vibration > VIB_WARN_HIGH)
      alert_level = 1;           /* WARN vibration */

    /* Tilt thresholds */
    if (fabsf(tilt) > TILT_CRIT_DEG)
      status_flags |= (1 << 1);  /* CRITICAL tilt */
    else if (fabsf(tilt) > TILT_WARN_DEG)
      alert_level = 1;           /* WARN tilt */

    /* Sensor fault flag */
    if (!rxData.sensor_ok)
      status_flags |= (1 << 2);

    /* Boot flag — first packet after reset */
    if (seq_num == 0)
      status_flags |= (1 << 3);

    /* CRITICAL overrides WARN */
    if ((status_flags & 0x03) != 0)
      alert_level = 2;

    /* --- Update alert state machine --- */
    uint8_t was_alert = alert_active;
    alert_active = (alert_level >= 1) ? 1 : 0;

    /* If alert state changes, start a 3-packet fast cooldown burst */
    uint8_t transition = (alert_active != was_alert) ? 1 : 0;
    if (transition) {
      cooldown_counter = 3;
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t tx_interval;

    if (alert_active) {
      tx_interval = pdMS_TO_TICKS(ALERT_PERIOD_MS); // 100ms
    } else if (cooldown_counter > 0) {
      tx_interval = pdMS_TO_TICKS(2000);            // 2 seconds redundant spacing
    } else {
      tx_interval = pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS); // 30 seconds
    }

    uint8_t force_tx = (transition || seq_num == 0) ? 1 : 0;

    if (!force_tx && (now - lastTxTick) < tx_interval) {
      /* Not time to transmit yet — skip */
      continue;
    }

    /* Redundant transmission cooldown tracker */
    if (cooldown_counter > 0 && !transition) {
      cooldown_counter--;
    }

    /* --- Construct 23-byte StructEye frame --- */
    txPacket.frame[0] = 0xAA;         /* Start byte */
    txPacket.frame[1] = NODE_ID;      /* Node ID */
    txPacket.frame[2] = seq_num++;    /* Sequence number */

    int16_t ax_s   = (int16_t)(rxData.accelX * 100);
    int16_t ay_s   = (int16_t)(rxData.accelY * 100);
    int16_t az_s   = (int16_t)(rxData.accelZ * 100);
    int16_t gx_s   = (int16_t)(rxData.gyroX  * 100);
    int16_t gy_s   = (int16_t)(rxData.gyroY  * 100);
    int16_t gz_s   = (int16_t)(rxData.gyroZ  * 100);
    uint16_t vib_s = (uint16_t)(vibration     * 100);
    int16_t tilt_s = (int16_t)(tilt           * 100);

    txPacket.frame[3]  = (ax_s >> 8) & 0xFF;
    txPacket.frame[4]  = ax_s & 0xFF;
    txPacket.frame[5]  = (ay_s >> 8) & 0xFF;
    txPacket.frame[6]  = ay_s & 0xFF;
    txPacket.frame[7]  = (az_s >> 8) & 0xFF;
    txPacket.frame[8]  = az_s & 0xFF;
    txPacket.frame[9]  = (gx_s >> 8) & 0xFF;
    txPacket.frame[10] = gx_s & 0xFF;
    txPacket.frame[11] = (gy_s >> 8) & 0xFF;
    txPacket.frame[12] = gy_s & 0xFF;
    txPacket.frame[13] = (gz_s >> 8) & 0xFF;
    txPacket.frame[14] = gz_s & 0xFF;
    txPacket.frame[15] = (vib_s >> 8) & 0xFF;
    txPacket.frame[16] = vib_s & 0xFF;
    txPacket.frame[17] = (tilt_s >> 8) & 0xFF;
    txPacket.frame[18] = tilt_s & 0xFF;

    txPacket.frame[19] = status_flags;
    txPacket.frame[20] = alert_level;

    /* CRC-16 over bytes 0–20 */
    uint16_t crc = crc16(txPacket.frame, 21);
    txPacket.frame[21] = (crc >> 8) & 0xFF;
    txPacket.frame[22] = crc & 0xFF;

    /* Push to LoRa TX queue — block up to 100ms if full */
    if (xQueueSend(xTxQueue, &txPacket, pdMS_TO_TICKS(100)) == pdPASS) {
      lastTxTick = now;

      /* Debug print */
      snprintf(uartMsg, sizeof(uartMsg),
               "[PROC] Seq=%u Vib=%.2f Tilt=%.2f Alert=%u\r\n",
               (unsigned)(seq_num - 1), vibration, tilt, alert_level);
      HAL_UART_Transmit(&huart3, (uint8_t *)uartMsg, strlen(uartMsg), 50);
    }
  }
}

/* ========================================================================
 * TASK 3: LoRa COMMUNICATION (CAD + TRANSMIT)
 * ========================================================================
 * - Blocks waiting on xTxQueue (zero CPU usage while idle).
 * - Performs CAD Listen-Before-Talk:
 *     1. Check channel with SX1278_PerformCAD()
 *     2. If busy → randomized backoff (20-80ms) using vTaskDelay
 *     3. Retry up to CAD_MAX_RETRIES times
 *     4. If still busy → force-transmit (better late than lost)
 * - Transmits the 23-byte frame via SPI burst.
 * - CAD backoffs use vTaskDelay (non-blocking to other tasks).
 * ======================================================================== */
void vLoRaCommunicationTask(void *pvParameters) {
  (void)pvParameters;

  TxPacket_t rxPacket;
  char uartMsg[80];

  for (;;) {
    /* Block indefinitely waiting for a packet from Task 2 */
    if (xQueueReceive(xTxQueue, &rxPacket, portMAX_DELAY) != pdPASS) {
      continue;
    }

    /* --- CAD Listen-Before-Talk --- */
    uint8_t tx_success = 0;
    for (int retries = 0; retries < CAD_MAX_RETRIES; retries++) {
      if (SX1278_PerformCAD()) {
        /* Channel busy — back off with randomized delay.
         * vTaskDelay yields the CPU to other tasks during the wait,
         * so sensor acquisition continues uninterrupted. */
        uint32_t backoff = get_random_ms(CAD_BACKOFF_MIN_MS, CAD_BACKOFF_MAX_MS);

        snprintf(uartMsg, sizeof(uartMsg),
                 "[CAD] Channel busy — backing off %lums (retry %d/%d)\r\n",
                 backoff, retries + 1, CAD_MAX_RETRIES);
        HAL_UART_Transmit(&huart3, (uint8_t *)uartMsg, strlen(uartMsg), 50);

        vTaskDelay(pdMS_TO_TICKS(backoff));
      } else {
        /* Channel free — transmit immediately */
        HAL_UART_Transmit(&huart3, (uint8_t *)"[TX] Channel free — transmitting\r\n", 34, 50);
        SX1278_Transmit(rxPacket.frame, 23);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); /* Toggle LD1 green LED */
        HAL_UART_Transmit(&huart3, (uint8_t *)"[TX] Done\r\n", 11, 50);
        tx_success = 1;
        break;
      }
    }

    if (!tx_success) {
      /* Fallback: force transmit after all CAD retries exhausted.
       * It's better to risk a collision than to silently drop an alert. */
      HAL_UART_Transmit(&huart3, (uint8_t *)"[TX] FORCE — CAD retries exhausted\r\n", 35, 50);
      SX1278_Transmit(rxPacket.frame, 23);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
      HAL_UART_Transmit(&huart3, (uint8_t *)"[TX] Done (forced)\r\n", 20, 50);
    }
  }
}

/* ========================================================================
 * TASK 4: SYSTEM MONITOR
 * ========================================================================
 * - Runs every 5 seconds at the lowest priority.
 * - Reports stack high-water marks for all tasks (minimum free stack words
 *   remaining since task creation — early warning for stack overflows).
 * - Reports free heap memory.
 * - Toggles LD2 (blue LED) as a system heartbeat indicator.
 * ======================================================================== */
void vSystemMonitorTask(void *pvParameters) {
  (void)pvParameters;

  char uartMsg[256];

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000)); /* Run every 5 seconds */

    /* Get stack high-water marks (minimum free stack words remaining) */
    UBaseType_t sensorHW  = uxTaskGetStackHighWaterMark(xSensorTaskHandle);
    UBaseType_t processHW = uxTaskGetStackHighWaterMark(xProcessTaskHandle);
    UBaseType_t loraHW    = uxTaskGetStackHighWaterMark(xLoRaTaskHandle);
    UBaseType_t monitorHW = uxTaskGetStackHighWaterMark(xMonitorTaskHandle);

    /* Get free heap */
    size_t freeHeap = xPortGetFreeHeapSize();

    /* Report */
    snprintf(uartMsg, sizeof(uartMsg),
             "\r\n--- SYSTEM MONITOR ---\r\n"
             "Free Heap: %u bytes\r\n"
             "Stack High Water Marks (min free words):\r\n"
             "  Sensor Task:  %u\r\n"
             "  Process Task: %u\r\n"
             "  LoRa Task:    %u\r\n"
             "  Monitor Task: %u\r\n"
             "Sensor Queue: %u/%d | TX Queue: %u/%d\r\n"
             "----------------------\r\n",
             (unsigned)freeHeap,
             (unsigned)sensorHW,
             (unsigned)processHW,
             (unsigned)loraHW,
             (unsigned)monitorHW,
             (unsigned)uxQueueMessagesWaiting(xSensorQueue), SENSOR_QUEUE_SIZE,
             (unsigned)uxQueueMessagesWaiting(xTxQueue), TX_QUEUE_SIZE);
    HAL_UART_Transmit(&huart3, (uint8_t *)uartMsg, strlen(uartMsg), 200);

    /* Toggle blue LED as system heartbeat */
    HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
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

  /* Debug: confirm UART is alive */
  HAL_UART_Transmit(&huart3, (uint8_t *)"UART OK\r\n", 9, 100);

  MX_USB_OTG_FS_PCD_Init();
  HAL_UART_Transmit(&huart3, (uint8_t *)"USB OK\r\n", 8, 100);

  MX_I2C1_Init();
  HAL_UART_Transmit(&huart3, (uint8_t *)"I2C1 OK\r\n", 9, 100);

  MX_ADC1_Init();
  HAL_UART_Transmit(&huart3, (uint8_t *)"ADC1 OK\r\n", 9, 100);

  MX_SPI2_Init();
  HAL_UART_Transmit(&huart3, (uint8_t *)"SPI2 OK\r\n", 9, 100);

  /* USER CODE BEGIN 2 */
  /* Manually initialize I2C1 and LoRa GPIO (since they aren't generated by
   * CubeMX yet) */
  User_Periph_Init();
  HAL_UART_Transmit(&huart3, (uint8_t *)"Periph OK\r\n", 11, 100);

  /* Seed PRNG from system tick for randomized CAD backoff */
  rng_state = HAL_GetTick() ^ 0xDEADBEEF;

  /* Debug: read version register raw before full init */
  {
    char dbg[64];
    uint8_t ver = SX1278_ReadRegister(0x42);
    snprintf(dbg, sizeof(dbg), "DEBUG: REG_VERSION raw = 0x%02X\r\n", ver);
    HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 100);
  }

  /* Initialize LoRa */
  if (SX1278_Init() == 1) {
    char *msg = "LoRa initialized OK (VERSION=0x12)\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
  } else {
    char *msg = "LoRa FAILED — SPI not reaching SX1278. Check wiring.\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
    while (1) {
      HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
      HAL_Delay(200);
    }
  }

  /* Initialize MPU6050 */
  if (MPU6050_Init(&hi2c1) == 1) {
    char *msg2 = "MPU6050 initialized OK\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)msg2, strlen(msg2), 100);

    /* Configure MPU6050 for Motion Detection (Hardware Interrupt) */
    uint8_t data;
    /* 1. Set HPF to 5Hz (ACCEL_CONFIG 0x1C) */
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1C, 1, &data, 1, 100);
    /* 2. Configure INT pin (INT_PIN_CFG 0x37) Latch enable, clear on read */
    data = 0x20;
    HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x37, 1, &data, 1, 100);
    /* 3. Enable Motion Interrupt (INT_ENABLE 0x38) */
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x38, 1, &data, 1, 100);
    /* 4. Set Threshold (MOT_THR 0x1F) 1 LSB = 2mg. 30 = 60mg. */
    data = 30;
    HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1F, 1, &data, 1, 100);
    /* 5. Set Duration (MOT_DUR 0x20) 1 LSB = 1ms. 2ms. */
    data = 2;
    HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x20, 1, &data, 1, 100);

    /* Clear any initial interrupt */
    HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3A, 1, &data, 1, 100);

  } else {
    char *msg2 = "MPU6050 failed to initialize\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)msg2, strlen(msg2), 100);
  }

  /* Enable EXTI0 interrupt for MPU6050 motion detection */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* -----------------------------------------------------------------------
   * CREATE FreeRTOS QUEUES
   * ----------------------------------------------------------------------- */
  xSensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(SensorData_t));
  xTxQueue     = xQueueCreate(TX_QUEUE_SIZE,     sizeof(TxPacket_t));

  if (xSensorQueue == NULL || xTxQueue == NULL) {
    HAL_UART_Transmit(&huart3,
      (uint8_t *)"FATAL: Queue creation failed!\r\n", 30, 100);
    Error_Handler();
  }

  /* -----------------------------------------------------------------------
   * CREATE FreeRTOS TASKS
   *
   * Priority hierarchy (higher number = higher priority):
   *   Task 3 LoRa Communication  = 4 (osPriorityAboveNormal)
   *   Task 1 Sensor Acquisition  = 3 (osPriorityNormal)
   *   Task 2 Threshold Detection = 3 (osPriorityNormal)
   *   Task 4 System Monitor      = 1 (osPriorityLow)
   * ----------------------------------------------------------------------- */
  BaseType_t ret;

  ret = xTaskCreate(vSensorAcquisitionTask, "SensorTask",
                    256, NULL, 3, &xSensorTaskHandle);
  if (ret != pdPASS) Error_Handler();

  ret = xTaskCreate(vThresholdDetectionTask, "ProcessTask",
                    256, NULL, 3, &xProcessTaskHandle);
  if (ret != pdPASS) Error_Handler();

  ret = xTaskCreate(vLoRaCommunicationTask, "LoRaTask",
                    512, NULL, 4, &xLoRaTaskHandle);
  if (ret != pdPASS) Error_Handler();

  ret = xTaskCreate(vSystemMonitorTask, "MonitorTask",
                    256, NULL, 1, &xMonitorTaskHandle);
  if (ret != pdPASS) Error_Handler();

  HAL_UART_Transmit(&huart3,
    (uint8_t *)"\r\n=== FreeRTOS Scheduler Starting ===\r\n\r\n", 41, 100);

  /* -----------------------------------------------------------------------
   * START THE SCHEDULER
   * This call never returns. All execution proceeds through tasks.
   * ----------------------------------------------------------------------- */
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* We should never get here as the scheduler takes over.
   * If we do, it means there was insufficient FreeRTOS heap. */
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin | SPI2_NSS_Pin | SPI2_RST_Pin | LD2_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_INT_Pin (EXTI Interrupt) */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin SPI2_NSS_Pin SPI2_RST_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin | SPI2_NSS_Pin | SPI2_RST_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_DIO0_Pin — plain input, polled via SPI */
  GPIO_InitStruct.Pin = SPI2_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_DIO0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init — priority set to 5 for FreeRTOS safety */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void User_Periph_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable Clocks */
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* I2C1 GPIO Configuration: PB8->SCL, PB9->SDA */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Initialize I2C1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  /* LoRa RST (PB5) Output Configuration */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* LoRa NSS / CS (PB12) Output Configuration (starts HIGH) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI2 GPIO Configuration: PB13->SCK, PB14->MISO, PB15->MOSI */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* LoRa DIO0 (PB4) — plain input, polled via SPI register */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* MPU6050 INT (PC0) — EXTI rising edge for motion interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called by the HAL library when TIM7 interrupts to
  *         increment the HAL system tick (uwTick). This is crucial because
  *         FreeRTOS uses SysTick, leaving TIM7 to run the HAL timebase.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}
/* USER CODE END 4 */

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
//neww

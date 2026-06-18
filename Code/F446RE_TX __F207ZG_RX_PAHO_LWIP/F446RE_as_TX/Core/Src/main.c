/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : StructEye Node 4 — STM32F446RE FreeRTOS LoRa Transmitter
 *
 *                   Architecture:
 *                     Task 1 — Sensor Acquisition   (EXTI + periodic poll)
 *                     Task 2 — Threshold Detection  (vibration/tilt analysis)
 *                     Task 3 — LoRa Communication   (CAD + TX)
 *                     Task 4 — System Monitor        (health telemetry)
 *
 *                   Inter-task communication via FreeRTOS Queues.
 *                   Motion interrupt (MPU6050 INT on PA8/EXTI8) wakes
 *                   Task 1 via vTaskNotifyGiveFromISR for sub-5ms latency.
 *
 *                   Pin Assignment (CN5/CN6/CN9 Arduino headers ONLY):
 *                     SPI1: PA5(SCK), PA6(MISO), PA7(MOSI) — CN5
 *                     LoRa: PB6(NSS), PA9(RST), PA10(DIO0) — per CubeMX
 *                     I2C1: PB8(SCL), PB9(SDA) — CN5
 *                     MPU:  PA8(INT) — CN5
 *                     UART: PA2(TX), PA3(RX) — CN9 (ST-Link VCP)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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
#define NODE_ID               0x04   /* Node 4 = F446RE Sensor Node */

/* Heartbeat & Alert timing (in RTOS ticks, 1 tick = 1ms) */
#define HEARTBEAT_PERIOD_MS   30000  /* 30 seconds normal heartbeat */
#define ALERT_TX_INTERVAL_MS  2000   /* 2 seconds between alert packets */
#define SENSOR_POLL_MS        50     /* 50ms sensor polling interval */
#define ALERT_STABLE_SAMPLES  5      /* debounce alert transitions */

/* Threshold values (Dynamic shaking above calibrated rest baseline) */
#define VIB_WARN_HIGH         1.2f   /* m/s² — dynamic shake warning */
#define VIB_CRIT_HIGH         3.0f   /* m/s² — severe dynamic shaking */
#define TILT_WARN_DEG         10.0f  /* degrees */
#define TILT_CRIT_DEG         15.0f  /* degrees */
/* Hysteresis (exit thresholds) to avoid WARN/OK chatter */
#define VIB_WARN_LOW          0.8f
#define VIB_CRIT_LOW          2.4f
#define TILT_WARN_LOW_DEG     8.0f
#define TILT_CRIT_LOW_DEG     12.0f

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
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
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
uint16_t crc16(const uint8_t *data, uint16_t len);

/* FreeRTOS Task Functions */
void vSensorAcquisitionTask(void *pvParameters);
void vThresholdDetectionTask(void *pvParameters);
void vLoRaCommunicationTask(void *pvParameters);
void vSystemMonitorTask(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf redirect to USART2 (ST-Link VCP) */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

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
 *   MPU6050 INT pin (PA8) → EXTI8 ISR → HAL_GPIO_EXTI_Callback →
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
      HAL_UART_Transmit(&huart2, (uint8_t *)intMsg, strlen(intMsg), 50);
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
 * - Runs event-driven transmit policy:
 *     1) Always send first packet after boot
 *     2) Send when alert level changes (0/1/2 transition)
 *     3) Otherwise send heartbeat every HEARTBEAT_PERIOD_MS (30s)
 * - Constructs the 23-byte StructEye frame with CRC-16.
 * - Pushes TxPacket_t to xTxQueue for Task 3.
 * ======================================================================== */
void vThresholdDetectionTask(void *pvParameters) {
  (void)pvParameters;

  SensorData_t rxData;
  TxPacket_t   txPacket;
  uint8_t      seq_num = 0;
  TickType_t   lastTxTick = 0;
  uint8_t      prev_alert_level = 0;
  uint8_t      pending_alert_level = 0;
  uint8_t      pending_alert_count = 0;
  char uartMsg[128];

  float base_gravity = 9.81f;
  uint16_t cal_samples = 0;
  float cal_sum = 0.0f;

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
        HAL_UART_Transmit(&huart2, (uint8_t *)uartMsg, strlen(uartMsg), 100);
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
    uint8_t candidate_alert = 0;
    float abs_tilt = fabsf(tilt);

    /* Hysteretic classifier for alert level (0=OK,1=WARN,2=CRITICAL). */
    if (prev_alert_level >= 2) {
      if (vibration > VIB_CRIT_LOW || abs_tilt > TILT_CRIT_LOW_DEG) {
        candidate_alert = 2;
      } else if (vibration > VIB_WARN_LOW || abs_tilt > TILT_WARN_LOW_DEG) {
        candidate_alert = 1;
      }
    } else if (prev_alert_level == 1) {
      if (vibration > VIB_CRIT_HIGH || abs_tilt > TILT_CRIT_DEG) {
        candidate_alert = 2;
      } else if (vibration > VIB_WARN_LOW || abs_tilt > TILT_WARN_LOW_DEG) {
        candidate_alert = 1;
      }
    } else {
      if (vibration > VIB_CRIT_HIGH || abs_tilt > TILT_CRIT_DEG) {
        candidate_alert = 2;
      } else if (vibration > VIB_WARN_HIGH || abs_tilt > TILT_WARN_DEG) {
        candidate_alert = 1;
      }
    }

    /* Sensor fault flag */
    if (!rxData.sensor_ok) {
      status_flags |= (1 << 2);
      candidate_alert = 2;
    }

    /* Boot flag — first packet after reset */
    if (seq_num == 0)
      status_flags |= (1 << 3);

    /* Debounce alert transitions to suppress noise-driven flapping.
     * Require 5 consecutive samples at the new level before accepting. */
    if (candidate_alert != pending_alert_level) {
      pending_alert_level = candidate_alert;
      pending_alert_count = 1;
    } else if (pending_alert_count < 255) {
      pending_alert_count++;
    }
    if (candidate_alert != prev_alert_level &&
        pending_alert_count < ALERT_STABLE_SAMPLES) {
      alert_level = prev_alert_level;
    } else {
      alert_level = candidate_alert;
    }

    /* Status bits reflect current critical conditions. */
    if (vibration > VIB_CRIT_HIGH) {
      status_flags |= (1 << 0);
    }
    if (abs_tilt > TILT_CRIT_DEG) {
      status_flags |= (1 << 1);
    }

    /* --- Update alert state machine (Matched to F207ZG Logic) --- */
    static uint8_t alert_active = 0;
    static uint8_t cooldown_counter = 0;

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
      tx_interval = pdMS_TO_TICKS(100);             // 100ms rapid-fire during alert
    } else if (cooldown_counter > 0) {
      tx_interval = pdMS_TO_TICKS(2000);            // 2 seconds redundant spacing
    } else {
      tx_interval = pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS); // 30 seconds heartbeat
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
      prev_alert_level = alert_level;

      /* Debug print */
      const char *reason;
      if (seq_num == 1) reason = "BOOT";
      else if (transition) reason = "EVENT";
      else if (alert_level > 0) reason = "ALERT";
      else reason = "HEARTBEAT";

      snprintf(uartMsg, sizeof(uartMsg),
               "[PROC] Seq=%u Vib=%.2f Tilt=%.2f Alert=%u (%s)\r\n",
               (unsigned)(seq_num - 1), vibration, tilt, alert_level,
               reason);
      HAL_UART_Transmit(&huart2, (uint8_t *)uartMsg, strlen(uartMsg), 50);
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
        HAL_UART_Transmit(&huart2, (uint8_t *)uartMsg, strlen(uartMsg), 50);

        vTaskDelay(pdMS_TO_TICKS(backoff));
      } else {
        /* Channel free — transmit immediately */
        HAL_UART_Transmit(&huart2, (uint8_t *)"[TX] Channel free — transmitting\r\n", 34, 50);
        SX1278_Transmit(rxPacket.frame, 23);
        HAL_UART_Transmit(&huart2, (uint8_t *)"[TX] Done\r\n", 11, 50);
        tx_success = 1;
        break;
      }
    }

    if (!tx_success) {
      /* Fallback: force transmit after all CAD retries exhausted.
       * It's better to risk a collision than to silently drop an alert. */
      HAL_UART_Transmit(&huart2, (uint8_t *)"[TX] FORCE — CAD retries exhausted\r\n", 35, 50);
      SX1278_Transmit(rxPacket.frame, 23);
      HAL_UART_Transmit(&huart2, (uint8_t *)"[TX] Done (forced)\r\n", 20, 50);
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
             "\r\n--- SYSTEM MONITOR (Node %d) ---\r\n"
             "Free Heap: %u bytes\r\n"
             "Stack High Water Marks (min free words):\r\n"
             "  Sensor Task:  %u\r\n"
             "  Process Task: %u\r\n"
             "  LoRa Task:    %u\r\n"
             "  Monitor Task: %u\r\n"
             "Sensor Queue: %u/%d | TX Queue: %u/%d\r\n"
             "----------------------\r\n",
             NODE_ID,
             (unsigned)freeHeap,
             (unsigned)sensorHW,
             (unsigned)processHW,
             (unsigned)loraHW,
             (unsigned)monitorHW,
             (unsigned)uxQueueMessagesWaiting(xSensorQueue), SENSOR_QUEUE_SIZE,
             (unsigned)uxQueueMessagesWaiting(xTxQueue), TX_QUEUE_SIZE);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartMsg, strlen(uartMsg), 200);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\n============================================================\r\n");
  printf("  StructEye Node 4 — STM32F446RE FreeRTOS LoRa Transmitter\r\n");
  printf("  Pins: SPI1(PA5/PA6/PA7) I2C1(PB8/PB9) LoRa(PB6/PA9/PA10)\r\n");
  printf("============================================================\r\n");

  /* Set NSS HIGH (deselected) and RST HIGH before LoRa init */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);

  /* Seed PRNG from system tick for randomized CAD backoff */
  rng_state = HAL_GetTick() ^ 0xDEADBEEF;

  /* Debug: read version register raw before full init */
  {
    char dbg[64];
    uint8_t ver = SX1278_ReadRegister(0x42);
    snprintf(dbg, sizeof(dbg), "DEBUG: REG_VERSION raw = 0x%02X\r\n", ver);
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), 100);
  }

  /* Initialize LoRa */
  if (SX1278_Init() == 1) {
    char *msg = "LoRa initialized OK (VERSION=0x12)\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
  } else {
    /* DIAGNOSTIC WIRE-WIGGLE LOOP */
    char msg[128];
    snprintf(msg, sizeof(msg), "\r\n[!] LoRa FAILED — SPI not reaching SX1278.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    snprintf(msg, sizeof(msg), "[!] Wiggle your wires now! I am polling SPI every 500ms...\r\n\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    
    while(1) {
        uint8_t ver = SX1278_ReadRegister(0x42);
        if (ver == 0x12) {
            snprintf(msg, sizeof(msg), ">>> SUCCESS! Connection restored! (ver=0x12) Reset the board!\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
        } else {
            snprintf(msg, sizeof(msg), "Wiggling... REG_VERSION = 0x%02X (Expected 0x12)\r\n", ver);
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
        }
        HAL_Delay(500);
    }
  }

  /* Initialize MPU6050 */
  if (MPU6050_Init(&hi2c1) == 1) {
    char *msg2 = "MPU6050 initialized OK\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg2, strlen(msg2), 100);

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
    HAL_UART_Transmit(&huart2, (uint8_t *)msg2, strlen(msg2), 100);
  }

  /* MPU INT on PA8 => EXTI9_5_IRQn */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* LoRa DIO0 on PA10 => EXTI15_10_IRQn (optional in TX mode) */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* -----------------------------------------------------------------------
   * CREATE FreeRTOS QUEUES
   * ----------------------------------------------------------------------- */
  xSensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(SensorData_t));
  xTxQueue     = xQueueCreate(TX_QUEUE_SIZE,     sizeof(TxPacket_t));

  if (xSensorQueue == NULL || xTxQueue == NULL) {
    HAL_UART_Transmit(&huart2,
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
                    512, NULL, 3, &xSensorTaskHandle);
  if (ret != pdPASS) Error_Handler();

  ret = xTaskCreate(vThresholdDetectionTask, "ProcessTask",
                    512, NULL, 3, &xProcessTaskHandle);
  if (ret != pdPASS) Error_Handler();

  ret = xTaskCreate(vLoRaCommunicationTask, "LoRaTask",
                    512, NULL, 4, &xLoRaTaskHandle);
  if (ret != pdPASS) Error_Handler();

  ret = xTaskCreate(vSystemMonitorTask, "MonitorTask",
                    512, NULL, 1, &xMonitorTaskHandle);
  if (ret != pdPASS) Error_Handler();

  HAL_UART_Transmit(&huart2,
    (uint8_t *)"\r\n=== FreeRTOS Scheduler Starting ===\r\n\r\n", 41, 100);

  /* -----------------------------------------------------------------------
   * START THE SCHEDULER
   * This call never returns. All execution proceeds through tasks.
   * ----------------------------------------------------------------------- */
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Should never reach here */
  Error_Handler();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU_INT_Pin LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin|LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RST_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
//NEw

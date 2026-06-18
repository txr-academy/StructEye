/*
 * StructEye - ESP32 Sensor Node 3 (FreeRTOS Multi-Task)
 * Node ID: 0x03
 * Hardware: ESP32 + MPU6050 + SX1278 LoRa
 *
 * Architecture (same as STM32 F207ZG):
 *   Task 1 — Sensor Acquisition   (GPIO interrupt + periodic poll)
 *   Task 2 — Threshold Detection  (vibration/tilt analysis)
 *   Task 3 — LoRa Communication   (CAD LBT + TX)
 *   Task 4 — System Monitor        (health telemetry)
 *
 * IPC (Inter-Process Communication) used:
 *   - FreeRTOS Queues      (xSensorQueue, xTxQueue)
 *   - Task Notifications   (ISR → Task 1 wake-up)
 *
 * NOTE: ESP32 Arduino already runs on FreeRTOS internally.
 *       setup() and loop() execute inside a FreeRTOS task.
 *       We create our own tasks and suspend the default loop task.
 */

#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <math.h>

/* ─── FreeRTOS headers (included with ESP32 Arduino core) ─── */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ═══════════════════════════════════════════════════════════════
 * CONFIGURATION — Change NODE_ID for each physical node
 * ═══════════════════════════════════════════════════════════════ */
#define NODE_ID 0x03

/* ─── Pin Definitions ─── */
#define MPU_ADDR  0x68
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK  18
#define LORA_NSS  5
#define LORA_RST  14
#define LORA_DIO0 26
#define MPU_INT   4    // GPIO4 connected to MPU6050 INT pin
#define LED_PIN   2

/* ─── Timing ─── */
#define HEARTBEAT_PERIOD_MS  30000  // 30s normal heartbeat
#define SENSOR_POLL_MS       50     // 50ms sensor polling interval
#define ALERT_STABLE_SAMPLES 3      // debounce alert transitions

/* ─── Thresholds (Dynamic shaking above calibrated baseline) ─── */
#define VIB_WARN_HIGH   1.2f
#define VIB_CRIT_HIGH   3.0f
#define TILT_WARN_DEG   10.0f
#define TILT_CRIT_DEG   15.0f
#define VIB_WARN_LOW    0.8f
#define VIB_CRIT_LOW    2.4f
#define TILT_WARN_LOW_DEG  8.0f
#define TILT_CRIT_LOW_DEG  12.0f

/* ─── CAD ─── */
#define CAD_MAX_RETRIES    15
#define CAD_BACKOFF_MIN_MS 20
#define CAD_BACKOFF_MAX_MS 80

/* ─── Queue Sizes ─── */
#define SENSOR_QUEUE_SIZE  10
#define TX_QUEUE_SIZE      5

/* ═══════════════════════════════════════════════════════════════
 * DATA STRUCTURES — passed between tasks via Queues (IPC)
 * ═══════════════════════════════════════════════════════════════ */

/* Sensor data: Task 1 → Task 2 via xSensorQueue */
typedef struct {
  float accelX, accelY, accelZ;  // m/s²
  float gyroX, gyroY, gyroZ;    // deg/s
  bool  sensor_ok;
} SensorData_t;

/* TX packet: Task 2 → Task 3 via xTxQueue */
typedef struct {
  uint8_t frame[23];
} TxPacket_t;

/* ═══════════════════════════════════════════════════════════════
 * GLOBAL HANDLES — FreeRTOS IPC objects
 * ═══════════════════════════════════════════════════════════════ */
TaskHandle_t  xSensorTaskHandle  = NULL;
TaskHandle_t  xProcessTaskHandle = NULL;
TaskHandle_t  xLoRaTaskHandle    = NULL;
TaskHandle_t  xMonitorTaskHandle = NULL;

QueueHandle_t xSensorQueue = NULL;  // SensorData_t: Task1 → Task2
QueueHandle_t xTxQueue     = NULL;  // TxPacket_t:   Task2 → Task3

RTC_DATA_ATTR uint8_t seq_num = 0;  // Survives deep sleep

/* ═══════════════════════════════════════════════════════════════
 * UTILITY FUNCTIONS
 * ═══════════════════════════════════════════════════════════════ */

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

/* Direct SPI register access (bypasses LoRa library private methods) */
uint8_t loraReadReg(uint8_t reg) {
  uint8_t val;
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_NSS, LOW);
  SPI.transfer(reg & 0x7F);
  val = SPI.transfer(0x00);
  digitalWrite(LORA_NSS, HIGH);
  SPI.endTransaction();
  return val;
}

void loraWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_NSS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(LORA_NSS, HIGH);
  SPI.endTransaction();
}

/* CAD — returns true if channel is busy */
bool isChannelBusy() {
  LoRa.idle();
  loraWriteReg(0x01, 0x85);  // LoRa mode + RXCONTINUOUS
  delay(2);
  uint8_t rssi_raw = loraReadReg(0x1B);
  LoRa.idle();
  return (rssi_raw > 70);
}

void configureMPU6050_WakeOnMotion() {
  // 1. Reset
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x80);
  Wire.endTransmission();
  delay(100);

  // 2. Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  // 3. HPF 5Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x01);
  Wire.endTransmission();

  // 4. INT pin config (latch)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37); Wire.write(0x20);
  Wire.endTransmission();

  // 5. Enable motion interrupt
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x38); Wire.write(0x40);
  Wire.endTransmission();

  // 6. Threshold = 30 (60mg)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1F); Wire.write(30);
  Wire.endTransmission();

  // 7. Duration = 2ms
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x20); Wire.write(2);
  Wire.endTransmission();
}

/* ═══════════════════════════════════════════════════════════════
 * ISR — MPU6050 Motion Interrupt (GPIO4)
 *
 * This is the fastest possible wake-up path:
 *   MPU6050 INT pin → GPIO ISR → xTaskNotifyGiveFromISR →
 *   portYIELD_FROM_ISR → Task 1 wakes in <10µs
 * ═══════════════════════════════════════════════════════════════ */
void IRAM_ATTR onMotionISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (xSensorTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(xSensorTaskHandle, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 1: SENSOR ACQUISITION
 * IPC Output: xSensorQueue (Queue)
 * IPC Input:  Task Notification (from ISR)
 * ═══════════════════════════════════════════════════════════════ */
void vSensorAcquisitionTask(void *pvParameters) {
  SensorData_t sensorData;

  for (;;) {
    /* Block until ISR notification OR timeout after SENSOR_POLL_MS */
    uint32_t notification = ulTaskNotifyTake(pdTRUE,
                                             pdMS_TO_TICKS(SENSOR_POLL_MS));

    if (notification > 0) {
      Serial.println(">>> HARDWARE INTERRUPT: Motion Detected! <<<");
    }

    /* Read MPU6050 — 14 bytes burst */
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);

    int16_t ax_h = Wire.read(); int16_t ax_l = Wire.read();
    int16_t ay_h = Wire.read(); int16_t ay_l = Wire.read();
    int16_t az_h = Wire.read(); int16_t az_l = Wire.read();
    int16_t temp_h = Wire.read(); int16_t temp_l = Wire.read();
    int16_t gx_h = Wire.read(); int16_t gx_l = Wire.read();
    int16_t gy_h = Wire.read(); int16_t gy_l = Wire.read();
    int16_t gz_h = Wire.read(); int16_t gz_l = Wire.read();

    int16_t ax = (ax_h << 8) | ax_l;
    int16_t ay = (ay_h << 8) | ay_l;
    int16_t az = (az_h << 8) | az_l;
    int16_t temp = (temp_h << 8) | temp_l;
    int16_t gx = (gx_h << 8) | gx_l;
    int16_t gy = (gy_h << 8) | gy_l;
    int16_t gz = (gz_h << 8) | gz_l;

    /* Clear MPU6050 interrupt latch */
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3A);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)1, true);
    Wire.read();

    /* Scale to engineering units */
    sensorData.accelX = (ax / 16384.0f) * 9.81f;
    sensorData.accelY = (ay / 16384.0f) * 9.81f;
    sensorData.accelZ = (az / 16384.0f) * 9.81f;
    sensorData.gyroX  = gx / 131.0f;
    sensorData.gyroY  = gy / 131.0f;
    sensorData.gyroZ  = gz / 131.0f;
    sensorData.sensor_ok = !(ax == 0 && ay == 0 && az == 0);

    /* Push to processing queue (IPC: Queue) */
    if (xQueueSend(xSensorQueue, &sensorData, 0) != pdPASS) {
      SensorData_t discard;
      xQueueReceive(xSensorQueue, &discard, 0);
      xQueueSend(xSensorQueue, &sensorData, 0);
    }
  }
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 2: PROCESSING & THRESHOLD DETECTION
 * IPC Input:  xSensorQueue (Queue — blocks until data arrives)
 * IPC Output: xTxQueue (Queue)
 * ═══════════════════════════════════════════════════════════════ */
void vThresholdDetectionTask(void *pvParameters) {
  SensorData_t rxData;
  TxPacket_t   txPacket;
  TickType_t   lastTxTick = 0;
  uint8_t      prev_alert_level = 0;
  uint8_t      pending_alert_level = 0;
  uint8_t      pending_alert_count = 0;

  float base_gravity = 9.81f;
  uint16_t cal_samples = 0;
  float cal_sum = 0.0f;

  for (;;) {
    if (xQueueReceive(xSensorQueue, &rxData, portMAX_DELAY) != pdPASS)
      continue;

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
        Serial.printf("[CAL] Auto-calibration done. Base Gravity = %.3f m/s2\n", base_gravity);
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
    if (accel_mag > 0.1f)
      tilt = acosf(maxAxis / accel_mag) * 180.0f / PI;

    uint8_t status_flags = 0;
    uint8_t alert_level  = 0;
    uint8_t candidate_alert = 0;
    float abs_tilt = fabsf(tilt);

    if (prev_alert_level >= 2) {
      if (vibration > VIB_CRIT_LOW || abs_tilt > TILT_CRIT_LOW_DEG)
        candidate_alert = 2;
      else if (vibration > VIB_WARN_LOW || abs_tilt > TILT_WARN_LOW_DEG)
        candidate_alert = 1;
    } else if (prev_alert_level == 1) {
      if (vibration > VIB_CRIT_HIGH || abs_tilt > TILT_CRIT_DEG)
        candidate_alert = 2;
      else if (vibration > VIB_WARN_LOW || abs_tilt > TILT_WARN_LOW_DEG)
        candidate_alert = 1;
    } else {
      if (vibration > VIB_CRIT_HIGH || abs_tilt > TILT_CRIT_DEG)
        candidate_alert = 2;
      else if (vibration > VIB_WARN_HIGH || abs_tilt > TILT_WARN_DEG)
        candidate_alert = 1;
    }

    if (!rxData.sensor_ok) {
      status_flags |= (1 << 2);
      candidate_alert = 2;
    }
    if (seq_num == 0)
      status_flags |= (1 << 3);

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

    if (vibration > VIB_CRIT_HIGH)
      status_flags |= (1 << 0);
    if (abs_tilt > TILT_CRIT_DEG)
      status_flags |= (1 << 1);

    uint8_t transition = (alert_level != prev_alert_level) ? 1 : 0;
    TickType_t now = xTaskGetTickCount();
    uint8_t force_tx = (transition || seq_num == 0) ? 1 : 0;

    if (!force_tx && (now - lastTxTick) < pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS))
      continue;

    /* Build 23-byte frame */
    txPacket.frame[0] = 0xAA;
    txPacket.frame[1] = NODE_ID;
    txPacket.frame[2] = seq_num++;

    int16_t ax_s   = (int16_t)(rxData.accelX * 100);
    int16_t ay_s   = (int16_t)(rxData.accelY * 100);
    int16_t az_s   = (int16_t)(rxData.accelZ * 100);
    int16_t gx_s   = (int16_t)(rxData.gyroX  * 100);
    int16_t gy_s   = (int16_t)(rxData.gyroY  * 100);
    int16_t gz_s   = (int16_t)(rxData.gyroZ  * 100);
    uint16_t vib_s = (uint16_t)(vibration     * 100);
    int16_t tilt_s = (int16_t)(tilt           * 100);

    txPacket.frame[3]  = (ax_s >> 8) & 0xFF;   txPacket.frame[4]  = ax_s & 0xFF;
    txPacket.frame[5]  = (ay_s >> 8) & 0xFF;   txPacket.frame[6]  = ay_s & 0xFF;
    txPacket.frame[7]  = (az_s >> 8) & 0xFF;   txPacket.frame[8]  = az_s & 0xFF;
    txPacket.frame[9]  = (gx_s >> 8) & 0xFF;   txPacket.frame[10] = gx_s & 0xFF;
    txPacket.frame[11] = (gy_s >> 8) & 0xFF;   txPacket.frame[12] = gy_s & 0xFF;
    txPacket.frame[13] = (gz_s >> 8) & 0xFF;   txPacket.frame[14] = gz_s & 0xFF;
    txPacket.frame[15] = (vib_s >> 8) & 0xFF;  txPacket.frame[16] = vib_s & 0xFF;
    txPacket.frame[17] = (tilt_s >> 8) & 0xFF; txPacket.frame[18] = tilt_s & 0xFF;

    txPacket.frame[19] = status_flags;
    txPacket.frame[20] = alert_level;

    uint16_t crc = crc16(txPacket.frame, 21);
    txPacket.frame[21] = (crc >> 8) & 0xFF;
    txPacket.frame[22] = crc & 0xFF;

    /* Push to LoRa TX queue (IPC: Queue) */
    if (xQueueSend(xTxQueue, &txPacket, pdMS_TO_TICKS(100)) == pdPASS) {
      lastTxTick = now;
      prev_alert_level = alert_level;
      Serial.printf("[PROC] Seq=%u Vib=%.2f Tilt=%.2f Alert=%u (%s)\n",
                    (unsigned)(seq_num - 1), vibration, tilt, alert_level,
                    transition ? "EVENT" : "HEARTBEAT");
    }
  }
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 3: LoRa COMMUNICATION (CAD + TX)
 * IPC Input: xTxQueue (Queue — blocks until packet arrives)
 * ═══════════════════════════════════════════════════════════════ */
void vLoRaCommunicationTask(void *pvParameters) {
  TxPacket_t rxPacket;

  for (;;) {
    if (xQueueReceive(xTxQueue, &rxPacket, portMAX_DELAY) != pdPASS)
      continue;

    /* CAD Listen-Before-Talk */
    bool tx_success = false;
    for (int retries = 0; retries < CAD_MAX_RETRIES; retries++) {
      if (isChannelBusy()) {
        uint32_t backoff = random(CAD_BACKOFF_MIN_MS, CAD_BACKOFF_MAX_MS);
        Serial.printf("[CAD] Channel busy — backing off %lums (%d/%d)\n",
                      backoff, retries + 1, CAD_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(backoff));  // Non-blocking to other tasks!
      } else {
        Serial.println("[TX] Channel free — transmitting");
        LoRa.beginPacket();
        LoRa.write(rxPacket.frame, 23);
        LoRa.endPacket();
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(20));
        digitalWrite(LED_PIN, LOW);
        Serial.println("[TX] Done");
        tx_success = true;
        break;
      }
    }

    if (!tx_success) {
      Serial.println("[TX] FORCE — CAD retries exhausted");
      LoRa.beginPacket();
      LoRa.write(rxPacket.frame, 23);
      LoRa.endPacket();
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(20));
      digitalWrite(LED_PIN, LOW);
    }
  }
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 4: SYSTEM MONITOR
 * No IPC — standalone monitoring task
 * ═══════════════════════════════════════════════════════════════ */
void vSystemMonitorTask(void *pvParameters) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    UBaseType_t sensorHW  = uxTaskGetStackHighWaterMark(xSensorTaskHandle);
    UBaseType_t processHW = uxTaskGetStackHighWaterMark(xProcessTaskHandle);
    UBaseType_t loraHW    = uxTaskGetStackHighWaterMark(xLoRaTaskHandle);
    UBaseType_t monitorHW = uxTaskGetStackHighWaterMark(xMonitorTaskHandle);

    Serial.printf("\n--- SYSTEM MONITOR (Node %d) ---\n", NODE_ID);
    Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Stack HWM: Sensor=%u Process=%u LoRa=%u Monitor=%u\n",
                  sensorHW, processHW, loraHW, monitorHW);
    Serial.printf("Queues: Sensor=%u/%d TX=%u/%d\n",
                  uxQueueMessagesWaiting(xSensorQueue), SENSOR_QUEUE_SIZE,
                  uxQueueMessagesWaiting(xTxQueue), TX_QUEUE_SIZE);
    Serial.println("-------------------------------");
  }
}

/* ═══════════════════════════════════════════════════════════════
 * SETUP — runs once, initializes hardware, creates RTOS tasks
 * ═══════════════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n========================================");
  Serial.printf("  StructEye ESP32 FreeRTOS Node %d\n", NODE_ID);
  Serial.println("========================================");

  Wire.begin(21, 22);
  randomSeed(analogRead(34) + NODE_ID * 100);

  /* Initialize LoRa */
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    delay(1000);
    ESP.restart();
  }
  LoRa.setTxPower(2);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  Serial.println("LoRa OK");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  /* Configure MPU6050 motion detection */
  configureMPU6050_WakeOnMotion();
  Serial.println("MPU6050 OK");

  /* Attach ISR — uses Task Notification (IPC) to wake Task 1 */
  pinMode(MPU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT), onMotionISR, RISING);

  /* ───────────────────────────────────────────────────────
   * CREATE IPC OBJECTS (Queues)
   * ─────────────────────────────────────────────────────── */
  xSensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(SensorData_t));
  xTxQueue     = xQueueCreate(TX_QUEUE_SIZE,     sizeof(TxPacket_t));

  if (xSensorQueue == NULL || xTxQueue == NULL) {
    Serial.println("FATAL: Queue creation failed!");
    ESP.restart();
  }

  /* ───────────────────────────────────────────────────────
   * CREATE FreeRTOS TASKS
   *
   * On ESP32, tasks can be pinned to specific cores:
   *   Core 0 = WiFi/BT stack (we don't use it)
   *   Core 1 = Arduino loop (default)
   *
   * We pin sensor-critical tasks to Core 1 and
   * the monitor task to Core 0 (idle core).
   * ─────────────────────────────────────────────────────── */
  xTaskCreatePinnedToCore(vSensorAcquisitionTask,  "SensorTask",
                          4096, NULL, 3, &xSensorTaskHandle, 1);

  xTaskCreatePinnedToCore(vThresholdDetectionTask, "ProcessTask",
                          4096, NULL, 3, &xProcessTaskHandle, 1);

  xTaskCreatePinnedToCore(vLoRaCommunicationTask,  "LoRaTask",
                          4096, NULL, 4, &xLoRaTaskHandle, 1);

  xTaskCreatePinnedToCore(vSystemMonitorTask,      "MonitorTask",
                          4096, NULL, 1, &xMonitorTaskHandle, 0);

  Serial.println("\n=== FreeRTOS Tasks Created — Scheduler Running ===\n");

  /* Stagger initial boot transmission by Node ID to desynchronize */
  delay(NODE_ID * 250);
}

/* ═══════════════════════════════════════════════════════════════
 * LOOP — empty because all work is done by FreeRTOS tasks
 *
 * The Arduino loop() runs as the lowest-priority "loopTask".
 * We just yield to let the RTOS scheduler run our real tasks.
 * ═══════════════════════════════════════════════════════════════ */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));  // Yield — all work in tasks above
}

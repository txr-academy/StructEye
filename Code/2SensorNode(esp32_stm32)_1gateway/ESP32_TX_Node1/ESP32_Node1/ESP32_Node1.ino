/*
 * StructEye - ESP32 Sensor Node 1
 * Node ID: 0x01
 * Hardware: ESP32 + MPU6050 + SX1278 LoRa
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <math.h>

#define NODE_ID       0x01
#define MPU_ADDR      0x68
#define LORA_MISO     19
#define LORA_MOSI     23
#define LORA_SCK      18
#define LORA_NSS      5
#define LORA_RST      14
#define LORA_DIO0     26

uint8_t seq_num = 0;

uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.printf("  StructEye ESP32 TX Node %d Started.\n", NODE_ID);
  Serial.println("========================================");

  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 not found!");
  } else {
    Serial.println("MPU6050 connected.");
  }

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  // Accel: +/- 2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();
  // Gyro: +/- 250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();

  Serial.println("Initializing LoRa...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
  } else {
    Serial.println("LoRa initialized.");
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(10);
    LoRa.setSyncWord(0x12);
  }
  Serial.println("System ready.\n");
}

void loop() {
  Serial.println("\n--- Reading Sensor Data ---");

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  float accelX = (ax / 16384.0) * 9.81;
  float accelY = (ay / 16384.0) * 9.81;
  float accelZ = (az / 16384.0) * 9.81;
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  float vibration = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);

  // Orientation-independent tilt: works no matter which axis points up
  float maxAxis = fabs(accelX);
  if (fabs(accelY) > maxAxis) maxAxis = fabs(accelY);
  if (fabs(accelZ) > maxAxis) maxAxis = fabs(accelZ);
  float tilt = 0.0;
  if (vibration > 0.1) {
    tilt = acos(maxAxis / vibration) * 180.0 / PI;
  }

  Serial.printf("Vibration: %.2f m/s2, Tilt: %.2f deg\n", vibration, tilt);

  uint8_t status_flags = 0;
  uint8_t alert_level = 0;

  if (vibration > 12.0) status_flags |= (1 << 0);
  if (fabs(tilt) > 15.0) status_flags |= (1 << 1);
  if (ax == 0 && ay == 0 && az == 0) status_flags |= (1 << 2);
  if (seq_num == 0) status_flags |= (1 << 3);

  if ((status_flags & 0x03) != 0) {
    alert_level = 2;
  } else if (vibration > 10.0 || fabs(tilt) > 10.0) {
    alert_level = 1;
  }

  uint8_t frame[23];
  frame[0] = 0xAA;
  frame[1] = NODE_ID;
  frame[2] = seq_num++;

  int16_t ax_s = (int16_t)(accelX * 100);
  int16_t ay_s = (int16_t)(accelY * 100);
  int16_t az_s = (int16_t)(accelZ * 100);
  int16_t gx_s = (int16_t)(gyroX * 100);
  int16_t gy_s = (int16_t)(gyroY * 100);
  int16_t gz_s = (int16_t)(gyroZ * 100);
  uint16_t vib_s = (uint16_t)(vibration * 100);
  int16_t tilt_s = (int16_t)(tilt * 100);

  frame[3]  = (ax_s >> 8) & 0xFF;  frame[4]  = ax_s & 0xFF;
  frame[5]  = (ay_s >> 8) & 0xFF;  frame[6]  = ay_s & 0xFF;
  frame[7]  = (az_s >> 8) & 0xFF;  frame[8]  = az_s & 0xFF;
  frame[9]  = (gx_s >> 8) & 0xFF;  frame[10] = gx_s & 0xFF;
  frame[11] = (gy_s >> 8) & 0xFF;  frame[12] = gy_s & 0xFF;
  frame[13] = (gz_s >> 8) & 0xFF;  frame[14] = gz_s & 0xFF;
  frame[15] = (vib_s >> 8) & 0xFF; frame[16] = vib_s & 0xFF;
  frame[17] = (tilt_s >> 8) & 0xFF; frame[18] = tilt_s & 0xFF;
  frame[19] = status_flags;
  frame[20] = alert_level;

  uint16_t crc = crc16(frame, 21);
  frame[21] = (crc >> 8) & 0xFF;
  frame[22] = crc & 0xFF;

  Serial.print("Dataframe (HEX): ");
  for (int i = 0; i < 23; i++) {
    if (frame[i] < 0x10) Serial.print("0");
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("Sending packet via LoRa...");
  LoRa.beginPacket();
  LoRa.write(frame, 23);
  LoRa.endPacket();
  Serial.println("LoRa Packet sent successfully.");
  Serial.println("Waiting 10 seconds...");
  delay(10000);
}

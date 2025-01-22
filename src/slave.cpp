/**
 * @file main.cpp
 * @author Victor Alessi
 * @brief This is the slave device code. Empompasses slave bt functionalities,
 * MPU readings and sendind data.
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include "BluetoothSerial.h"

/**
 * @brief MPU DEFINITIONS
 *
 */
Adafruit_MPU6050 mpu;

// Counters and accumulators for averaging
int sample_count = 0;
float sum_ax = 0, sum_ay = 0, sum_az = 0;
float sum_gx = 0, sum_gy = 0, sum_gz = 0;

/**
 * @brief BLUETOOTH DEFINITIONS
 *
 */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

String device_name = "ESP32-BT-Slave"; // BT: Variable used to store the CLIENT(slave) bluetooth device name
String MACadd = "64:B7:08:29:35:72";
uint8_t address[6]; // BT: Variable used to store the CLIENT(Slave) MAC address used for the connection; Use your own andress in the same format

#define MAX_BUFFER_SIZE 256
#define MAX_BUFFER_SIZE_6 1550 //6*256+space for char

// Define buffers as character arrays
char message[MAX_BUFFER_SIZE_6]; // used for displaying on the serial monitor
char buffer[MAX_BUFFER_SIZE_6];  // used to concatenate the data and send the message as a single package
char buffer_acceleration_X[MAX_BUFFER_SIZE];
char buffer_acceleration_Y[MAX_BUFFER_SIZE];
char buffer_acceleration_Z[MAX_BUFFER_SIZE];
char buffer_gyro_X[MAX_BUFFER_SIZE];
char buffer_gyro_Y[MAX_BUFFER_SIZE];
char buffer_gyro_Z[MAX_BUFFER_SIZE];
bool ack_received = true;
int i = 0;
int last_ack = 0;

void setup(void)
{
  // SERIAL BEGIN
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // MPU INIT
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // BLUETOOTH INIT
  esp_read_mac(address, ESP_MAC_BT);
  SerialBT.begin(device_name); // Bluetooth device name
  while (!SerialBT.connected(1000))
  {
    Serial.println("Waiting for connection...");
  }
  ack_received = true;
  while (i < 6)
  {
    Serial.print(address[i], HEX);
    Serial.print("\t");
    i++;
  }
  Serial.println();

  memset(message, 0, sizeof(message));
  memset(buffer, 0, sizeof(buffer));
  memset(buffer_acceleration_X, 0, sizeof(buffer_acceleration_X));
  memset(buffer_acceleration_Y, 0, sizeof(buffer_acceleration_Y));
  memset(buffer_acceleration_Z, 0, sizeof(buffer_acceleration_Z));
  memset(buffer_gyro_X, 0, sizeof(buffer_gyro_X));
  memset(buffer_gyro_Y, 0, sizeof(buffer_gyro_Y));
  memset(buffer_gyro_Z, 0, sizeof(buffer_gyro_Z));
}

void loop()
{
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;

  if (mpu.getEvent(&a, &g, &temp))
  {
    sum_ax += a.acceleration.x;
    sum_ay += a.acceleration.y;
    sum_az += a.acceleration.z;

    sum_gx += g.gyro.x;
    sum_gy += g.gyro.y;
    sum_gz += g.gyro.z;

    sample_count++;
    Serial.println(ack_received);
    if (sample_count >= 100 && ack_received) {
      snprintf(buffer, sizeof(buffer), "%f,%f,%f,%f,%f,%f!",
               sum_ax / sample_count, sum_ay / sample_count, sum_az / sample_count,
               sum_gx / sample_count, sum_gy / sample_count, sum_gz / sample_count);
      SerialBT.write((uint8_t *)buffer, strlen(buffer));
      ack_received = false; // Attendre le prochain ACK

      // Reset for next batch
      memset(message, 0, sizeof(message));
      memset(buffer, 0, sizeof(buffer));
      memset(buffer_acceleration_X, 0, sizeof(buffer_acceleration_X));
      memset(buffer_acceleration_Y, 0, sizeof(buffer_acceleration_Y));
      memset(buffer_acceleration_Z, 0, sizeof(buffer_acceleration_Z));
      memset(buffer_gyro_X, 0, sizeof(buffer_gyro_X));
      memset(buffer_gyro_Y, 0, sizeof(buffer_gyro_Y));
      memset(buffer_gyro_Z, 0, sizeof(buffer_gyro_Z));
      sum_ax = 0;
      sum_ay = 0;
      sum_az = 0;
      sum_gx = 0;
      sum_gy = 0;
      sum_gz = 0;
      sample_count = 0;
    }

    if (millis() - last_ack > 1000) {
      ack_received = true;
    }

    // Vérifier si un ACK est reçu
    if (SerialBT.available()) {
      char ack = SerialBT.read(); // Lire un caractère depuis le buffer
      if (ack == 'A') {           // 'A' représente un accusé de réception
        ack_received = true;
        last_ack = millis();
      }
    }
  }
}
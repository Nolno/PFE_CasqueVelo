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
#include <Wire.h>
#include <Arduino.h>
#include "MPU.h"
#include "BluetoothSerial.h"


#define INTERRUPT_PIN 15

//############################ MPU  SENSOR ###################################
MPU mpu(INTERRUPT_PIN);


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
uint8_t address[6];
// BT: Variable used to store the CLIENT(Slave) MAC address used for the connection; Use your own andress in the same format

#define MAX_BUFFER_SIZE 256
#define MAX_BUFFER_SIZE_6 1550 //6*256+space for char

// Define buffers as character arrays
char message[MAX_BUFFER_SIZE_6]; // used for displaying on the serial monitor
char buffer[MAX_BUFFER_SIZE_6]; // used to concatenate the data and send the message as a single package

float avg_yaw = 0, avg_pitch = 0, avg_roll = 0;


bool ack_received = true;
int i = 0;
unsigned long last_ack = 0;
unsigned long temps;

void setup()
{
    Wire.begin();
    // SERIAL BEGIN
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    //############################ MPU  Setup ###################################
    mpu.initialize();

    //############################ BLUETOOTH Setup ###################################
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
}

void loop()
{
    mpu.update();
    if (ack_received)
    {
        mpu.getAveragedYPR(avg_yaw, avg_pitch, avg_roll);
        snprintf(buffer, sizeof(buffer), "%f,%f,%f!",
                 avg_yaw, avg_pitch, avg_roll);
        SerialBT.write((uint8_t*)buffer, strlen(buffer));
        Serial.println(buffer);
        ack_received = false; // Attendre le prochain ACK

        // Reset for next batch
        memset(message, 0, sizeof(message));
        memset(buffer, 0, sizeof(buffer));
    }

    if (millis() - last_ack > 1000)
    {
        ack_received = true;
    }

    // Vérifier si un ACK est reçu
    if (SerialBT.available())
    {
        char ack = SerialBT.read(); // Lire un caractère depuis le buffer
        if (ack == 'A')
        {
            // 'A' représente un accusé de réception
            ack_received = true;
            last_ack = millis();
        }
    }
}

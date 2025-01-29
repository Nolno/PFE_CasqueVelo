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
#include "MPU6050_6Axis_MotionApps20.h"
#include "BluetoothSerial.h"


#define INTERRUPT_PIN 15

//############################ MPU  SENSOR ###################################
MPU6050 mpu;
/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false; // Set true if DMP init was successful
uint8_t MPUIntStatus; // Holds actual interrupt status byte from MPU
uint8_t devStatus; // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q; // [w, x, y, z]         Quaternion container
VectorInt16 aa; // [x, y, z]            Accel sensor measurements
VectorInt16 gy; // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal; // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            Gravity vector
float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// Counters and accumulators for averaging
int sample_count = 0;
float sum_yaw = 0, sum_pitch = 0, sum_roll = 0;
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
uint8_t address[6];
// BT: Variable used to store the CLIENT(Slave) MAC address used for the connection; Use your own andress in the same format

#define MAX_BUFFER_SIZE 256
#define MAX_BUFFER_SIZE_6 1550 //6*256+space for char

// Define buffers as character arrays
char message[MAX_BUFFER_SIZE_6]; // used for displaying on the serial monitor
char buffer[MAX_BUFFER_SIZE_6]; // used to concatenate the data and send the message as a single package
#define BUFFER_SIZE 100
float yaw_buffer[BUFFER_SIZE];
float pitch_buffer[BUFFER_SIZE];
float roll_buffer[BUFFER_SIZE];
int buffer_index = 0;  // Index actuel dans le buffer
bool buffer_full = false;  // Indique si on a rempli au moins 100 valeurs
float avg_yaw = 0, avg_pitch = 0, avg_roll = 0;


bool ack_received = true;
int i = 0;
int last_ack = 0;
unsigned long temps;


volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady()
{
    MPUInterrupt = true;
}

void addSample(float new_yaw, float new_pitch, float new_roll);

void getAveragedValues(float &avg_yaw, float &avg_pitch, float &avg_roll);
void setup()
{
    Wire.begin();
    // SERIAL BEGIN
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    //############################ MPU  Setup ###################################
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    /*Verify connection*/
    Serial.println(F("Testing MPU6050 connection..."));
    if (mpu.testConnection() == false)
    {
        Serial.println("MPU6050 connection failed");
        while (true);
    }
    Serial.println("MPU6050 connection successful");
    //############################ DMP  Setup ###################################
    /* Initializate and configure the DMP*/
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    /* Making sure it worked (returns 0 if so) */
    if (devStatus == 0)
    {
        constexpr int loops = 100;
        mpu.CalibrateAccel(loops); // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(loops);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP...")); //Turning ON DMP
        mpu.setDMPEnabled(true);

        /*Enable Arduino interrupt detection*/
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code ")); //Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }
    pinMode(A5, OUTPUT);
    Serial.println("time,yaw,pitch,roll");
    temps = millis();

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

}

void loop()
{

    /* Read a packet from FIFO */
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
    {
        // Get the Latest packet

        /* Display Euler angles in degrees */
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        // applyRotationCorrection(q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        addSample(ypr[0], ypr[1], ypr[2]);

        if (ack_received)
        {
            getAveragedValues(avg_yaw, avg_pitch, avg_roll);
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
}

void addSample(float new_yaw, float new_pitch, float new_roll) {
    yaw_buffer[buffer_index] = new_yaw;
    pitch_buffer[buffer_index] = new_pitch;
    roll_buffer[buffer_index] = new_roll;

    buffer_index = (buffer_index + 1) % BUFFER_SIZE; // Tourne en boucle
    if (buffer_index == 0) buffer_full = true;  // Après 100 valeurs, on est plein
}

void getAveragedValues(float &avg_yaw, float &avg_pitch, float &avg_roll) {
    int count = buffer_full ? BUFFER_SIZE : buffer_index;
    if (count == 0) return;  // Évite division par 0

    float sum_yaw = 0, sum_pitch = 0, sum_roll = 0;
    for (int i = 0; i < count; i++) {
        sum_yaw += yaw_buffer[i];
        sum_pitch += pitch_buffer[i];
        sum_roll += roll_buffer[i];
    }

    avg_yaw = sum_yaw / count;
    avg_pitch = sum_pitch / count;
    avg_roll = sum_roll / count;
}

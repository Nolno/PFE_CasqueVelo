//
// Created by Antoine on 29/01/2025.
//

#include "MPU.h"

MPU::MPU(int interruptPin) : INTERRUPT_PIN(interruptPin), buffer_index(0), buffer_full(false) {}

void MPU::initialize() {
    Wire.begin();
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    /*Verify connection*/
    Serial.println(F("Testing MPU6050 connection..."));
    if (mpu.testConnection() == false) {
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
    if (devStatus == 0) {
        constexpr int loops = 100;
        mpu.CalibrateAccel(loops);  // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(loops);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));   //Turning ON DMP
        mpu.setDMPEnabled(true);
    }
    else {
        Serial.print(F("DMP Initialization failed (code ")); //Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }
    pinMode(A5, OUTPUT);
    Serial.println("time,yaw,pitch,roll");
    temps = millis();

    // On attend que le buffer se remplisse
    while (!buffer_full) {
        update();
    }
}

void MPU::update() {
    if (!DMPReady) return; // Stop the program if DMP programming fails.

    /* Read a packet from FIFO */
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet
        /* Display Euler angles in degrees */
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        addSample(ypr[0], ypr[1], ypr[2]);
        this->ypr[0] = ypr[0];
        this->ypr[1] = ypr[1];
        this->ypr[2] = ypr[2];
    }
}

float MPU::getYaw() {
    return ypr[0];
}

float MPU::getPitch() {
    return ypr[1];
}

float MPU::getRoll() {
    return ypr[2];
}

void MPU::getYPR(float (&ypr)[3]) {
    ypr[0] = this->ypr[0];
    ypr[1] = this->ypr[1];
    ypr[2] = this->ypr[2];
}

void MPU::addSample(float new_yaw, float new_pitch, float new_roll) {
    yaw_buffer[buffer_index] = new_yaw;
    pitch_buffer[buffer_index] = new_pitch;
    roll_buffer[buffer_index] = new_roll;

    buffer_index = (buffer_index + 1) % BUFFER_SIZE; // Tourne en boucle
    if (buffer_index == 0) buffer_full = true;  // Après 100 valeurs, on est plein
}

void MPU::getAveragedYPR(float &avg_yaw, float &avg_pitch, float &avg_roll) {
    if (!buffer_full) {
        avg_yaw = 0;
        avg_pitch = 0;
        avg_roll = 0;
        return;
    }

    float sum_yaw = 0;
    float sum_pitch = 0;
    float sum_roll = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum_yaw += yaw_buffer[i];
        sum_pitch += pitch_buffer[i];
        sum_roll += roll_buffer[i];
    }

    avg_yaw = sum_yaw / BUFFER_SIZE;
    avg_pitch = sum_pitch / BUFFER_SIZE;
    avg_roll = sum_roll / BUFFER_SIZE;
}

#include "MPU.h"

volatile bool MPU::MPUInterrupt = false;

MPU::MPU(int interruptPin) : INTERRUPT_PIN(interruptPin), buffer_index(0), buffer_full(false) {}

void MPU::initialize() {
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing MPU6050 connection..."));
    if (mpu.testConnection() == false) {
        Serial.println("MPU6050 connection failed");
        while (true);
    }
    Serial.println("MPU6050 connection successful");
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    if (devStatus == 0) {
        constexpr int loops = 100;
        mpu.CalibrateAccel(loops);
        mpu.CalibrateGyro(loops);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(A5, OUTPUT);
    Serial.println("time,yaw,pitch,roll");
    temps = millis();
    while (!buffer_full) {
        update();
    }
}

void MPU::update() {
    if (!DMPReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
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
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;
    if (buffer_index == 0) buffer_full = true;
}

void MPU::getAveragedYPR(float& avg_yaw, float& avg_pitch, float& avg_roll) {
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

void MPU::getAveragedYPR(float (&ypr)[3]) {
    float avg_yaw, avg_pitch, avg_roll;
    getAveragedYPR(avg_yaw, avg_pitch, avg_roll);
    ypr[0] = avg_yaw;
    ypr[1] = avg_pitch;
    ypr[2] = avg_roll;
}

void MPU::DMPDataReady() {
    MPUInterrupt = true;
}
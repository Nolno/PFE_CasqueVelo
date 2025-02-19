#include "MPU.h"

volatile bool MPU::MPUInterrupt = false;

MPU::MPU(int interruptPin) : yaw_buffer{}, pitch_buffer{}, roll_buffer{}, buffer_index(0), buffer_full(false),
                             startTime(0), ypr{},
                             DMPReady(false),
                             FIFOBuffer{},
                             packetSize(0),
                             devStatus(0),
                             MPUIntStatus(0),
                             INTERRUPT_PIN(interruptPin), temps(0)
{
}

void MPU::initialize()
{
    Serial.println("Initialisation ICM20948...");
    if (!icm.begin_I2C())
    {
        Serial.println("ICM20948 non détecté !");
        while (true) delay(10);
    }
    Serial.println("ICM20948 trouvé !");

    startTime = millis();
    Serial.println("Début de la calibration du magnétomètre...");


    while (isCalibrating)
    {
        sensors_event_t accel, gyro, mag, temp;
        icm.getEvent(&accel, &gyro, &temp, &mag);
        if (millis() - startTime < 30000)
        {
            mag_min_x = min(mag_min_x, mag.magnetic.x);
            mag_max_x = max(mag_max_x, mag.magnetic.x);
            mag_min_y = min(mag_min_y, mag.magnetic.y);
            mag_max_y = max(mag_max_y, mag.magnetic.y);
            mag_min_z = min(mag_min_z, mag.magnetic.z);
            mag_max_z = max(mag_max_z, mag.magnetic.z);
        }
        else
        {
            mag_offset_x = (mag_max_x + mag_min_x) / 2;
            mag_offset_y = (mag_max_y + mag_min_y) / 2;
            mag_offset_z = (mag_max_z + mag_min_z) / 2;

            mag_scale_x = (mag_max_x - mag_min_x) / 2;
            mag_scale_y = (mag_max_y - mag_min_y) / 2;
            mag_scale_z = (mag_max_z - mag_min_z) / 2;

            Serial.print("Offset: ");
            Serial.print(mag_offset_x);
            Serial.print(", ");
            Serial.print(mag_offset_y);
            Serial.print(", ");
            Serial.println(mag_offset_z);
            Serial.print("Scale: ");
            Serial.print(mag_scale_x);
            Serial.print(", ");
            Serial.print(mag_scale_y);
            Serial.print(", ");
            Serial.println(mag_scale_z);

            Serial.println("Calibration terminée !");
            isCalibrating = false;
        }
    }
}

void MPU::update()
{
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    float mag_x = (mag.magnetic.x - mag_offset_x) / mag_scale_x;
    float mag_y = (mag.magnetic.y - mag_offset_y) / mag_scale_y;
    float mag_z = (mag.magnetic.z - mag_offset_z) / mag_scale_z;
    // Serial.print("Mag: "); Serial.print(mag_x); Serial.print(", "); Serial.print(mag_y); Serial.print(", "); Serial.println(mag_z);

    float a_x = accel.acceleration.x;
    float a_y = accel.acceleration.y;
    float a_z = accel.acceleration.z;

    float roll = atan2(a_y, a_z) * RAD_TO_DEG;
    float pitch = atan2(-a_x, sqrt(a_y * a_y + a_z * a_z)) * RAD_TO_DEG;

    float X_h = mag_x * cos(pitch * (M_PI / 180)) +
        mag_y * sin(roll * (M_PI / 180)) * sin(pitch * (M_PI / 180)) +
        mag_z * cos(roll * (M_PI / 180)) * sin(pitch * (M_PI / 180));

    float Y_h = mag_y * cos(roll * (M_PI / 180)) -
        mag_z * sin(roll * (M_PI / 180));

    double yaw = atan2(-Y_h, X_h) * RAD_TO_DEG;
    yaw = yaw + 180; // Correction pour avoir [0, 360°]

    // Appliquer le filtre de Kalman
    float filtered_yaw = applyKalmanFilter(yaw);

    // Affichage des angles
    // Serial.print("Roll: "); Serial.print(roll);
    // Serial.print("\tPitch: "); Serial.print(pitch);
    // Serial.print("\tYaw (brut): "); Serial.print(yaw);
    // Serial.print("\tYaw (Kalman): "); Serial.println(filtered_yaw);
    addSample(yaw, pitch, roll);
    this->ypr[0] = yaw;
    this->ypr[1] = pitch;
    this->ypr[2] = roll;
}

float MPU::applyKalmanFilter(float measured_yaw)
{
    // Étape 1 : Calcul du Gain de Kalman
    kalman_gain = kalman_error / (kalman_error + sensor_error);

    // Étape 2 : Mise à jour de l'estimation (avec correction dynamique)
    float difference = measured_yaw - kalman_yaw;

    // Appliquer un boost si la variation est importante
    if (abs(difference) > 10)
    {
        kalman_yaw += difference * correction_factor;
    }
    else
    {
        kalman_yaw = kalman_yaw + kalman_gain * difference;
    }

    // Étape 3 : Mise à jour de l'incertitude
    kalman_error = (1 - kalman_gain) * kalman_error;

    return kalman_yaw;
}

float MPU::getYaw()
{
    return ypr[0];
}

float MPU::getPitch()
{
    return ypr[1];
}

float MPU::getRoll()
{
    return ypr[2];
}

void MPU::getYPR(float (&ypr)[3])
{
    ypr[0] = this->ypr[0];
    ypr[1] = this->ypr[1];
    ypr[2] = this->ypr[2];
}

void MPU::addSample(float new_yaw, float new_pitch, float new_roll)
{
    yaw_buffer[buffer_index] = new_yaw;
    pitch_buffer[buffer_index] = new_pitch;
    roll_buffer[buffer_index] = new_roll;
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;
    if (buffer_index == 0) buffer_full = true;
}

void MPU::getAveragedYPR(float& avg_yaw, float& avg_pitch, float& avg_roll)
{
    if (!buffer_full)
    {
        float local_ypr[3];
        getYPR(local_ypr);
        avg_yaw = local_ypr[0];
        avg_pitch = local_ypr[1];
        avg_roll = local_ypr[2];
        return;
    }
    float sum_yaw = 0;
    float sum_pitch = 0;
    float sum_roll = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        sum_yaw += yaw_buffer[i];
        sum_pitch += pitch_buffer[i];
        sum_roll += roll_buffer[i];
    }
    avg_yaw = sum_yaw / BUFFER_SIZE;
    avg_pitch = sum_pitch / BUFFER_SIZE;
    avg_roll = sum_roll / BUFFER_SIZE;
    // Serial.print("Yaw: "); Serial.print(avg_yaw); Serial.print("\tPitch: "); Serial.print(avg_pitch); Serial.print("\tRoll: "); Serial.println(avg_roll);
}

void MPU::getAveragedYPR(float (&ypr)[3])
{
    float avg_yaw, avg_pitch, avg_roll;
    getAveragedYPR(avg_yaw, avg_pitch, avg_roll);
    ypr[0] = avg_yaw;
    ypr[1] = avg_pitch;
    ypr[2] = avg_roll;
}

void MPU::DMPDataReady()
{
    MPUInterrupt = true;
}

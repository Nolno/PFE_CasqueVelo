/*#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <MadgwickAHRS.h>


Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro, *icm_mag;
Madgwick filter;  // Filtre de fusion des capteurs



struct MagnetometerCalibration {
    float offset_x, offset_y, offset_z;
};


MagnetometerCalibration calibrateMagnetometer() {
    Serial.println("Calibrage du magnétomètre...");
    Serial.println("Tourne le capteur dans toutes les directions pendant 10 secondes.");

    float min_x = 1000, max_x = -1000;
    float min_y = 1000, max_y = -1000;
    float min_z = 1000, max_z = -1000;

    unsigned long start_time = millis();
    while (millis() - start_time < 10000) { // 10 secondes
        sensors_event_t mag;
        icm_mag->getEvent(&mag);

        if (mag.magnetic.x < min_x) min_x = mag.magnetic.x;
        if (mag.magnetic.x > max_x) max_x = mag.magnetic.x;
        if (mag.magnetic.y < min_y) min_y = mag.magnetic.y;
        if (mag.magnetic.y > max_y) max_y = mag.magnetic.y;
        if (mag.magnetic.z < min_z) min_z = mag.magnetic.z;
        if (mag.magnetic.z > max_z) max_z = mag.magnetic.z;

        delay(100);
    }

    MagnetometerCalibration calib;
    calib.offset_x = (max_x + min_x) / 2;
    calib.offset_y = (max_y + min_y) / 2;
    calib.offset_z = (max_z + min_z) / 2;

    Serial.println("Calibration terminée !");
    Serial.print("Offsets: X="); Serial.print(calib.offset_x);
    Serial.print(", Y="); Serial.print(calib.offset_y);
    Serial.print(", Z="); Serial.println(calib.offset_z);

    return calib;
}
MagnetometerCalibration magCalib;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("Initialisation du ICM20948...");

    if (!icm.begin_I2C()) {
        Serial.println("Erreur: Impossible de détecter le ICM20948");
        while (1) delay(10);
    }

    Serial.println("ICM20948 détecté !");

    icm_temp = icm.getTemperatureSensor();
    icm_accel = icm.getAccelerometerSensor();
    icm_gyro = icm.getGyroSensor();
    icm_mag = icm.getMagnetometerSensor();

    filter.begin(100);  // Fréquence d'échantillonnage (100 Hz)
    magCalib = calibrateMagnetometer();

}

void loop() {
    sensors_event_t accel, gyro, temp, mag;
    icm_temp->getEvent(&temp);
    icm_accel->getEvent(&accel);
    icm_gyro->getEvent(&gyro);
    icm_mag->getEvent(&mag);

    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;
    float gx = gyro.gyro.x * 57.2958;
    float gy = gyro.gyro.y * 57.2958;
    float gz = gyro.gyro.z * 57.2958;
    
    // Appliquer la correction d'offset au magnétomètre
    float mx = mag.magnetic.x - magCalib.offset_x;
    float my = mag.magnetic.y - magCalib.offset_y;
    float mz = mag.magnetic.z - magCalib.offset_z;

    // Appliquer le filtre Madgwick avec les valeurs corrigées
    //filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();

    Serial.print("Roll: "); Serial.print(roll);
    Serial.print("°\tPitch: "); Serial.print(pitch);
    Serial.print("°\tYaw: "); Serial.print(yaw);
    Serial.println("°");

    delay(10);
}*/

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#define RAD_TO_DEG 57.2957795

Adafruit_ICM20948 icm;
unsigned long startTime;
bool isCalibrating = true;

// Variables de calibration
float mag_min_x = 1000, mag_max_x = -1000;
float mag_min_y = 1000, mag_max_y = -1000;
float mag_min_z = 1000, mag_max_z = -1000;

float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;
float mag_scale_x = 1, mag_scale_y = 1, mag_scale_z = 1;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initialisation ICM20948...");
  if (!icm.begin_I2C()) {
    Serial.println("ICM20948 non détecté !");
    while (1) delay(10);
  }
  Serial.println("ICM20948 trouvé !");
  
  startTime = millis();
  Serial.println("Début de la calibration du magnétomètre...");
}

// Variables pour le Filtre de Kalman
float kalman_yaw = 0;  // Estimation du Yaw
float kalman_error = 4; // Erreur estimée initiale
float kalman_gain = 0;  // Gain de Kalman
float sensor_error = 2; // Bruit de mesure (ajuste si nécessaire)

float correction_factor=0.5;

float applyKalmanFilter(float measured_yaw) {
    // Étape 1 : Calcul du Gain de Kalman
    kalman_gain = kalman_error / (kalman_error + sensor_error);
    
    // Étape 2 : Mise à jour de l'estimation (avec correction dynamique)
    float difference = measured_yaw - kalman_yaw;
  
    // Appliquer un boost si la variation est importante
    if (abs(difference) > 10) { 
      kalman_yaw += difference * correction_factor; 
    } else {
      kalman_yaw = kalman_yaw + kalman_gain * difference;
    }
  
    // Étape 3 : Mise à jour de l'incertitude
    kalman_error = (1 - kalman_gain) * kalman_error;
  
    return kalman_yaw;
  }

void loop() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  if (isCalibrating) {
    if (millis() - startTime < 30000) {
      mag_min_x = min(mag_min_x, mag.magnetic.x);
      mag_max_x = max(mag_max_x, mag.magnetic.x);
      mag_min_y = min(mag_min_y, mag.magnetic.y);
      mag_max_y = max(mag_max_y, mag.magnetic.y);
      mag_min_z = min(mag_min_z, mag.magnetic.z);
      mag_max_z = max(mag_max_z, mag.magnetic.z);
    } else {
      mag_offset_x = (mag_max_x + mag_min_x) / 2;
      mag_offset_y = (mag_max_y + mag_min_y) / 2;
      mag_offset_z = (mag_max_z + mag_min_z) / 2;

      mag_scale_x = (mag_max_x - mag_min_x) / 2;
      mag_scale_y = (mag_max_y - mag_min_y) / 2;
      mag_scale_z = (mag_max_z - mag_min_z) / 2;

      Serial.println("Calibration terminée !");
      isCalibrating = false;
    }
  } else {
    float mag_x = (mag.magnetic.x - mag_offset_x) / mag_scale_x;
    float mag_y = (mag.magnetic.y - mag_offset_y) / mag_scale_y;
    float mag_z = (mag.magnetic.z - mag_offset_z) / mag_scale_z;

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

    float yaw = atan2(-Y_h, X_h) * RAD_TO_DEG;
    if (yaw < 0) yaw += 360;  // Correction pour avoir [0, 360°]

    // Appliquer le filtre de Kalman
    float filtered_yaw = applyKalmanFilter(yaw);

    // Affichage des angles
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print("\tPitch: "); Serial.print(pitch);
    Serial.print("\tYaw (brut): "); Serial.print(yaw);
    Serial.print("\tYaw (Kalman): "); Serial.println(filtered_yaw);
  }

  delay(100);
}

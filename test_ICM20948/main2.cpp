#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

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

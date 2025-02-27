#include "MPU.h"

// Constructeur de la classe MPU - Initialisation des variables
MPU::MPU() : ypr{}, yawBuffer{}, pitchBuffer{}, rollBuffer{}, bufferIndex(0),
                             bufferFull(false), startTime(0), temps(0)
{
}

// Initialisation du MPU
void MPU::initialize()
{
    Serial.println("Initialisation ICM20948...");
    if (!icm.begin_I2C()) // Initialisation du MPU via I2C
    {
        Serial.println("ICM20948 non détecté !");
        while (true) delay(10);
    }
    Serial.println("ICM20948 trouvé !");

    // Calibration du magnétomètre
    startTime = millis();
    Serial.println("Début de la calibration du magnétomètre...");

    sensors_event_t accel, gyro, mag, temp;
    bool isCalibrating = true;
    while (isCalibrating)
    {
        // Récupérer les données du MPU (accéléromètre, gyroscope, magnétomètre, température)
        icm.getEvent(&accel, &gyro, &temp, &mag);
        if (millis() - startTime < 30000) // Calibrer pendant 30 secondes
        {
            // Trouver les valeurs min et max pour chaque axe
            magMinX = min(magMinX, mag.magnetic.x);
            magMaxX = max(magMaxX, mag.magnetic.x);
            magMinY = min(magMinY, mag.magnetic.y);
            magMaxY = max(magMaxY, mag.magnetic.y);
            magMinZ = min(magMinZ, mag.magnetic.z);
            magMaxZ = max(magMaxZ, mag.magnetic.z);
        }
        else
        {// Calculer l'offset et le facteur d'échelle à la fin de la calibration
            magOffsetX = (magMaxX + magMinX) / 2;
            magOffsetY = (magMaxY + magMinY) / 2;
            magOffsetZ = (magMaxZ + magMinZ) / 2;

            magScaleX = (magMaxX - magMinX) / 2;
            magScaleY = (magMaxY - magMinY) / 2;
            magScaleZ = (magMaxZ - magMinZ) / 2;

            Serial.print("Offset: ");
            Serial.print(magOffsetX);
            Serial.print(", ");
            Serial.print(magOffsetY);
            Serial.print(", ");
            Serial.println(magOffsetZ);
            Serial.print("Scale: ");
            Serial.print(magScaleX);
            Serial.print(", ");
            Serial.print(magScaleY);
            Serial.print(", ");
            Serial.println(magScaleZ);

            Serial.println("Calibration terminée !");
            isCalibrating = false;
        }
    }
    // Initialisation du filtre de Kalman
    icm.getEvent(&accel, &gyro, &temp, &mag);

    // Lecture des données du magnétomètre
    float mag_x = (mag.magnetic.x - magOffsetX) / magScaleX;
    float mag_y = (mag.magnetic.y - magOffsetY) / magScaleY;
    float mag_z = (mag.magnetic.z - magOffsetZ) / magScaleZ;
    // Serial.print("Mag: "); Serial.print(mag_x); Serial.print(", "); Serial.print(mag_y); Serial.print(", "); Serial.println(mag_z);

    // Lecture des données de l'accéléromètre
    float a_x = accel.acceleration.x;
    float a_y = accel.acceleration.y;
    float a_z = accel.acceleration.z;

    // Calcul des angles d'Euler (Roll, Pitch, Yaw)
    float roll = atan2(a_y, a_z) * RAD_TO_DEG;
    float pitch = atan2(-a_x, sqrt(a_y * a_y + a_z * a_z)) * RAD_TO_DEG;

    // Calcul de l'angle de lacet (Yaw) à partir du magnétomètre (avec compensation des angles de tangage et de roulis)
    float X_h = mag_x * cos(pitch * (M_PI / 180)) +
        mag_y * sin(roll * (M_PI / 180)) * sin(pitch * (M_PI / 180)) +
        mag_z * cos(roll * (M_PI / 180)) * sin(pitch * (M_PI / 180));

    float Y_h = mag_y * cos(roll * (M_PI / 180)) -
        mag_z * sin(roll * (M_PI / 180));

    float yaw = atan2(-Y_h, X_h) * RAD_TO_DEG;
    yaw = yaw + 180; // Correction pour avoir [0, 360°]
    float initialYaw = yaw;
    kalmanYaw = initialYaw;
    kalmanX = cos(initialYaw * DEG_TO_RAD);
    kalmanY = sin(initialYaw * DEG_TO_RAD);

}

// Routine de mise à jour du MPU - Doit être appelée régulièrement dans la boucle principale (loop)
void MPU::update()
{
    // Récupérer les données du MPU (accéléromètre, gyroscope, magnétomètre, température)
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    // Lecture des données du magnétomètre
    float mag_x = (mag.magnetic.x - magOffsetX) / magScaleX;
    float mag_y = (mag.magnetic.y - magOffsetY) / magScaleY;
    float mag_z = (mag.magnetic.z - magOffsetZ) / magScaleZ;
    // Serial.print("Mag: "); Serial.print(mag_x); Serial.print(", "); Serial.print(mag_y); Serial.print(", "); Serial.println(mag_z);

    // Lecture des données de l'accéléromètre
    float a_x = accel.acceleration.x;
    float a_y = accel.acceleration.y;
    float a_z = accel.acceleration.z;

    // Calcul des angles d'Euler (Roll, Pitch, Yaw)
    float roll = atan2(a_y, a_z) * RAD_TO_DEG;
    float pitch = atan2(-a_x, sqrt(a_y * a_y + a_z * a_z)) * RAD_TO_DEG;

    // Calcul de l'angle de lacet (Yaw) à partir du magnétomètre (avec compensation des angles de tangage et de roulis)
    float X_h = mag_x * cos(pitch * (M_PI / 180)) +
        mag_y * sin(roll * (M_PI / 180)) * sin(pitch * (M_PI / 180)) +
        mag_z * cos(roll * (M_PI / 180)) * sin(pitch * (M_PI / 180));

    float Y_h = mag_y * cos(roll * (M_PI / 180)) -
        mag_z * sin(roll * (M_PI / 180));

    float yaw = atan2(-Y_h, X_h) * RAD_TO_DEG;
    yaw = yaw + 180; // Correction pour avoir [0, 360°]

    // Appliquer le filtre de Kalman
    float filtered_yaw = applyKalmanFilter(yaw);

    // Affichage des angles
    // Serial.print("Roll: "); Serial.print(roll);
    // Serial.print("\tPitch: "); Serial.print(pitch);
    // Serial.print("\tYaw (brut): "); Serial.print(yaw);
    // Serial.print("\tYaw (Kalman): "); Serial.println(filtered_yaw);
    addSample(filtered_yaw, pitch, roll); // Ajouter l'échantillon dans le buffer de calcul de la moyenne

    // assigner les valeurs des angles
    this->ypr[0] = yaw;
    this->ypr[1] = pitch;
    this->ypr[2] = roll;
}

float MPU::getAngleDifference(float a, float b) {
    float diff = fmod(b - a, 360); // Calcul de la différence brute
    if (diff < -180) diff += 360; // Ajustement si trop négatif
    if (diff > 180) diff -= 360; // Ajustement si trop positif
    return diff;
}

float MPU::applyKalmanFilter(float measured_yaw)
{
    // Étape 1 : Conversion de l'angle mesuré en coordonnées cartésiennes
    float measuredX = cos(measured_yaw * DEG_TO_RAD);
    float measuredY = sin(measured_yaw * DEG_TO_RAD);

    // Étape 2 : Estimation de l'angle actuel pour comparer
    float predictedYaw = atan2(kalmanY, kalmanX) * RAD_TO_DEG;
    float difference = getAngleDifference(predictedYaw, measured_yaw);

    // Boost de réactivité si le changement est important
    if (abs(difference) > 5) {
        kalmanGain *= 1.5;
    }

    // Étape 3 : Régulation des erreurs pour éviter une dérive excessive
    sensorError = max(sensorError * 0.9, 0.1);
    kalmanError = min(kalmanError * 1.1, 10.0);

    // Calcul du Gain de Kalman
    kalmanGain = kalmanError / (kalmanError + sensorError);

    // Étape 4 : Mise à jour en X et Y
    kalmanX += kalmanGain * (measuredX - kalmanX);
    kalmanY += kalmanGain * (measuredY - kalmanY);

    // Étape 5 : Normalisation
    float norm = sqrt(kalmanX * kalmanX + kalmanY * kalmanY);
    kalmanX /= norm;
    kalmanY /= norm;

    // Étape 6 : Conversion en angle filtré
    kalmanYaw = atan2(kalmanY, kalmanX) * RAD_TO_DEG;
    if (kalmanYaw < 0) kalmanYaw += 360;

    // Étape 7 : Mise à jour de l'incertitude
    kalmanError = (1 - kalmanGain) * kalmanError;

    return kalmanYaw;
}





// Accesseurs pour les angles d'Euler
float MPU::getYaw() const
{
    return ypr[0];
}

float MPU::getPitch() const
{
    return ypr[1];
}

float MPU::getRoll() const
{
    return ypr[2];
}

void MPU::getYPR(float (&ypr)[3]) const
{
    ypr[0] = this->ypr[0];
    ypr[1] = this->ypr[1];
    ypr[2] = this->ypr[2];
}


void MPU::addSample(float new_yaw, float new_pitch, float new_roll)
{
    // Ajouter les valeurs dans le buffer
    yawBuffer[bufferIndex] = new_yaw;
    pitchBuffer[bufferIndex] = new_pitch;
    rollBuffer[bufferIndex] = new_roll;
    // Incrémenter l'index
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    // Vérifier si le buffer est plein
    if (bufferIndex == 0) bufferFull = true;
}


void MPU::getAveragedYPR(float& avg_yaw, float& avg_pitch, float& avg_roll) const
{
    // Si le buffer n'est pas plein, on renvoie les valeurs brutes
    if (!bufferFull)
    {
        float local_ypr[3];
        getYPR(local_ypr);
        avg_yaw = local_ypr[0];
        avg_pitch = local_ypr[1];
        avg_roll = local_ypr[2];
        return;
    }

    // Calcul de la moyenne
    float sum_yaw = 0;
    float sum_pitch = 0;
    float sum_roll = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) // Parcourir le buffer
    {
        // Somme des valeurs
        sum_yaw += yawBuffer[i];
        sum_pitch += pitchBuffer[i];
        sum_roll += rollBuffer[i];
    }
    // Calcul de la moyenne
    avg_yaw = sum_yaw / BUFFER_SIZE;
    avg_pitch = sum_pitch / BUFFER_SIZE;
    avg_roll = sum_roll / BUFFER_SIZE;
    // Serial.print("Yaw: "); Serial.print(avg_yaw); Serial.print("\tPitch: "); Serial.print(avg_pitch); Serial.print("\tRoll: "); Serial.println(avg_roll);
}

void MPU::getAveragedYPR(float (&ypr)[3]) const
{   // Surcharge de la méthode pour renvoyer les valeurs dans un tableau
    float avg_yaw, avg_pitch, avg_roll;
    getAveragedYPR(avg_yaw, avg_pitch, avg_roll);
    ypr[0] = avg_yaw;
    ypr[1] = avg_pitch;
    ypr[2] = avg_roll;
}



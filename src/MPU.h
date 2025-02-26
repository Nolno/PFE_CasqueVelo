/**
* @file MPU.h
 * @author Antoine Martin
 * @brief Code de gestion du MPU. Ce code fonctionne sur un ESP32-WROOM-32E avec un IMU ICM20948 de chez Adafruit.
 * @version 1.0
 * @date 25/02/2024
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef MPU_H
#define MPU_H
#define BUFFER_SIZE 50

#include <Arduino.h>
#include <Adafruit_ICM20948.h>
/**
* @brief Classe de gestion du MPU. La classe permet de récupérer les angles d'orientation du MPU(yaw, pitch, roll).
* L'initialisation et le calibrage du MPU et du DMP est effectué dans la classe, ainsi que les éventuels traitements des données.
*/
class MPU {
private:
    Adafruit_ICM20948 icm; /**< Objet pour la gestion du MPU */
    // Angles d'orientation
    float ypr[3]; /**< Tableau pour stocker les valeurs des angles d'orientation [yaw, pitch, roll] */
    float yaw_buffer[BUFFER_SIZE]; /**< Buffer pour stocker les valeurs de l'angle de lacet */
    float pitch_buffer[BUFFER_SIZE]; /**< Buffer pour stocker les valeurs de l'angle de tangage */
    float roll_buffer[BUFFER_SIZE]; /**< Buffer pour stocker les valeurs de l'angle de roulis */
    int buffer_index;  /**< Index actuel dans le buffer */
    bool buffer_full;  /**< Indique si on a rempli au moins x valeurs */

    // Variables pour la calibration
    unsigned long startTime; /**< Temps de démarrage de la calibration */
    // Variables de calibration
    float mag_min_x = 1000, mag_max_x = -1000; // Valeurs min et max pour l'axe X
    float mag_min_y = 1000, mag_max_y = -1000; // Valeurs min et max pour l'axe Y
    float mag_min_z = 1000, mag_max_z = -1000; // Valeurs min et max pour l'axe Z

    float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0; // Offset pour chaque axe
    float mag_scale_x = 1, mag_scale_y = 1, mag_scale_z = 1; // Facteur d'échelle pour chaque axe

    // Variables pour le Filtre de Kalman
    float kalman_yaw = 0;  // Estimation du Yaw
    float kalman_error = 4; // Erreur estimée initiale
    float kalman_gain = 0.5;  // Gain de Kalman
    float sensor_error = 2; // Bruit de mesure (ajuste si nécessaire)

    float correction_factor=0.5; // Facteur de correction pour la moyenne

    unsigned long temps; /**< Temps actuel en millisecondes */


    /**
    * @brief Méthode pour ajouter un échantillon dans le buffer utilisé pour le calcul de la moyenne.
    */
    void addSample(float new_yaw, float new_pitch, float new_roll);
    /**
     * @brief Applique un filtre de Kalman sur la mesure de l'angle de lacet.
     * @param measured_yaw Valeur mesurée de l'angle de lacet.
     * @return
     */
    float applyKalmanFilter(float measured_yaw);
    /**
     * @brief Calcule la différence entre deux angles. La différence est calculée en tenant compte de la périodicité de 360°.
     * @param a Premier angle.
     * @param b Deuxième angle.
     * @return Différence entre les deux angles.
     */
    float angleDifference(float a, float b);
public:
    /**
    * @brief Constructeur de la classe MPU.
    */
    explicit MPU();

    /**
    * @brief Méthode pour initialiser le MPU. Cette méthode doit être appelée une seule fois au démarrage du programme.
    */
    void initialize();

    /**
    * @brief Routine de mise à jour du MPU. Appeler cette méthode régulièrement dans la boucle principale (loop).
    */
    void update();

    /**
    * @brief Méthode pour récupérer la valeur de l'angle de lacet.
    */
    float getYaw();

    /**
    * @brief Méthode pour récupérer la valeur de l'angle de tangage.
    */
    float getPitch();

    /**
    * @brief Méthode pour récupérer la valeur de l'angle de roulis.
    */
    float getRoll();

    /**
    * @brief Méthode pour récupérer les valeurs des angles d'orientation (yaw, pitch, roll) en degrés.
    */
    void getYPR(float (&ypr)[3]);

    /**
    * @brief Méthode pour obtenir les valeurs moyennes des angles d'orientation en degrés.
    * @param avg_yaw Valeur moyenne de l'angle de lacet.
    * @param avg_pitch Valeur moyenne de l'angle de tangage.
    * @param avg_roll Valeur moyenne de l'angle de roulis.
    */
    void getAveragedYPR(float &avg_yaw, float &avg_pitch, float &avg_roll);

    /**
     * @brief Méthode pour obtenir les valeurs moyennes des angles d'orientation en degrés.
     * @param ypr Tableau contenant les valeurs des angles d'orientation [yaw, pitch, roll]
     */
    void getAveragedYPR(float (&ypr)[3]);
};

#endif //MPU_H

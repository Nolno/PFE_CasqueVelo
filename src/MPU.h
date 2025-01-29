//
// Created by Antoine on 29/01/2025.
//

#ifndef MPU_H
#define MPU_H
#define BUFFER_SIZE 100

#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"

/**
* @brief Classe de gestion du MPU6050. La classe permet de récupérer les angles d'orientation du MPU6050 (yaw, pitch, roll).
* L'initialisation et le calibrage du MPU et du DMP est effectué dans la classe, ainsi que les éventuels traitements des données.
*/
class MPU {
private:
  float yaw_buffer[BUFFER_SIZE];
  float pitch_buffer[BUFFER_SIZE];
  float roll_buffer[BUFFER_SIZE];
  int buffer_index;  /**< Index actuel dans le buffer */
  bool buffer_full;  /**< Indique si on a rempli au moins x valeurs */


  MPU6050 mpu; /**< Objet MPU6050 */
  Quaternion q; /**< Quaternion pour stocker les valeurs de l'orientation */
  VectorFloat gravity; /**< Vecteur pour stocker les valeurs de l'accélération */
  float ypr[3]; /**< Tableau pour stocker les valeurs des angles d'orientation [yaw, pitch, roll] */
  bool DMPReady; /**< Indique si le DMP est prêt à être utilisé */
  uint8_t FIFOBuffer[64]; /**< Tampon de stockage FIFO */
  uint16_t packetSize; /**< Taille du paquet DMP attendu (par défaut 42 octets) */
  uint8_t devStatus; /**< Statut de retour après chaque opération de l'appareil (0 = succès, !0 = erreur) */
  uint8_t MPUIntStatus; /**< Contient le byte de statut d'interruption réel du MPU */
  int const INTERRUPT_PIN; /**< Définit la broche d'interruption #0 */
  unsigned long temps; /**< Temps actuel en millisecondes */

public:
  /**
  * @brief Constructeur de la classe MPU.
  */
  MPU(int interruptPin);

  /**
  * @brief Méthode pour initialiser le MPU6050 et le DMP.
  */
  void initialize();

  /**
  * @brief Méthode pour mettre à jour les valeurs des angles d'orientation.
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
  * @brief Méthode pour récupérer les valeurs des angles d'orientation (yaw, pitch, roll).
  */
  void getYPR(float (&ypr)[3]);

  /**
  * @brief Méthode pour ajouter un échantillon dans les buffers.
  */
  void addSample(float new_yaw, float new_pitch, float new_roll);

  /**
  * @brief Méthode pour obtenir les valeurs moyennes des angles d'orientation.
  */
  void getAveragedYPR(float &avg_yaw, float &avg_pitch, float &avg_roll);

  void DMPDataReady();

};



#endif //MPU_H

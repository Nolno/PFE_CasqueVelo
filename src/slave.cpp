/**
 * @file main.cpp
 * @author Antoine Martin
 * @brief Code de fonctionnement du slave. Ce code est exécuté sur un ESP32-WROOM-32E.
 * @version 1.0
 * @date 25/02/2024
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <Wire.h>
#include <Arduino.h>
#include "MPU.h"
#include "BluetoothSerial.h"

//############################ PINS ###################################
#define INTERRUPT_PIN 15 /**< Pin d'interruption du MPU */

//############################ MPU ###################################
MPU mpu; /**< Objet pour la gestion du MPU */


//############################ BLUETOOTH ###################################
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

String deviceName = "ESP32-BT-Slave"; /**< BT: Stockage du nom du périphérique Bluetooth SLAVE */
String MACadd = "64:B7:08:29:35:72"; /**< Adresse MAC du périphérique Bluetooth SLAVE */
uint8_t address[6]; /**< Adresse MAC du périphérique Bluetooth SLAVE */

#define MAX_BUFFER_SIZE 256 /**< Taille maximale du buffer de réception */
#define MAX_BUFFER_SIZE_6 1550 /**< Taille maximale du buffer de réception : 6*256 + espace pour 1 caractère (char) */

// Déclaration des buffers comme des arrays de char
char message[MAX_BUFFER_SIZE_6]; /**< Buffer pour stocker les données reçues via Bluetooth */
char buffer[MAX_BUFFER_SIZE_6]; /**< Buffer pour stocker et concaténer les données à envoyer via Bluetooth */

float avg_yaw = 0, avg_pitch = 0, avg_roll = 0;
/**< Variables pour stocker les valeurs moyennes des angles d'orientation */

bool ack_received = true; /**< Stocke l'état de l'accusé de réception */
int i = 0; /**< Variable pour parcourir les adresses MAC */
unsigned long last_ack = 0; /**< Temps du dernier ACK reçu */
unsigned long temps; /**< Temps actuel en millisecondes */
/**
 * @brief
 *  Fonction de configuration du programme. Cette fonction est exécutée une seule fois au démarrage du programme.
 */
void setup()
{
    Wire.begin(); // Initialisation de la communication I2C
    Serial.begin(115200); // Initialisation de la communication série
    while (!Serial) // Attendre que la communication série soit prête
        delay(10);

    //############################ MPU  Setup ###################################
    mpu.initialize(); // Initialisation du MPU - inclut le calibrage du magnétomètre

    //############################ BLUETOOTH Setup ###################################
    esp_read_mac(address, ESP_MAC_BT); // Récupérer l'adresse MAC du périphérique Bluetooth
    SerialBT.begin(deviceName); // Initialisation du Bluetooth en mode SLAVE
    while (!SerialBT.connected(1000)) // Attendre la connexion
    {
        Serial.println("Waiting for connection...");
    }
    ack_received = true; // Initialiser l'accusé de réception à true
    while (i < 6) // Afficher l'adresse MAC du périphérique Bluetooth
    {
        Serial.print(address[i], HEX);
        Serial.print("\t");
        i++;
    }
    Serial.println();
    delay(3000);
    memset(message, 0, sizeof(message)); // Initialiser le buffer de réception
    memset(buffer, 0, sizeof(buffer)); // Initialiser le buffer d'envoi
}

/**
 * @brief
 *  Fonction principale du programme. Cette fonction est exécutée en boucle après la fonction setup().
 */
void loop()
{
    mpu.update(); // Routine du MPU
    if (ack_received) // Si un ACK est reçu (le master est prêt à recevoir de nouvelles données)
    {
        mpu.getAveragedYPR(avg_yaw, avg_pitch, avg_roll); // Récupérer les angles de l'orientation moyens
        snprintf(buffer, sizeof(buffer), "%f,%f,%f!",avg_yaw, avg_pitch, avg_roll); // Stocker les valeurs dans le buffer
        SerialBT.write((uint8_t*)buffer, strlen(buffer));  // Envoyer les données via Bluetooth
        Serial.println(buffer); // Afficher les données dans le moniteur série

        ack_received = false; // Attendre le prochain ACK
        // Reset for next batch
        memset(message, 0, sizeof(message));
        memset(buffer, 0, sizeof(buffer));
    }


    // Vérifier si un ACK est reçu
    if (SerialBT.available())
    {
        char ack = SerialBT.read(); // Lire un caractère depuis le buffer
        if (ack == 'A')
        {
            ack_received = true; // L'ACK est reçu
            last_ack = millis();
        }
    }

    // Si un ACK n'est pas reçu depuis plus d'une seconde, on envoie à nouveau les données
    if (millis() - last_ack > 1000)
    {
        ack_received = true;
    }
}

/**
   * @file main.cpp
   * @author Antoine Martin
   * @brief  Code principal du projet, permettant de récupérer les données des capteurs et de les envoyer via Bluetooth.
   * Inclut également la gestion de l'alarme et de l'enregistrement des données sur la carte SD. Le code doit être
   * exécuté sur un ESP32 WROOM 32E.
   * @version 1.0
   * @date 25/02/2025
   *
   * @copyright Copyright (c) 2025
   *
   */


#include <Alarm.h>
#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include "MPU.h"
#include "Tachymeter.h"
#include "pitches.h"


/******************************************************************************
 *                                   DEFINES                                  *
 ******************************************************************************/

// Baudrate pour la communication série. Doit être le même que celui configuré dans le platformio.ini
#define BAUDRATE 115200

//############################ CAPTEUR A EFFET HAL ###################################

// Fréquence minimale pour laquelle le capteur à effet Hall est capable de détecter les impulsions
#define MIN_FREQUENCY 0.1
// Facteur de conversion secondes/microsecondes
#define MICRO_CONSTANT 1.0E-6
// Rayon de la roue en mètres : À MODIFIER EN FONCTION DE LA ROUE UTILISÉE
#define WHEEL_RADIUS 0.31

//############################ BLUETOOTH ###################################
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

//############################ PINS ###################################

#define CS_PIN 5 /**< CS pin de la carte SD */
#define PIN_BUZZER 2 // Pin du buzzer
#define PIN_HAL 25  // Pin du capteur à effet Hall
#define PIN_LED 33 // Pin de la LED
#define INTERRUPT_PIN 15 // Pin de l'interruption du MPU


/******************************************************************************
 *                         Prototypes des fonctions locales                   *
 ******************************************************************************/

//############################ BLUETOOTH ###################################
/**
 *  @brief Fonction pour connecter le MASTER au SLAVE via Bluetooth
 */
void connectSlave();

/**
 * @brief
 *  Fonction de callback pour le statut du Bluetooth
 * @param event Événement du Bluetooth
 * @param param Paramètre du callback
 */
void getBluetoothStatus(esp_spp_cb_event_t event, esp_spp_cb_param_t* param);

/**
 * @brief
 *  Récupère les valeurs de l'accélération et de l'orientation du MPU de slave via Bluetooth
 * @param ypr_slave
 *  Tableau contenant les angles de l'orientation de slave [yaw, pitch, roll] [lacet, tangage, roulis] en radians
 */
void getSlaveData(float (&ypr_slave)[3]);
//############################ SON ###################################
/**
 * @brief
 *  Son à jouer pour indiquer la connexion au slave (dernière étape de l'initialisation).
 */
void playBtConnexionSuccessSound();

/**
 * @brief
 *  Son à jouer pour indiquer la fin de la calibration.
 */
void playCalibrationSound();
//############################ SD CARD ###################################
/**
 * @brief
 *  Fonction pour enregistrer les données sur la carte SD, et les afficher dans le moniteur série
 * @param ypr_master
 *  Tableau contenant les angles de l'orientation de master [yaw, pitch, roll] [lacet, tangage, roulis] en degrés
 * @param ypr_slave
 *  Tableau contenant les angles de l'orientation de slave [yaw, pitch, roll] [lacet, tangage, roulis] en degrés
 * @param ypr_diff
 *  Tableau contenant les différences entre les angles de l'orientation de master et de slave [yaw, pitch, roll] [lacet, tangage, roulis] en degrés
 * @param speed
 *  Vitesse du vélo en km/h
 * @param alarmState
 *  État de l'alarme (true = alarme activée, false = alarme désactivée)
 */
void logData(float (&ypr_master)[3], float (&ypr_slave)[3], float (&ypr_diff)[3], double speed,
             boolean alarmState);

/******************************************************************************
 *                             VARIABLES GLOBALES                             *
 ******************************************************************************/
//############################ ALARME ###################################

// Objet pour la gestion de l'alarme
Alarm alarmSystem(PIN_BUZZER, PIN_LED, 70, 3, 0.1, 0.1);
//############################ MPU ###################################

// Objet pour la gestion du MPU
MPU mpu(INTERRUPT_PIN);
float ypr[3]; // Tableau contenant les angles de l'orientation [yaw, pitch, roll] du master en degrés
float yprSlave[3]; // Tableau contenant les angles de l'orientation [yaw, pitch, roll] du slave en degrés
// ############################ TACHYMÈTRE ###################################

// Objet pour la gestion du tachymètre
Tachymeter tachymeter(PIN_HAL, PIN_BUZZER, WHEEL_RADIUS, 0.55);
//############################ BLUETOOTH ###################################

// Variables pour la gestion de la connexion Bluetooth
unsigned long previousMillisReconnect;
/**< Variable pour stocker le temps écoulé depuis la dernière tentative de reconnexion */
bool SlaveConnected; /**< Variable pour stocker l'état de la connexion Bluetooth */
int recatt = 0; /**< Variable utilisée pour stocker le nombre d'essais de reconnexion */

String myName = "ESP32-BT-Master";
/**< Variable contenant le nom du périphérique Bluetooth MASTER; juste pour l'impression */
String slaveName = "ESP32-BT-Slave";
/**< Variable contenant le nom du périphérique Bluetooth SLAVE; juste pour l'impression */
String MACadd = "64:B7:08:29:35:72";
/**< Variable contenant l'adresse MAC du périphérique Bluetooth SLAVE; juste pour l'impression */
uint8_t slaveAddress[6] = {0x64, 0xB7, 0x08, 0x29, 0x35, 0x72};
/**< Variable contenant l'adresse MAC du périphérique Bluetooth SLAVE, utilisée pour la connexion */

BluetoothSerial SerialBT; /**< Objet pour la communication série Bluetooth */

// Variables pour la réception des données Bluetooth
char receivedBuffer[1550]; /**< Buffer pour stocker les données reçues via Bluetooth */
int bufferIndex = 0; /**< Index pour parcourir le buffer */
char receivedChar2; /**< Caractère reçu via Bluetooth */
char* token; /**< tokens extraits du buffer */
constexpr char t[2] = ","; /**< Délimiteur pour la fonction strtok */

//############################ CARTE SD ###################################
File myFile; /**< Objet pour la gestion de la carte SD */

// ############################ BOUTON ###################################
constexpr int buttonPin = 32; /**< Broche du bouton */
int buttonState = 0; /**< État du bouton */

// Variables pour stocker les angles initiaux
float initialMasterYaw = 0; /**< Yaw initial du master — utilisé pour la mise à zéro */
float initialSlaveYaw = 0; /**< Yaw initial du slave — utilisé pour la mise à zéro */

// Variable pour stocker l'état du système
bool systemStarted = false; /**< État du système (true = système démarré, false = système arrêté) */


/******************************************************************************
 *                              SETUP AND LOOP                                *
 ******************************************************************************/

/**
 * @brief
 *  Fonction de configuration du programme. Cette fonction est exécutée une seule fois au démarrage du programme.
 */
void setup()
{
    Serial.begin(115200); //  Initialisation de la communication série
    Wire.begin(); //  Initialisation de la communication I2C
    //############################ SETUP DU BOUTON ###################################
    pinMode(buttonPin, INPUT_PULLUP); //  Initialisation de la broche du bouton, avec une résistance de tirage pour que le bouton soit HIGH par défaut
    //############################ SETUP DE LA LED ###################################
    pinMode(PIN_LED, OUTPUT); //  Initialisation de la broche de la LED
    //############################ INITIALISATION DE L'ALARME ###################################
    alarmSystem.init(); //  Initialisation de l'alarme
    //############################ INITIALISATION DU TACHYMÈTRE ###################################
    tachymeter.initialize(); //  Initialisation du tachymètre
    //############################ INITIALISATION DU MPU ###################################
    mpu.initialize(); //  Initialisation du MPU - Inclut la calibration
    playCalibrationSound();  //  Jouer le son de fin de calibration
    //############################ SD CARD Setup ###################################
    pinMode(CS_PIN, OUTPUT);  //  Initialisation de la broche CS - permet de sélectionner la carte SD

    Serial.print("Initializing SD card... ");
    if (!SD.begin(CS_PIN))
    {
        Serial.println("Card initialization failed!");
        // ReSharper disable once CppDFAEndlessLoop - Ignore le warning de boucle infinie
        while (true); //  Si la carte SD n'est pas détectée, on arrête le programme
    }
    Serial.println("Initialization done.");

    Serial.println("Creating example.csv...");
    myFile = SD.open("/example.csv", FILE_WRITE); //  Créer un fichier CSV pour stocker les données
    // Écrire les en-têtes dans le fichier
    myFile.println(
        "time,yaw_master,pitch_master,roll_master,yaw_slave,pitch_slave,roll_slave,yaw_diff,pitch_diff,roll_diff,speed,alarmState,limit_angle,limit_time");
    myFile.close();  //  Fermer le fichier

    //############################ BLUETOOTH Setup ################################
    SlaveConnected = false; //  Flag pour indiquer si le slave est connecté : initialisé à false (non connecté)

    SerialBT.register_callback(getBluetoothStatus); // Enregistrer la fonction de callback pour le statut du Bluetooth
    SerialBT.begin(myName, true); //   Initialisation du Bluetooth en mode MASTER
    Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
    connectSlave(); // Connexion au slave
}

/**
 * @brief
 *  Fonction principale du programme. Cette fonction est exécutée en boucle après la fonction setup().
 */
void loop()
{
    mpu.update(); // Routine du MPU
    tachymeter.update(); // Routine du tachymètre


    // Bluetooth - MPU routine
    if (!SlaveConnected) // Si le slave n'est pas connecté on tente de se reconnecter
    {
        if (millis() - previousMillisReconnect >= 10000)
        {
            previousMillisReconnect = millis();
            recatt++;
            Serial.print("Trying to reconnect. Attempt No.: ");
            Serial.println(recatt);
            Serial.println("Stopping Bluetooth...");
            SerialBT.end();
            Serial.println("Bluetooth stopped !");
            Serial.println("Starting Bluetooth...");
            SerialBT.begin(myName, true);
            Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n",
                          myName.c_str());
            connectSlave();
            Serial.println(
                "time,yaw_master,pitch_master,roll_master,yaw_slave,pitch_slave,roll_slave,yaw_diff,pitch_diff,roll_diff,speed");
        }
    }
    if (SerialBT.available()) // Si des données sont disponibles sur le port série Bluetooth
    {
        // On lit les données des capteurs
        mpu.getAveragedYPR(ypr); // Récupérer les angles de l'orientation du master
        getSlaveData(yprSlave); // Récupérer les angles de l'orientation du slave
        double speedKmh = tachymeter.getSpeed() * 3.6; // Récupérer la vitesse en km/h

        // Lit l'état du bouton - Le système démarre lorsque le bouton est pressé, les angles sont mis à zéro
        buttonState = digitalRead(buttonPin);
        if (buttonState == LOW)
        { // Le bouton est pressé
            Serial.println("Mise à zéro");
            // Initialiser les angles de départ
            initialMasterYaw = ypr[0];
            initialSlaveYaw = yprSlave[0];
            // Démarrer le système
            systemStarted = true;
        }
        // Routine principale : si le système est démarré (le bouton a été pressé au moins une fois)
        if (systemStarted)
        {
            // Récupérer les angles de l'orientation du master
            ypr[0] = ypr[0] - initialMasterYaw;
            if (ypr[0] < 0) // Correction pour avoir un angle entre [0, 360°] au lieu de [-180, 180°]
            {
                ypr[0] = 360 + ypr[0];
            }
            // Même chose pour le slave
            yprSlave[0] = yprSlave[0] - initialSlaveYaw;
            if (yprSlave[0] < 0)
            {
                yprSlave[0] = 360 + yprSlave[0];
            }
            // REMARQUE : Les angles de tangage et de roulis ne sont pas modifiés. Il sera nécessaire de le faire si
            // on souhaite les utiliser.

            // Calculer les différences entre les angles de l'orientation du master et du slave.
            // La formule suivante permet de calculer la différence entre deux angles en tenant compte de la périodicité
            // de 360°. Par exemple, la différence entre 10° et 350° est de 20° et non de 340°.
            float ypr_diff[3] = {
                180 - abs(abs(ypr[0] - yprSlave[0]) - 180),
                180 - abs(abs(ypr[1] - yprSlave[1]) - 180),
                180 - abs(abs(ypr[2] - yprSlave[2]) - 180)
            };

            // Mise à jour de l'alarme avec les différences d'angles et la vitesse actuelle
            boolean alarmState = alarmSystem.update(ypr_diff, speedKmh);
            // Enregistrement des données dans le fichier CSV de la carte SD
            logData(ypr, yprSlave, ypr_diff, speedKmh, alarmState);
        }
        else // Si le système n'est pas démarré, on affiche les angles bruts (a des fins de débogage uniquement)
        {
            Serial.print("Système non démarré --> Yaw : ");
            Serial.print(ypr[0]);
            Serial.print(" Roll : ");
            Serial.print(ypr[1]);
            Serial.print(" Pitch : ");
            Serial.println(ypr[2]);
            // REMARQUE : Tant que le système n'est pas démarré, les données ne sont pas enregistrées sur la carte SD.
        }
    }
}


void getSlaveData(float (&ypr_slave)[3])
{
    bufferIndex = 0; // Réinitialiser l'index avant lecture
    memset(receivedBuffer, 0, sizeof(receivedBuffer)); // Effacer le buffer

    // Lecture des données reçues via Bluetooth
    while (SerialBT.available())
    {
        receivedChar2 = SerialBT.read(); // Lire le caractère
        if (receivedChar2 == '!') // Fin de la transmission
        {
            receivedBuffer[bufferIndex] = '\0'; // Terminer la chaîne
            break;
        }
        // On n'a pas reçu le caractère de fin, on stocke donc le caractère dans le buffer
        if (bufferIndex < sizeof(receivedBuffer) - 1) // Vérifier les limites du tableau
        {
            receivedBuffer[bufferIndex] = receivedChar2; // Stocker le caractère
            bufferIndex++; // Incrémenter l'index
        }
        else
        {
            Serial.println("Buffer overflow detected!"); // Détection de dépassement
            break;
        }
    }
    // Extraire les données du buffer. Les données sont séparées par des virgules (délimiteur défini par t, voir plus haut)
    token = strtok(receivedBuffer, t);
    if (token != nullptr) // Si le token n'est pas null (données reçues)
    {
        ypr_slave[0] = atof(token); // Convertir la chaîne en float
        token = strtok(nullptr, t); // Lire le prochain token
    }
    else
    {   // Si le token est null, on initialise la valeur à 0
        ypr_slave[0] = 0;
    }
    // On répète le processus pour les deux autres valeurs
    if (token != nullptr)
    {
        ypr_slave[1] = atof(token);
        token = strtok(nullptr, t);
    }
    else
    {
        ypr_slave[1] = 0;
    }
    if (token != nullptr)
    {
        ypr_slave[2] = atof(token);
    }
    else
    {
        ypr_slave[2] = 0;
    }
    // Envoyer un caractère d'Acknowledgement au slave (On indique au slave que les données ont été reçues et traitées)
    // On est apte à recevoir de nouvelles données
    SerialBT.write('A');
}


void getBluetoothStatus(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
    // Gestion des événements Bluetooth
    if (event == ESP_SPP_OPEN_EVT)
    {
        Serial.println("Client Connected");
        SlaveConnected = true;
        recatt = 0;
    }
    else if (event == ESP_SPP_CLOSE_EVT)
    {
        Serial.println("Client Disconnected");
        SlaveConnected = false;
    }
}


void connectSlave()
{
    // Tant que le slave n'est pas connecté, on tente de se connecter
    while (!SerialBT.connected())
    {
        Serial.println("Function BT connection executed");
        Serial.printf("Connecting to slave BT device named \"%s\" and MAC address \"%s\" is started.\n",
                      slaveName.c_str(),
                      MACadd.c_str());
        SerialBT.connect(slaveAddress); // Connexion au slave via son adresse MAC
    }
    playBtConnexionSuccessSound(); // Jouer le son de connexion
    // On vide le buffer de réception
    while (SerialBT.available())
    {
        SerialBT.read();
    }
}


void logData(float (&ypr_master)[3], float (&ypr_slave)[3], float (&ypr_diff)[3], double speedKmh,
             boolean alarmState)
{
    myFile = SD.open("/example.csv", FILE_APPEND); // Ouvrir le fichier en mode ajout
    if (myFile) // Si le fichier est ouvert avec succès
    {
        // Créer une chaîne de caractères pour stocker les données au format CSV
        String dataString = "";
        dataString += String(millis()) + ",";
        dataString += String(ypr_master[0]) + "," + String(ypr_master[1]) + "," + String(ypr_master[2]) + ",";
        dataString += String(ypr_slave[0]) + "," + String(ypr_slave[1]) + "," + String(ypr_slave[2]) + ",";
        dataString += String(ypr_diff[0]) + "," + String(ypr_diff[1]) + "," + String(ypr_diff[2]) + ",";
        dataString += String(speedKmh) + "," + String(alarmState) + ",";

        // Ajouter les limites calculées
        std::pair<double, double> limits = alarmSystem.getLimits(speedKmh);
        dataString += String(limits.first) + "," + String(limits.second);

        // Imprimer la chaîne dans le fichier
        myFile.println(dataString);
        myFile.flush(); // Vider le buffer
        myFile.close(); // Fermer le fichier

        // Créer une chaîne de caractères pour l'affichage aligné dans le moniteur série
        char formattedString[512];
        sprintf(formattedString,
                "time: %lu YAW --> Master: %7.2f Slave: %7.2f Diff: %7.2f PITCH --> Master: %7.2f Slave: %7.2f Diff: %7.2f ROLL --> Master: %7.2f Slave: %7.2f Diff: %7.2f Speed: %7.2f limit_angle: %7.2f limit_time: %7.2f",
                millis(),
                ypr_master[0], ypr_slave[0], ypr_diff[0],
                ypr_master[1], ypr_slave[1], ypr_diff[1],
                ypr_master[2], ypr_slave[2], ypr_diff[2],
                speedKmh,
                limits.first, limits.second);

        // Imprimer la chaîne formatée dans la console série
        Serial.println(formattedString);
    }
    else // Si le fichier n'a pas pu être ouvert
    {
        Serial.println("Error opening file.");
    }
}


void playCalibrationSound()
{
    // Définir la mélodie à jouer. Les notes sont définies dans le fichier pitches.h
    int melody[] = {NOTE_G4, NOTE_B4, NOTE_D5};
    constexpr int noteDuration = 1000 / 4; // Durée de chaque note (en ms)
    constexpr int pauseBetweenNotes = noteDuration * 1.30; // Pause entre chaque note
    for (const int thisNote : melody) // Jouer chaque note
    {
        tone(PIN_BUZZER, thisNote, noteDuration); // Jouer la note pendant la durée spécifiée
        delay(pauseBetweenNotes); // Attendre
        noTone(PIN_BUZZER); // Arrêter le son
    }
}


void playBtConnexionSuccessSound()
{
    // Mélodie de connexion réussie
    int duration = 125; // Durée de chaque note (en ms)

    tone(PIN_BUZZER, NOTE_E5, duration);
    delay(duration);
    tone(PIN_BUZZER, NOTE_CS5, duration);
    delay(duration);
    tone(PIN_BUZZER, NOTE_CS5, duration);
    delay(duration);
    delay(duration);
    // Descente
    tone(PIN_BUZZER, NOTE_B4, duration);
    delay(duration);
    tone(PIN_BUZZER, NOTE_CS5, duration);
    delay(duration);
    tone(PIN_BUZZER, NOTE_E5, duration);
    delay(duration);
    noTone(PIN_BUZZER); // Stop le son
    delay(duration);
}

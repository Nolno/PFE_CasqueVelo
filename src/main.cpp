/**
   * @file main.cpp
   * @author Victor Alessi
   * @brief  This is the master device code. Encompasses master bt functionalities,
   * sd card data formatting and saving, speed sensor readings, and MPU readings
   * @version 0.1
   * @date 2024-03-22
   *
   * @copyright Copyright (c) 2024
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

//############################ HALL SENSOR ###################################
// Minimum frequency for the tachymeter
#define MIN_FREQUENCY                        0.1
// Conversion factor from seconds to microseconds
#define MICRO_CONSTANT                       1.0E-6

// Wheel radius in meters
#define WHEEL_RADIUS 0.31


// Baudrate for the serial communication
#define BAUDRATE 115200

//############################ MPU  SENSOR ###################################

#define MAX_BUFFER_SIZE 256
#define MAX_BUFFER_SIZE_6 1550

#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

//############################ SD CARD     ###################################
/**
 * @brief SD SETUP
 *
 */
#define CS_PIN 5 /**< Chip select pin for SD card */
#define PIN_BUZZER 2
#define PIN_HAL 25
#define PIN_LED 33 // Pin connected to the LED
#define INTERRUPT_PIN 15


/******************************************************************************
 *                         LOCAL FUNCTION PROTOTYPES                          *
 ******************************************************************************/

//############################ MPU  SENSOR ###################################
// Prototypes
void SlaveConnect(); /**< Function to connect to the slave device */
void check_bt_connection(bool stat);
void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t* param); /**< Callback function for Bluetooth status */
/**
 * @brief
 *  Récupère les valeurs de l'accélération et de l'orientation du MPU de slave via Bluetooth
 * @param ypr_slave
 *  Tableau contenant les angles de l'orientation de slave [yaw, pitch, roll] [lace, tangage, roulis] en radians
 */
void getSlaveData(float (&ypr_slave)[3]);
void playTotallySpiesSound();
//############################ SD CARD     ###################################
void data_logging(float (&ypr_master)[3], float (&ypr_slave)[3], float (&ypr_diff)[3], double speed);
/**< Function to log data */
/******************************************************************************
 *                             GLOBAL VARIABLES                             *
 ******************************************************************************/
//############################ ALARM ###################################
Alarm alarmSystem(PIN_BUZZER, PIN_LED, 80, 3, 0.1, 0.1); /**< Alarm object */
//############################ MPU  SENSOR ###################################
MPU mpu(INTERRUPT_PIN);
float ypr[3]; // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
float ypr_slave[3]; // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector for slave
// ############################ HALL SENSOR ###################################
Tachymeter tachymeter(PIN_HAL, PIN_BUZZER, WHEEL_RADIUS, 0.55);
//############################ BLUETOOTH ###################################
const long interval = 100;
int ledState = LOW;
unsigned long previousMillisLED = 0;
unsigned long previousMillisReconnect; /**< Variable used for comparing millis counter for the reconnection timer */
bool ledBtState = false; /**< Variable used to change the indication LED state */
bool SlaveConnected; /**< Variable used to store the current connection state (true=connected/false=disconnected) */
int recatt = 0; /**< Variable used to count the reconnection attempts */

String myName = "ESP32-BT-Master";
/**< Variable used to store the SERVER(Master) bluetooth device name; just for printing */
String slaveName = "ESP32-BT-Slave";
/**< Variable used to store the CLIENT(Slave) bluetooth device name; just for printing */
String MACadd = "64:B7:08:29:35:72";
/**< Variable used to store the CLIENT(Slave) bluetooth device Mac address; just for printing */
uint8_t address[6] = {0x64, 0xB7, 0x08, 0x29, 0x35, 0x72};
/**< Variable used to store the CLIENT(Slave) MAC address used for the connection */

BluetoothSerial SerialBT; /**< Bluetooth Serial Object */

//############################ SD CARD     ###################################
File myFile; /**< File object for SD card */
int i = 0; /**< Counter variable */

// ############################ BUTTON ###################################
const int buttonPin = 32; // Broche à laquelle le bouton est connecté
int buttonState = 0; // Variable pour lire l'état du bouton

// Déclaration des variables contenant la différence entre les angles (pitch et yaw) mesurés et reçus.
float initial_diff_pitch;
float initial_diff_roll;
float initial_diff_yaw;

float initial_master_yaw = 0;
float initial_slave_yaw = 0;

float diff_yaw;
float diff_pitch;
float diff_roll;

bool systemStarted = false;

// Compteur de seconde afin de connaitre le temps d'inattention (angle du casque > ANGLE)
bool cpt_second = false;
char receivedBuffer[1550];
int bufferIndex = 0;
char receivedChar2;
char* token;
const char t[2] = ",";
// Define buffers as character arrays
char message[MAX_BUFFER_SIZE_6]; // used for displaying on the serial monitor
char buffer[MAX_BUFFER_SIZE_6]; // used to concatenate the data and send the message as a single package
unsigned long temps;


void playInitialisationSound();

void setup()
{
    Serial.begin(115200); //  Sets the data rate in bits per second (baud) for serial data transmission
    Wire.begin();
    //############################ BUTTON Setup ###################################
    pinMode(buttonPin, INPUT_PULLUP);
    //############################ LED and BUZZER Setup ###################################
    pinMode(PIN_LED, OUTPUT);
    //############################ ALARM Setup ###################################
    alarmSystem.init();
    //############################ Tachymeter setup ###################################
    Tachymeter::instance = &tachymeter;
    tachymeter.initialize();
    //############################ MPU  Setup ###################################
    mpu.initialize();
    int melody[] = {NOTE_G4, NOTE_B4, NOTE_D5};
    for (int thisNote = 0; thisNote < 3; thisNote++)
    {
        int noteDuration = 1000 / 4;
        tone(PIN_BUZZER, melody[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        noTone(PIN_BUZZER);
    }
    pinMode(A5, OUTPUT);
    //############################ SD CARD Setup ###################################
    pinMode(CS_PIN, OUTPUT);

    Serial.print("Initializing SD card... ");
    if (!SD.begin(CS_PIN))
    {
        Serial.println("Card initialization failed!");
        while (1); //  Stop the program
    }
    Serial.println("Initialization done.");

    if (SD.exists("/example.csv"))
    {
        Serial.println("example.csv exists.");
    }
    else
    {
        Serial.println("example.csv doesn't exist.");
    }

    Serial.println("Creating example.csv...");
    myFile = SD.open("/example.csv", FILE_WRITE);
    myFile.println(
        "time,yaw_master,pitch_master,roll_master,yaw_slave,pitch_slave,roll_slave,yaw_diff,pitch_diff,roll_diff,speed,limit_angle,limit_time");
    myFile.close();

    if (SD.exists("/example.csv"))
    {
        Serial.println("example.csv exists.");
    }
    else
    {
        Serial.println("example.csv doesn't exist.");
    }

    //############################ BLUETOOTH Setup ################################
    SlaveConnected = false; //  Set the variable false = CLIENT is not connected

    SerialBT.register_callback(Bt_Status); // Define the Bt_Status callback
    SerialBT.begin(myName, true); //   Starts the bluetooth device as SERVER(Master)
    Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
    SlaveConnect(); // Connect to the CLIENT(Slave)

    Serial.println(
        "time,yaw_master,pitch_master,roll_master,yaw_slave,pitch_slave,roll_slave, yaw_diff, pitch_diff, roll_diff");
    // // On attend 5s après le 1er message pour initialiser l'écart initial
    // delay(5000);
    // // On vide le buffer bluetooth
    // while (SerialBT.available())
    // {
    //     SerialBT.read();
    // }
    // SerialBT.write('A');
    // mpu.getAveragedYPR(ypr);
    // // attendre la réception des données du slave
    // while (!SerialBT.available())
    // {
    //     // Attendre ici jusqu'à ce que des données soient disponibles
    //     delay(10); // Petit délai pour éviter de bloquer le processeur
    // }
    // getSlaveData(ypr_slave);
    // initial_diff_pitch = ypr[1] - ypr_slave[1];
    // initial_diff_roll = ypr[2] - ypr_slave[2];
    // initial_diff_yaw = ypr[0] - ypr_slave[0];
    // Serial.println("Initial diff yaw: " + String(initial_diff_yaw));
    // // Signal sonore pour indiquer la fin de l'initialisation
    // playInitialisationSound();
}


void loop()
{
    // MPU routine
    mpu.update();
    // Tachymeter routine
    tachymeter.update();

    // Bluetooth - MPU routine
    if (!SlaveConnected)
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
            SlaveConnect();
            Serial.println(
                "time,yaw_master,pitch_master,roll_master,yaw_slave,pitch_slave,roll_slave,yaw_diff,pitch_diff,roll_diff,speed");
        }
    }
    if (SerialBT.available())
    {
        // On lit les données des capteurs
        mpu.getAveragedYPR(ypr);
        getSlaveData(ypr_slave);
        double speedKmh = tachymeter.getSpeed() * 3.6;

        // Lit l'état du bouton - Le système démarre lorsque le bouton est pressé, les angles sont mis à zéro
        buttonState = digitalRead(buttonPin);
        if (buttonState == LOW)
        {
            // Le bouton est pressé
            Serial.println("Mise à zéro");
            initial_master_yaw = ypr[0];
            initial_slave_yaw = ypr_slave[0];
            systemStarted = true;
        }

        if (systemStarted)
        {
            ypr[0] = ypr[0] - initial_master_yaw;
            if (ypr[0] < 0)
            {
                ypr[0] = 360 + ypr[0];
            }
            ypr_slave[0] = ypr_slave[0] - initial_slave_yaw;
            if (ypr_slave[0] < 0)
            {
                ypr_slave[0] = 360 + ypr_slave[0];
            }

            float ypr_diff[3] = {
                180 - abs(abs(ypr[0] - ypr_slave[0]) - 180),
                180 - abs(abs(ypr[1] - ypr_slave[1]) - 180),
                180 - abs(abs(ypr[2] - ypr_slave[2]) - 180)
            };

            // Routines
            alarmSystem.update(ypr_diff, speedKmh);
            data_logging(ypr, ypr_slave, ypr_diff, speedKmh);
        }
        else
        {
            Serial.print("Système non démarré --> Yaw : ");Serial.print(ypr[0]);
            Serial.print(" Roll : ");Serial.print(ypr[1]);
            Serial.print(" Pitch : ");Serial.println(ypr[2]);
        }
    }
}


void getSlaveData(float (&ypr_slave)[3])
{
    bufferIndex = 0; // Réinitialiser l'index avant lecture
    memset(receivedBuffer, 0, sizeof(receivedBuffer)); // Effacer le buffer
    // wait for the data to be available


    while (SerialBT.available())
    {
        receivedChar2 = SerialBT.read();
        if (receivedChar2 == '!')
        {
            receivedBuffer[bufferIndex] = '\0'; // Terminer la chaîne
            break;
        }
        if (bufferIndex < sizeof(receivedBuffer) - 1) // Vérifier les limites du tableau
        {
            receivedBuffer[bufferIndex] = receivedChar2; // Stocker le caractère
            bufferIndex++;
        }
        else
        {
            Serial.println("Buffer overflow detected!"); // Détection de dépassement
            break;
        }
    }
    token = strtok(receivedBuffer, t);
    if (token != nullptr)
    {
        ypr_slave[0] = atof(token);
        token = strtok(nullptr, t);
    }
    else
    {
        ypr_slave[0] = 0;
    }
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
    SerialBT.write('A');
}


void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
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


void SlaveConnect()
{
    while (!SerialBT.connected())
    {
        Serial.println("Function BT connection executed");
        Serial.printf("Connecting to slave BT device named \"%s\" and MAC address \"%s\" is started.\n",
                      slaveName.c_str(),
                      MACadd.c_str());
        SerialBT.connect(address);
    }
    playTotallySpiesSound();
    // Empty the bluetooth serial buffer
    while (SerialBT.available())
    {
        SerialBT.read();
    }
}


void data_logging(float (&ypr_master)[3], float (&ypr_slave)[3], float (&ypr_diff)[3], double speedKmh)
{
    if (myFile)
    {
        // Créer une chaîne de caractères pour stocker les données au format CSV
        String dataString = "";
        dataString += String(millis()) + ",";
        dataString += String(ypr_master[0]) + "," + String(ypr_master[1]) + "," + String(ypr_master[2]) + ",";
        dataString += String(ypr_slave[0]) + "," + String(ypr_slave[1]) + "," + String(ypr_slave[2]) + ",";
        dataString += String(ypr_diff[0]) + "," + String(ypr_diff[1]) + "," + String(ypr_diff[2]) + ",";
        dataString += String(speedKmh) + ",";

        // Ajouter les limites calculées
        std::pair<double, double> limits = alarmSystem.getLimits(speedKmh);
        dataString += String(limits.first) + "," + String(limits.second);

        // Imprimer la chaîne dans le fichier
        myFile.println(dataString);
        myFile.flush();
        myFile.close();

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
    else
    {
        Serial.println("Error opening file.");
    }
    myFile = SD.open("/example.csv", FILE_APPEND);
}


void playTotallySpiesSound()
{
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


void playInitialisationSound()
{
    int duration = 125; // Durée de chaque note (en ms)

    tone(PIN_BUZZER, NOTE_E5, duration);
    delay(duration);
    tone(PIN_BUZZER, NOTE_E5, duration);
    delay(duration);
}

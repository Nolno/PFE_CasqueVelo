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

#include <Arduino.h>
#include <SD.h>
#include "BluetoothSerial.h"
#include "MPU.h"
#include "Tachymeter.h"
#include <Wire.h>


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
// ############################ HALL SENSOR ###################################
Tachymeter tachymeter(PIN_HAL, PIN_BUZZER, WHEEL_RADIUS, 0.55);
//############################ MPU  SENSOR ###################################
// Prototypes
void SlaveConnect(); /**< Function to connect to the slave device */
void check_bt_connection(bool stat);
void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t* param); /**< Callback function for Bluetooth status */

//############################ SD CARD     ###################################
void data_logging(float (&acc)[3], float (&acc_slave)[3]); /**< Function to log data */
void alarm_system(float (&ypr)[3], float (&ypr_slave)[3]); /**< Function to start the alarm system*/

/******************************************************************************
 *                             GLOBAL VARIABLES                             *
 ******************************************************************************/
//############################ MPU  SENSOR ###################################
MPU mpu(INTERRUPT_PIN);

float ypr[3]; // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
float ypr_slave[3]; // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector for slave

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

// Déclaration des variables contenant la différence entre les angles (pitch et yaw) mesurés et reçus.
float initial_diff_pitch;
float initial_diff_roll;
float initial_diff_yaw;
float diff_yaw;
float diff_pitch;
float diff_roll;

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


void setup()
{
    Wire.begin();
    //############################ LED and BUZZER Setup ###################################
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    //############################ Tachymeter setup ###################################
    Tachymeter::instance = &tachymeter;
    tachymeter.initialize();
    //############################ BLUETOOTH Setup ################################
    SlaveConnected = false; //  Set the variable false = CLIENT is not connected
    Serial.begin(115200); //  Sets the data rate in bits per second (baud) for serial data transmission

    SerialBT.register_callback(Bt_Status); // Define the Bt_Status callback
    SerialBT.begin(myName, true); //   Starts the bluetooth device as SERVER(Master)
    Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
    SlaveConnect(); // Connect to the CLIENT(Slave)
    //############################ MPU  Setup ###################################
    mpu.initialize();
    pinMode(A5, OUTPUT);
    Serial.println("time,yaw,pitch,roll");
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
    myFile.print(
        "slave_ax,slave_ay,slave_az,slave_gx,slave_gy,slave_gz,master_ax,master_ay,master_az,master_gx,master_gy,master_gz,angular_velocity,linear_velocity\n");
    myFile.close();

    if (SD.exists("/example.csv"))
    {
        Serial.println("example.csv exists.");
    }
    else
    {
        Serial.println("example.csv doesn't exist.");
    }
}


/**
 * @brief
 *  Récupère les valeurs de l'accélération et de l'orientation du MPU de slave via Bluetooth
 * @param ypr_slave
 *  Tableau contenant les angles de l'orientation de slave [yaw, pitch, roll] [lace, tangage, roulis] en radians
 */
void getSlaveData(float (&ypr_slave)[3]);


void loop()
{
    // MPU routine
    mpu.update();
    // Tachymeter routine
    tachymeter.update();


    float acc[3]; // Tableau contenant les valeurs de l'accélération de master [x, y, z]
    float gyro[3]; // Tableau contenant les valeurs de l'orientation de master [x, y, z]
    float acc_slave[3]; // Tableau contenant les valeurs de l'accélération de slave [x, y, z]
    float gyro_slave[3]; // Tableau contenant les valeurs de l'orientation de slave [x, y, z]
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
        }
    }

    if (SerialBT.available())
    {
        mpu.getAveragedYPR(ypr);
        // Serial.print("ypr\t");
        Serial.print(millis() - temps);
        Serial.print(",");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print(",");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print(",");
        Serial.println(ypr[2] * 180 / M_PI);
        getSlaveData(ypr_slave);
        Serial.print("slave :");
        Serial.print(ypr_slave[0] * 180 / M_PI);
        Serial.print(",");
        Serial.print(ypr_slave[1] * 180 / M_PI);
        Serial.print(",");
        Serial.println(ypr_slave[2] * 180 / M_PI);

        // alarm_system(ypr, ypr_slave);
        // data_logging();
    }
}


void getSlaveData(float (&ypr_slave)[3])
{
    bufferIndex = 0; // Réinitialiser l'index avant lecture
    memset(receivedBuffer, 0, sizeof(receivedBuffer)); // Effacer le buffer

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
    ypr_slave[0] = atoff(token);
    token = strtok(nullptr, t);
    ypr_slave[1] = atoff(token);
    token = strtok(nullptr, t);
    ypr_slave[2] = atoff(token);
    token = strtok(nullptr, t);
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
    Serial.println("Function BT connection executed");
    Serial.printf("Connecting to slave BT device named \"%s\" and MAC address \"%s\" is started.\n", slaveName.c_str(),
                  MACadd.c_str());
    SerialBT.connect(address);
}

void data_logging(float (&acc)[3], float (&acc_slave)[3])
{
    myFile = SD.open("/example.csv", FILE_WRITE);

    if (myFile)
    {
        Serial.println("File opened successfully.");
        // Schema d'une ligne du fichier CSV : master_ax,master_ay,master_az,slave_ax,slave_ay,slave_az
        myFile.print(acc[0]);
        myFile.print(",");
        myFile.print(acc[1]);
        myFile.print(",");
        myFile.print(acc[2]);
        myFile.print(",");
        myFile.print(acc_slave[0]);
        myFile.print(",");
        myFile.print(acc_slave[1]);
        myFile.print(",");
        myFile.print(acc_slave[2]);
        myFile.println();
        myFile.close();
        Serial.println("File closed.");
    }
    else
    {
        Serial.println("Error opening file.");
    }
}

void alarm_system(float (&ypr)[3], float (&ypr_slave)[3])
{
    diff_yaw = ypr[0] - ypr_slave[0];
    diff_pitch = ypr[1] - ypr_slave[1];
    diff_roll = ypr[2] - ypr_slave[2];
    if (initial_diff_pitch == 0)
        initial_diff_pitch = diff_pitch;
    if (initial_diff_roll == 0)
        initial_diff_roll = diff_roll;
    if (initial_diff_yaw == 0)
        initial_diff_yaw = diff_yaw;

    Serial.print("Angles received : yaw = ");
    Serial.print(ypr_slave[0]);
    Serial.print(" pitch = ");
    Serial.print(ypr_slave[1]);
    Serial.print(" roll = ");
    Serial.println(ypr_slave[2]);
    Serial.print("Diff : yaw = ");
    Serial.print(diff_yaw);
    Serial.print(" pitch = ");
    Serial.print(diff_pitch);
    Serial.print(" roll = ");
    Serial.println(diff_roll);
    Serial.print("Diff acc INIT: yaw = ");
    Serial.print(initial_diff_yaw);
    Serial.print(" pitch = ");
    Serial.print(initial_diff_pitch);
    Serial.print(" roll = ");
    Serial.println(initial_diff_roll);
    Serial.println();

    if ((abs(diff_roll) - abs(initial_diff_roll) > 0.7) || (abs(diff_pitch) - abs(initial_diff_pitch) > 0.7) || (
        abs(diff_yaw) - abs(initial_diff_yaw) > 0.7))
    {
        cpt_second = true;
        if ((abs(diff_roll) - abs(initial_diff_roll) > 0.7))
        {
            Serial.println("yaw");
        }
        if ((abs(diff_pitch) - abs(initial_diff_pitch) > 0.7))
        {
            Serial.println("pitch");
        }
        if ((abs(diff_yaw) - abs(initial_diff_yaw) > 0.7))
        {
            Serial.println("roll");
        }
    }
    else
    {
        cpt_second = false;
        digitalWrite(PIN_LED, LOW);
        // noTone(PIN_BUZZER);
    }

    // Activation de l'alarme
    if (cpt_second)
    {
        if (millis() - previousMillisLED >= interval)
        {
            // save the last time you blinked the LED
            previousMillisLED = millis();

            // if the LED is off turn it on and vice versa:
            if (ledState == LOW)
            {
                ledState = HIGH;
                Serial.println("LED ON");
            }
            else
            {
                ledState = LOW;
                Serial.println("LED OFF");
            }

            // set the LED with the ledState of the variable:
            digitalWrite(PIN_LED, ledState);
        }
        // tone(PIN_BUZZER, 500);
        cpt_second = false;
    }
    bufferIndex = 0;
    memset(receivedBuffer, 0, sizeof(receivedBuffer));
}

std::pair<double, double> calculerLimites(double v_kmh, double A_max, double T_max, double k, double k_prime) {


    angle_limits = (A_max * std::exp(-k * v_kmh));
    temps_limits = (T_max * std::exp(-k_prime * v_kmh));

    return {angle_limits, temps_limits};
}

boolean depasserLimites(double angle, double temps, double v_kmh, double A_max, double T_max, double k, double k_prime){
    angle_limits, temps_limits = calculerLimites(double v_kmh, double A_max, double T_max, double k, double k_prime);
    if( angle>=angle_limits && temps>=temps_limits){
        return true;
    } 
    else{
        return false;
    }
}

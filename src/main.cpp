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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


/******************************************************************************
 *                                   DEFINES                                  *
 ******************************************************************************/

//############################ HALL SENSOR ###################################
/**
 * @brief
 *     Minimum frequency accepted for the measurement of wheel speed, in Hertz.
*/
#define MIN_FREQUENCY                        0.1
/**
 * @brief
 *     10^-6 micro preffix constant
*/
#define MICRO_CONSTANT                       1.0E-6

/**
 * @brief
 *      Radius of the wheel
*/
#define RADIUS 50E-2

/**
 * @brief
 *      Serial communication baudrate
*/
#define BAUDRATE 115200

//############################ MPU  SENSOR ###################################
/**
 * @brief MPU DEFINITIONS
 *
 */
Adafruit_MPU6050 mpu;

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
#define PIN_LED_LEFT_L2 33
#define PIN_BUZZER 2

/******************************************************************************
 *                         LOCAL FUNCTION PROTOTYPES                          *
 ******************************************************************************/
// ############################ HALL SENSOR ###################################
/**
 * @brief
 *  Interrupt handler function, responsible for obtaining average period readings.
 *
*/
void IRAM_ATTR pulse_event();

//############################ MPU  SENSOR ###################################
// Prototypes
void SlaveConnect(); /**< Function to connect to the slave device */
void check_bt_connection(bool stat);
void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t* param); /**< Callback function for Bluetooth status */

//############################ SD CARD     ###################################
void data_logging(float (&acc)[3], float (&acc_slave)[3]); /**< Function to log data */
void alarm_system(float (&acc)[3], float (&acc_slave)[3]); /**<Function to start the alarm system*/


/******************************************************************************
 *                             GLOBAL VARIABLES                             *
 ******************************************************************************/
// ############################ HALL SENSOR ###################################
/** @brief
 *    Maximum period accepted for the measurement of wheel speed, defined by
 * 1/MIN_FREQUENCY. In seconds
 */
float max_period = 1 / MIN_FREQUENCY;

float measurement_runtime = 0;
float runtime = 0;
float time_between_pulses = 0;


float angular_speed = 0;
float linear_speed = 0;


uint8_t pulse_counter = 0;
uint8_t pulse_counter_last = 0;

bool first_cycle = 1;
uint8_t j = 0;
//############################ MPU  SENSOR ###################################
/**
 * @brief BLUETOOTH SETUP
 *
 */
const long interval = 1000;
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

char message_hall[MAX_BUFFER_SIZE];
char buffer_hall[MAX_BUFFER_SIZE];
char buffer_angular[MAX_BUFFER_SIZE];
char buffer_linear[MAX_BUFFER_SIZE];

//############################ SD CARD     ###################################
File myFile; /**< File object for SD card */
int i = 0; /**< Counter variable */

// Déclaration des variables contenant la différence entre les angles (pitch et yaw) mesurés et reçus.
float initial_diff_Y;
float initial_diff_Z;
float diff_acc_X;
float diff_acc_Y;
float diff_acc_Z;

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
char buffer_acceleration_X[MAX_BUFFER_SIZE];
char buffer_acceleration_Y[MAX_BUFFER_SIZE];
char buffer_acceleration_Z[MAX_BUFFER_SIZE];
char buffer_gyro_X[MAX_BUFFER_SIZE];
char buffer_gyro_Y[MAX_BUFFER_SIZE];
char buffer_gyro_Z[MAX_BUFFER_SIZE];

void setup()
{
    /**
     * @brief Hall sensor setup
     *
     */
    pinMode(25, INPUT_PULLUP);
    attachInterrupt(25, pulse_event, RISING); //   Enable interruption pin 25 when going from LOW to HIGH.

    /**
     * @brief Bluetooth setup
     *
     */

    SlaveConnected = false; //  Set the variable false = CLIENT is not connected
    Serial.begin(115200); //  Sets the data rate in bits per second (baud) for serial data transmission

    SerialBT.register_callback(Bt_Status); // Define the Bt_Status callback
    SerialBT.begin(myName, true); //   Starts the bluetooth device as SERVER(Master)
    Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
    SlaveConnect(); // Connect to the CLIENT(Slave)

    /**
     * @brief SD setup
     *
     */

    pinMode(CS_PIN, OUTPUT);
    pinMode(PIN_LED_LEFT_L2, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

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

    /**
     * @brief MPU Setup
     *
     */
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }

    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

/**
 * @brief
 *  Récupère les valeurs de l'accélération du MPU de master
 * @param acc
 *  Tableau contenant les valeurs de l'accélération de master [x, y, z]
 */
void getAcc(float (&acc)[3]);

/**
 * @brief
 *  Récupère les valeurs de l'orientation du MPU de master
 * @param gyro
 *  Tableau contenant les valeurs de l'orientation de master [x, y, z]
 */
void getGyro(float (&gyro)[3]);

/**
 * @brief
 *  Récupère les valeurs de l'accélération et de l'orientation du MPU de slave via Bluetooth
 * @param acc_slave
 *  Tableau contenant les valeurs de l'accélération de slave [x, y, z]
 * @param gyro_slave
 *  Tableau contenant les valeurs de l'orientation de slave [x, y, z]
 */
void getSlaveData(float (&acc_slave)[3], float (&gyro_slave)[3]);


void loop()
{
    // Serial.println(pulse_counter);
    // //  Speed sensor routine
    // if ((pulse_counter - pulse_counter_last) > 1)
    // {
    //     // Get current application runtime
    //     runtime = micros();
    //     time_between_pulses = runtime - measurement_runtime;
    //     time_between_pulses = time_between_pulses * (1e-6);
    //
    //
    //     // Ignore intervals if they are too long
    //     if (time_between_pulses > max_period)
    //     {
    //         time_between_pulses = 0;
    //         pulse_counter_last = pulse_counter;
    //     }
    //     else
    //     {
    //         angular_speed = 2 * PI / time_between_pulses;
    //         linear_speed = RADIUS * angular_speed;
    //         Serial.println(angular_speed);
    //         Serial.println(linear_speed);
    //         pulse_counter_last = pulse_counter;
    //     }
    // }
    // else
    // {
    //     angular_speed = pulse_counter;
    //     linear_speed = pulse_counter;
    // }
    // snprintf(buffer_angular, sizeof(buffer_angular), "%f", angular_speed);
    // snprintf(buffer_linear, sizeof(buffer_linear), "%f", linear_speed);
    // sprintf(buffer_hall, ",%s,%s", buffer_angular, buffer_linear);
    // strcpy(message_hall, buffer_hall);
    // Serial.println(message_hall);
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
    { // Si des données sont disponibles
        getAcc(acc);
        getGyro(gyro);
        // Serial.write(SerialBT.read());
        getSlaveData(acc_slave, gyro_slave);
        alarm_system(acc, acc_slave);
        // data_logging();
    }
}


void getAcc(float (&acc)[3])
{
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp))
    {
        acc[0] = a.acceleration.x;
        acc[1] = a.acceleration.y;
        acc[2] = a.acceleration.z;
    }
    else
    {
        acc[0] = 0;
        acc[1] = 0;
        acc[2] = 0;
    }
}

void getGyro(float (&gyro)[3])
{
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp))
    {
        gyro[0] = g.gyro.x;
        gyro[1] = g.gyro.y;
        gyro[2] = g.gyro.z;
    }
    else
    {
        gyro[0] = 0;
        gyro[1] = 0;
        gyro[2] = 0;
    }
}

void getSlaveData(float (&acc_slave)[3], float (&gyro_slave)[3])
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
    acc_slave[0] = atoff(token);
    token = strtok(nullptr, t);
    acc_slave[1] = atoff(token);
    token = strtok(nullptr, t);
    acc_slave[2] = atoff(token);
    token = strtok(nullptr, t);
    gyro_slave[0] = atoff(token);
    token = strtok(nullptr, t);
    gyro_slave[1] = atoff(token);
    token = strtok(nullptr, t);
    gyro_slave[2] = atoff(token);
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
    myFile = SD.open("/example.csv", FILE_APPEND);

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
        // while (SerialBT.available())
        // {
        //     Serial.println("B");
        //     char receivedChar = SerialBT.read(); //reads "%s,%s,%s,%s,%s,%s!"
        //
        //     Serial.print(receivedChar);
        //
        //     if (receivedChar != '!')
        //     {
        //         myFile.print(receivedChar);
        //     }
        //     else
        //     {
        //         myFile.print(message);
        //         myFile.print(message_hall);
        //         myFile.println();
        //         break;
        //     }
        // }
        myFile.close();
        Serial.println("File closed.");
    }
    else
    {
        Serial.println("Error opening file.");
    }
}

void alarm_system(float (&acc)[3], float (&acc_slave)[3])
{
    diff_acc_X = acc[0] - acc_slave[0];
    diff_acc_Y = acc[1] - acc_slave[1];
    diff_acc_Z = acc[2] - -acc_slave[2];
    if (initial_diff_Y == 0)
        initial_diff_Y = diff_acc_Y;
    if (initial_diff_Z == 0)
        initial_diff_Z = diff_acc_Z;
    /*Serial.print("ACC X RECEIVED: ");
    Serial.print(acc_X_received);
    Serial.print(" ACC Y RECEIVED:");
    Serial.println(acc_Y_received);
    Serial.print(" ACC Z RECEIVED: ");
    Serial.print(acc_Z_received);
    Serial.println("Diff acc X : ");
    Serial.print(diff_acc_X);
    Serial.print(" Diff acc Y : ");
    Serial.print(diff_acc_Y);*/
    Serial.print(" Diff acc Z :  ");
    Serial.print(diff_acc_Z);
    Serial.print(" Diff acc Z INIT :  ");
    Serial.print(initial_diff_Z);

    if ((abs(diff_acc_Z) - abs(initial_diff_Z)) > 0.7)
        cpt_second = true;
    else
    {
        cpt_second = false;
        digitalWrite(PIN_LED_LEFT_L2, LOW);
        noTone(PIN_BUZZER);
    }

    // Activation de l'alarme
    if (cpt_second)
    {
        if (millis() - previousMillisLED >= interval)
        {
            // save the last time you blinked the LED
            previousMillisLED = millis();

            // if the LED is off turn it on and vice-versa:
            if (ledState == LOW)
            {
                ledState = HIGH;
            }
            else
            {
                ledState = LOW;
            }

            // set the LED with the ledState of the variable:
            digitalWrite(PIN_LED_LEFT_L2, ledState);
        }
        tone(PIN_BUZZER, 500);

        cpt_second = false;
    }
    bufferIndex = 0;
    memset(receivedBuffer, 0, sizeof(receivedBuffer));
}

/**
 * @brief
 *  Interrupt handler function, responsible for obtaining average period readings.
 *
*/
void IRAM_ATTR pulse_event()
{
    pulse_counter++;
    measurement_runtime = micros();
}

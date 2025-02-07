/*  #include <Arduino.h>
 #include <SD.h>
 #include "BluetoothSerial.h"
 #include <Adafruit_MPU6050.h>
 #include <Adafruit_Sensor.h>
 #include <Wire.h>
 #include <math.h>
 #include "Tachymeter.h"


int pulse_counter = 0;
#define PIN_BUZZER 2
float WEEL_RADIUS = 0.31;
volatile bool buzzer_flag = false;
unsigned long previousMillis = 0;
 */

/**
 * @brief
 *  Interrupt handler function, responsible for obtaining average period readings.
 *
*/
/* void IRAM_ATTR pulse_event();


void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial.println("Hello World");

    /**
     * @brief Hall sensor setup
     *
     */
/*    pinMode(25, INPUT_PULLUP);
    attachInterrupt(25, pulse_event, RISING); //   Enable interruption pin 25 when going from LOW to HIGH.
    pinMode(PIN_BUZZER, OUTPUT);
    previousMillis = millis();

}

void loop() {
  if (pulse_counter == 0) {
    previousMillis = millis();
  }
    delay(100);
    if (buzzer_flag) {
        buzzer_flag = false;
        tone(PIN_BUZZER, 500);
        delay(100);
        noTone(PIN_BUZZER);
        float speed = WEEL_RADIUS * 3.14 * 1000 / (millis() - previousMillis);
        Serial.print("Speed : "); Serial.print(speed); Serial.print(" m/s "); Serial.print(speed * 3.6); Serial.println(" km/h");

        previousMillis = millis();
    }
}

/**
 * @brief
 *  Interrupt handler function, responsible for obtaining average period readings.
 *
*/
/*
void IRAM_ATTR pulse_event()
{
    pulse_counter++;
    Serial.println(pulse_counter);
    buzzer_flag = true;
}
 */
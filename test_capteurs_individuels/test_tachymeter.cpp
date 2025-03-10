#include <Arduino.h>
#include "Tachymeter.h"
#include <Wire.h>
#include <SPI.h>

// Configuration des pins et des constantes
#define PIN_SENSOR 25
#define PIN_BUZZER 2
#define WHEEL_RADIUS 0.31

// Instancier l'objet Tachymeter
Tachymeter tachymeter(PIN_SENSOR, PIN_BUZZER, WHEEL_RADIUS, 0.55);

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial.println("Tachymeter Test");

    // Associer l'instance pour l'interruption et initialiser
    Tachymeter::instance = &tachymeter;
    tachymeter.initialize();
}

void loop() {
    // Mise à jour du tachymètre (gérer les événements)
    tachymeter.update();

    // Exemple d'utilisation : récupérer et afficher la vitesse
    float speed = tachymeter.getSpeed();
    Serial.print("Current Speed: ");
    Serial.print(speed);
    Serial.print(" m/s ");
    Serial.print(speed * 3.6);
    Serial.println(" km/h");

    delay(10); // Pause pour éviter de saturer le moniteur série
}

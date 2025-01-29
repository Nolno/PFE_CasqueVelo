#include "Tachymeter.h"

// Initialisation de l'instance statique
Tachymeter *Tachymeter::instance = nullptr;


Tachymeter::Tachymeter(int sensorPin, int buzzerPin, float wheelRadius, float stationarySpeed)
    : sensorPin(sensorPin), buzzerPin(buzzerPin), wheelRadius(wheelRadius), stationarySpeed(stationarySpeed),
      pulseCounter(0), eventFlag(false), previousMillis(0), currentSpeed(0) {}

// Initialisation
void Tachymeter::initialize() {
    pinMode(sensorPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(sensorPin), []() { instance->pulseEvent(); }, RISING);
    stationaryPeriod = (wheelRadius * 2 * PI) / stationarySpeed;
    previousMillis = millis();
}

// Mise à jour (appelée dans loop pour traiter les événements)
void Tachymeter::update() {
    if (eventFlag) {
        eventFlag = false; // Réinitialiser le drapeau d'événement

        // Faire sonner le buzzer pour signaler un passage
        tone(buzzerPin, 500);
        delay(50);
        noTone(buzzerPin);

        // Calculer la vitesse
        unsigned long currentMillis = millis();
        float period = (currentMillis - previousMillis) / 1000.0; // Période en secondes
        if (period > 0) {
            currentSpeed = (wheelRadius * 2 * PI) / period; // Vitesse en m/s
        }
        else {
            currentSpeed = 0;
        }
        previousMillis = currentMillis;
    }
    else {
        // Vérifier si le véhicule est arrêté
        unsigned long currentMillis = millis();
        float period = (currentMillis - previousMillis) / 1000.0; // Période en secondes
        if (period > stationaryPeriod) {
            currentSpeed = 0; // Véhicule arrêté
        }
    }
}

// Méthode pour récupérer la vitesse actuelle
float Tachymeter::getSpeed() const {
    return currentSpeed; // Retourner la vitesse calculée
}

// Méthode privée : gestion de l'interruption
void Tachymeter::pulseEvent() {
    pulseCounter++;
    eventFlag = true; // Signaler qu'un événement a eu lieu
}

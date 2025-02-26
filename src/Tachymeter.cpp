#include "Tachymeter.h"

// Initialisation de l'instance statique
Tachymeter *Tachymeter::instance = nullptr;


Tachymeter::Tachymeter(const int sensorPin, const int buzzerPin, const float wheelRadius, const float stationarySpeed)
    : stationarySpeed(stationarySpeed), sensorPin(sensorPin), buzzerPin(buzzerPin), wheelRadius(wheelRadius),
      pulseCounter(0), eventFlag(false), previousMillis(0), currentSpeed(0)
{
    stationaryPeriod = (wheelRadius * 2 * PI) / stationarySpeed;
}

// Initialisation
void Tachymeter::initialize() {
    // Configurer les broches
    pinMode(sensorPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    // Attacher l'interruption pour gérer les impulsions
    attachInterrupt(digitalPinToInterrupt(sensorPin), []() { instance->pulseEvent(); }, RISING);
    previousMillis = millis();
}

void Tachymeter::update() {
    if (eventFlag) { // Un événement a eu lieu, c'est-à-dire que l'aimant est passé devant le capteur
        eventFlag = false; // Réinitialiser le drapeau d'événement

        // Faire sonner le buzzer pour signaler un passage
        tone(buzzerPin, 500);
        delay(50);
        noTone(buzzerPin);

        // Calculer la vitesse
        const unsigned long currentMillis = millis(); // Temps actuel
        const float period = (currentMillis - previousMillis) / 1000.0; // Période entre deux passages de l'aimant en secondes
        if (period > 0) { // Éviter la division par zéro
            currentSpeed = (wheelRadius * 2 * PI) / period; // Vitesse en m/s
        }
        else {
            currentSpeed = 0;
        }
        previousMillis = currentMillis; // Mettre à jour le temps de la dernière impulsion
    }
    else {
        // Vérifier si le véhicule est arrêté
        unsigned long currentMillis = millis();
        float period = (currentMillis - previousMillis) / 1000.0; // Période en secondes
        // On considère le véhicule comme arrêté si la période dépasse le seuil (la roue tourne trop lentement,
        // donc l'aimant ne passe pas devant le capteur)
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

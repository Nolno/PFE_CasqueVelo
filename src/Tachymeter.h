#ifndef TACHYMETER_H
#define TACHYMETER_H

#include <Arduino.h>

class Tachymeter {
public:
    /**
     * @brief Constructeur de la classe Tachymeter.
     *
     * @param sensorPin Numéro de la broche du capteur de vitesse.
     * @param buzzerPin Numéro de la broche du buzzer.
     * @param wheelRadius Rayon de la roue (en mètres).
     * @param stationarySpeed Vitesse seuil pour considérer le véhicule comme arrêté (en m/s). Valeur par défaut : 0.55.
     */
    Tachymeter(int sensorPin, int buzzerPin, float wheelRadius, float stationarySpeed = 0.55);


    // Méthodes publiques

    void initialize();
    void update();
    float getSpeed() const; // Méthode pour récupérer la vitesse actuelle

    // Instance statique pour interruptions
    static Tachymeter *instance;

private:
    // Attributs privés
    float stationarySpeed; // Vitesse seuil pour considérer le véhicule comme arrêté (m/s)
    float stationaryPeriod; // Période seuil pour considérer le véhicule comme arrêté (s)
    int sensorPin;
    int buzzerPin;
    float wheelRadius;
    volatile int pulseCounter;
    volatile bool eventFlag;
    unsigned long previousMillis;
    float currentSpeed; // Stocke la vitesse calculée

    // Méthodes privées
    void pulseEvent();
};

#endif // TACHYMETER_H

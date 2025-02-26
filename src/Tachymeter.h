#ifndef TACHYMETER_H
#define TACHYMETER_H

#include <Arduino.h>

/**
 * @brief Classe pour gérer un tachymètre.
 *
 * Le tachymètre mesure la vitesse du véhicule en fonction des impulsions d'un capteur de vitesse.
 * Une impulsion est générée à chaque tour de roue complet, quand un aimant passe devant un capteur à effet Hall.
 */
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
    /**
     * @brief Initialiser le tachymètre.
     *
     * Cette méthode configure les broches du capteur de vitesse et du buzzer, et attache l'interruption pour gérer les impulsions.
     */
    void initialize();

    /**
     * @brief Routine du tachymètre.
     *
     * Cette méthode doit être appelée dans la boucle de contrôle principale (loop) pour traiter les événements.
     */
    void update();

    /**
     * @brief Récupérer la vitesse actuelle.
     *
     * @return La vitesse actuelle en m/s.
     */
    float getSpeed() const;

    // Instance statique pour interruptions
    static Tachymeter *instance;

private:
    // Attributs privés
    float stationarySpeed; /**< Vitesse seuil pour considérer le véhicule comme arrêté (m/s) */
    float stationaryPeriod; /**< Période seuil pour considérer le véhicule comme arrêté (s) */
    int sensorPin; /**< Numéro de la broche du capteur de vitesse */
    int buzzerPin; /**< Numéro de la broche du buzzer */
    float wheelRadius; /**< Rayon de la roue (m) */
    volatile int pulseCounter; /**< Compteur d'impulsions */
    volatile bool eventFlag; /**< Drapeau d'événement */
    unsigned long previousMillis; /**< Temps de la dernière impulsion (ms) */
    float currentSpeed; /**< Vitesse actuelle (m/s) */

    // Méthodes privées
    /**
     * @brief Gestion de l'interruption.
     *
     * Cette méthode est appelée à chaque impulsion du capteur de vitesse.
     */
    void pulseEvent();
};

#endif // TACHYMETER_H

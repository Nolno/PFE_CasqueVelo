/**
* @file Alarm.h
 * @author Antoine Martin
 * @brief Code de gestion de l'alarme. Ce code fonctionne sur un ESP32-WROOM-32E avec une LED et un buzzer.
 * @version 1.0
 * @date 25/02/2024
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef ALARM_H
#define ALARM_H
#include <Arduino.h>


/**
 * @brief Classe de gestion de l'alarme.
 *
 * La classe permet de gérer l'alarme en fonction de l'angle de dépassement
 * et du temps de dépassement de l'angle. Si l'angle dépasse un certain seuil pendant un certain temps,
 * l'alarme est déclenchée.
 *
 * Les angles et temps limites sont calculés en fonction de la vitesse du vélo.
 *
 * L'alarme est composée d'une LED et d'un buzzer.
 */
class Alarm {
private :
    int pinLED; /**< Pin connecté à la LED */
    int pinBuzzer; /**< Pin connecté au buzzer */
    bool alarmState; /**< État de l'alarme */
    bool silence; /**< État de silence de l'alarme */
    unsigned long initialExceedTime; /**< Temps initial de dépassement de l'angle */
    bool blinkState; /**< État de clignotement de la LED */
    unsigned long lastBlinkChange; /**< Dernier changement d'état de clignotement */
    double maxAngle; /**< Seuil maximum de l'angle à vitesse nulle */
    double maxTime; /**< Seuil maximum de temps de dépassement de l'angle à vitesse nulle */
    double angleAttenuation; /**< Facteur d'atténuation pour l'angle */
    double timeAttenuation; /**< Facteur d'atténuation pour le temps */

    // Contrôle de l'alarme
    void start(); /**< Méthode pour démarrer l'alarme. Allume la LED et active le buzzer */
    void stop() const; /**< Méthode pour arrêter l'alarme. Éteint la LED et désactive le buzzer */

public:
    /**
    * @brief Constructeur de la classe Alarm.
    * @param pinBuzzer : Pin connecté au buzzer
    * @param pinLED : Pin connecté à la LED
    * @param maxAngle : Seuil maximum de l'angle à vitesse nulle (en degrés)
    * @param maxTime : Seuil maximum de temps de dépassement de l'angle à vitesse nulle (en secondes)
    * @param angleAttenuation : Facteur d'atténuation pour l'angle
    * @param timeAttenuation
    *
    */
    Alarm(int pinBuzzer, int pinLED, double maxAngle=80, double maxTime=3, double angleAttenuation=0.1, double timeAttenuation=0.1);

    /**
    * @brief Méthode pour initialiser l'alarme. Cette méthode doit être appelée une seule fois au démarrage du programme.
    */
    void init() const;

    /**
     * @brief Méthode de mise à jour de l'alarme. Elle vérifie si, à la vitesse actuelle, l'angle actuel est en dépassement
     * de seuil d'angle depuis un temps supérieur au seuil de temps. Si c'est le cas, l'alarme est déclenchée.
     *
     * Doit être appelée régulièrement dans la boucle principale (loop).
     *
     * Remarque : Les angles de tangage et de roulis ne sont pas utilisés pour le moment.
     *
     * @param ypr_diff tableau contenant les différences d'angles (yaw, pitch, roll) entre le master et le slave
     * @param speed vitesse du vélo en km/h
     * @return true si l'alarme est déclenchée, false sinon
     */
    boolean update(const float (&ypr_diff)[3], double speed);

    /**
     * @brief Récupère les limites d'angle et de temps en fonction de la vitesse du vélo. Les limites sont calculées
     * à partir des limites à vitesse nulle et sont atténuées en fonction de la vitesse  via des facteurs d'atténuation,
     * suivant une exponentielle décroissante.
     * @param v_kmh vitesse du vélo en km/h
     * @return une paire de double contenant les limites d'angle et de temps
     */
    std::pair<double, double> getLimits(double v_kmh) const;
};



#endif //ALARM_H

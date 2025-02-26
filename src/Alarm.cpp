//
// Created by Antoine on 03/02/2025.
//

#include "Alarm.h"

Alarm::Alarm(const int pinBuzzer, const int pinLED, const double maxAngle, const double maxTime,
             const double angleAttenuation,
             const double timeAttenuation)
    : pinLED(pinLED), pinBuzzer(pinBuzzer), alarmState(false), silence(false), initialExceedTime(0),
      blinkState(false), lastBlinkChange(0), maxAngle(maxAngle), maxTime(maxTime),
      angleAttenuation(angleAttenuation), timeAttenuation(timeAttenuation)
{
}

void Alarm::init() const
{
    pinMode(pinLED, OUTPUT); // Configure le pin de la LED en sortie
    // pinMode(pinBuzzer, OUTPUT); // Configure le pin du buzzer en sortie

    // Buzzer test
    tone(pinBuzzer, 500);
    delay(50);
    noTone(pinBuzzer);

    // Initialise la LED état bas (éteinte)
    digitalWrite(pinLED, LOW);
}

boolean Alarm::update(const float (&ypr_diff)[3], const double speed)
{
    // Récupère les limites en fonction de la vitesse
    const std::pair<double, double> limits = getLimits(speed);
    const double limitAngle = limits.first;
    const double limitTime = limits.second;
    // Serial.print("Limit angle: "); Serial.print(limitAngle); Serial.print(" Limit time: "); Serial.println(limitTime);

    // Récupère les différences d'angles
    const double diffYaw = ypr_diff[0];
    // REMARQUE : Les angles de tangage et de roulis ne sont pas utilisés pour le moment
    // double diffPitch = ypr_diff[1];
    // double diffRoll = ypr_diff[2];

    // On vérifie si l'angle actuel dépasse la limite
    if (abs(diffYaw) > limitAngle)
    {
        // Si le temps de dépassement est nul (l'angle n'était pas dépassé au temps t-1)
        if (initialExceedTime == 0)
        {
            // On enregistre le temps actuel comme temps initial de dépassement
            initialExceedTime = millis();
        }
        // Si le temps de dépassement est supérieur au temps limite
        if (millis() - initialExceedTime > limitTime * 1000)
        {
            // On déclenche l'alarme
            alarmState = true;
        }
    }
    else
    {
        // Si l'angle n'est plus dépassé, on réinitialise le temps de dépassement
        initialExceedTime = 0;
        // On arrête l'alarme
        alarmState = false;
    }

    // Si l'alarme est déclenchée, on la démarre
    if (alarmState)
    {
        start();
        return true;
    }
    // Sinon, on l'arrête
    else
    {
        stop();
        return false;
    }
}

std::pair<double, double> Alarm::getLimits(const double v_kmh) const
{
    // Calcul des limites d'angle et de temps en fonction de la vitesse : fonction exponentielle décroissante
    double limitAngle = (maxAngle * std::exp(-(angleAttenuation * v_kmh)));
    double limitTime = (maxTime * std::exp((-timeAttenuation * v_kmh)));

    // Retourne les limites
    return std::make_pair(limitAngle, limitTime);
}

void Alarm::start()
{
    Serial.println("Alarm started");
    // Si l'alarme n'est pas en mode silence
    if (!silence)
    {
        // Active le buzzer
        tone(pinBuzzer, 500);
    }

    // Clignotement de la LED toutes les 500ms
    if (lastBlinkChange == 0 || millis() - lastBlinkChange > 500)
    {
        blinkState = !blinkState;
        lastBlinkChange = millis();
    }
    if (blinkState)
    {
        digitalWrite(pinLED, HIGH);
    }
    else
    {
        digitalWrite(pinLED, LOW);
    }
}

void Alarm::stop() const
{
    // On coupe le buzzer et on éteint la LED
    digitalWrite(pinLED, LOW);
    noTone(pinBuzzer);
}

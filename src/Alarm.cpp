//
// Created by Antoine on 03/02/2025.
//

#include "Alarm.h"

Alarm::Alarm(int pinBuzzer, int pinLED, double maxAngle, double maxTime, double angleAttenuation,
             double timeAttenuation)
    : pinBuzzer(pinBuzzer), pinLED(pinLED), maxAngle(maxAngle), maxTime(maxTime), angleAttenuation(angleAttenuation),
      timeAttenuation(timeAttenuation), alarmState(false), silence(false), initialExceedTime(0), blinkState(false)
{
}

void Alarm::init()
{
    pinMode(pinLED, OUTPUT); // Set the LED pin as output
    // pinMode(pinBuzzer, OUTPUT); // Set the buzzer pin as output
    tone(pinBuzzer, 500);
    delay(50);
    noTone(pinBuzzer);
    digitalWrite(pinLED, LOW); // Turn off the LED
}

boolean Alarm::update(float (&ypr_diff)[3], double speed)
{
    std::pair<double, double> limits = getLimits(speed);
    double limitAngle = limits.first;
    double limitTime = limits.second;
    Serial.print("Limit angle: "); Serial.print(limitAngle); Serial.print(" Limit time: "); Serial.println(limitTime);
    double diff_yaw = ypr_diff[0];
    double diff_pitch = ypr_diff[1];
    double diff_roll = ypr_diff[2];

    // Check if the angle exceeds the threshold
    if (abs(diff_yaw) > limitAngle)
    {
        if (initialExceedTime == 0)
        {
            initialExceedTime = millis();
        }
        // Check if the time exceeds the threshold
        if (millis() - initialExceedTime > limitTime * 1000)
        {
            alarmState = true;
        }
    }
    else
    {
        initialExceedTime = 0;
        alarmState = false;
    }

    if (alarmState)
    {
        start();
        return true;
    }
    else
    {
        stop();
        return false;
    }
}

std::pair<double, double> Alarm::getLimits(double v_kmh)
{
    double limitAngle = (maxAngle * std::exp(-(angleAttenuation * v_kmh)));
    double limitTime = (maxTime * std::exp((-timeAttenuation * v_kmh)));

    return std::make_pair(limitAngle, limitTime);
}

void Alarm::start()
{
    Serial.println("Alarm started");
    if (!silence)
    {
        tone(pinBuzzer, 500);
    }
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

void Alarm::stop()
{
    digitalWrite(pinLED, LOW);
    // digitalWrite(pinBuzzer, LOW);
    noTone(pinBuzzer);
}

//
// Created by Antoine on 03/02/2025.
//

#include "Alarm.h"

Alarm::Alarm(int pinBuzzer, int pinLED, double maxAngle, double maxTime) : pinBuzzer(pinBuzzer), pinLED(pinLED), maxAngle(maxAngle), maxTime(maxTime), alarmState(false), initialExceedTime(0), blinkState(false), angleAttenuation(0.0), timeAttenuation(0.0) {}


void Alarm::init() {
    pinMode(pinBuzzer, OUTPUT); // Set the buzzer pin as output
    pinMode(pinLED, OUTPUT); // Set the LED pin as output

    digitalWrite(pinBuzzer, LOW); // Turn off the buzzer
    digitalWrite(pinLED, LOW); // Turn off the LED
}

boolean Alarm::update(float (&ypr)[3], float (&ypr_slave)[3], float speed) {
    std::pair<double, double> limits = getLimits(speed);
    double limitAngle = limits.first;
    double limitTime = limits.second;

    double diff_yaw = ypr[0] - ypr_slave[0];
    double diff_pitch = ypr[1] - ypr_slave[1];
    double diff_roll = ypr[2] - ypr_slave[2];

    // Check if the angle exceeds the threshold
    if (abs(diff_yaw) > limitAngle) {
        if (initialExceedTime == 0) {
            initialExceedTime = millis();
        }
        // Check if the time exceeds the threshold
        if (millis() - initialExceedTime > limitTime) {
            alarmState = true;
        }
    } else {
        initialExceedTime = 0;
        alarmState = false;
    }

    if (alarmState) {
        start();
        return true;
    } else {
        stop();
        return false;
    }
}

std::pair<double, double> Alarm::getLimits(double v_kmh) {

    double limitAngle = (maxAngle * std::exp(angleAttenuation * v_kmh));
    double limitTime = (maxTime * std::exp(timeAttenuation * v_kmh));

    return std::make_pair(limitAngle, limitTime);
}

void Alarm::start() {
    if (blinkState) {
        digitalWrite(pinLED, HIGH);
    } else {
        digitalWrite(pinLED, LOW);
    }
    tone(pinBuzzer, 500);
}

void Alarm::stop() {
    digitalWrite(pinLED, LOW);
    noTone(pinBuzzer);
}

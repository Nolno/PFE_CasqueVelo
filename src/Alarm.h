//
// Created by Antoine on 03/02/2025.
//

#ifndef ALARM_H
#define ALARM_H
#include <Arduino.h>



class Alarm {
private :
    int pinLED; // Pin connected to the LED
    int pinBuzzer; // Pin connected to the buzzer
    bool alarmState; // State of the alarm (true = on, false = off)
    bool silence; // Whether the alarm
    unsigned long initialExceedTime; // Time when the angle start exceeded the threshold
    bool blinkState;
    unsigned long lastBlinkChange;
    double maxAngle; // Maximum angle threshold
    double maxTime; // Maximum time threshold
    double angleAttenuation; // Attenuation factor for the angle
    double timeAttenuation; // Attenuation factor for the time

    /**
    * @brief Start the alarm, turn on the buzzer and handle the LED blinking
    */
    void start(); // Start the alarm
    /**
    * @brief Stop the alarm, turn off the buzzer and the LED
    */
    void stop(); // Stop the alarm

public:
    /**
    * @brief Constructor
    * @param pinBuzzer : Pin connected to the buzzer
    * @param pinLED : Pin connected to the LED
    * @param maxAngle : Maximum angle threshold (in degrees)
    * @param maxTime : Maximum time threshold (in seconds)
    */
    Alarm(int pinBuzzer, int pinLED, double maxAngle=80, double maxTime=3, double angleAttenuation=0.1, double timeAttenuation=0.1);

    /**
    * @brief Initialize the alarm system. Set the buzzer and LED pins as output and turn them off
    */
    void init();

    /**
     * @brief Alarm update function. Check if the alarm needs to be triggered based on the yaw, pitch and roll angles
     * and the speed. If the threshold angle is exceeded for more than the threshold time, the alarm is triggered.
     * @param ypr list of yaw, pitch and roll angles for the master device [yaw, pitch, roll]
     * @param ypr_slave list of yaw, pitch and roll angles for the slave device [yaw, pitch, roll]
     * @param speed speed of the master device in km/h
     * @return true if the alarm is triggered, false otherwise
     */
    boolean update(float (&ypr_diff)[3], double speed);

    /**
     * @brief Get the limits for the angle and time based on the speed, the maximum angle and time, and the attenuation
     * factors. Maximums and attenuation factors are set in the constructor.
     * @param v_kmh speed of the master device in km/h
     * @return a pair containing the angle and time limits
     */
    std::pair<double, double> getLimits(double v_kmh);
};



#endif //ALARM_H

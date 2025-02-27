#include <Arduino.h>

const int buttonPin = 32;  // Broche à laquelle le bouton est connecté
int buttonState = 0;      // Variable pour lire l'état du bouton

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);  // Configure la broche comme entrée avec résistance pull-up interne
    Serial.begin(115200);             // Initialise la communication série
}

void loop() {
    buttonState = digitalRead(buttonPin);  // Lit l'état du bouton
    Serial.println(buttonState);
    if (buttonState == LOW) {
        // Le bouton est pressé
        Serial.println("Bouton pressé !");
        // Ajoutez ici le code pour l'action à effectuer
   }

    delay(50);  // Petit délai pour éviter les rebonds du bouton
}

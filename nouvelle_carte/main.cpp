// Includes
#include <Arduino.h>
#include <Wire.h>


void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);  // Attendre un peu avant de scanner

  Serial.println("I2C Scanner");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Capteur trouvé à l'adresse: 0x");
      Serial.println(address, HEX);
    }
  }
}

void loop() {
  // Rien à faire dans la loop
}

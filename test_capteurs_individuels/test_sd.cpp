#include <Arduino.h>
#include <SD.h>

#define CS_PIN 5 /**< CS pin de la carte SD */


File myFile; /**< Objet pour la gestion de la carte SD */

void setup()
{
    Serial.begin(115200); // Initialisation de la communication série
    pinMode(CS_PIN, OUTPUT); // Initialisation de la broche CS - permet de sélectionner la carte SD

    Serial.print("Initializing SD card... ");
    if (!SD.begin(CS_PIN))
    {
        Serial.println("Card initialization failed!");
        while (true); // Si la carte SD n'est pas détectée, on arrête le programme
    }
    Serial.println("Initialization done.");
    // On créé un fichier "test.txt" sur la carte SD
    myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile)
    {
        // Si le fichier est ouvert, on écrit une chaîne de caractères
        myFile.println("Hello from SD card!");
        // On ferme le fichier
        myFile.close();
        Serial.println("File written successfully.");
    }
    else
    {
        // Si le fichier n'a pas pu être ouvert, on affiche un message d'erreur
        Serial.println("Error opening file.");
    }
}

void loop()
{
    // On ouvre le fichier "test.txt" en mode écriture
    myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile)
    {
        // Si le fichier est ouvert, on écrit une chaîne de caractères
        myFile.println("Hello from SD card!");
        // On ferme le fichier
        myFile.close();
        Serial.println("File written successfully.");
    }
    else
    {
        // Si le fichier n'a pas pu être ouvert, on affiche un message d'erreur
        Serial.println("Error opening file.");
    }
    // On attend 1 seconde
    delay(1000);
}

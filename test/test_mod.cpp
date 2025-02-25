#include <unity.h>
#include "Alarm.h"
#include <iostream>
#include <list> 
#include <limits>

//Déclaration des variables : les valeurs attendues des tests de valeurs fonctionnelles
double valeur_angle1_5;
double valeur_temps1_5;
double valeur_angle1_10;
double valeur_temps1_10;
double valeur_angle1_15;
double valeur_temps1_15;

//Déclaration des variables : les valeurs attendues des tests de limites fonctionnelles
double limite_angle_0;
double limite_temps_0;
double limite_angle_virgule;
double limite_temps_virgule;
double hors_limite;
double limitMin;
double limitMax;

//Déclaration de la liste des valeurs des tests de valeurs fonctionnelles
std::list<double> listValeurs_jeu1 {};
//Déclaration de la liste des valeurs des tests de limites fonctionnelles
std::list<double> listLimit{};

/* #define PIN_BUZZER 2
#define PIN_HAL 25
#define PIN_LED 33 // Pin connected to the LED */


void setUp() {
  //Déclaration et initialisation de l'objet alarmSystem_test1
  alarmSystem_test1 = new Alarm(/* PIN_BUZZER, PIN_LED, */ 80, 3, 0.1, 0.1);
  alarmSystem_test1->init();  

  //Initialisation des variables des tests de valeurs fonctionnelles
  valeur_angle1_5 = 48.52;
  valeur_temps1_5 = 1.82;
  valeur_angle1_10 = 29.43;
  valeur_temps1_10 = 1.10;
  valeur_angle1_15 = 17.85;
  valeur_temps1_15 = 0.67;

  //Initialisation des variables des tests de limites fonctionnelles
  limite_angle_0 = 80;
  limite_temps_0 = 3;
  limite_angle_virgule = 76;
  limite_temps_virgule = 2;
  hors_limite = 0.0;
  limitMin = std::numeric_limits<double>::lowest() ;
  limitMax = std::numeric_limits<double>::max() ;

  //Initialisation de la liste des valeurs des tests de valeurs fonctionnelles
  listValeurs_jeu1 = {valeur_angle1_5, valeur_angle1_10, valeur_angle1_15, valeur_temps1_5, valeur_temps1_10, valeur_temps1_15};
  //Initialisation de la liste des valeurs des tests de limites fonctionnelles
  listLimit = {limite_angle_0, limite_temps_0, limite_angle_virgule, limite_temps_virgule, hors_limite, limitMin, limitMax};
}

void tearDown(void) {
  //Suppression de l'objet alarmSystem_test1
    delete alarmSystem_test1;

    //Suppression des valeurs de la liste des tests de valeurs fonctionnelles
    for (double valeur : listValeurs_jeu1) {
      valeur = 0;
    }
    //Suppression des valeurs de la liste des tests de limites fonctionnelles
    for (double valeur : listLimit) {
      valeur = 0;
    }
}

//Tests de valeurs fonctionnelles
 void test_valeur_jeu1(void){
      std::cout << "Running test_valeur_jeu1" << std::endl;
      //Test de l'angle à 5 km/h
      TEST_ASSERT_EQUAL(valeur_angle1_5, alarmSystem_test1->getLimits(5).first);
      //Test du temps à 5 km/h
      TEST_ASSERT_EQUAL(valeur_temps1_5, alarmSystem_test1->getLimits(5).second);
      std::cout << "Test 5km done" << std::endl;

      //Tests de l'angle à 10km/h
      TEST_ASSERT_EQUAL(valeur_angle1_10, alarmSystem_test1->getLimits(10).first);
      //Tests du temps à 10km/h
      TEST_ASSERT_EQUAL(valeur_temps1_10, alarmSystem_test1->getLimits(10).second);
      std::cout << "Test 10km done" << std::endl;

      //Test de l'angle à 15km/h
      TEST_ASSERT_EQUAL(valeur_angle1_15, alarmSystem_test1->getLimits(15).first);
      //Test du temps à 15km/h
      TEST_ASSERT_EQUAL(valeur_temps1_15, alarmSystem_test1->getLimits(15).second);
  std::cout << "Test 15km done" << std::endl;
  std::cout << "Finishing test_valeur_jeu1" << std::endl;
}

  //Tests des limites fonctionnelles
  void test_limit(void){
  std::cout << "Running test_limit" << std::endl;
  //Test de l'angle avec une vitesse nulle
  TEST_ASSERT_EQUAL(limite_angle_0, alarmSystem_test1->getLimits(0).first);
  //Test du temps avec une vitesse nulle
  TEST_ASSERT_EQUAL(limite_temps_0, alarmSystem_test1->getLimits(0).second);
  std::cout << "Test vitesse nulle done" << std::endl;

  //Test de l'angle avec une valeur à virgule
  TEST_ASSERT_EQUAL(limite_angle_virgule, alarmSystem_test1->getLimits(0.5).first);
  //Test du temps avec une valeur à virgule
  TEST_ASSERT_EQUAL(limite_temps_virgule, alarmSystem_test1->getLimits(0.5).second);
  std::cout << "Test vitesse virgule done" << std::endl;

  //Test de l'angle avec une vitesse négative
  TEST_ASSERT_EQUAL(hors_limite, alarmSystem_test1->getLimits(-10).first);
  //Test du temps avec une vitesse négative
  TEST_ASSERT_EQUAL(hors_limite, alarmSystem_test1->getLimits(-10).second);
  std::cout << "Test vitesse negative done" << std::endl;

  //Test de l'angle avec une vitesse égale à la valeur minimale des doubles
  TEST_ASSERT_EQUAL(hors_limite, alarmSystem_test1->getLimits(limitMin).first);
  //Test du temps avec une vitesse égale à la valeur minimale des doubles
  TEST_ASSERT_EQUAL(hors_limite, alarmSystem_test1->getLimits(limitMin).second);
  std::cout << "Test vitesse min done" << std::endl;
  
  //Test de l'angle avec une vitesse égale à la valeur maximale des doubles
  TEST_ASSERT_EQUAL(hors_limite, alarmSystem_test1->getLimits(limitMax).first);
  //Test du temps avec une vitesse égale à la valeur maximale des doubles
  TEST_ASSERT_EQUAL(hors_limite, alarmSystem_test1->getLimits(limitMax).second);
  std::cout << "Test vitesse max done" << std::endl;
  std::cout << "Finishing test_limit_v5" << std::endl;
}  

int runUnityTests(void) {
  //Initialisation de l'unité de test
  UNITY_BEGIN();
  std::cout << "Running tests" << std::endl;
  //Compile les tests
  RUN_TEST(test_valeur_jeu1);
  RUN_TEST(test_limit);
  return UNITY_END();
}

//Fonction principale qui appelle l'unité de tests
int main(void) {
  return runUnityTests();
}
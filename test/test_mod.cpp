#include <Arduino.h>
#include <unity.h>
#include "main.h"

String STR_TO_TEST;
bool limit;


void setUp(void) {
    // set stuff up here
    STR_TO_TEST = "Hello, world!";
    limit = false;
}

void tearDown(void) {
    // clean stuff up here
    STR_TO_TEST = "";
    limit = NULL;
}

void test_limit(void){
    TEST_ASSERT_EQUAL(limit, depasserLimites(45, 70, 10, 70, 5, 0.1, 0.2));
}

void test_string_concat(void) {
    String hello = "Hello, ";
    String world = "world!";
    TEST_ASSERT_EQUAL_STRING(STR_TO_TEST.c_str(), (hello + world).c_str());
}



void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();

    RUN_TEST(test_string_concat);
    RUN_TEST(test_limit);


    UNITY_END(); // stop unit testing
}

void loop()
{
}
#include <Arduino.h>

const int solenoidPin = 12;

int solenoidOnTime = 500;

void activateSolenoid() {
    digitalWrite(solenoidPin, LOW);
    delay(solenoidOnTime);
    digitalWrite(solenoidPin, HIGH);
}

void setup() {
    pinMode(solenoidPin, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    activateSolenoid();
    delay(1000);
}
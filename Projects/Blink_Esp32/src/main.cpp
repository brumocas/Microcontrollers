#include <Arduino.h>

const int ledPin_Red = 33; // GPIO33
const int ledPin_Green = 25; // GPIO25

void setup() {
  pinMode(ledPin_Red, OUTPUT);
  pinMode(ledPin_Green, OUTPUT);

}

void loop() {
  digitalWrite(ledPin_Red, HIGH); // Turn the LED on (HIGH)
  digitalWrite(ledPin_Green, LOW);
  delay(1000); // Wait for 1 second
  digitalWrite(ledPin_Red, LOW); // Turn the LED off (LOW)
  digitalWrite(ledPin_Green, HIGH);
  delay(1000); // Wait for 1 second
}
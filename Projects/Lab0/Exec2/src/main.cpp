#include <Arduino.h>

const int ledPin_Red = 25; // GPIO33

void setup() {
  pinMode(ledPin_Red, OUTPUT);

}

void loop() {

  // Turn the LED on (HIGH)
  digitalWrite(ledPin_Red, HIGH); 
  Serial.println("Led on");
  delay(1000);

  // Turn the LED off (LOW)
  digitalWrite(ledPin_Red, LOW); 
  Serial.println("Led off");
  delay(1000);
}
#include <Arduino.h>

const int ledPin_Red = 33; // GPIO33
const int ledPin_Green = 25; // GPIO25
const int button1 = 36; // GPIO36 
int button1_state; // To store button 1 state

void setup() {
  pinMode(ledPin_Red, OUTPUT);
  pinMode(ledPin_Green, OUTPUT);
  pinMode(button1, INPUT_PULLDOWN);
}

void loop() {

  button1_state = digitalRead(button1);
  delay(25);

  if(button1_state == HIGH){

  // Turn the LED on (HIGH)
  digitalWrite(ledPin_Red, HIGH); 
  digitalWrite(ledPin_Green, HIGH);
  Serial.println("Led on");

  }
  else
  {
    
  // Turn the LED off (LOW)
  digitalWrite(ledPin_Red, LOW); 
  digitalWrite(ledPin_Green, LOW);
  Serial.println("Led off");

  }
  
}
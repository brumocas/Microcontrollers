#include <Arduino.h>
#include <Servo.h>
#include "ServosInfo.h"
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>

#define Servo1_Pin 9
#define Servo2_Pin 10
#define Servo3_Pin 11
#define Servo4_Pin 12
#define Built_IN_LED 25

VL53L0X tof;
float distance, prev_distance;

int LED_state;

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Input variables

// Output variables


// Our finite state machines
fsm_t fsm1;

unsigned long interval, last_cycle;
unsigned long loop_micros;
int8_t angle;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}


void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  
  servo1.servo.attach(Servo1_Pin, 500,  2500);
  servo2.servo.attach(Servo2_Pin, 500 , 2500);
  servo3.servo.attach(Servo3_Pin, 500 , 2500);
  servo4.servo.attach(Servo4_Pin, 500 , 2500);
  // Start the serial port with 115200 baudrate
  Serial.begin(115200); 

  Wire.setSDA(9);
  Wire.setSCL(8);

  Wire.begin();

  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  

  interval = 40;
  angle = -90;
  set_state(fsm1, 0);
}

void loop() 
{    

    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
      if (b == '-') angle-=angle;
      if (b == '+') angle+=angle;
    }  
    // To measure the time between loop() calls
    unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      
      // Read the inputs

      // FSM processing

      // Update tis for all state machines
      unsigned long cur_time = millis();   // Just one call to millis()
      fsm1.tis = cur_time - fsm1.tes; 

      // Calculate next state for the first state machine

      // Update the states
      set_state(fsm1, fsm1.new_state);

      // Actions set by the current state of the first state machine

      // Set the outputs


      //Test tof
      if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
      }
 
      // Start new distance measure
     tof.startReadRangeMillimeters(); 

      // Toggle builtin LED    
  
      Serial.print(" Dist: ");
      Serial.print(distance, 3);
      Serial.println();
      digitalWrite(LED_BUILTIN, HIGH);
      

      //Testing servos
      /* 
      servo1.setPosition(-90.00);
      servo2.setPosition(-90.00);
      servo3.setPosition(-90.00);
      servo4.setPosition(-90.00);
      delay(500);
      servo1.setPosition(0.00);
      servo2.setPosition(0.00);
      servo3.setPosition(0.00);
      servo4.setPosition(0.00);
      delay(500);
      servo1.setPosition(90.00);
      servo2.setPosition(90.00);
      servo3.setPosition(90.00);
      servo4.setPosition(90.00);
      delay(500);
      */
    
      // Debug using the serial port
      /* 
      Serial.print(" fsm1.state: ");
      Serial.print(fsm1.state);

      Serial.print(" loop: ");
      Serial.print(micros() - loop_micros);
      Serial.println();
      */  
    }
    
}


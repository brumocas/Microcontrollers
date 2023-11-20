#include <Arduino.h>
#include <Servo.h>
#include "ServosInfo.h"

#define Servo1_Pin 9
#define Servo2_Pin 10
#define Servo3_Pin 11
#define Servo4_Pin 12

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
  servo1.servo.attach(Servo1_Pin, 500,  2500);
  servo2.servo.attach(Servo2_Pin, 500 , 2500);
  servo3.servo.attach(Servo3_Pin, 500 , 2500);
  servo4.servo.attach(Servo4_Pin, 500 , 2500);
  // Start the serial port with 115200 baudrate
  Serial.begin(115200);
  angle = -90;

  interval = 40;
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


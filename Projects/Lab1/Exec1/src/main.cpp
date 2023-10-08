#include <Arduino.h>

#define LED1_pin 6
#define LED2_pin 7
#define LED3_pin 8
#define LED4_pin 9
#define LED5_pin 10

#define S1_pin 2
#define S2_pin 3
#define S3_pin 4

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Input variables
uint8_t S1, prevS1;
uint8_t S2, prevS2;
uint8_t S3, prevS3;

// Output variables
uint8_t LED_1, LED_2, LED_3, LED_4, LED_5;

// Our finite state machines
fsm_t fsm1, fsm2, fsm3;

unsigned long interval, last_cycle;
unsigned long loop_micros;
uint16_t default_time, blink_time, aux;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}


void setup() 
{
  pinMode(LED1_pin, OUTPUT);
  pinMode(LED2_pin, OUTPUT);
  pinMode(LED3_pin, OUTPUT);
  pinMode(LED4_pin, OUTPUT);
  pinMode(LED5_pin, OUTPUT);

  pinMode(S1_pin, INPUT_PULLUP);
  pinMode(S2_pin, INPUT_PULLUP);
  pinMode(S3_pin, INPUT_PULLUP);

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);

  // In ms
  default_time = 2000; 
  blink_time = 1000;
  aux = 0;

  interval = 40;
  set_state(fsm1, 0);
  set_state(fsm2, 0);
  set_state(fsm3, 0);
}

void loop() 
{
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
      b = Serial.read();       
      // Terminal not needed
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
      prevS1 = S1;
      prevS2 = S2;
      prevS3 = S3;
      S1 = !digitalRead(S1_pin);
      S2 = !digitalRead(S2_pin);
      S3 = !digitalRead(S3_pin);

      // FSM processing

      // Update tis for all state machines
      unsigned long cur_time = millis();   // Just one call to millis()
      fsm1.tis = cur_time - fsm1.tes;
      fsm2.tis = cur_time - fsm2.tes;
      fsm3.tis = cur_time - fsm3.tes;


      // Calculate next state for the first state machine
      if (fsm1.state == 0 && S1 && !prevS1){
        fsm1.new_state = 1;
        aux = fsm2.tis;
      } else if (fsm1.state == 1 && S1 && !prevS1){
        fsm1.new_state = 0;
        fsm2.tes = millis() - aux;
      }

      // Calculate next state for the second state machine
      if (fsm2.state == 0 && fsm1.state == 1){
        fsm2.new_state = 1;
      } else if (fsm2.state == 1 && fsm2.tis > default_time && fsm1.state == 1){
        fsm2.new_state = 2;
      } else if (fsm2.state == 2 && fsm2.tis > default_time && fsm1.state == 1){
        fsm2.new_state = 3;
      } else if (fsm2.state == 3 && fsm2.tis > default_time && fsm1.state == 1){
        fsm2.new_state = 4;
      } else if (fsm2.state == 4 && fsm2.tis > default_time && fsm1.state == 1){
        fsm2.new_state = 5;
      } else if (fsm2.state == 5 && fsm2.tis > default_time && fsm1.state == 1){
        fsm2.new_state = 6;
      } else if (fsm2.state == 6 && fsm2.tis > blink_time && fsm1.state == 1){
        fsm2.new_state = 7;
      } else if (fsm2.state == 7 && fsm2.tis > blink_time && fsm1.state == 1){  
        fsm2.new_state = 6; 
      }

      // Calculate next state for the third state machine
      if (fsm3.state == 0 && S2 && !prevS2){
        fsm3.new_state = 1;
        default_time += 1000;
      } else if (fsm3.state == 1 && S3 && !prevS3){
        fsm3.new_state = 0;
        default_time = 2000;
      } else if (fsm3.state == 1 && S2 && !prevS2){
        default_time += 1000;
      } 

      // Update the states
      set_state(fsm1, fsm1.new_state);
      set_state(fsm2, fsm2.new_state);
      set_state(fsm3, fsm3.new_state);

      // Actions set by the current state of the first state machine
      // No actions here

      // Actions set by the current state of the second state machine
      LED_1 = (fsm2.state == 1) || (fsm2.state == 7);
      LED_2 = (fsm2.state == 1) || (fsm2.state == 2) || (fsm2.state == 7);
      LED_3 = (fsm2.state == 1) || (fsm2.state == 2) || (fsm2.state == 3) || (fsm2.state == 7);
      LED_4 = (fsm2.state == 1) || (fsm2.state == 2) || (fsm2.state == 3) || (fsm2.state == 4) || (fsm2.state == 7);
      LED_5 = (fsm2.state == 1) || (fsm2.state == 2) || (fsm2.state == 3) || (fsm2.state == 4) || (fsm2.state == 5) || (fsm2.state == 7);

      // Actions set by the current state of the third state machine
      // No actions here

      // Set the outputs
      digitalWrite(LED1_pin, LED_1);
      digitalWrite(LED2_pin, LED_2);
      digitalWrite(LED3_pin, LED_3);
      digitalWrite(LED4_pin, LED_4);
      digitalWrite(LED5_pin, LED_5);
      digitalWrite(LED3_pin, LED_3);

      // Debug using the serial port
      Serial.print("Sdown: ");
      Serial.print(S1);

      Serial.print(" Sup: ");
      Serial.print(S2);

      Serial.print(" Sgo: ");
      Serial.print(S3);

      Serial.print(" CSM: ");
      Serial.print(fsm1.state);

      Serial.print(" LSM: ");
      Serial.print(fsm2.state);

      Serial.print(" RTSM: ");
      Serial.print(fsm3.state);

      Serial.print(" LED_1: ");
      Serial.print(LED_1);

      Serial.print(" LED_2: ");
      Serial.print(LED_2);

      Serial.print(" LED_3: ");
      Serial.print(LED_3);

      Serial.print(" LED_4: ");
      Serial.print(LED_4);

      Serial.print(" LED_5: ");
      Serial.print(LED_5);

      Serial.print(" default_time: ");
      Serial.print(default_time);

      Serial.print(" blink_time: ");
      Serial.print(blink_time);

      Serial.print(" fsm2.tis: ");
      Serial.print(fsm2.tis);

      Serial.print(" aux: ");
      Serial.print(aux);

      /*
      Serial.print(" blink: ");
      Serial.print(blink_period);
      */

      /* 
      Serial.print(" loop: ");
      Serial.print(micros() - loop_micros);
      */

      Serial.println();
      
    }
    
}


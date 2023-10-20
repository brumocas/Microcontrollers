#include <Arduino.h>

#define MAXIMUM_NUM_NEOPIXELS 5
#include <NeoPixelConnect.h>

#define LED_PIN 22

// Create an instance of NeoPixelConnect and initialize it
// to use GPIO pin 22 as the control pin, for a string
// of 8 neopixels. Name the instance p
NeoPixelConnect strip(LED_PIN, MAXIMUM_NUM_NEOPIXELS, pio0, 0);

#define S1_pin 2
#define S2_pin 3
#define S3_pin 4

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

typedef struct {
  // RGB struct
  unsigned int led_state, R,G,B;
} led_RGB;

// Input variables
uint8_t S1, prevS1;
uint8_t S2, prevS2;
uint8_t S3, prevS3;

// Output variables
led_RGB led1, led2, led3, led4, led5;
uint8_t LED_1,LED_2,LED_3,LED_4,LED_5;

// Our finite state machines
fsm_t fsm1, fsm2, fsm3;

unsigned long aux,aux_default;



unsigned long interval, last_cycle;
unsigned long loop_micros;
uint16_t default_time, blink_time;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

// Set individual led state
void set_led( led_RGB& led, int led_state, int red, int green, int blue){
  led.led_state = led_state;
  led.R = red;
  led.G = green;
  led.B = blue;
}

// Lit led strip
 void set_led_strip(led_RGB led1, led_RGB led2, led_RGB led3, led_RGB led4, led_RGB led5){
       
    if (led1.led_state)
    {
      strip.neoPixelSetValue(0, led1.R, led1.G, led1.B);
    } else{
      strip.neoPixelSetValue(0, 0, 0, 0);
    }

    if (led2.led_state)
    {
      strip.neoPixelSetValue(1, led2.R, led2.G, led2.B);
    } else{
      strip.neoPixelSetValue(1, 0, 0, 0);
    }

    if (led3.led_state)
    {
      strip.neoPixelSetValue(2, led3.R, led3.G, led3.B);
    } else{
      strip.neoPixelSetValue(2, 0, 0, 0);
    }

    if (led4.led_state)
    {
      strip.neoPixelSetValue(3, led4.R, led4.G, led4.B);
    } else{
      strip.neoPixelSetValue(3, 0, 0, 0);
    }

    if (led5.led_state)
    {
      strip.neoPixelSetValue(4, led5.R, led5.G, led5.B);
    } else{
      strip.neoPixelSetValue(4, 0, 0, 0);
    }
    strip.neoPixelShow();

 }


void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
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
    const unsigned long now = millis();
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
      } else if (fsm1.state == 1 && S1 && !prevS1){
        fsm1.new_state = 0;
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

      // Set led
      set_led(led1, LED_1, 30, 200, 0);
      set_led(led2, LED_2, 30, 200, 0);
      set_led(led3, LED_3, 30, 200, 0);
      set_led(led4, LED_4, 30, 200, 0);
      set_led(led5, LED_5, 30, 200, 0);

      // Set the outputs
      set_led_strip(led1, led2, led3, led4, led5);
      
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
      Serial.print(led1.led_state);

      Serial.print(" LED_2: ");
      Serial.print(led2.led_state);

      Serial.print(" LED_3: ");
      Serial.print(led3.led_state);

      Serial.print(" LED_4: ");
      Serial.print(led4.led_state);

      Serial.print(" LED_5: ");
      Serial.print(led5.led_state);

      Serial.print(" default_time: ");
      Serial.print(default_time);

      Serial.print(" blink_time: ");
      Serial.print(blink_time);

      
      Serial.print(" fsm2.tis: ");
      Serial.print(fsm2.tis);

      Serial.print(" fsm1.tes: ");
      Serial.print(fsm1.tes);

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


#include <Arduino.h>

#define MAXIMUM_NUM_NEOPIXELS 5
#include <NeoPixelConnect.h>

#define LED_PIN 6

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

// Define an enumeration named "Color" with additional color constants
enum Color {
    VIOLET_led,
    BLUE_led,
    CYAN_led,
    GREEN_led,
    YELLOW_led,
    ORANGE_led,
    WHITE_led,
    RED_led
};
Color led_color;

// Define an enumeration named "Color" with additional color constants
enum MODE {
    NORMAL,
    BLINK,
    FADE
};
MODE led_mode;

// Input variables
uint8_t S1, prevS1; //Sdown
uint8_t S2, prevS2; //Sup
uint8_t S3, prevS3; //Sgo

// Output variables
led_RGB led1, led2, led3, led4, led5;
uint8_t LED_1,LED_2,LED_3,LED_4,LED_5;

// Our finite state machines
fsm_t fsm1, fsm2, fsm3, fsm4, fsm5, fsm6, fsm7, fsm8, fsm9, fsm10, fsm11, fsm12, fsm13;

unsigned long interval, last_cycle, loop_micros;

uint16_t default_time, blink_time, aux;
uint configuration_mode, countLeds;
bool blink_mode ;

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

  if(led_state != -1){
    led.led_state = led_state;
  }

  if (red != -1)
  {
    led.R = red;
  }

  if (green != -1){
    led.G = green;
  }

  if(blue != -1){
    led.B = blue;
  }
}

// Set all leds with one color
void set_led_strip_color(led_RGB& led1, led_RGB& led2, led_RGB& led3, led_RGB& led4, led_RGB& led5, Color color){

  switch (color) {
        case VIOLET_led:
            set_led(led1, -1, 100, 0, 100);
            set_led(led2, -1, 100, 0, 100);
            set_led(led3, -1, 100, 0, 100);
            set_led(led4, -1, 100, 0, 100);
            set_led(led5, -1, 100, 0, 100);
            break;
        case BLUE_led:
            set_led(led1, -1, 0, 0, 255);
            set_led(led2, -1, 0, 0, 255);
            set_led(led3, -1, 0, 0, 255);
            set_led(led4, -1, 0, 0, 255);
            set_led(led5, -1, 0, 0, 255);
            break;
        case CYAN_led:
            set_led(led1, -1, 0, 255, 255);
            set_led(led2, -1, 0, 255, 255);
            set_led(led3, -1, 0, 255, 255);
            set_led(led4, -1, 0, 255, 255);
            set_led(led5, -1, 0, 255, 255);
            break;
        case GREEN_led:
            set_led(led1, -1, 0, 255, 0);
            set_led(led2, -1, 0, 255, 0);
            set_led(led3, -1, 0, 255, 0);
            set_led(led4, -1, 0, 255, 0);
            set_led(led5, -1, 0, 255, 0);
            break;
        case YELLOW_led:
            set_led(led1, -1, 255, 255, 0);
            set_led(led2, -1, 255, 255, 0);
            set_led(led3, -1, 255, 255, 0);
            set_led(led4, -1, 255, 255, 0);
            set_led(led5, -1, 255, 255, 0);
            break;
        case ORANGE_led:
            set_led(led1, -1, 255, 60, 0);
            set_led(led2, -1, 255, 60, 0);
            set_led(led3, -1, 255, 60, 0);
            set_led(led4, -1, 255, 60, 0);
            set_led(led5, -1, 255, 60, 0);
            break;
        case WHITE_led:
            set_led(led1, -1, 255, 255, 255);
            set_led(led2, -1, 255, 255, 255);
            set_led(led3, -1, 255, 255, 255);
            set_led(led4, -1, 255, 255, 255);
            set_led(led5, -1, 255, 255, 255);
            break;
        case RED_led:
            set_led(led1, -1, 255, 0, 0);
            set_led(led2, -1, 255, 0, 0);
            set_led(led3, -1, 255, 0, 0);
            set_led(led4, -1, 255, 0, 0);
            set_led(led5, -1, 255, 0, 0);
            break;
        default:
            // Handle the default case, or do nothing
            break;
    }

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
  blink_time = 500;
  aux = 0;

  configuration_mode = 0;
  blink_mode = false;
  led_mode = NORMAL;
  led_color = ORANGE_led;

  // Set leds to light blue
  set_led_strip_color(led1, led2, led3, led4, led5, led_color);

  interval = 40;
  set_state(fsm1,0);
  set_state(fsm2,0);
  countLeds = 5;
  set_state(fsm3,0);
  set_state(fsm4,0);
  set_state(fsm5,0);
  set_state(fsm6,0);
  set_state(fsm7,0);
  set_state(fsm8,0);
  set_state(fsm9,0);
  set_state(fsm10,0);
  set_state(fsm11,0);
  set_state(fsm12,0);
  set_state(fsm13,0);
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
      fsm4.tis = cur_time - fsm4.tes;
      fsm5.tis = cur_time - fsm5.tes;
      fsm6.tis = cur_time - fsm6.tes;
      fsm7.tis = cur_time - fsm7.tes;
      fsm8.tis = cur_time - fsm8.tes;
      fsm9.tis = cur_time - fsm9.tes;
      fsm10.tis = cur_time - fsm10.tes;
      fsm11.tis = cur_time - fsm11.tes;
      fsm12.tis = cur_time - fsm12.tes;
      fsm13.tis = cur_time - fsm13.tes;

      // Calculate next state for the first state machine
      if ((fsm1.state == 0 && S1 && !prevS1)){
        fsm1.new_state = 1;
        fsm2.tes = cur_time - aux;
      } else if ((fsm1.state == 1 && S1 && !prevS1)){
        fsm1.new_state = 0;
        aux = fsm2.tis;
      } else if ((fsm1.state == 1 || fsm1.state == 0) && configuration_mode != 0){
        fsm1.new_state = 2;
        aux = 0;
      } else if (fsm1.state == 2 && configuration_mode == 0){
        fsm1.new_state = 0;
      }
      

      // Calculate next state for the second state machine
      if (fsm2.state == 0 && fsm1.state == 1 && fsm2.tis > default_time && countLeds != 0){        
        fsm2.tes = cur_time;
        fsm2.new_state = 0;
        countLeds--;
      } else if (fsm2.state == 0 && fsm2.tis > default_time && fsm1.state == 1 && countLeds == 0){ 
        fsm2.new_state = 1;
        blink_mode = true;
      } else if (fsm2.state == 1 && fsm2.tis > blink_time && fsm1.state == 1 ){ 
        fsm2.new_state = 2;
      } else if (fsm2.state == 2 && fsm2.tis > blink_time && fsm1.state == 1 ){ 
        fsm2.new_state = 1;
      } else if ((fsm2.state == 2 || fsm2.state == 1 || fsm2.state == 0) && S3 && !prevS3){
        fsm2.new_state = 0;
        countLeds = 5;
        blink_mode = false;
      }

      // Calculate next state for the third state machine
      if (fsm3.state == 0 && S2 && !prevS2 ){
        fsm3.new_state = 0;
        if(countLeds < 5 && !blink_mode && !configuration_mode){
        countLeds++;
        }
      } 

      //Calculate new state for the Configuration Mode (CMSM) state machine
      if (fsm4.state == 0 && S2 ){
        fsm4.new_state = 1;
      } else if(fsm4.state == 1 && fsm4.tis > 3000 && S2){
        fsm4.new_state = 2;
        configuration_mode = 1;
      } else if (fsm4.state == 2 && S2 && !prevS2){
        fsm4.new_state = 3;
        configuration_mode = 2;
      } else if (fsm4.state == 3 && S2 && !prevS2){
        fsm4.new_state = 4;
        configuration_mode = 3;
      } else if ((fsm4.state == 2 || fsm4.state == 3 || fsm4.state == 4) && fsm4.tis > 3000 && S2){
        fsm4.new_state = 0;
        configuration_mode = 0;
      } else if(!prevS2){
        fsm4.tes = cur_time;
      }

      //Calculate new state for the Configuration Mode 1 (CM1SM) state machine
      if (fsm5.state == 0 && configuration_mode == 1){
        fsm5.new_state = 1;
        default_time = 1000;
      } else if(fsm5.state == 1 && S1 && !prevS1){
        fsm5.new_state = 2;
        default_time = 2000;
      } else if(fsm5.state == 2 && S1 && !prevS1){
        fsm5.new_state = 3;
        default_time = 5000;
      } else if(fsm5.state == 3 && S1 && !prevS1){
        fsm5.new_state = 4;
        default_time = 10000;
      } else if(fsm5.state == 4 && S1 && !prevS1){
        fsm5.new_state = 1;
        default_time = 1000;
      } else if ((fsm5.state == 1 || fsm5.state == 2 || fsm5.state == 3 || fsm5.state == 4 || fsm5.state == 5) && configuration_mode != 1){ 
        fsm5.new_state = 0;
      }
      
      //Calculate new state for the Configuration Mode 1 lED 1 (CM1L1SM) state machine
      if (fsm6.state == 0 && configuration_mode == 1){
        fsm6.new_state = 1;
      } else if(fsm6.state == 1 && fsm6.tis > 500){
        fsm6.new_state = 2;
      } else if(fsm6.state == 2 && fsm6.tis > 500){
        fsm6.new_state = 1;
      } else if((fsm6.state == 1 || fsm6.state == 2) && configuration_mode != 1){
        fsm6.new_state = 0;  
      }

      //Calculate new state for the Configuration Mode 1 lED 5 (CM1L5SM) state machine
      if (fsm7.state == 0 && configuration_mode == 1){
        fsm7.new_state = 1;
      } else if(fsm7.state == 1 && fsm7.tis > default_time){
        fsm7.new_state = 2;
      } else if(fsm7.state == 2 && fsm7.tis > default_time){
        fsm7.new_state = 1;
      } else if((fsm7.state == 1 || fsm7.state == 2) && configuration_mode != 1){
        fsm7.new_state = 0;  
      }

      //Calculate new state for the Configuration Mode 2 (CM2SM) state machine
      if (fsm8.state == 0 && configuration_mode == 2){
        fsm8.new_state = 1;
        led_mode = NORMAL;
      } else if(fsm8.state == 1 && S1 && !prevS1){
        fsm8.new_state = 2;
        led_mode = BLINK;
      } else if(fsm8.state == 2 && S1 && !prevS1){
        fsm8.new_state = 3;
        led_mode = FADE;
      } else if(fsm8.state == 3 && S1 && !prevS1){
        fsm8.new_state = 1;
        led_mode = NORMAL;
      } else if((fsm8.state == 1 || fsm8.state == 2 || fsm8.state == 3 ) && configuration_mode != 2){
        fsm8.new_state = 0;
      }

      //Calculate new state for the Configuration Mode 2 lED 2 (CM2L2SM) state machine
      if (fsm9.state == 0 && configuration_mode == 2){
        fsm9.new_state = 1;
      } else if(fsm9.state == 1 && fsm9.tis > 500){
        fsm9.new_state = 2;
      } else if(fsm9.state == 2 && fsm9.tis > 500){
        fsm9.new_state = 1;
      } else if((fsm9.state == 1 || fsm9.state == 2) && configuration_mode != 2){
        fsm9.new_state = 0;  
      }

      //Calculate new state for the Configuration Mode 2 lED 5 (CM2L5SM) state machine
      //fsm10


      //Calculate new state for the Configuration Mode 3 (CM3SM) state machine
      if (fsm11.state == 0 && configuration_mode == 3){
        fsm11.new_state = 1;
        led_color = VIOLET_led;
      } else if(fsm11.state == 1 && S1 && !prevS1){
        fsm11.new_state = 2;
        led_color = BLUE_led;
      } else if(fsm11.state == 2 && S1 && !prevS1){
        fsm11.new_state = 3;
        led_color = CYAN_led;
      } else if(fsm11.state == 3 && S1 && !prevS1){
        fsm11.new_state = 4;
        led_color = GREEN_led;
      } else if(fsm11.state == 4 && S1 && !prevS1){
        fsm11.new_state = 5;
        led_color = YELLOW_led;
      } else if(fsm11.state == 5 && S1 && !prevS1){
        fsm11.new_state = 6;
        led_color = ORANGE_led;
      } else if(fsm11.state == 6 && S1 && !prevS1){
        fsm11.new_state = 7;
        led_color = WHITE_led;
      } else if(fsm11.state == 7 && S1 && !prevS1){
        fsm11.new_state = 1;
        led_color = VIOLET_led;
      } else if((fsm11.state == 1 || fsm11.state == 2 || fsm11.state == 3 || fsm11.state == 4 || fsm11.state == 5 || fsm11.state == 6 || fsm11.state == 7) && configuration_mode != 3){
        fsm11.new_state = 0;
      }

      //Calculate new state for the Configuration Mode 3  lED 3 (CM2L2SM) state machine
      if (fsm12.state == 0 && configuration_mode == 3){
        fsm12.new_state = 1;
      } else if(fsm12.state == 1 && fsm12.tis > 500){
        fsm12.new_state = 2;
      } else if(fsm12.state == 2 && fsm12.tis > 500){
        fsm12.new_state = 1;
      } else if((fsm12.state == 1 || fsm12.state == 2) && configuration_mode != 3){
        fsm12.new_state = 0;  
      }

      //Calculate new state for the Configuration Mode 3 lED 5 (CM2L5SM) state machine
      if (fsm13.state == 0 && configuration_mode == 3){
        fsm13.new_state = 1;
      } else if(fsm13.state == 1 && configuration_mode != 3){
        fsm13.new_state = 0;  
      }

      // Update the states
      set_state(fsm1, fsm1.new_state);
      set_state(fsm2, fsm2.new_state);
      set_state(fsm3, fsm3.new_state);
      set_state(fsm4, fsm4.new_state);
      set_state(fsm5, fsm5.new_state);
      set_state(fsm6, fsm6.new_state);
      set_state(fsm7, fsm7.new_state);
      set_state(fsm8, fsm8.new_state);
      set_state(fsm9, fsm9.new_state);
      set_state(fsm10, fsm10.new_state);
      set_state(fsm11, fsm11.new_state);
      set_state(fsm12, fsm12.new_state);
      set_state(fsm13, fsm13.new_state);

      // Actions set by the current state of the second state machine
      LED_1 = ((countLeds >= 1 || fsm2.state == 1) && !configuration_mode) || (fsm6.state == 1);
      LED_2 = ((countLeds >= 2 || fsm2.state == 1) && !configuration_mode) || (fsm9.state == 1);
      LED_3 = ((countLeds >= 3 || fsm2.state == 1) && !configuration_mode) || (fsm12.state == 1);
      LED_4 = ((countLeds >= 4 || fsm2.state == 1) && !configuration_mode);
      LED_5 = ((countLeds >= 5 || fsm2.state == 1) && !configuration_mode) || (fsm7.state == 1) || (fsm13.state == 1) ;;
      
      // Set led
      set_led(led1, LED_1, -1, -1, -1);
      set_led(led2, LED_2, -1, -1, -1);
      set_led(led3, LED_3, -1, -1, -1);
      set_led(led4, LED_4, -1, -1, -1);
      set_led(led5, LED_5, -1, -1, -1);

      // Set the outputs
      
      if (configuration_mode){
        // manage leds color
        set_led_strip_color(led1, led2, led3, led4, led5, led_color);
        // set leds to 1, 2 and 3 to RED for better 
        set_led(led1, -1, 255, 0, 0);
        set_led(led2, -1, 255, 0, 0);
        set_led(led3, -1, 255, 0, 0);
      } else if (blink_mode){
        set_led_strip_color(led1, led2, led3, led4, led5, RED_led);
      } else{
        // manage led 3 modes
        switch (led_mode)
        {
        case NORMAL:
          set_led_strip_color(led1, led2, led3, led4, led5, led_color);
          break;
        case BLINK:
          set_led_strip_color(led1, led2, led3, led4, led5, led_color);
          break;
        case FADE:
          set_led_strip_color(led1, led2, led3, led4, led5, led_color);
          break;
        default:
          break;
        }
        set_led_strip_color(led1, led2, led3, led4, led5, led_color);

      }

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

      Serial.print(" CMST: ");
      Serial.print(fsm4.state);

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

      Serial.print(" led_mode: ");
      Serial.print(led_mode);

      Serial.print(" led_color: ");
      Serial.print(led_color);

      /*
      Serial.print(" blink_time: ");
      Serial.print(blink_time);
      
      Serial.print(" fsm2.tis: ");
      Serial.print(fsm2.tis);

      Serial.print(" fsm3.tis: ");
      Serial.print(fsm3.tis);

      Serial.print(" aux: ");
      Serial.print(aux);
    */
   
      Serial.print(" configuration mode: ");
      Serial.print(configuration_mode);

      Serial.print(" cOUNTleDS: ");
      Serial.print(countLeds);

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


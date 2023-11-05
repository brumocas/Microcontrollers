#include <Arduino.h>

#define MAXIMUM_NUM_NEOPIXELS 5
#include <NeoPixelConnect.h>

// Led Strip input pin
#define LED_PIN 6

NeoPixelConnect strip(LED_PIN, MAXIMUM_NUM_NEOPIXELS, pio0, 0);

// Input buttons pins
#define S1_pin 2 // Sdown
#define S2_pin 3 // Sup
#define S3_pin 4 // Sgo

typedef struct {
  int state, new_state;
  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// RGB struct for each LED state
typedef struct { 
  // led logic state
  uint8_t led_state;
  // led RGB values
  float R,G,B;
  // led RGB values for fade control, saved first RGB values when starting fade for the next iterations 
  float RF, GF, BF;
} led_RGB;

// Enum of the possible colors of the LED strip
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

// Enum of the possible LED modes
enum MODE {
    NORMAL,
    BLINK, 
    FADE,
};
MODE led_mode;

// Input variables
uint8_t S1, prevS1; //Sdown
uint8_t S2, prevS2; //Sup
uint8_t S3, prevS3; //Sgo

// Output variables
// Struct for each led to control
led_RGB led1, led2, led3, led4, led5;

// Our finite state machines
fsm_t fsm1, fsm2, fsm3, fsm4, fsm5, fsm6, fsm7, fsm8, fsm9, fsm10, fsm11, fsm12, fsm13;
// auxiliar time variables
unsigned long interval, last_cycle, loop_micros, default_time, blink_time, aux, blink_aux, idle_aux;

// auxiliar variables
// congiguration_mode --> indicates the LED current mode
// leds_counts --> indicates number of lit LEDs outside the configuration mode
// step --> indicates the number of subtractions when the FADE mode is on
// blink_mode --> indicates if we are in the RED blink mode
// up --> used in idle mode to count up and down
uint8_t configuration_mode, leds_count, pos;
float step;
bool blink_mode, idle_mode, up;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

// Function to set the LED RGB struct
// When the parameter passed == -1 the LED parameter is not changed
void set_led( led_RGB& led, int led_state, float red, float green, float blue){
  // Set LED logical state
  if(led_state != -1){
    led.led_state = led_state;
  }
  // Set LED RED value
  if (red != -1)
  { 
    led.R = red;
  }
  // Set LED GREEN value
  if (green != -1 ){
    led.G = green;
  }
  // Set LED BLUE value
  if(blue != -1 ){
    led.B = blue;
  }
}

// Function to set all LEDs strip to the same color
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
            set_led(led1, -1, 255, 40, 0);
            set_led(led2, -1, 255, 40, 0);
            set_led(led3, -1, 255, 40, 0);
            set_led(led4, -1, 255, 40, 0);
            set_led(led5, -1, 255, 40, 0);
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
            // Do nothing
            break;
    }

}

// Funtion to lit the LED strip
 void set_led_strip(led_RGB& led1, led_RGB& led2, led_RGB& led3, led_RGB& led4, led_RGB& led5){

    // Set led1 output   
    if (led1.led_state)
    {
      strip.neoPixelSetValue(0, led1.R, led1.G, led1.B);
    } else{
      strip.neoPixelSetValue(0, 0, 0, 0);
    }
    // Set led2 output
    if (led2.led_state)
    {
      strip.neoPixelSetValue(1, led2.R, led2.G, led2.B);
    } else{
      strip.neoPixelSetValue(1, 0, 0, 0);
    }
    // Set led3 output
    if (led3.led_state)
    {
      strip.neoPixelSetValue(2, led3.R, led3.G, led3.B);
    } else{
      strip.neoPixelSetValue(2, 0, 0, 0);
    }
    // Set led4 output 
    if (led4.led_state)
    {
      strip.neoPixelSetValue(3, led4.R, led4.G, led4.B);
    } else{
      strip.neoPixelSetValue(3, 0, 0, 0);
    }
    // Set led5 output 
    if (led5.led_state)
    {
      strip.neoPixelSetValue(4, led5.R, led5.G, led5.B);
    } else{
      strip.neoPixelSetValue(4, 0, 0, 0);
    }
    // Show the leds result
    strip.neoPixelShow();

 }

// Function to save the first RGB value for the FADE mode for a given LED
void setFade(led_RGB& led){
  led.RF = led.R;
  led.GF = led.G;
  led.BF = led.B;
}

// Function to save the first RGB value for the FADE mode for all LEDs
void setAllFade(led_RGB& led1, led_RGB& led2, led_RGB& led3, led_RGB& led4, led_RGB& led5){
  setFade(led1);
  setFade(led2);
  setFade(led3);
  setFade(led4);
  setFade(led5);
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
  // Blink time in RED mode
  blink_time = 500;
  // aux variable when in paused, used to save fsm2.tis
  aux = 0;
  // aux variable when in BLINK LED mode to check if fsm2.tis > default_time/2
  blink_aux = 0;
  // aux variable to check idle mode
  idle_aux = 0;
  // variable to save current LED mode, if configuration_mode == 0 ---> out of configuration mode
  configuration_mode = 0;
  // variable to save if we are in LED BLINK RED mode
  blink_mode = false;
  // variable to save if we are in LED IDLE mode
  idle_mode = false;
  // For idle mode to save the color enum position
  pos = 0;

  // Starting LED mode
  led_mode = BLINK;
  // Starting LED color
  led_color = ORANGE_led;
  // Set leds to starting color 
  set_led_strip_color(led1, led2, led3, led4, led5, led_color);

  interval = 40;
  set_state(fsm1,0);
  set_state(fsm2,0);
  leds_count = 5;
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

      // Calculate next state for the CONTROL STATE MACHINE (CSM)     
      if ((fsm1.state == 0 && S1 && !prevS1)){
        fsm1.new_state = 1;
        fsm2.tes = cur_time - aux;
        idle_aux = cur_time;
      } else if ((fsm1.state == 1 && S1 && !prevS1)){
        fsm1.new_state = 0;
        aux = fsm2.tis;
      } else if ((fsm1.state == 1 || fsm1.state == 0) && configuration_mode != 0){
        fsm1.new_state = 2;
        aux = 0;
      } else if (fsm1.state == 2 && configuration_mode == 0){
        fsm1.new_state = 0;
      }

      // Calculate next state for the LED STATE MACHINE (LSM)
      if (fsm2.state == 0 && fsm1.state == 1 && fsm2.tis > default_time && leds_count != 0){        
        fsm2.tes = cur_time;
        fsm2.new_state = 0;
        leds_count--;

        if(led_mode == FADE){
          set_led_strip_color(led1, led2, led3, led4, led5, led_color);
          setAllFade(led1, led2, led3, led4, led5);
        }

      } else if (fsm2.state == 0 && fsm2.tis > default_time/2 && fsm1.state == 1 && leds_count != 0 && led_mode == BLINK){ 
        fsm2.new_state = 3;
        blink_aux = cur_time;
        leds_count--;
      } else if (fsm2.state == 3 && fsm2.tis > 50 && fsm1.state == 1 ){ 
        fsm2.new_state = 4;
        leds_count++;
      } else if (fsm2.state == 4 && fsm2.tis > 50 && fsm1.state == 1 ){ 
        fsm2.new_state = 3;
        leds_count--;
      } else if (fsm2.state == 4 && cur_time - blink_aux > default_time/2 && fsm1.state == 1 ){ 
        fsm2.new_state = 0;
        leds_count--;
      } else if (fsm2.state == 0 && fsm2.tis > default_time && fsm1.state == 1 && leds_count == 0){ 
        fsm2.new_state = 1;
        blink_mode = true;
        idle_aux = cur_time;
      } else if (fsm2.state == 1 && fsm2.tis > blink_time && fsm1.state == 1 ){ 
        fsm2.new_state = 2;
      } else if (fsm2.state == 2 && fsm2.tis > blink_time && fsm1.state == 1 ){ 
        fsm2.new_state = 1;
      } else if ((fsm2.state == 2 || fsm2.state == 1 || fsm2.state == 0|| fsm2.state == 3 || fsm2.state == 4 ) && ((S3 && !prevS3) || fsm1.state == 2)){
        fsm2.new_state = 0;
        leds_count = 6;
        blink_mode = false;
        idle_mode = false;
        if(configuration_mode == 0)
        set_led_strip_color(led1, led2, led3, led4, led5, led_color);
      } else if ((fsm2.state == 1 || fsm2.state == 2 ) && cur_time - idle_aux > 10000 && fsm1.state == 1){
        fsm2.new_state = 5;
        blink_mode = false;
        idle_mode = true;
        up = true;
        pos = 0;
        leds_count = 0;
        idle_aux = 0;
      } else if (fsm2.state == 5 && fsm2.tis > 500 ){
        fsm2.new_state = 6;
        if(up == true){
          leds_count++;
          if (leds_count >= 5)
          up = false;
        } else{
          leds_count--;
          if (leds_count <= 0){
            up = true;
            pos = pos + 1;
          }
        }
        if (pos == 6)
        pos = 0;

      } else if (fsm2.state == 6 && fsm2.tis > 500 ){
        fsm2.new_state = 5;
      }else if ((fsm2.state == 5 || fsm2.state == 6 ) && ((S3 && !prevS3) || (S2 && !prevS2) || (S1 && !prevS1) || fsm1.state == 2)){
        fsm2.new_state = 0;
        idle_mode = false;
        blink_aux = 0;
        leds_count = 6;
        if(configuration_mode == 0)
        set_led_strip_color(led1, led2, led3, led4, led5, led_color);
      }
      
      // Calculate next state for the INCREASE LED STATE MACHINE (ILSTM)
      if (fsm3.state == 0 && S2 && !prevS2 ){
        fsm3.new_state = 0;
        if(leds_count < 5 && !blink_mode && !configuration_mode){
        leds_count++;
        }
      } 

      //Calculate new state for the CONFIGURATION MODE STATE MACHINE (CMSM) 
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
        set_led_strip_color(led1, led2, led3, led4, led5, led_color);
      } else if(!prevS2){
        fsm4.tes = cur_time;
        
      }

      //Calculate new state for the CONFIGURATION MODE 1 STATE MACHINE (CM1SM)
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
      } else if ((fsm5.state == 1 || fsm5.state == 2 || fsm5.state == 3 || fsm5.state == 4) && configuration_mode != 1){ 
        fsm5.new_state = 0;
      }
      
      //Calculate new state for the CONFIGURATION MODE 1 LED 1 STATE MACHINE (CM1L1SM) 
      if (fsm6.state == 0 && configuration_mode == 1){
        fsm6.new_state = 1;
      } else if(fsm6.state == 1 && fsm6.tis > 500){
        fsm6.new_state = 2;
      } else if(fsm6.state == 2 && fsm6.tis > 500){
        fsm6.new_state = 1;
      } else if((fsm6.state == 1 || fsm6.state == 2) && configuration_mode != 1){
        fsm6.new_state = 0;  
      }

      //Calculate new state for the CONFIGURATION MODE 1 STATE MACHINE LED 5 STATE MACHINE (CM1L5SM) 
      if (fsm7.state == 0 && configuration_mode == 1){
        fsm7.new_state = 1;
      } else if(fsm7.state == 1 && fsm7.tis > default_time){
        fsm7.new_state = 2;
      } else if(fsm7.state == 2 && fsm7.tis > default_time){
        fsm7.new_state = 1;
      } else if((fsm7.state == 1 || fsm7.state == 2) && configuration_mode != 1){
        fsm7.new_state = 0;  
      }

      //Calculate new state for the CONFIGURATION MODE 2 STATE MACHINE (CM2SM) 
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

      //Calculate new state for the CONFIGURATION MODE 2 STATE MACHINE LED 2 STATE MACHINE (CM2L2SM) 
      if (fsm9.state == 0 && configuration_mode == 2){
        fsm9.new_state = 1;
      } else if(fsm9.state == 1 && fsm9.tis > 500){
        fsm9.new_state = 2;
      } else if(fsm9.state == 2 && fsm9.tis > 500){
        fsm9.new_state = 1;
      } else if((fsm9.state == 1 || fsm9.state == 2) && configuration_mode != 2){
        fsm9.new_state = 0;  
      }

      //Calculate new state for the CONFIGURATION MODE 2 STATE MACHINE LED 5 STATE MACHINE (CM2L5SM) 
      //fsm10
      if (fsm10.state == 0 && configuration_mode == 2){
        fsm10.new_state = 1;
      } else if(fsm10.state == 1 && (led_mode == NORMAL)){
        fsm10.new_state = 2;
      }else if(fsm10.state == 2 && fsm10.tis > default_time){
        fsm10.new_state = 3; 
      } else if(fsm10.state == 3 && fsm10.tis > default_time){
        fsm10.new_state = 2;
      } else if(fsm10.state == 1 && (led_mode == BLINK)){
        fsm10.new_state = 4;
      } else if(fsm10.state == 4 && fsm10.tis > default_time/2){
        blink_aux = cur_time;
        fsm10.new_state = 5;
      } else if(fsm10.state == 5 && fsm10.tis > 50){
        fsm10.new_state = 6;
      } else if(fsm10.state == 6 && fsm10.tis > 50){
        fsm10.new_state = 5;
      } else if((fsm10.state == 5 || fsm10.state == 6) && (cur_time - blink_aux > default_time/2)){
        fsm10.new_state = 7;
      } else if(fsm10.state == 7  && fsm10.tis > default_time){
        fsm10.new_state = 4;
      } else if(fsm10.state == 1 && (led_mode == FADE)){
        fsm10.new_state = 8;
        set_led_strip_color(led1, led2, led3, led4, led5, led_color);
        setFade(led5);
      } else if(fsm10.state == 8 && fsm10.tis > default_time){
        fsm10.new_state = 9;
      } else if(fsm10.state == 9 && fsm10.tis > default_time){
        fsm10.new_state = 8;
        set_led_strip_color(led1, led2, led3, led4, led5, led_color);
        setFade(led5);
      } else if((fsm10.state == 1 || fsm10.state == 2 || fsm10.state == 3 || fsm10.state == 4 || fsm10.state == 5 || fsm10.state == 6 || fsm10.state == 7 || fsm10.state == 8 || fsm10.state == 9) && configuration_mode != 2){
        fsm10.new_state = 0;  
      } else if((fsm10.state == 1 || fsm10.state == 2 || fsm10.state == 3 || fsm10.state == 4 || fsm10.state == 5 || fsm10.state == 6 || fsm10.state == 7 || fsm10.state == 8 || fsm10.state == 9) && S1 && prevS1){
        fsm10.new_state = 1;  
      }

      
      //Calculate new state for the CONFIGURATION MODE 3 STATE MACHINE (CM3SM) 
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

      //Calculate new state for the CONFIGURATION MODE 3 STATE MACHINE LED 3 STATE MACHINE (CM3L3SM) 
      if (fsm12.state == 0 && configuration_mode == 3){
        fsm12.new_state = 1;
      } else if(fsm12.state == 1 && fsm12.tis > 500){
        fsm12.new_state = 2;
      } else if(fsm12.state == 2 && fsm12.tis > 500){
        fsm12.new_state = 1;
      } else if((fsm12.state == 1 || fsm12.state == 2) && configuration_mode != 3){
        fsm12.new_state = 0;  
      }

      //Calculate new state for the CONFIGURATION MODE 3 STATE MACHINE LED 5 STATE MACHINE (CM3L5SM) 
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

      // Actions 
      led1.led_state = ((leds_count >= 1 || fsm2.state == 1) && !configuration_mode) || (fsm6.state == 1);
      led2.led_state = ((leds_count >= 2 || fsm2.state == 1) && !configuration_mode) || (fsm9.state == 1);
      led3.led_state = ((leds_count >= 3 || fsm2.state == 1) && !configuration_mode) || (fsm12.state == 1);
      led4.led_state = ((leds_count >= 4 || fsm2.state == 1) && !configuration_mode);
      led5.led_state = ((leds_count >= 5 || fsm2.state == 1) && !configuration_mode) || (fsm7.state == 1) || (fsm13.state == 1) || (fsm10.state == 3) || (fsm10.state == 5) || (fsm10.state == 4) || (fsm10.state == 8);

      // Set LED color values
      if (configuration_mode){
        
        // Manage LED in configuration mode
        if (configuration_mode == 2 && led_mode == FADE){
          step = (default_time/interval);
          set_led(led5, -1, led5.R - (led5.RF/step), led5.G - (led5.GF/step), led5.B - (led5.BF/step));                                                                                               
          set_led(led2, -1, 255, 0, 0);              
        } else{
          set_led_strip_color(led1, led2, led3, led4, led5, led_color);
          // set LEDs to 1, 2 and 3 to RED for better understanding
          set_led(led1, -1, 255, 0, 0);
          set_led(led2, -1, 255, 0, 0);
          set_led(led3, -1, 255, 0, 0);
        }

      } else if (blink_mode){
        // set LEDs to RED in blink mode
        set_led_strip_color(led1, led2, led3, led4, led5, RED_led);
      }  else if (idle_mode){
        // set LEDs with the given enum color position
        set_led_strip_color(led1, led2, led3, led4, led5, (Color)pos);

      } else{
        // Manage LED normal mode
        if(led_mode == FADE && fsm1.state == 1){
        step = (default_time/interval);
        if(leds_count == 1)
          set_led(led1, -1, led1.R - (led1.RF/step), led1.G - (led1.GF/step), led1.B - (led1.BF/step));       
        if(leds_count == 2)
          set_led(led2, -1, led2.R - (led2.RF/step), led2.G - (led2.GF/step), led2.B - (led2.BF/step));     
        if(leds_count == 3)    
          set_led(led3, -1, led3.R - (led3.RF/step), led3.G - (led3.GF/step), led3.B - (led3.BF/step));        
        if(leds_count == 4)
          set_led(led4, -1, led4.R - (led4.RF/step), led4.G - (led4.GF/step), led4.B - (led4.BF/step));         
        if(leds_count == 5)
          set_led(led5, -1, led5.R - (led5.RF/step), led5.G - (led5.GF/step), led5.B - (led5.BF/step));            
        }else if(led_mode == FADE){
          // Hold fade mode when in paused
        }else {
          // MORMAL mode or BLINK mode
          set_led_strip_color(led1, led2, led3, led4, led5, led_color);         
        } 
      }

      // lit LED strip
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

      Serial.print(" configuration mode: ");
      Serial.print(configuration_mode);

      Serial.print(" ledscount: ");
      Serial.print(leds_count);

      Serial.print(" iDLE timer: ");
      Serial.print(cur_time - idle_aux);

      /*
      Serial.print(" cur : ");
      Serial.print(cur_time);
      */

      Serial.println();
      
    }
    
}


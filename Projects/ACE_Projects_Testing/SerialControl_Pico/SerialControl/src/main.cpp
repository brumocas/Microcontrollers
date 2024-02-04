// 2022 Paulo Costa
// Using PWM for LED intensity and serial commands example

#include <Arduino.h>
#include <EEPROM.h>
#include "RP2040_PWM.h"

// pin 25 is the built in LED for the Pi Pico (NOT for the Pi Pico W)
#define PWM0_pin 25

//creates pwm instance
RP2040_PWM* PWM_Instance;
float LED_frequency;
uint16_t LED_intensity;

unsigned long interval, last_cycle;
unsigned long loop_micros;

#include "commands.h"

commands_t serial_commands;



void process_command(char command, float value)
{
  if (command == 'L') {  // The 'L' command sets the LED_intensity to the value that follows
    LED_intensity = value;
    if (LED_intensity > 100) LED_intensity = 100;  // Avoid "impossible" values for LED_intensity

  } else if (command == 'o') { // The 'o' command sets the LED_intensity to zero
    LED_intensity = 0;

  } // Put here more commands...
}


void setup() 
{
  serial_commands.init(process_command);

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);


  //assigns pin 25 (built in LED), with frequency of [LED_frequency] Hz and a duty cycle of [LED_intensity]%
  LED_frequency = 20000;
  PWM_Instance = new RP2040_PWM(PWM0_pin, LED_frequency, LED_intensity);

  interval = 10;
}

void loop() 
{
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
      
      b = Serial.read();    

      if (b == '-')  { // Press '-' to decrease the LED intensity
        LED_intensity -= 10; 
      
      } else if (b == '+') {  // Press '+' to increase the LED intensity
        LED_intensity += 10;
      
      } else serial_commands.process_char(b);

      if (LED_intensity > 100) LED_intensity = 100;  // Avoid "impossible" values for LED_intensity
    }  


    // To measure the time between loop() calls
    //unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      
      // Update the LED intensity
      PWM_Instance->setPWM(PWM0_pin, LED_frequency, LED_intensity);

      // Debug using the serial port
      Serial.print(" state: ");
      Serial.print(serial_commands.state);

      Serial.print(" count: ");
      Serial.print(serial_commands.count);

      Serial.print(" Command: ");
      Serial.print(serial_commands.command);

      Serial.print(" LED_intensity: ");
      Serial.print(LED_intensity);

      Serial.print(" loop: ");
      Serial.println(micros() - loop_micros);
    }
    
}


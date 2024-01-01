// 2022 Paulo Costa
// RGB sensor TCS34725 and serial commands example

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>

unsigned long interval, last_cycle;
unsigned long loop_micros;

#include "commands.h"

commands_t serial_commands;
int show_lux;

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
//Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);


// tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
// We don't need to wait for the next integration cycle because we wait "interval" ms before requesting another read
// (make sure that interval > Integration_Time)
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}


void process_command(char command, float value)
{
  if (command == 'L') {  // The 'L' command controls if we calculate the Lux value
    
    show_lux = value;    // L1<enter> to turn on Lux calculation, L0<enter> to switch off

  } else if (command == 'o') { // The 'o' command ...

  } // Put here more commands...
}


void setup() 
{
  serial_commands.init(process_command);

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);
                    
                   // Connect TCS34725 Vin to 3.3
  Wire.setSDA(8);  // Connect TCS34725 SDA to gpio 8
  Wire.setSCL(9);  // Connect TCS34725 SCL to gpio 9

  Wire.begin();
  
  while (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }

  Serial.println("Found sensor");

  interval = 50;
}

void loop() 
{
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
   
      b = Serial.read();    
      serial_commands.process_char(b);
    }  


    // To measure the time between loop() calls
    //unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      
      uint16_t r, g, b, c, colorTemp, lux;

      //tcs.getRawData(&r, &g, &b, &c);
      getRawData_noDelay(&r, &g, &b, &c);
      // colorTemp = tcs.calculateColorTemperature(r, g, b);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      if (show_lux) lux = tcs.calculateLux(r, g, b);

      Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
      if (show_lux) Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
      Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
      Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
      Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
      Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");

      // Debug using the serial port
      //Serial.print(" state: ");
      //Serial.print(serial_commands.state);

      //Serial.print(" count: ");
      //Serial.print(serial_commands.count);

      Serial.print(" Command: ");
      Serial.print(serial_commands.command);

      Serial.print(" loop: ");
      Serial.println(micros() - loop_micros);
    }
    
}


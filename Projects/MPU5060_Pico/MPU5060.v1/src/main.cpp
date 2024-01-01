// 2022 Paulo Costa
// Raw IMU (MP5060) reading and conversion to SI, libray and example

#include <Arduino.h>

unsigned long interval, last_cycle;
unsigned long loop_micros;

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// Variables for raw readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

float acc_gain, w_gain;
float acc_gain_table[4] = {2, 4, 8, 16};         // units -> g
float w_gain_table[4] = {250, 500, 1000, 2000};  // units ->  degrees/sec


void setup() 
{
  // Start the serial port with 115200 baudrate
  Serial.begin(115200);

  Wire.setSDA(8); 
  Wire.setSCL(9);
  Wire.begin();      

  // initialize device
  while(1) {
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    if (accelgyro.testConnection()) {
      Serial.println("MPU6050 connection successful");    
      acc_gain = 9.8 * acc_gain_table[accelgyro.getFullScaleAccelRange()] / 32768.0;
      w_gain = acc_gain_table[accelgyro.getFullScaleGyroRange()] / 32768.0;
      break;

    } else {
      Serial.println("MPU6050 connection failed");              
      delay(250);

    }
  }

  interval = 100;
}

void loop() 
{
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
   
      b = Serial.read();    
      // Do stuf with the received byte
      // ...
    }  


    // To measure the time between loop() calls
    //unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      

      // read raw accel/gyro measurements from device
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      // these methods (and a few others) are also available
      //accelgyro.getAcceleration(&ax, &ay, &az);
      //accelgyro.getRotation(&gx, &gy, &gz);

      // display tab-separated accel/gyro x/y/z values
      Serial.print("a:\t");
      Serial.print(ax * acc_gain); Serial.print("\t");
      Serial.print(ay * acc_gain); Serial.print("\t");
      Serial.print(az * acc_gain); Serial.print("\t");
      Serial.print("w:\t");
      Serial.print(gx * w_gain); Serial.print("\t");
      Serial.print(gy * w_gain); Serial.print("\t");
      Serial.print(gz * w_gain); Serial.print("\t");
         
      Serial.print(" loop: ");
      Serial.println(micros() - loop_micros);
    }
    
}


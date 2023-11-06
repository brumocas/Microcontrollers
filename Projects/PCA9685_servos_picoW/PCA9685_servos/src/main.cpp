// 2022 Paulo Costa, based on the servo example from the Adafruit_PWMServoDriver library
// PCA9685 Servo driver and serial commands example

#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>

unsigned long interval, last_cycle;
unsigned long loop_micros;

#include "commands.h"

commands_t serial_commands;
int show_lux;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40, Wire);

#define NUM_SERVOS 16

int servo_pos[NUM_SERVOS];
int servo_idx;

void process_command(char command, float value)
{
  if (command == 's') {  // The 's' command Selects the current Servo
    
    servo_idx = value;
    if (servo_idx < 0) servo_idx = 0;
    if (servo_idx > NUM_SERVOS - 1) servo_idx = NUM_SERVOS - 1;

  } else if (command == 'p') { // The 'p' command sets current Servo pulse
                               // Ton = value/4096 * 20 ms
    if (value < 0) value = 0;
    if (value > 4095) value = 4095;  
    servo_pos[servo_idx] = value;  

  } // Put here more commands...
}

int find_ServoDriver(int addr)
{

  // test PCA9685 presence
  // write8(PCA9685_MODE1, MODE1_RESTART);

  Wire.beginTransmission(addr);
  Wire.write(PCA9685_MODE1);
  Wire.write(MODE1_RESTART);

  int err = Wire.endTransmission();

  return !err;
  
}


void setup() 
{
  serial_commands.init(process_command);

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);
                    
                   // Connect PCA9685 Vcc to 3.3
  Wire.setSDA(8);  // Connect PCA9685 SDA to gpio 8
  Wire.setSCL(9);  // Connect PCA9685 SCL to gpio 9
  
  /*Wire.begin();

  while (!find_ServoDriver(PCA9685_I2C_ADDRESS)) {
    Serial.println("No PCA9685 found ... check your connections");
    delay(200);
  }*/
  
  while (!ServoDriver.begin()) {
    Serial.println("No PCA9685 found ... check your connections");
    delay(200);
  }

  delay(10);

  Serial.println("Found PCA9685");

  ServoDriver.begin();

  // In theory the internal oscillator (clock) is 25MHz but it really isn't
  // that precise. You can 'calibrate' this by tweaking this number until
  // you get the PWM update frequency you're expecting!
  // The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
  // is used for calculating things like writeMicroseconds()
  // Analog servos run at ~50 Hz updates, It is importaint to use an
  // oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
  // 1) Attach the oscilloscope to one of the PWM signal pins and ground on
  //    the I2C PCA9685 chip you are setting the value for.
  // 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
  //    expected value (50Hz for most ESCs)
  // Setting the value here is specific to each individual I2C PCA9685 chip and
  // affects the calculations for the PWM update frequency. 
  // Failure to correctly set the int.osc value will cause unexpected PWM results
     
  ServoDriver.setOscillatorFrequency(28000000);
  ServoDriver.setPWMFreq(50);  // Analog servos run at ~50 Hz updates (20 ms period)
  delay(10);

  interval = 50;

  int i = 0;
      
  for(i = 0; i < NUM_SERVOS; i++) {
     servo_pos[i] = 205;
  }
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
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      
      int i = 0;
      for(i = 0; i < NUM_SERVOS; i++) {
        ServoDriver.setPWM(i, 0, servo_pos[i]);
      }
      
      // Debug using the serial port
      //Serial.print(" state: ");
      //Serial.print(serial_commands.state);

      Serial.print(" S[0]: ");
      Serial.print(servo_pos[0]);

      Serial.print(" pwm[0]: ");
      Serial.print(ServoDriver.getPWMOff(0));

      Serial.print(" Command: ");
      Serial.print(serial_commands.command);
      Serial.print(serial_commands.data);

      Serial.print(" loop: ");
      Serial.println(micros() - loop_micros);
    }
    
}


// 2022 Paulo Costa
// Pico W LED access

#include <Arduino.h>
#include <WiFi.h>

#include "pico/cyw43_arch.h"

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof;
float distance, prev_distance;

int LED_state;
unsigned long interval;
unsigned long currentMicros, previousMicros;
int loop_count;

void setup() 
{
  interval = 40 * 1000;

  Serial.begin(115200);

  Wire.setSDA(8);
  Wire.setSCL(9);

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

  //WiFi.begin
}

#define CYW43_WL_GPIO_LED_PIN 0

void loop() 
{
  currentMicros = micros();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
    }
 
    // Start new distance measure
    tof.startReadRangeMillimeters(); 

    // Toggle builtin LED    
    loop_count++;
    if (loop_count > 5) {
      LED_state = !LED_state;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
      loop_count = 0;
    }

    Serial.print(" Dist: ");
    Serial.print(distance, 3);
    Serial.println();
  }
}

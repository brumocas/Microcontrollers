#include <Arduino.h>

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

  pinMode(25, OUTPUT);
  digitalWrite(25, !digitalRead(25));

  Wire1.setSDA(10);
  Wire1.setSCL(11);

  Wire1.begin();

  tof.setBus(&Wire1);
  tof.setTimeout(500);

  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  
  
}

void loop() 
{
  currentMicros = micros();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    digitalWrite(25, !digitalRead(25));

    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
    }
 
    // Start new distance measure
    tof.startReadRangeMillimeters();    

    Serial.print(" Dist: ");
    Serial.print(distance, 3);
    Serial.println();
  }
}

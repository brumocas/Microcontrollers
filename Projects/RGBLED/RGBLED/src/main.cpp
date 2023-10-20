#include <Arduino.h>

#define MAXIMUM_NUM_NEOPIXELS 5
#include <NeoPixelConnect.h>

#define LED_PIN 22

// Create an instance of NeoPixelConnect and initialize it
// to use GPIO pin 22 as the control pin, for a string
// of 8 neopixels. Name the instance p
NeoPixelConnect strip(LED_PIN, MAXIMUM_NUM_NEOPIXELS, pio0, 0);

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

}

void loop() 
{
  for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
    strip.neoPixelSetValue(i, 255, i * 16, 0);
  }
  digitalWrite(LED_BUILTIN, 1);
  strip.neoPixelFill(255, 0, 0);
  strip.neoPixelShow();
  Serial.print("On");
  delay(500);

  digitalWrite(LED_BUILTIN, 0);
  strip.neoPixelClear();
  delay(500);
  Serial.println("Off");
}


#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include <VL53L0X.h>

#define builtInLED 25

/*------------------------------------------------Fsm-------------------------------------------*/
unsigned long interval;
unsigned long currentMicros, previousMicros;
int loop_count;

// State machine struct
typedef struct {
  int state, new_state;
  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

// Our finite state machines
fsm_t fsm1;




/*---------------------------------------Color Sensor-------------------------------------------*/
// Initialise with specific int time and gain values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

// Global Variables for the color sensor
int show_lux=1;
uint16_t r, g, b, c, colorTemp, lux;
// Define an enum named Color with three constants: RED, GREEN, and BLUE
enum Color {
    RED,
    GREEN,
    BLUE,
    INVALID
};
Color color;

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

// Guess current color
Color getColorValue(){
  
  getRawData_noDelay(&r, &g, &b, &c);

  if(r > g && r > b){
    return RED;
  } else if (g > r && g > b){
    return GREEN;
  } else if (b > r && b > g){
    return BLUE;
  } else {
    return INVALID;
  }

}



/*---------------------------------------Time of Flight-------------------------------------------*/
VL53L0X tof;
float distance, prev_distance;

// Get tof distance and prev_distance value in millimeters
void tofGetValue(){
    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
    }

    // Start new distance measure
    tof.startReadRangeMillimeters();   
}







void setup() 
{ 
  // PCT can be edit here in microseconds
  interval = 40 * 1000;

  // Start Serial Communication
  Serial.begin(115200);

  // Set built in Led for debug 
  pinMode(builtInLED, OUTPUT);
  digitalWrite(builtInLED, !digitalRead(25));


  // Connect TCS34725 Vin to 3.3
  Wire.setSDA(8);  // Connect TCS34725 SDA to gpio 8
  Wire.setSCL(9);  // Connect TCS34725 SCL to gpio 9
  Wire.begin();

  while (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }

  // Connect Tof to 3.3 V
  Wire1.setSDA(10);
  Wire1.setSCL(11);
  Wire1.begin();

  tof.setBus(&Wire1);
  tof.setTimeout(500);

  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }
  Serial.println("Found distance sensor");

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  


  // Init fsms
  set_state(fsm1, 0);
}

void loop() 
{
  currentMicros = micros();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    // Read inputs 

    // Read tof distance and pre_distance values
    tofGetValue();

    // Read color sensor values
    color = getColorValue();
    /* 
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    if (show_lux) lux = tcs.calculateLux(r, g, b);
    */


    // Update tis for all state machines
    unsigned long cur_time = currentMicros/1e3;   // Just one call to millis()
    fsm1.tis = cur_time - fsm1.tes;

    // FSM processing
    if(fsm1.state == 0 && fsm1.tis >= 2000){
      fsm1.new_state = 1;
    } else if (fsm1.state == 1 && fsm1.tis >= 2000){
      fsm1.new_state = 0;
    }














    // Update the states
    set_state(fsm1, fsm1.new_state);




    // Outputs
    //digitalWrite(25, !digitalRead(25));

    if (fsm1.state == 0){
      digitalWrite(builtInLED, HIGH);
    } else if (fsm1.state == 1){
      digitalWrite(builtInLED, LOW);
    }




    // Prints for debug


    // Distance value
    Serial.print(" Dist: ");
    Serial.print(distance, 3);
    Serial.print("  ");

    // Color value
    Serial.print(" Color: ");
    Serial.print(color);
    Serial.print("  ");


    Serial.print(" Fsm1: ");
    Serial.print(fsm1.state);
    Serial.print("  ");

    Serial.print(" Fsm1.tis: ");
    Serial.print(fsm1.tis);
    Serial.print("  ");
    

    /*
    Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    if (show_lux) Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    */

        
    Serial.println();
  }
}

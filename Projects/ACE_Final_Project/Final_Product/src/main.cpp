#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <ServosInfo.h>
#include <cmath>
#include <Interpolate.h>


/*------------------------------------------------Wifi------------------------------------------*/
const char* ssid = "bruninho";
const char* password = "123bruni";

//const char* ssid = "Escritório/Office";
//const char* password = "EEDAB1237C";

WiFiServer server(80);

String header;
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

// Function to get client connections with new information
void getClientInfo();


/*------------------------------------------------Fsm-------------------------------------------*/
unsigned long interval;
unsigned long currentMicros, previousMicros;
int loop_count;
bool clockwise = false;

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
fsm_t fsm1, fsm2, fsm3, fsm4, fsm5, fsm6;


/*------------------------------------------------Mode Control-------------------------------------------*/
enum Mode {
    HOLD,
    REMOTE,
    INITIAL_POS,
    SORT,
    SORT3X3_GRID,
    DISTANCE
};
Mode mode;

// Get current mode string
String modeToString(Mode mode) {
  switch (mode) {
    case REMOTE:
      return "Remote";
    case INITIAL_POS:
      return "Init";
    case SORT:
      return "Sort";
    case HOLD:
      return "Hold";
    case DISTANCE:
      return "Distance";
    case SORT3X3_GRID:
      return "Sort3X3_GRID";
    default:
      return "Unknown Mode";
  }
}


/*---------------------------------------Color Sensor-------------------------------------------*/
// Initialise with specific int time and gain values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

// Global Variables for the color sensor
int show_lux = 1;
uint16_t r, g, b, c, colorTemp, lux;
// Defined enum named Color with three constants: RED, GREEN, BLUE and YELLOW
enum Color {
    RED,
    GREEN,
    BLUE,
    YELLOW, 
    INVALID
};

// Structure to store HSV values
struct HSV {
    float h; // Hue
    float s; // Saturation
    float v; // Value
}; 
Color color, sensed_color;

// Function to convert RGB to HSV
HSV rgbToHsv(int r, int g, int b) {
    HSV hsv;
    
    float min_val = std::min(std::min(r, g), b);
    float max_val = std::max(std::max(r, g), b);
    
    // Calculate the hue
    if (max_val == min_val) {
        hsv.h = 0; // undefined, but for simplicity, set to 0
    } else if (max_val == r) {
        hsv.h = 60 * (0 + (g - b) / (max_val - min_val));
    } else if (max_val == g) {
        hsv.h = 60 * (2 + (b - r) / (max_val - min_val));
    } else { // max_val == b
        hsv.h = 60 * (4 + (r - g) / (max_val - min_val));
    }
    
    // Make sure hue is in the range [0, 360)
    while (hsv.h < 0) {
        hsv.h += 360;
    }
    
    // Calculate saturation
    hsv.s = (max_val == 0) ? 0 : (1 - (min_val / max_val));
    
    // Calculate value
    hsv.v = max_val / 255.0;
    
    return hsv;
}

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


// Function to guess color based on HSV values
Color getColorValue() {
    uint16_t r, g, b;
    getRawData_noDelay(&r, &g, &b, &c); 

    // Convert RGB to HSV
    HSV hsv = rgbToHsv(r, g, b);

    // Determine color based on the hue component
    if (hsv.h >= 0 && hsv.h < 30) {
        return RED;
    } else if (hsv.h >= 30 && hsv.h < 90) {
        return YELLOW;
    } else if (hsv.h >= 90 && hsv.h < 150) {
        return GREEN;
    } else if (hsv.h >= 150 && hsv.h < 210) {
        return BLUE;
    } else if (hsv.h >= 210 && hsv.h <= 270) {
        return BLUE; 
    } else {
        return INVALID;
    }
}


/*---------------------------------------Time of Flight-------------------------------------------*/
VL53L0X tof;
// Distance in cm
float distance, prev_distance, sum_distance = 0;
int count = 0;


// Get tof distance and prev_distance value in cm
void tofGetValue(){
    if (tof.readRangeAvailable()) {
      prev_distance = distance;

      distance = tof.readRangeMillimeters() * 1e-1;
    }

    // Start new distance measure
    tof.startReadRangeMillimeters();   
}


/*----------------------------------------------Servos-------------------------------------------*/
// Servos aux angles used
int angle1_aux = SERVO1_INIT, angle2_aux = SERVO2_INIT, angle3_aux = SERVO3_INIT, angle4_aux = SERVO4_INIT;
int line = 1, column = 1;
float sort_distance;
int sort_angle;

// Pins 
#define LED 25
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 7

// Pick SORT 
void pickPosSORT(float distance){

    if (distance >= 7.5 && distance <= 17.5){
    angle3_aux = getS3Interpolated(distance);
    angle4_aux = getS4Interpolated(distance);
  } 
}

// Rotate SORT 
void rotatePosSORT(int angle){
  angle2_aux = angle;

}

// Lego Default position
void pickPos(){

  angle3_aux = 145;
  angle4_aux = 119;
}

// Grab Lego
void grabLego(){
  angle1_aux = 90;
}

// Drop Lego
void dropLego(){
  angle1_aux = 45;
}

// Robot up
void upPos(){
  angle3_aux = 11;
  angle4_aux = 119;
}

// Rotate sense Pos
void rotateSensePos(){
  angle2_aux = 30;
}

// Color sensing 
void sensePos(){
  angle3_aux = 80;
  angle4_aux = 60;
}

// Rotate center
void rotateCenterPos(){
  angle2_aux = 90;
}

// Go to GREEN position
void rotateGreenPos(){
  angle2_aux = 170;
}

// Go to RED position
void rotateRedPos(){
  angle2_aux = 180;
}

// Go to BLUE position
void rotateBluePos(){
  angle2_aux = 160;
}

// Go to red position
void rotateYellowPos(){
  angle2_aux = 150;
}


// Pick SORT3X3_GRID angle 2
void pickSORT3X3_2(int x, int y){

  if(x == 1 && y == 1){
    angle2_aux = 110;
  } else if(x == 2 && y == 1){
    angle2_aux = 107;
  } else if(x == 3 && y == 1){
    angle2_aux = 105;
  } else if(x == 1 && y == 2){
    angle2_aux = 90;
  } else if(x == 2 && y == 2){
    angle2_aux = 90;
  } else if(x == 3 && y == 2){
    angle2_aux = 90;
  } else if(x == 1 && y == 3){
    angle2_aux = 75;
  } else if(x == 2 && y == 3){
    angle2_aux = 78;
  } else if(x == 3 && y == 3){
    angle2_aux = 78;
  }
} 

// Pick SORT3X3_GRID angle3 and angle4
void pickSORT3X3_34(int x, int y){

  if(x == 1 && y == 1){
    angle3_aux = 100;
    angle4_aux = 40;
  } else if(x == 2 && y == 1){
    angle3_aux = 119;
    angle4_aux = 76;
  } else if(x == 3 && y == 1){
    angle3_aux = 135;
    angle4_aux = 114;
  } else if(x == 1 && y == 2){
    angle3_aux = 103;
    angle4_aux = 40;
  } else if(x == 2 && y == 2){
    angle3_aux = 114;
    angle4_aux = 71;
  } else if(x == 3 && y == 2){
    angle3_aux = 145;
    angle4_aux = 123;
  } else if(x == 1 && y == 3){
    angle3_aux = 100;
    angle4_aux = 40;
  } else if(x == 2 && y == 3){
    angle3_aux = 115;
    angle4_aux = 85;
  } else if(x == 3 && y == 3){
    angle3_aux = 140;
    angle4_aux = 112;
  }
} 


void setup() 
{ 
    
  // PCT can be edit here in microseconds
  interval = 40 * 1000;

  // Start Serial Communication
  Serial.begin(115200);

  // Set built in Led for debug 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  s1.servo.attach(SERVO1_PIN, 500, 2500);
  s2.servo.attach(SERVO2_PIN, 500, 2500);
  s3.servo.attach(SERVO3_PIN, 500, 2500);
  s4.servo.attach(SERVO4_PIN, 500, 2500);

  // Set init positions
  s1.servo.writeMicroseconds(s1.getPWM());
  s2.servo.writeMicroseconds(s2.getPWM());
  s3.servo.writeMicroseconds(s3.getPWM());
  s4.servo.writeMicroseconds(s4.getPWM());

  
  // Connect TCS34725 Vin to 3.3
  Wire.setSDA(8);  
  Wire.setSCL(9);  
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
  tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  
  

  // Connect wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to wifi....");
  }

  Serial.println("");
  Serial.print("WiFi connected at IP Address ");
  Serial.println(WiFi.localIP());
  server.begin();

  // Set starting mdoe
  mode = HOLD;

  // Starting SORT Position
  sort_distance = 16.75;
  sort_angle = 90;

  // Init fsms
  set_state(fsm1, 0);
  set_state(fsm2, 0);
  set_state(fsm3, 0);
  set_state(fsm4, 0);
  set_state(fsm5, 0);
  set_state(fsm6, 0);
}

void loop()   
{ 

  // Check if there is a new client connection/new data available
  getClientInfo();
  
  // Get current time
  currentMicros = micros();

  // The Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    // Update tis for all state machines
    unsigned long cur_time = currentMicros/1e3;   // Just one call to millis()
    fsm1.tis = cur_time - fsm1.tes;
    fsm2.tis = cur_time - fsm2.tes;
    fsm3.tis = cur_time - fsm3.tes;
    fsm4.tis = cur_time - fsm4.tes;
    fsm5.tis = cur_time - fsm5.tes;
    fsm6.tis = cur_time - fsm6.tes;
    
    

    // Read inputs 

    // Read color sensor values
    color = getColorValue();

    // Read tof distance and pre_distance values
    tofGetValue();    
    /* 
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    if (show_lux) lux = tcs.calculateLux(r, g, b);
    */

    // FSM processing

    // fsm1, LED state machine
    if(fsm1.state == 0 && fsm1.tis >= 500){
      fsm1.new_state = 1;
    } else if (fsm1.state == 1 && fsm1.tis >= 500){
      fsm1.new_state = 0;
    }

    // Control State Machine
    if(mode == REMOTE || mode == INITIAL_POS){
      fsm3.new_state = 0;
      fsm4.new_state = 0;
      fsm5.new_state = 0;
      fsm6.new_state = 0;
      line = 1;
      column = 1;
    }

    if(mode == SORT){
      fsm4.new_state = 0;
      fsm5.new_state = 0;
      fsm6.new_state = 0;
      line = 1;
      column = 1;
    }

    if(mode == DISTANCE){
      fsm3.new_state = 0;
      fsm4.new_state = 0;
      fsm6.new_state = 0;
    }

    if(mode == SORT3X3_GRID){
      fsm3.new_state = 0;
      fsm4.new_state = 0;
      fsm5.new_state = 0;
    }

    // fsm3, Sort state machine
    if(fsm3.state == 0 && mode == SORT){
      // Go to init Pos
      fsm3.new_state = 1;
      angle1_aux = SERVO1_INIT;
      rotatePosSORT(sort_angle);
      angle3_aux = SERVO3_INIT;
      angle4_aux = SERVO4_INIT;
    } else if(fsm3.state == 1 && s1.curr_Angle == s1.next_Angle && s2.curr_Angle == s2.next_Angle && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Go to lego positon
      fsm3.new_state = 2;
      pickPosSORT(sort_distance);
    } else if (fsm3.state == 2 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Grab lego
      fsm3.new_state = 3;
      grabLego();
    } else if (fsm3.state == 3 && s1.curr_Angle == s1.next_Angle){
      // Up Pos
      fsm3.new_state = 4;
      upPos();
    } else if (fsm3.state == 4 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Rotate Lego to sensing color pos
      fsm3.new_state = 5;
      rotateSensePos();
    } else if (fsm3.state == 5 && s2.curr_Angle == s2.next_Angle){
      // Find lego color
      fsm3.new_state = 6;
      sensePos();
    } else if (fsm3.state == 6 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && fsm3.tis >= 2000){
      // Up Pos
      fsm3.new_state = 7;
      upPos();
    }else if (fsm3.state == 7 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && sensed_color == GREEN){
      // Rotate Green
      fsm3.new_state = 8;
      rotateGreenPos();
    } else if (fsm3.state == 7 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && sensed_color == BLUE){
      // Rotate Blue
      fsm3.new_state = 12;
      rotateBluePos();
    } else if (fsm3.state == 7 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && sensed_color == RED){
      // Rotate Red
      fsm3.new_state = 13;
      rotateRedPos();
    } else if (fsm3.state == 7 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && sensed_color == YELLOW){
      // Rotate Yellow
      fsm3.new_state = 14;
      rotateYellowPos();
    } else if ( (fsm3.state == 8 || fsm3.state == 12 || fsm3.state == 13 || fsm3.state == 14) && s2.curr_Angle == s2.next_Angle){
      // Go to lego drop position
      fsm3.new_state = 9;
      pickPos();
    } else if (fsm3.state == 9 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Drop lego
      fsm3.new_state = 10;  
      dropLego();
    } else if (fsm3.state == 10 && s1.curr_Angle == s1.next_Angle && fsm3.tis >= 500){
      // Go to up position
      fsm3.new_state = 11;
      upPos();
    } else if (fsm3.state == 11 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Go to center position
      fsm3.new_state = 15;
      rotatePosSORT(sort_angle);
    } else if (fsm3.state == 15 && s2.curr_Angle == s2.next_Angle){
      // Go to up position
      fsm3.new_state = 1;
    }


    // fsm5, Distance state machine
    if(fsm5.state == 0 && mode == DISTANCE){
      // Go to init Pos
      fsm5.new_state = 1;
      angle1_aux = SERVO1_INIT;
      angle2_aux = SERVO2_INIT;
      angle3_aux = SERVO3_INIT;
      angle4_aux = SERVO4_INIT;
      count = 0;
      sum_distance = 0;
    } else if(fsm5.state == 1 && s1.curr_Angle == s1.next_Angle && s2.curr_Angle == s2.next_Angle && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Start Sweep
      fsm5.new_state = 2;
    } else if(fsm5.state == 2 && distance <= 17 && distance > prev_distance){
      // Measure average distance
      fsm5.new_state = 3; 
    } else if (fsm5.state == 3 && fsm5.tis >= 500){
      // Go to average_distance Pos
      fsm5.new_state = 4;
      angle3_aux = getS3Interpolated(sum_distance/count);
      angle4_aux = getS4Interpolated(sum_distance/count);
      angle1_aux = 0;
    } else if (fsm5.new_state == 4 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && s1.curr_Angle == s1.next_Angle){
      // Grab   
      fsm5.new_state = 5;
      grabLego();
    } else if (fsm5.state == 5 && s1.curr_Angle == s1.next_Angle){
      // Up Robot
      fsm5.new_state = 6;
      upPos();
    } else if(fsm5.state == 6 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle && s4.next_Angle){
      // Rotate TO 180 Pos
      fsm5.new_state = 7;
      angle2_aux = 180;
    } else if(fsm5.state == 7 && s2.curr_Angle == s2.next_Angle){
      // Open Arm
      fsm5.new_state = 8;
      pickPos();
    } else if(fsm5.state == 8 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Drop Lego
      fsm5.new_state = 9;
      dropLego();
    } else if(fsm5.state == 9 && s1.curr_Angle == s1.next_Angle){
      // Robot Up
      fsm5.new_state = 10;
      upPos();
    }else if(fsm5.state == 10 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && fsm5.tis >= 1000){
      // Go to init
      fsm5.new_state = 0;
    }


    // fsm6, Sort3X3_GRID state machine
    if(fsm6.state == 0 && mode == SORT3X3_GRID){
      // Go to init Pos
      fsm6.new_state = 1;
      angle1_aux = SERVO1_INIT;
      angle2_aux = SERVO2_INIT;
      angle3_aux = SERVO3_INIT;
      angle4_aux = SERVO4_INIT;
    } else if(fsm6.state == 1 && s1.curr_Angle == s1.next_Angle && s2.curr_Angle == s2.next_Angle && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Go to column number 1
      fsm6.new_state = 2;
      pickSORT3X3_2(line, column);
    } else if(fsm6.state == 2 && s2.curr_Angle == s2.next_Angle){
      // x and y Pos
      fsm6.new_state = 3;
      pickSORT3X3_34(line, column);
    } else if(fsm6.state == 3 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Grab lego
      fsm6.new_state = 4;
      grabLego();
    } else if(fsm6.state == 4 && s1.curr_Angle == s1.next_Angle){
      // Up Pos
      fsm6.new_state = 5;
      upPos();
    } else if(fsm6.state == 5  && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Rotate sensing pos
      fsm6.new_state = 6;
      rotateSensePos();
    } else if(fsm6.state == 6  && s2.curr_Angle==s2.next_Angle){
      // Sensing Pos
      fsm6.new_state = 7;
      sensePos();
    } else if(fsm6.state == 7  && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Sensing color
      fsm6.new_state = 8;
    } else if(fsm6.state == 8 && fsm6.tis > 2000){
      // Up Robot
      fsm6.new_state = 9;
      upPos();
    } else if (fsm6.state == 9 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle && sensed_color == RED){
      // RED
      fsm6.new_state = 10;
      rotateRedPos();
    } else if (fsm6.state == 9 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle && sensed_color == BLUE){
      // BLUE
      fsm6.new_state = 11;
      rotateBluePos();
    } else if (fsm6.state == 9 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle && sensed_color == GREEN){
      // GREEN
      fsm6.new_state = 12;
      rotateGreenPos();
    } else if (fsm6.state == 9 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle && sensed_color == YELLOW){
      // YELLOW
      fsm6.new_state = 13;
      rotateYellowPos();
    } else if ( (fsm6.state == 10 || fsm6.state == 11 || fsm6.state == 12 || fsm6.state == 13 ) && s2.curr_Angle == s2.next_Angle){
      // Drop Pos
      fsm6.new_state = 14;
      pickPos();
    } else if ( fsm6.state == 14 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Drop lego
      fsm6.new_state = 15;
      dropLego();
    } else if ( fsm6.state == 15 && s1.curr_Angle == s1.next_Angle){
      // Up Pos
      fsm6.new_state = 16;
      upPos();
    } else if (fsm6.state == 16 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Go to init
      fsm6.new_state = 0;
      // Increase line
      column+=1;
      // If line ends
      if(column > 3){
        column = 1;
        line+=1;
      }
      // If grid ends
      if(line > 3 ){
        column = 1;
        line = 1;
      }
    }

    // Update the states
    set_state(fsm1, fsm1.new_state);
    set_state(fsm2, fsm2.new_state);
    set_state(fsm3, fsm3.new_state);
    set_state(fsm4, fsm4.new_state);
    set_state(fsm5, fsm5.new_state);
    set_state(fsm6, fsm6.new_state);


    // Outputs

    if (mode == REMOTE || mode == SORT || mode == DISTANCE || mode == SORT3X3_GRID)
    { 
      // State machines change angles 
      s1.next_Angle = angle1_aux;
      s2.next_Angle = angle2_aux;
      s3.next_Angle = angle3_aux;
      s4.next_Angle = angle4_aux;
    }

    if (mode == INITIAL_POS)
    { 
      // Go to Initial Position
      angle1_aux = SERVO1_INIT;
      angle2_aux = SERVO2_INIT;
      angle3_aux = SERVO3_INIT;
      angle4_aux = SERVO4_INIT;

      s1.next_Angle = SERVO1_INIT;
      s2.next_Angle = SERVO2_INIT;
      s3.next_Angle = SERVO3_INIT;
      s4.next_Angle = SERVO4_INIT;

    }

    // fsm1 outputs
    if (fsm1.state == 0){
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (fsm1.state == 1){
      digitalWrite(LED_BUILTIN, LOW);
    }

    // fsm3 and fsm6 outputs
    if (fsm3.state == 6 || fsm6.state == 8)
    {
      sensed_color = color;
    }

    // fsm5 outputs
    
    // Sweep area
    if (fsm5.state == 2){
      
      if(clockwise){
        angle2_aux -= 1;
      } else if(!clockwise){
        angle2_aux +=1;
      }

      if (s2.curr_Angle == 50)
        clockwise = false;
      else if (s2.curr_Angle == 180 )
        clockwise = true;
      
    }

    // Measure average distance
    if (fsm5.state == 3){
      sum_distance += distance;
      count +=1;
    }
  
    // Update Servos outputs
    s1.servo.writeMicroseconds(s1.getPWM());
    s2.servo.writeMicroseconds(s2.getPWM());
    s3.servo.writeMicroseconds(s3.getPWM());
    s4.servo.writeMicroseconds(s4.getPWM());


    // Prints for debug
    
    // Color value
    Serial.print(" Color: ");
    Serial.print(color);
    Serial.print("  ");

    // Color value
    Serial.print(" Sensed Color: ");
    Serial.print(sensed_color);
    Serial.print("  ");

    // Control mode value
    Serial.print(" Mode: ");
    Serial.print(modeToString(mode));
    Serial.print("  ");

    // Fsm1 state
    Serial.print(" Fsm1: ");
    Serial.print(fsm1.state);
    Serial.print("  ");

    /*
    // Fsm2 state
    Serial.print(" Fsm2: ");
    Serial.print(fsm2.state);
    Serial.print("  ");
    */

    // Fsm3 state
    Serial.print(" Fsm3: ");
    Serial.print(fsm3.state);
    Serial.print("  ");
    
    /*
    // Fsm4 state
    Serial.print(" Fsm4: ");
    Serial.print(fsm4.state);
    Serial.print("  ");
    */

    // Fsm5 state
    Serial.print(" Fsm5: ");
    Serial.print(fsm5.state);
    Serial.print("  ");

    // Fsm5 state
    Serial.print(" Fsm6: ");
    Serial.print(fsm6.state);
    Serial.print("  ");

    // Current angles values
    Serial.print(" A1: " + String(s1.next_Angle));
    Serial.print(" A2: " + String(s2.next_Angle));
    Serial.print(" A3: " + String(s3.curr_Angle));
    Serial.print(" A4: " + String(s4.curr_Angle));

    // line and column values
    Serial.print(" l: " + String(line));
    Serial.print(" c: " + String(column));

    // Distance value
    Serial.print(" Dist: ");
    Serial.print(distance, 3);
    Serial.print("  ");

    // Average Distance value
    Serial.print(" Ave_Dist: ");
    Serial.print(sum_distance/count, 3);
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

// Function to get client connections with new information from webServer
void getClientInfo(){

 WiFiClient client = server.available();

  if (client) {
    currentTime = millis();
    previousTime = currentTime;
    //Serial.println("New Client.");
    String currentLine = "";
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();
      if (client.available()) {
        char c = client.read();
        header += c;
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            if (header.indexOf("GET /Remote") >= 0) {
              mode = REMOTE;
            }

            if (header.indexOf("GET /Init") >= 0) {
              mode = INITIAL_POS;
            }

            if (header.indexOf("GET /Sort") >= 0) {
              mode = SORT;
            }    

            if (header.indexOf("GET /Distance") >= 0) {
              mode = DISTANCE;
            }       

            if (header.indexOf("GET /Sort3X3_GRID") >= 0) {
              mode = SORT3X3_GRID;
            }


            // Set web page style
            client.println("<style>");
            client.println("html {");
            client.println("    font-family: Helvetica;");
            client.println("    display: inline-block;");
            client.println("    margin: 0px auto;");
            client.println("    text-align: center;");
            client.println("}");
            client.println("p {");
            client.println("    text-align: center; /* Center the text within <p> elements */");
            client.println("}");
            client.println(".button, .button2 {");
            client.println("    width: 200px; /* Set a fixed width for the buttons */");
            client.println("    background-color: #4CAF50;");
            client.println("    border: none;");
            client.println("    color: white;");
            client.println("    padding: 16px 0; /* Adjusted padding for both buttons */");
            client.println("    text-decoration: none;");
            client.println("    font-size: 30px;");
            client.println("    margin: 2px;");
            client.println("    cursor: pointer;");
            client.println("}");
            client.println(".button2 {");
            client.println("    background-color: #F23A3A;");
            client.println("}");
            client.println("</style>");

                          
            // Web Page title
            client.println("<body><h1>ACE Robot Control</h1>");
            // Web mode state
            //client.println("<p>Current Robot mode is " + modeToString(mode) + "</p>");


            if (mode == INITIAL_POS) {
              client.println("<p><a href=\"/Init\"><button class=\"button\">Init</button></a></p>");
            } else {
              client.println("<p><a href=\"/Init\"><button class=\"button button2\">Init</button></a></p>");
            }

            // Update Web new values
            if (mode == REMOTE) {
              client.println("<p><a href=\"/Remote\"><button class=\"button\">Remote</button></a></p>");
            } else {
              client.println("<p><a href=\"/Remote\"><button class=\"button button2\">Remote</button></a></p>");
            }

            if (mode == SORT) {
              client.println("<p><a href=\"/Sort\"><button class=\"button\">Sort</button></a></p>");
            } else {
              client.println("<p><a href=\"/Sort\"><button class=\"button button2\">Sort</button></a></p>");
            }

            if (mode == SORT3X3_GRID) {
              client.println("<p><a href=\"/Sort3X3_GRID\"><button class=\"button\">Sort3X3_GRID</button></a></p>");
            } else {
              client.println("<p><a href=\"/Sort3X3_GRID\"><button class=\"button button2\">Sort3X3_GRID</button></a></p>");
            }

            if (mode == DISTANCE) {
              client.println("<p><a href=\"/Distance\"><button class=\"button\">Distance</button></a></p>");
            } else {
              client.println("<p><a href=\"/Distance\"><button class=\"button button2\">Distance</button></a></p>");
            }

            // Get new angles values
            if (header.indexOf("GET /number1?value=") >= 0) {
              // Update number value for angle1
              angle1_aux = header.substring(header.indexOf("/number1?value=") + 15, header.indexOf(" ", header.indexOf("/number1?value="))).toInt();
            }

            if (header.indexOf("GET /number2?value=") >= 0) {
              // Update number value for angle2
              angle2_aux = header.substring(header.indexOf("/number2?value=") + 15, header.indexOf(" ", header.indexOf("/number2?value="))).toInt();
            }

            if (header.indexOf("GET /number3?value=") >= 0) {
              // Update number value for angle3
              angle3_aux = header.substring(header.indexOf("/number3?value=") + 15, header.indexOf(" ", header.indexOf("/number3?value="))).toInt();
            }

            if (header.indexOf("GET /number4?value=") >= 0) {
              // Update number value for angle4
              angle4_aux = header.substring(header.indexOf("/number4?value=") + 15, header.indexOf(" ", header.indexOf("/number4?value="))).toInt();
            }

            if (header.indexOf("GET /number5?value=") >= 0) {
              // Update number value for angle4
              step = header.substring(header.indexOf("/number5?value=") + 15, header.indexOf(" ", header.indexOf("/number5?value="))).toInt();
              if(step > 10){
                step = 10;
              } else if (step <= 0){
                step = 1;
              }
            }

            if (header.indexOf("GET /number6?value=") >= 0) {
              // Update number value for prev distance
              sort_distance = header.substring(header.indexOf("/number6?value=") + 15, header.indexOf(" ", header.indexOf("/number6?value="))).toFloat();
            }

            if (header.indexOf("GET /number7?value=") >= 0) {
              // Update number value for prev angle
              sort_angle = header.substring(header.indexOf("/number7?value=") + 15, header.indexOf(" ", header.indexOf("/number7?value="))).toInt();
            }

            // Show angles if in REMOTE moDE
            if (mode == INITIAL_POS){

              // Web to change step value
              client.println("<p>Current step value is " + String(step) + "</p>");
              client.println("<p><input type=\"number\" min=\"0\" max=\"10\" value=\"" + String(step) + "\" id=\"myNumber5\"></p>");
              client.println("<script>");
              client.println("var number = document.getElementById('myNumber5');");
              client.println("number.onchange = function() {");
              client.println("  window.location.href = '/number5?value=' + this.value;");
              client.println("}");
              client.println("</script>");

              // Web mode state for sort distance
              client.println("<p>Current SORT distance to base is " + String(sort_distance) + "</p>");
              client.println("<p><input type=\"number\" min=\"7.5\" max=\"17.5\" value=\"" + String(sort_distance) + "\" id=\"myNumber6\"></p>");
              client.println("<script>");
              client.println("var number = document.getElementById('myNumber6');");
              client.println("number.onchange = function() {");
              client.println("  window.location.href = '/number6?value=' + this.value;");
              client.println("}");
              client.println("</script>");

              
              // Web mode state for sort angle
              client.println("<p>Current SORT angle to base is " + String(sort_angle) + "</p>");
              client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(sort_angle) + "\" id=\"myNumber7\"></p>");
              client.println("<script>");
              client.println("var number = document.getElementById('myNumber7');");
              client.println("number.onchange = function() {");
              client.println("  window.location.href = '/number7?value=' + this.value;");
              client.println("}");
              client.println("</script>");
              
            }


            // Show angles if in REMOTE moDE
            if (mode == REMOTE){

              // Web mode state for angle1
              client.println("<p>Current Angle1 value is " + String(s1.next_Angle) + "</p>");
              client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle1_aux) + "\" id=\"myNumber1\"></p>");
              client.println("<script>");
              client.println("var number = document.getElementById('myNumber1');");
              client.println("number.onchange = function() {");
              client.println("  window.location.href = '/number1?value=' + this.value;");
              client.println("}");
              client.println("</script>");


              // Web mode state for angle2
              client.println("<p>Current Angle2 value is " + String(s2.next_Angle) + "</p>");
              client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle2_aux) + "\" id=\"myNumber2\"></p>");
              client.println("<script>");
              client.println("var number = document.getElementById('myNumber2');");
              client.println("number.onchange = function() {");
              client.println("  window.location.href = '/number2?value=' + this.value;");
              client.println("}");
              client.println("</script>");


              // Web mode state for angle3
              client.println("<p>Current Angle3 value is " + String(s3.next_Angle) + "</p>");
              client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle3_aux) + "\" id=\"myNumber3\"></p>");
              client.println("<script>");
              client.println("var number = document.getElementById('myNumber3');");
              client.println("number.onchange = function() {");
              client.println("  window.location.href = '/number3?value=' + this.value;");
              client.println("}");
              client.println("</script>");


              // Web mode state for angle4
              client.println("<p>Current Angle4 value is " + String(s4.next_Angle) + "</p>");
              client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle4_aux) + "\" id=\"myNumber4\"></p>");
              client.println("<script>");
              client.println("var number = document.getElementById('myNumber4');");
              client.println("number.onchange = function() {");
              client.println("  window.location.href = '/number4?value=' + this.value;");
              client.println("}");
              client.println("</script>");

            }

            client.println("</body></html>");
            client.println();
            break;

          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    header = "";
    client.stop();
    //Serial.println("Client disconnected.");
    //Serial.println("");
  }

}
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <ServosInfo.h>
#include <cmath>


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
fsm_t fsm1, fsm2, fsm3, fsm4;


/*------------------------------------------------Mode Control-------------------------------------------*/
enum Mode {
    HOLD,
    REMOTE,
    INITIAL_POS,
    SORT,
    SORT3X3
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
    case SORT3X3:
      return "Sort3X3";
    case HOLD:
      return "Hold";
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
float distance, prev_distance;

// Get tof distance and prev_distance value in millimeters
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

// Pins 
#define LED 25
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 7

// Pick lego 
void pickLegoPos(){
  angle3_aux = 140;
  angle4_aux = 119;
}

// Grab Lego
void grabLego(){
  angle1_aux = 90;
}

// Drop Lego
void dropLego(){
  angle1_aux = 30;
}

// Robot up
void upPos(){
  angle3_aux = 11;
  angle4_aux = 119;
}

// Rotate sense
void rotateSensePos(){
  angle2_aux = 14;
}

// Color sensing 
void senseLegoPos(){
  angle3_aux = 70;
  angle4_aux = 60;
}

// Rotate center
void rotateCenter(){
  angle2_aux = 90;
}

// Go to GREEN position
void rotateGreenPos(){
  angle2_aux = 110;
}

// Go to RED position
void rotateRedPos(){
  angle2_aux = 130;
}

// Go to BLUE position
void rotateBluePos(){
  angle2_aux = 70;
}

// Go to red position
void rotateYellowPos(){
  angle2_aux = 50;
}

// PicK Grid line from column
void pickSORT3X3_L(int x){

  if(x == 1){
    // Pos 1xY
    angle3_aux = 90;
    angle4_aux = 40;
  } else if(x == 2){
    // Pos 2xY
    angle3_aux = 105;
    angle4_aux = 80;
  } else if(x == 3){
    // Pos 3xY
    angle3_aux = 145;
    angle4_aux = 140;
  }  

}

// Pick Column from grid
void pickSORT3X3_C(int y){

  if (y == 1){
    // Column 1
    angle2_aux = 120;
  } else if(y == 2){
    // Column 2
    angle2_aux = 90;
  } else if( y == 3){
    // Column 3
    angle3_aux == 60;
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
  Wire.setSDA(8);  // Connect TCS34725 SDA to gpio 10
  Wire.setSCL(9);  // Connect TCS34725 SCL to gpio 11
  Wire.begin();

  while (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }

  /*
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
  */


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


  // Init fsms
  set_state(fsm1, 0);
  set_state(fsm2, 0);
  set_state(fsm3, 0);
  set_state(fsm4, 0);
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
    
    
    
    // Read inputs 

    // Read color sensor values
    color = getColorValue();

    /*
    // Read tof distance and pre_distance values
    tofGetValue();    
    */

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

    // fsm2, (bugged!!!)
    if(fsm2.state == 0 ){
      fsm2.new_state = 1;
    }

    // fsm3, Sort state machine
    if(fsm3.state == 0 && mode == SORT){
      // Go to init Pos
      fsm3.new_state = 1;
      angle1_aux = SERVO1_INIT;
      angle2_aux = SERVO2_INIT;
      angle3_aux = SERVO3_INIT;
      angle4_aux = SERVO4_INIT;
    } else if(fsm3.state == 3 && s1.curr_Angle == s1.next_Angle && s2.curr_Angle == s2.next_Angle && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Go to lego positon
      fsm3.new_state = 2;
      pickLegoPos();
    } else if (fsm3.state == 2 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Grab lego
      fsm3.new_state = 3;
      grabLego();
    } else if (fsm3.state == 3 && s1.curr_Angle == s1.next_Angle){
      // Up Pos
      fsm3.new_state = 4;
      upPos();
    } else if (fsm3.state == 3 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Rotate Lego to sensing color pos
      fsm3.new_state = 5;
      rotateSensePos();
    } else if (fsm3.state == 5 && s2.curr_Angle == s2.next_Angle){
      // Find lego color
      fsm3.new_state = 6;
      senseLegoPos();
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
      pickLegoPos();
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
      fsm3.new_state = 12;
      rotateCenter(); 
    } else if (fsm3.state == 12 && s2.curr_Angle == s2.next_Angle){
      // Go to up position
      fsm3.new_state = 0;
    }

    
    // fsm4, SORT3X3 state machine
    if(fsm4.state == 0 && mode == SORT3X3){
      // Go to init Pos
      fsm4.new_state = 1;
      angle1_aux = SERVO1_INIT;
      angle2_aux = SERVO2_INIT;
      angle3_aux = SERVO3_INIT;
      angle4_aux = SERVO4_INIT;
    } else if(fsm4.state == 1 && s1.curr_Angle == s1.next_Angle && s2.curr_Angle == s2.next_Angle && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Go to column number 1
      fsm4.new_state = 2;
      pickSORT3X3_C(1);
    } else if(fsm4.state == 2 && s2.curr_Angle == s2.next_Angle){
      // 1X1 Pos
      fsm4.new_state = 3;
      pickSORT3X3_L(1);
    } else if(fsm4.state == 3 && s3.curr_Angle == s3.next_Angle && s4.curr_Angle == s4.next_Angle){
      // Grab lego
      fsm4.new_state = 4;
      grabLego();
    } else if(fsm4.state == 4 && s2.curr_Angle == s2.next_Angle){
      // Continue ....
      fsm4.new_state == 0;
    }


    // Update the states
    set_state(fsm1, fsm1.new_state);
    set_state(fsm2, fsm2.new_state);
    set_state(fsm3, fsm3.new_state);
    set_state(fsm4, fsm4.new_state);


    // Outputs

    if (mode == REMOTE || mode == SORT || mode == SORT3X3)
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

    // fsm3 outputs
    if (fsm3.state == 5)
    {
      sensed_color = color;
    }
  

    // Update Servos outputs
    s1.servo.writeMicroseconds(s1.getPWM());
    s2.servo.writeMicroseconds(s2.getPWM());
    s3.servo.writeMicroseconds(s3.getPWM());
    s4.servo.writeMicroseconds(s4.getPWM());



    // Prints for debug
    
    // Distance value
    Serial.print(" Dist: ");
    Serial.print(distance, 3);
    Serial.print("  ");

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

    // Fsm2 state
    Serial.print(" Fsm2: ");
    Serial.print(fsm2.state);
    Serial.print("  ");

    // Fsm3 state
    Serial.print(" Fsm3: ");
    Serial.print(fsm3.state);
    Serial.print("  ");

    // Current angles values
    Serial.print(" Angle1: " + String(s1.next_Angle));
    Serial.print(" Angle2: " + String(s2.next_Angle));
    Serial.print(" Angle3: " + String(s3.curr_Angle));
    Serial.print(" Angle4: " + String(s4.curr_Angle));

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

            if (header.indexOf("GET /Sort3X3") >= 0) {
              mode = SORT3X3;
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

            // Update Web new values
            if (mode == REMOTE) {
              client.println("<p><a href=\"/Remote\"><button class=\"button\">Remote</button></a></p>");
            } else {
              client.println("<p><a href=\"/Remote\"><button class=\"button button2\">Remote</button></a></p>");
            }

            if (mode == INITIAL_POS) {
              client.println("<p><a href=\"/Init\"><button class=\"button\">Init</button></a></p>");
            } else {
              client.println("<p><a href=\"/Init\"><button class=\"button button2\">Init</button></a></p>");
            }

            if (mode == SORT) {
              client.println("<p><a href=\"/Sort\"><button class=\"button\">Sort</button></a></p>");
            } else {
              client.println("<p><a href=\"/Sort\"><button class=\"button button2\">Sort</button></a></p>");
            }

            if (mode == SORT3X3) {
              client.println("<p><a href=\"/Sort3X3\"><button class=\"button\">Sort3X3</button></a></p>");
            } else {
              client.println("<p><a href=\"/Sort3X3\"><button class=\"button button2\">Sort3X3</button></a></p>");
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
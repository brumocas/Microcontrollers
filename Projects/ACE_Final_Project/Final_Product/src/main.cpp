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
fsm_t fsm1, fsm2;


/*------------------------------------------------Mode Control-------------------------------------------*/
enum Mode {
    HOLD,
    MODE1,
    MODE2,
    MODE3,
    MODE4
};
Mode mode;

// Get current mode string
String modeToString(Mode mode) {
  switch (mode) {
    case MODE1:
      return "Mode 1";
    case MODE2:
      return "Mode 2";
    case MODE3:
      return "Mode 3";
    case MODE4:
      return "Mode 4";
    case HOLD:
      return "HOLD";
    default:
      return "Unknown Mode";
  }
}


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
float angle1 = 0, angle2 = 0, angle3 = 0, angle4 = 0;
float anglejoint1_deg = 0;
float anglejoint2_deg = 0;

float X = 0;
float Y = 7;
float Z = 8;

enum Angle{
 Servo1,
 Servo2,
 Servo3,
 Servo4
}; 
Angle state;


// Pins 
#define LED 25
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 7

// Define measures
#define L 8.00

// Init angles
#define SERVO1_INIT 30
#define SERVO2_INIT 90
#define SERVO3_INIT 120
#define SERVO4_INIT 180


  void changeAngle(Angle state, int sum){{
    
    switch (state)
    {
    case Servo1:
      angle1 +=sum;
      break;

      case Servo2:
      angle2 +=sum;
      break;
      
      case Servo3:
      angle3 +=sum;
      break;
      
      case Servo4:
      angle4 +=sum;
      break;
    
    default:
      break;
    }
  }

  }

  // Go to x,y,z given position
  void goToPos(float x, float y, float z){
  
  // Check the required position is valid

  /*
  if ( y^2 + z^2 > (2*L)^2 ){
    return;
  }
  */
  
  /*
  float o2 = -2 * std::atan(std::sqrt( ((2 * L) * (2 * L) / (y * y + z * z)) - 1));
  float o1 = std::atan2(z, y) - std::atan2(L * std::sin(o2), L * (1 + std::cos(o2))) - M_PI / 2;

  o1 = o1 * 180 / M_PI;
  o2 = o2 * 180 / M_PI;

  Serial.print(" o1:"  +  String(o1));
  Serial.print(" o2:"  +  String(o2));


  anglejoint1_deg = 90 - o1;
  anglejoint2_deg = o2 + 270;

  angle3 = anglejoint1_deg;
  angle4 = anglejoint2_deg;
  */

  }


  // For CMD angle remote control
  void getAngleCMD(){

      uint8_t b;
    if (Serial.available() > 0) {
    b = Serial.read();
    
    if (b == '-') {
      changeAngle(state, -10);
    } else if (b == '+') {
      changeAngle(state, 10);
    } 

    if (b == 'l'){
      if (state == Servo1){
        state = Servo2;
      } else if (state == Servo2){
        state = Servo3;
      } else if (state == Servo3){
        state = Servo4;
      } else if (state == Servo4){
        state = Servo1;
      }

    }
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
  s1.servo.writeMicroseconds(s1.getPWM(SERVO1_INIT));
  angle1 = SERVO1_INIT;
  s2.servo.writeMicroseconds(s3.getPWM(SERVO2_INIT));
  angle2 = SERVO2_INIT;
  s3.servo.writeMicroseconds(s2.getPWM(SERVO3_INIT));
  angle3 = SERVO3_INIT;
  s4.servo.writeMicroseconds(s4.getPWM(SERVO4_INIT));
  angle4 = SERVO4_INIT;

  // Set angle to move
  state = Servo1;


  /*
  // Connect TCS34725 Vin to 3.3
  Wire.setSDA(8);  // Connect TCS34725 SDA to gpio 10
  Wire.setSCL(9);  // Connect TCS34725 SCL to gpio 11
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
}

void loop()   
{ 

  // Check if there is a new client connection
  getClientInfo();
  
  // Get current time
  currentMicros = micros();

  // The Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    // Update tis for all state machines
    unsigned long cur_time = currentMicros/1e3;   // Just one call to millis()
    fsm1.tis = cur_time - fsm1.tes;

    
    // Read inputs 
    getAngleCMD();

    /*
    // Read tof distance and pre_distance values
    tofGetValue();

    // Read color sensor values
    color = getColorValue();
    
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


    // fsm2 











    // Update the states
    set_state(fsm1, fsm1.new_state);
  

    // Outputs
    if (fsm1.state == 0){
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (fsm1.state == 1){
      digitalWrite(LED_BUILTIN, LOW);
    }

    
    if (mode == MODE1)
    {
      s1.servo.writeMicroseconds(s1.getPWM(angle1));
      s2.servo.writeMicroseconds(s2.getPWM(angle2));
      s3.servo.writeMicroseconds(s3.getPWM(angle3));
      s4.servo.writeMicroseconds(s4.getPWM(angle4));
    }

    
    if (mode == MODE2)
    {
      //goToPos(X , 8, 7);
      s1.servo.writeMicroseconds(s1.getPWM(SERVO1_INIT));
      s2.servo.writeMicroseconds(s2.getPWM(SERVO2_INIT));
      s3.servo.writeMicroseconds(s3.getPWM(SERVO3_INIT));
      s4.servo.writeMicroseconds(s4.getPWM(SERVO4_INIT));
    }

    if (mode == MODE3)
    {
      //goToPos(X , 8, 6);  
    }

    
    if (mode == MODE4)
    {
      //goToPos(X , 8, 0);  
    }
    

    
    //s1.servo.writeMicroseconds(s1.getPWM(angle1));
    //s2.servo.writeMicroseconds(s2.getPWM(angle2));
    //s3.servo.writeMicroseconds(s3.getPWM(angle3));
    //s4.servo.writeMicroseconds(s4.getPWM(angle4));


    // Prints for debug


    // Distance value
    Serial.print(" Dist: ");
    Serial.print(distance, 3);
    Serial.print("  ");

    // Color value
    Serial.print(" Color: ");
    Serial.print(color);
    Serial.print("  ");

    Serial.print(" Mode: ");
    Serial.print(modeToString(mode));
    Serial.print("  ");

    // Fsm1 state
    Serial.print(" Fsm1: ");
    Serial.print(fsm1.state);
    Serial.print("  ");

    Serial.print(" Angle1: " + String(angle1));
    Serial.print(" Angle2: " + String(angle2));
    Serial.print(" Angle3: " + String(angle3));
    Serial.print(" Angle4: " + String(angle4));

    /*
    Serial.print(" X: " + String(X));
    Serial.print(" Y: " + String(Y));
    Serial.print(" Z: " + String(Z));
    */

    /* 
    Serial.print(" Fsm1.tis: ");
    Serial.print(fsm1.tis);
    Serial.print("  ");
    */

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


// Function to get client connections with new information
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

            if (header.indexOf("GET /Mode1") >= 0) {
              //Serial.println("Mode1 selected");
              mode = MODE1;
            }

            if (header.indexOf("GET /Mode2") >= 0) {
              //Serial.println("Mode2 selected");
              mode = MODE2;
            }

            if (header.indexOf("GET /Mode3") >= 0) {
              //Serial.println("Mode3 selected");
              mode = MODE3;
            }    

            if (header.indexOf("GET /Mode4") >= 0) {
              //Serial.println("Mode4 selected");
              mode = MODE4;
            }    

            // Set web page style
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #F23A3A;}</style></head>");
            
            // Web Page title
            client.println("<body><h1>ACE Robot Mode Control</h1>");
            // Web mode state
            client.println("<p>Current Robot mode is " + modeToString(mode) + "</p>");

            if (mode == MODE1) {
              client.println("<p><a href=\"/Mode1\"><button class=\"button\">Mode 1</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode1\"><button class=\"button button2\">Mode 1</button></a></p>");
            }

            if (mode == MODE2) {
              client.println("<p><a href=\"/Mode2\"><button class=\"button\">Mode 2</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode2\"><button class=\"button button2\">Mode 2</button></a></p>");
            }

            if (mode == MODE3) {
              client.println("<p><a href=\"/Mode3\"><button class=\"button\">Mode 3</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode3\"><button class=\"button button2\">Mode 3</button></a></p>");
            }

            if (mode == MODE4) {
              client.println("<p><a href=\"/Mode4\"><button class=\"button\">Mode 4</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode4\"><button class=\"button button2\">Mode 4</button></a></p>");
            }



            if (header.indexOf("GET /number1?value=") >= 0) {
              // Update number value for angle1
              angle1 = header.substring(header.indexOf("/number1?value=") + 15, header.indexOf(" ", header.indexOf("/number1?value="))).toInt();
            }

            if (header.indexOf("GET /number2?value=") >= 0) {
              // Update number value for angle2
              angle2 = header.substring(header.indexOf("/number2?value=") + 15, header.indexOf(" ", header.indexOf("/number2?value="))).toInt();
            }

            if (header.indexOf("GET /number3?value=") >= 0) {
              // Update number value for angle3
              angle3 = header.substring(header.indexOf("/number3?value=") + 15, header.indexOf(" ", header.indexOf("/number3?value="))).toInt();
            }

            if (header.indexOf("GET /number4?value=") >= 0) {
              // Update number value for angle4
              angle4 = header.substring(header.indexOf("/number4?value=") + 15, header.indexOf(" ", header.indexOf("/number4?value="))).toInt();
            }

            // Web mode state for angle1
            client.println("<p>Current Angle1 value is " + String(angle1) + "</p>");
            client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle1) + "\" id=\"myNumber1\"></p>");
            client.println("<script>");
            client.println("var number = document.getElementById('myNumber1');");
            client.println("number.onchange = function() {");
            client.println("  window.location.href = '/number1?value=' + this.value;");
            client.println("}");
            client.println("</script>");

            // Web mode state for angle2
            client.println("<p>Current Angle2 value is " + String(angle2) + "</p>");
            client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle2) + "\" id=\"myNumber2\"></p>");
            client.println("<script>");
            client.println("var number = document.getElementById('myNumber2');");
            client.println("number.onchange = function() {");
            client.println("  window.location.href = '/number2?value=' + this.value;");
            client.println("}");
            client.println("</script>");

            // Web mode state for angle3
            client.println("<p>Current Angle3 value is " + String(angle3) + "</p>");
            client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle3) + "\" id=\"myNumber3\"></p>");
            client.println("<script>");
            client.println("var number = document.getElementById('myNumber3');");
            client.println("number.onchange = function() {");
            client.println("  window.location.href = '/number3?value=' + this.value;");
            client.println("}");
            client.println("</script>");

            // Web mode state for angle4
            client.println("<p>Current Angle4 value is " + String(angle4) + "</p>");
            client.println("<p><input type=\"number\" min=\"0\" max=\"180\" value=\"" + String(angle4) + "\" id=\"myNumber4\"></p>");
            client.println("<script>");
            client.println("var number = document.getElementById('myNumber4');");
            client.println("number.onchange = function() {");
            client.println("  window.location.href = '/number4?value=' + this.value;");
            client.println("}");
            client.println("</script>");


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
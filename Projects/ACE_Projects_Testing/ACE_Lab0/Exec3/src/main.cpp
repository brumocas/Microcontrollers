#include <Arduino.h>

#define LED1_pin 25
#define LED2_pin 7

#define S1_pin 2
#define S2_pin 3

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Input variables
uint8_t S1, prevS1;
uint8_t S2, prevS2;

#define L 8

// Output variables
uint8_t LED_1, LED_2;

// Our finite state machines
fsm_t fsm1, fsm2;

unsigned long interval, last_cycle;
unsigned long loop_micros;
uint16_t blink_period;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

float anglejoint1_deg = 0;
float anglejoint2_deg = 0;
int angle3 = 0;
int angle4 = 0;


void setup() 
{


  // Start the serial port with 115200 baudrate
  Serial.begin(115200);


  interval = 40;
}

void loop() 
{
   
    // To measure the time between loop() calls
    unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      

    float y = 8;
    float z = 7;
      
    
    float o2 = -2 * std::atan(std::sqrt( ((2 * L) * (2 * L) / (y * y + z * z)) - 1));
    float o1 = std::atan2(z, y) - std::atan2(L * std::sin(o2), L * (1 + std::cos(o2))) - M_PI / 2;

    o1 = o1 * 180 / M_PI;
    o2 = o2 * 180 / M_PI;

    Serial.print(" o1: "  +  String(o1));
    Serial.print(" o2: "  +  String(o1));
    Serial.print(" Angle3: " + String(anglejoint1_deg));
    Serial.print(" Angle4: " + String(anglejoint2_deg));


    anglejoint1_deg = 90 - o1;
    anglejoint2_deg = o2 + 270;

    angle3 = int( anglejoint1_deg);
    angle4 = int(anglejoint2_deg);

    Serial.println();

    }
    
}


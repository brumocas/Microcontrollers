#include <Servo.h>
#include <ServosInfo.h>

unsigned long interval ;
unsigned long currentMicros, previousMicros;
int loop_count;
int angle1, angle2, angle3, angle4;

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
#define l1 8.3
#define l2 8
#define d2 5.5

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

  // Go to position
 
  
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


void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  

  s1.servo.attach(SERVO1_PIN, 500, 2500);
  s2.servo.attach(SERVO2_PIN, 500, 2500);
  s3.servo.attach(SERVO3_PIN, 500, 2500);
  s4.servo.attach(SERVO4_PIN, 500, 2500);

  // Set init positions
  s1.servo.writeMicroseconds(s1.getPWM(SERVO1_INIT));
  angle1 = SERVO1_INIT;
  s2.servo.writeMicroseconds(s1.getPWM(SERVO2_INIT));
  angle2 = SERVO2_INIT;
  s3.servo.writeMicroseconds(s1.getPWM(SERVO3_INIT));
  angle3 = SERVO3_INIT;
  s4.servo.writeMicroseconds(s4.getPWM(SERVO4_INIT));
  angle4 = SERVO4_INIT;

  // PCT can be edit here in microseconds
  interval = 40 * 1000;

  // Set angle to move
  state = Servo1;

}
  
void loop() {
  
  currentMicros = micros();

  getAngleCMD();

  


  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    
    s1.servo.writeMicroseconds(s1.getPWM(angle1));
    s2.servo.writeMicroseconds(s2.getPWM(angle2));
    s3.servo.writeMicroseconds(s3.getPWM(angle3));
    s4.servo.writeMicroseconds(s4.getPWM(angle4));
    

    digitalWrite(25, !digitalRead(25));

    // Debug output
    Serial.println(" Servo angle1: " + String(angle1) + " Servo angle2: " + String(angle2)+ " Servo angle3: " + String(angle3) + " Servo angle4: " + String(angle4));  
    digitalWrite(LED, !digitalRead(LED));
  }
}



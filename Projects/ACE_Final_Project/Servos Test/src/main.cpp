#include <Servo.h>
#include <ServosInfo.h>

unsigned long interval ;
unsigned long currentMicros, previousMicros;
int loop_count;
int angle ;

// Pins 
#define LED 25
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 7

// Init angles
#define SERVO1_INIT 30
#define SERVO2_INIT 90
#define SERVO3_INIT 0
#define SERVO4_INIT 10

  void getAngleCMD(){

      uint8_t b;
  if (Serial.available() > 0) {
    b = Serial.read();

    if (b == '-') {
      angle -= 10;
    } else if (b == '+') {
      angle += 10;
    } 

    if (angle >= 180) angle = 180;
    else if (angle <= 0) angle = 0;

  }
}


void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);

  //s1.servo.attach(SERVO1_PIN, 500, 2500);
  //s2.servo.attach(SERVO2_PIN, 500, 2500);
  //s3.servo.attach(SERVO3_PIN, 500, 2500);
  s4.servo.attach(SERVO4_PIN, 500, 2500);

  // Set init positions
  //s1.servo.writeMicroseconds(s1.getPWM(SERVO1_INIT));
  //s2.servo.writeMicroseconds(s1.getPWM(SERVO2_INIT));
  angle = SERVO4_INIT;
  //s3.servo.writeMicroseconds(s1.getPWM(SERVO3_INIT));
  s4.servo.writeMicroseconds(s4.getPWM(SERVO4_INIT));

  // PCT can be edit here in microseconds
  interval = 40 * 1000;

}

void loop() {
  currentMicros = micros();

  getAngleCMD();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    //s3.servo.writeMicroseconds(s3.getPWM(angle));
    s4.servo.writeMicroseconds(s4.getPWM(angle));
    Serial.println("  Servo angle: " + String(angle));  // Debug output
    digitalWrite(LED, !digitalRead(LED));
  }
}



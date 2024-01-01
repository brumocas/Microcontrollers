#include <Servo.h>
#include <ServosInfo.h>


#define SERVO1_PIN 7

float i;
float angle;

void calibrateServo(ServoInfo servo){
  i = 0;
  angle = -90;

  delay(2500);

  for (int k = 0; k < TIME_MAX; k++)
  {
    servo.servo.writeMicroseconds(490 + i);  
    
    servo.time[k] = 500 + i;
    servo.angle[k] = angle;
    angle = map(500 + i, 500, 2500, -90 , 90);
    i+=TIME_STEP;
    delay(100);
  }

  for (int i = 0; i < TIME_MAX; i++)
  {
    Serial.print(servo.time[i]);
    Serial.print(", ");
    Serial.println(servo.angle[i]);
  }

  Serial.println();

  for (int i = 0; i < TIME_MAX; i++)
  {
    Serial.print(servo.angle[i]);
    Serial.print(", ");
  }

}

void setup() {
  Serial.begin(9600);
  servo.servo.attach(SERVO1_PIN, 500, 2500);  // Attach the servo to pin 9 

  //calibrateServo(servo);
}
void loop() {


  
  for (int i = 0; i < 2000; i++)
  {
    servo.servo.writeMicroseconds(490 + i);
    Serial.println(490 + i);
    delay(100);
  }


  
  
}

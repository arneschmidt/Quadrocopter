

#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define MOTOR1_PIN 6
#define MOTOR2_PIN 7
#define MOTOR3_PIN 8
#define MOTOR4_PIN 9

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

byte byteRead;

void setup() {
  Serial.begin(9600);
  Serial.println("Wait for motor value input");
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);

}

void loop() {  
  // Wait for input
  while (!Serial.available());
  int value = Serial.parseInt();
  if(value > 2000 || value < 1000)
  {
    Serial.println("value not in range [1000,2000]");
  }
  else
  {
  Serial.println("Motor Value:");
  Serial.println(value);
  
    // Send min output
  motor1.writeMicroseconds(value);
  motor2.writeMicroseconds(value);
  motor3.writeMicroseconds(value);
  motor4.writeMicroseconds(value);
  } 

}

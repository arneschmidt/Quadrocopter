

#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

#define MOTOR_CAL 7




#define MOTOR1_PIN 6 //BL
#define MOTOR2_PIN 7  //FL  
#define MOTOR3_PIN 8  //FR
#define MOTOR4_PIN 9  //BR


Servo motor_cal;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


byte byteRead;

void setup() {
  Serial.begin(9600);
  Serial.println("Calibrate one Motor");
  Serial.print("Calibrate motor PIN D");
  Serial.println(MOTOR_CAL);
  motor_cal.attach(MOTOR_CAL);
  motor_cal.writeMicroseconds(MAX_SIGNAL);
  if(6 != MOTOR_CAL)
  {
    motor1.attach(MOTOR1_PIN);
    motor1.writeMicroseconds(MIN_SIGNAL);
  }
  if(7 != MOTOR_CAL)
  {
    motor2.attach(MOTOR2_PIN);
    motor2.writeMicroseconds(MIN_SIGNAL);
  }
  if(8 != MOTOR_CAL)
  {
    motor3.attach(MOTOR3_PIN);
    motor3.writeMicroseconds(MIN_SIGNAL);
  }
  if(9 != MOTOR_CAL)
  {
    motor4.attach(MOTOR4_PIN);
    motor4.writeMicroseconds(MIN_SIGNAL); 
  }

}

void loop() {  

  Serial.println("Now writing maximum output.");
  motor_cal.writeMicroseconds(MAX_SIGNAL);
  
  while (!Serial.available());
  Serial.read();
  
    // Send min output
  Serial.println("Sending minimum output");
  motor_cal.writeMicroseconds(MIN_SIGNAL);

  
  // Wait for input
  while (!Serial.available());
  Serial.read();

}

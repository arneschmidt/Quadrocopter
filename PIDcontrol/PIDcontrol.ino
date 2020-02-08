
#include <Wire.h>
#include <Servo.h>
#include "/home/arne/Dokumente/Quadrocopter/readIMU/LSM9DS0.h"

// Parameter
#define DT  0.02         // Loop time

//IMU Filter:
#define AA  0.2       // complementary filter constant
#define BB 0.5
#define G_GAIN 0.070    // [deg/s/LSB]
#define PitchOffset 8.0
#define RollOffset -2.5

/*
#define Q  2
#define R  100
#define Q_vel 2
#define R_vel 2
*/
float Q = 1;
float R = 1000;
float Q_vel = 1;
float R_vel = 100;
float p[] = {1,1,1,1};


float xGyroBias = 0;
float yGyroBias = 0;
float zGyroBias = 0;


//PID
double P =  2.0;
double I = 0.0;
double D = 1.0;

//Control
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_FL_PIN 7
#define MOTOR_FR_PIN 8
#define MOTOR_BL_PIN 6
#define MOTOR_BR_PIN 9

//RC Control
#define thrustCH 4
#define pitchCH 3
#define yawCH 5
#define rollCH 2
#define switchACH 11
#define switchBCH 12

int RC_PitchOffset = 0;
int RC_YawOffset = 0;
int RC_RollOffset = 0;


// Motor init
Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

// IMU init
byte buff[6];
int accRaw[3];
int magRaw[3];
int gyrRaw[3];
float rate_gyr_y = 0.0;   // [deg/s]
float rate_gyr_x = 0.0;    // [deg/s]
float rate_gyr_z = 0.0;     // [deg/s]
float rate_gyr_x_old = 0.0;
float rate_gyr_y_old = 0.0;
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float AccYangle = 0.0;
float AccXangle = 0.0;
float X = 0.0;
float Y = 0.0;
float X_vel = 0.0;
float Y_vel = 0.0;
float X_des = 0.0;
float Y_des = 0.0;
float X_err = 0.0;
float Y_err = 0.0;
float Z = 0;

int thrust = 0;
boolean motorOff = false;

unsigned long startTime;

void inputPIDconst()
{
 Serial.print("P?");
 while (!Serial.available());
 P = Serial.parseFloat();
 Serial.println(P);
 Serial.print("I?");
 while (!Serial.available());
 I = Serial.parseFloat();
 Serial.println(I);
 Serial.print("D?");
 while (!Serial.available());
 D = Serial.parseFloat();
 Serial.println(D);
}

void inputKalmanFilter()
{
    
 Serial.print("system noise q?");
 while (!Serial.available());
 Q = Serial.parseFloat();
 Serial.println(Q);
 Serial.print("mes noise r?");
 while (!Serial.available());
 R = Serial.parseFloat();
 Serial.println(R);
  Serial.print("system noise q_vel?");
 while (!Serial.available());
 Q_vel = Serial.parseFloat();
 Serial.println(Q_vel);
 Serial.print("mes noise r_vel?");
 while (!Serial.available());
 R_vel = Serial.parseFloat();
 Serial.println(R_vel); 

}

void kalmanFilter(float & x, float x_vel, float z, float & pp, float q, float r)
{
  float xPred = x + x_vel*DT;
  float pPred = pp + q;
  float K = pPred/(pPred + r);
  x = xPred + K*(z - xPred);
  pp = (1-K)*pPred;
}

void getRCInput()
{
  float scaleFactor = 0.05;
  float frequencyOffset = 2000.0;
  int waitingTime = 20000;
  int switchA = round(pulseIn(switchACH, HIGH,waitingTime) > 2000);
  int thrustInt =  pulseIn(thrustCH,HIGH,waitingTime);
  int pitchInt =  pulseIn(pitchCH,HIGH,waitingTime) - RC_PitchOffset;
  int yawInt =  pulseIn(yawCH,HIGH,waitingTime) - RC_YawOffset;
  int rollInt =  pulseIn(rollCH,HIGH,waitingTime) - RC_RollOffset;
  if(thrustInt != 0)
  {
    thrust = (thrustInt - 1600  + switchA*400)*AA + thrust*(1-AA);
    Y_des = (-(pitchInt)*scaleFactor)*AA + Y_des*(1-AA);
    X_des = ((rollInt)*scaleFactor)*AA + X_des*(1-AA);
    Z = round(((yawInt)*scaleFactor)*AA + Z*(1-AA));//*scaleFactor);
  }
  else
  {
    if(thrust > 0)
      thrust -= 10;
    Y_des *= 0.5;
    X_des *= 0.5;
    Z *= 0.5;   
  }
  motorOff = (pulseIn(switchBCH, HIGH,waitingTime) > 2000);
}
void printRCInput()
{
  Serial.print("thrust: ");
  Serial.print(thrust);
  Serial.print("\t X_des: ");
  Serial.print(X_des);
  Serial.print("\t Y_des: ");
  Serial.print(Y_des);
  Serial.print("\t Z: ");
  Serial.print(Z);
  Serial.print("\t motorOff ");
  Serial.println(motorOff);
   
}

void printRawData()
 {  
  Serial.print("#AccX\t");
  Serial.print(AccXangle);
  Serial.print("\t###  AccY  ");
  Serial.print(AccYangle);
  
  Serial.print("\t rate_gyr_x");
  Serial.print(rate_gyr_x);
  Serial.print("\t rate_gyr_y");
  Serial.print(rate_gyr_y);
  /*
  Serial.print(" #GyrX\t");
  Serial.print(gyroXangle);
  Serial.print(" #GyrY  \t");
  Serial.print(gyroYangle);
  Serial.print(" #GyrZ\t");
  Serial.print(gyroZangle);
  */
 }
 void printFilteredData()
{
  Serial.print(" #X \t");
  Serial.print(X);
  Serial.print(" #Y \t");
  Serial.print(Y);
  
  //Serial.print(" #heading ");
  //Serial.print(heading); 
  Serial.print(" #X_vel \t");
  Serial.print(X_vel);
  Serial.print(" #Y_vel \t");
  Serial.println(Y_vel);
  
}
  
void calcAngles() {
  //Read the measurements from the sensors
  readFrom(ACC_ADDRESS, 0x80 | OUT_X_L_A, 6, buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));
  readFrom(MAG_ADDRESS, 0x80 | OUT_X_L_M, 6, buff);
  magRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  magRaw[1] = (int)(buff[2] | (buff[3] << 8));
  magRaw[2] = (int)(buff[4] | (buff[5] << 8));
  readFrom(GYR_ADDRESS, 0x80 | OUT_X_L_G, 6, buff);
  gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
  gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));


  //Convert Gyro raw to degrees per second
  rate_gyr_x = (float) - gyrRaw[0] * G_GAIN - xGyroBias;
  rate_gyr_y = (float) gyrRaw[1]  * G_GAIN - yGyroBias;
  rate_gyr_z = (float) gyrRaw[2]  * G_GAIN - zGyroBias;

  //Calculate the angles from the gyro
  /*gyroXangle+=rate_gyr_x*DT;
  gyroYangle+=rate_gyr_y*DT;
  gyroZangle+=rate_gyr_z*DT;
  */
  //Convert Accelerometer values to degrees
  AccXangle = (float) (atan2(accRaw[2],accRaw[1]) + M_PI/2)*RAD_TO_DEG - 180 - RollOffset; //+M_PI
  AccYangle = (float) (atan2(accRaw[2],accRaw[0]) + M_PI/2)*RAD_TO_DEG + 180 - PitchOffset; //+M_PI

  if(AccXangle > 180)   AccXangle -= 360;
  if(AccYangle > 180)   AccYangle -= 360;


  //Complementary filter used to combine the accelerometer and gyro values.
/*  X_vel = -X;
  Y_vel = -Y;
  rate_gyr_x = BB * rate_gyr_x + (1 - BB) * rate_gyr_x_old;
  rate_gyr_y = BB * rate_gyr_y + (1 - BB) * rate_gyr_y_old; 
  rate_gyr_x_old = rate_gyr_x;
  rate_gyr_y_old = rate_gyr_y;

  X=AA*(X-rate_gyr_x*DT) +(1 - AA) * AccXangle;
  Y=AA*(Y+rate_gyr_y*DT) +(1 - AA) * AccYangle;
    X_vel += X;
  Y_vel += Y;
  X_vel /= DT;
  Y_vel /= DT;
*/
  kalmanFilter(X_vel, 0, rate_gyr_x, p[2], Q_vel, R_vel);
  kalmanFilter(Y_vel, 0, rate_gyr_y, p[3], Q_vel, R_vel);
  kalmanFilter(X, X_vel, AccXangle, p[0], Q, R);
  kalmanFilter(Y, Y_vel, AccYangle, p[1], Q, R);

  //Compute heading  
  float heading = 180 * atan2(magRaw[1],magRaw[0])/M_PI;
  
  //Convert heading to 0 - 360
  if(heading < 0) heading += 360;
}

void calibrateGyro()
{
  Serial.println("Calibration..");
  int n = 200;
  float gyroXSum = 0;
  float gyroYSum = 0;
  float gyroZSum = 0;
  
  for(int i = 0; i < n; i++)
  {
    startTime = millis();
    calcAngles();
    
    gyroXSum += rate_gyr_x;
    gyroYSum += rate_gyr_y;
    gyroZSum += rate_gyr_z;
    
    while(millis() - startTime < (DT*1000))
    {
     delay(1);
    }
  }
  
  xGyroBias = gyroXSum / float(n);
  yGyroBias = gyroYSum / float(n);  
  zGyroBias = gyroZSum / float(n);
  
  Serial.print("xGyroBias:"); Serial.print(xGyroBias); Serial.print(" yGyroBias: "); Serial.println(yGyroBias);
}

void calibrateRC()
{
  Serial.println(" Calibrate RC..");
  int n = 100;
  float RC_PitchOffsetSum = 0;
  float RC_RollOffsetSum = 0;
  float RC_YawOffsetSum = 0;
  
  int waitingTime = 20000;
 for(int i = 0; i < n; i++)
  {
    startTime = millis();
     int pitchInt =  pulseIn(pitchCH,HIGH,waitingTime);
     int yawInt =  pulseIn(yawCH,HIGH,waitingTime);
     int rollInt =  pulseIn(rollCH,HIGH,waitingTime);
    
    if(pitchInt != 0 && yawInt != 0 && rollInt != 0)
    {
      RC_PitchOffsetSum += pitchInt;
      RC_YawOffsetSum += yawInt;
      RC_RollOffsetSum += rollInt;
    }
    
    while(millis() - startTime < (DT*1000))
    {
     delay(1);
    }
  }
  
  RC_PitchOffset = RC_PitchOffsetSum / n;
  RC_YawOffset = RC_YawOffsetSum / n;
  RC_RollOffset = RC_RollOffsetSum / n;
  
  Serial.print(" RC_PitchOffset: ");
  Serial.print(RC_PitchOffset);
  Serial.print(" RC_YawOffset ");
  Serial.print(RC_YawOffset);
  Serial.print(" RC_RollOffset ");
  Serial.println(RC_RollOffset);

}

void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device (initiate again)
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

int pidControl(float & a, float & a_des, float & a_err, float & a_vel)
{
  float error = a_des - a;
  a_err += error;
  return (int) P*(error) + I*(a_err - error) - D*a_vel;
}

void setMotorValue(Servo & motor, int value)
{
  value += 1000;
  if(value > MAX_SIGNAL) value = MAX_SIGNAL;
  else if(value < MIN_SIGNAL) value = MIN_SIGNAL;
  motor.writeMicroseconds(value);
  //println(value);
}
void getInput()
{
  if(Serial.available() > 0){
  int input = Serial.read();
  if(input == 49)
  { motorOff = true;
  thrust = 0; }
  else if(input == 50) thrust -=10;
  else if(input == 51) thrust +=10;
  else if(input == 52) thrust -=100;
  else if(input == 53) thrust +=100;
  else if(input == 57) motorOff = false;
  Serial.println(thrust);
  }
}

void setup() {
  Serial.begin(9600);  // start serial for output
  //---------RC SETUP------------------------------------------------------
  pinMode(thrustCH, INPUT);
  pinMode(pitchCH,INPUT);
  pinMode(yawCH,INPUT);
  pinMode(rollCH,INPUT);
  pinMode(switchACH,INPUT);
  pinMode(switchBCH,INPUT);
  //---------IMU SETUP------------------------------------------------------
  Wire.begin();        // join i2c bus (address optional for master)
  //Enable accelerometer
  writeTo(ACC_ADDRESS,CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuos update,  100Hz data rate
  writeTo(ACC_ADDRESS,CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

  //Enable the magnetometer
  writeTo(MAG_ADDRESS,CTRL_REG5_XM, 0b11110000);   // Temp enable, M data rate = 50Hz
  writeTo(MAG_ADDRESS,CTRL_REG6_XM, 0b01100000);   // +/-12gauss
  writeTo(MAG_ADDRESS,CTRL_REG7_XM, 0b00000000);   // Continuous-conversion mode

  // Enable Gyro
  writeTo(GYR_ADDRESS, CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
  writeTo(GYR_ADDRESS, CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
  
  //---------MOTOR SETUP----------------------------------------------------
  motorFL.attach(MOTOR_FL_PIN);
  motorFR.attach(MOTOR_FR_PIN);
  motorBL.attach(MOTOR_BL_PIN);
  motorBR.attach(MOTOR_BR_PIN);
  motorFL.writeMicroseconds(MIN_SIGNAL);
  motorFR.writeMicroseconds(MIN_SIGNAL);
  motorBL.writeMicroseconds(MIN_SIGNAL);
  motorBR.writeMicroseconds(MIN_SIGNAL);
  
  calibrateGyro();
  calibrateRC();
  //inputPIDconst();
  //inputKalmanFilter();
}

void loop(){
  startTime = millis();
  calcAngles();
  getRCInput();
  //getInput();
  printRCInput();
  // if X_control is positive, pitch must be higher
  int X_control = pidControl(X, X_des, X_err, X_vel);
  int Y_control = pidControl(Y, Y_des, Y_err, Y_vel);
  //Serial.print("X_control");
  //Serial.print(X_control);
  //Serial.print(" \t Y_control");
  //Serial.println(Y_control);
  //printRawData();
  //printFilteredData();
  
  //Serial.print("thrust:");
  //Serial.println(thrust);
  //Serial.print("motorOff:");
  //Serial.println(motorOff);
  
  if(thrust > 0 && motorOff == false){
  setMotorValue(motorFL, thrust + X_control + Y_control - Z);
  setMotorValue(motorFR, thrust - X_control + Y_control + Z);
  setMotorValue(motorBL, thrust + X_control - Y_control + Z);
  setMotorValue(motorBR, thrust - X_control - Y_control - Z);
  }
  else if(motorOff == true)
  {
    setMotorValue(motorFL, 0);
    setMotorValue(motorFR, 0);
    setMotorValue(motorBL, 0);
    setMotorValue(motorBR, 0);
  }
  
  
    //Each loop should be at least 20ms.
  while(millis() - startTime < (DT*1000))
  {
    delay(1);
  }
 
  //Serial.println(millis()- startTime);
}





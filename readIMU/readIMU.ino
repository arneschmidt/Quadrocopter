
#include <Wire.h>
#include "/home/arne/Dokumente/Quadrocopter/readIMU/LSM9DS0.h"

#define DT  0.02          // Loop time
#define AA  0.9       // complementary filter constant
#define G_GAIN 0.070    // [deg/s/LSB]
#define PitchOffset 1.5 
#define RollOffset -4

byte buff[6];
int accRaw[3];
int magRaw[3];
int gyrRaw[3];
float rate_gyr_y = 0.0;   // [deg/s]
float rate_gyr_x = 0.0;    // [deg/s]
float rate_gyr_z = 0.0;     // [deg/s]
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float AccYangle = 0.0;
float AccXangle = 0.0;
float CFangleX = 0.0;
float CFangleY = 0.0;
float CFoldX = 0.0;
float CFoldY = 0.0;
float CFvelX = 0.0;
float CFvelY = 0.0;


unsigned long startTime;

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9800);  // start serial for output
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
}
float pid_pitch(float x_des){}
float pid_roll(float y_des){}

void loop() {
 startTime = millis();

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
  rate_gyr_x = (float) gyrRaw[0] * G_GAIN;
  rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
  rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;

  //Calculate the angles from the gyro
  gyroXangle+=rate_gyr_x*DT;
  gyroYangle+=rate_gyr_y*DT;
  gyroZangle+=rate_gyr_z*DT;

  //Convert Accelerometer values to degrees
  AccXangle = (float) (atan2(accRaw[2],accRaw[1]) + M_PI/2)*RAD_TO_DEG; //+M_PI
  AccYangle = (float) (atan2(accRaw[2],accRaw[0]) + M_PI/2)*RAD_TO_DEG; //+M_PI

  if(AccXangle > 180)   AccXangle -= 360;
  if(AccYangle > 180)   AccYangle -= 360;


  //Complementary filter used to combine the accelerometer and gyro values.
  CFoldX = CFangleX;
  CFoldY = CFangleY;
  CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
  CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;
  CFvelX = CFangleX - CFoldX;
  CFvelY = CFangleY - CFoldY;


  //Compute heading  
  float heading = 180 * atan2(magRaw[1],magRaw[0])/M_PI;
  
  //Convert heading to 0 - 360
          if(heading < 0)
            heading += 360;
  /*          
  Serial.print("#AccX\t");
  Serial.print(AccXangle);
  Serial.print("\t###  AccY  ");
  Serial.print(AccYangle);
  
  Serial.print(" #GyrX\t");
  Serial.print(gyroXangle);
  Serial.print(" #GyrY  \t");
  Serial.print(gyroYangle);
  Serial.print(" #GyrZ\t");
  Serial.print(gyroZangle);
  
  Serial.print(" #CFangleX ");
  Serial.print(CFangleX);
  Serial.print(" #CFangleY ");
  Serial.print(CFangleY);
  Serial.print(" #heading ");
  Serial.print(heading); 
  Serial.print(" #CFvelX ");
  Serial.print(CFvelX);
  Serial.print(" #CFvelY ");
  Serial.print(CFvelY);
  */
  Serial.print(" --Loop Time--\t");

  //Each loop should be at least 20ms.
  while(millis() - startTime < (DT*1000))
        {
            delay(1);
        }
  Serial.println( millis()- startTime);
 


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





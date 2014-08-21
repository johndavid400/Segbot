// Testing the Invensense MPU6050 IMU
// modified by JD Warren - 2014
// Sensor from Banggood.com
// http://www.banggood.com/6DOF-MPU-6050-3-Axis-Gyro-With-Accelerometer-Sensor-Module-For-Arduino-p-80862.html
//
// Originally written by Arduino User JohnChi
// august 12, 2014.
// public domain
//
#include<Wire.h>
const int GyroA=0x68;  // I2C address of the MPU-6050
float AcX,AcY,AcZ,Temp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(GyroA);
  Wire.write(0x6B);  // MPU6050_PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  wire();
  read_accel();
  Serial.println("");
  delay(20);
}

void wire(){
  Wire.beginTransmission(GyroA);
  Wire.write(0x3B);  // starting with register 0x3B (MPU6050_ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(GyroA,14,true);  // read a total of 14 registers
}

void read_accel(){
  AcX = Wire.read()*256+Wire.read();
  AcX = map(AcX, -15000, 15000, -90, 90);
  //AcX=Wire.read()*256+Wire.read();  // 0x3B (MPU6050_ACCEL_XOUT_H) & 0x3C (MPU6050_ACCEL_XOUT_L)
  //AcY=Wire.read()*256+Wire.read();  // 0x3D (MPU6050_ACCEL_YOUT_H) & 0x3E (MPU6050_ACCEL_YOUT_L)
  //AcZ=Wire.read()*256+Wire.read();  // 0x3F (MPU6050_ACCEL_ZOUT_H) & 0x40 (MPU6050_ACCEL_ZOUT_L)
  print_accel();
}

void read_gyro(){
  GyX=Wire.read()*256+Wire.read();  // 0x43 (MPU6050_GYRO_XOUT_H) & 0x44 (MPU6050_GYRO_XOUT_L)
  //GyY=Wire.read()*256+Wire.read();  // 0x45 (MPU6050_GYRO_YOUT_H) & 0x46 (MPU6050_GYRO_YOUT_L)
  //GyZ=Wire.read()*256+Wire.read();  // 0x47 (MPU6050_GYRO_ZOUT_H) & 0x48 (MPU6050_GYRO_ZOUT_L)
  print_gyro();
}

void read_temp(){
  Temp=(Wire.read()*256+Wire.read())/340.00+36.53;  // 0x41 (MPU6050_TEMP_OUT_H) & 0x42 (MPU6050_TEMP_OUT_L)
  print_temp();
}

void print_accel(){
  Serial.print("AcX = "); Serial.print(AcX);
  //Serial.print("  AcY = "); Serial.print(AcY);
  //Serial.print("  AcZ = "); Serial.print(AcZ);
}
void print_gyro(){
  Serial.print(" GyX = "); Serial.print(GyX);
  //Serial.print(" GyY = "); Serial.print(GyY);
  //Serial.print(" GyZ = "); Serial.println(GyZ);
}
void print_temp(){
  Serial.print(" Temp = "); Serial.print(Temp);   
}

// www.prototyperobotics.com
// JD Warren 2013
// 
// Arduino Uno
//
// This code is for using the accelerometer and gyroscope sensors sold at Radio Shack
// Gyroscope - Parallax 27911-RT 3-Axis Gyroscope Module
// Accelerometer - Memsic 2125 Dual-axis Accelerometer
// This code is a work-in-progress and will be labeled as version 4 (v4), once completed

// declare output pins for x and y accelerometer
int y_accel = 7;

// raw values for x and y readings
int y_raw = 0;

// adjusted values for x and y readings
int y_adj = 0;

// values for min/max accelerometer readings
int accel_low = 3700;
int accel_high = 6300;

// I2C setup for gyroscope
#include <Wire.h>
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
int Addr = 105;                 // I2C address of gyro
int x;

// end of variable declaration

void setup(){
  Serial.begin(9600);
  Wire.begin();
  // setting up inputs for x and y accelerometer
  pinMode(y_accel, INPUT);
  writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
  writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
  writeI2C(CTRL_REG4, 0x80);    // Set scale (500 deg/sec)
  delay(100);                   // Wait to synchronize 
}

void loop(){
  // read accelerometer values
  read_accel();
  // read gyroscope values
  read_gyroscope();
  // print values
  print_accel();
  // delay
  delay(50);
}

void read_accel(){
  // read the y axis of the accelerometer
  y_raw = pulseIn(y_accel, HIGH);
  y_adj = map(y_raw, accel_low, accel_high, -90, 90);
}

void print_accel(){
  // print y values
  Serial.print("Accel Y: ");
  Serial.print(y_adj);
  Serial.print("     ");
  // end of line
  Serial.println("");
}

void read_gyroscope(){
  // Get new values
  getGyroValues();              
  // In following Dividing by 114 reduces noise
  Serial.print("Gyro X:"); 
  Serial.print(x / 114);
  Serial.print("    ");
}

void getGyroValues () {
  byte MSB, LSB;
  MSB = readI2C(0x29);
  LSB = readI2C(0x28);
  x = ((MSB << 8) | LSB);
}

int readI2C (byte regAddr) {
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);                // Register address to read
  Wire.endTransmission();             // Terminate request
  Wire.requestFrom(Addr, 1);          // Read a byte
  while(!Wire.available()) { 
  };       // Wait for receipt
  return(Wire.read());                // Get result
}

void writeI2C (byte regAddr, byte val) {
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}



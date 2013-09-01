// This code is to get angle readings from the L3G4200 gyroscope from Radio Shack using I2C
// Gyroscope is made by Parallax
// Model: 27911-RT
// Catalog #: 276-119 

// I2C setup for gyroscope
#include <Wire.h>
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
int Addr = 105;                 // I2C address of gyro
float gyro_rate;

void gyro_setup(){
  Wire.begin();
  writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
  writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
  writeI2C(CTRL_REG4, 0x80);    // Set scale (500 deg/sec)
  delay(100);                   // Wait to synchronize  
}

void read_gyroscope(){
  // Get new values
  getGyroValues();
  // sum gyro angle
  gyro_angle = gyro_angle + (gyro_rate * gyro_scale);
}

void getGyroValues () {
  byte MSB, LSB;
  MSB = readI2C(0x29);
  LSB = readI2C(0x28);
  // for some reason we have to divide the gyro_rate by 114? Thanks Parallax
  gyro_rate = ((MSB << 8) | LSB) / 114;
}

int readI2C (byte regAddr) {
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);                // Register address to read
  Wire.endTransmission();             // Terminate request
  Wire.requestFrom(Addr, 1);          // Read a byte
  while(!Wire.available()) {};        // Wait for receipt
  return(Wire.read());                // Get result
}

void writeI2C (byte regAddr, byte val) {
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}

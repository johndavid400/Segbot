// This code is for using the accelerometer and gyroscope sensors sold at Radio Shack
// Gyroscope - Parallax 27911-RT 3-Axis Gyroscope Module
// Accelerometer - Memsic 2125 Dual-axis Accelerometer

// This code is a work-in-progress and will be labeled as version 4 (v4), once completed

boolean read_x = false;
boolean read_y = true;

// declare output pins for x and y accelerometer
int x_accel = 8;
int y_accel = 7;

// raw values for x and y readings
int x_raw = 0;
int y_raw = 0;

// adjusted values for x and y readings
int x_adj = 0;
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
int x, y, z;

// end of variable declaration

void setup(){
  Serial.begin(9600);
  Wire.begin();
  // setting up inputs for x and y accelerometer
  pinMode(x_accel, INPUT);
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
  if(read_x){
    // read the x axis of the accelerometer
    x_raw = pulseIn(x_accel, HIGH);
    x_adj = map(x_raw, accel_low, accel_high, -90, 90);
  }
  if(read_y){
    // read the y axis of the accelerometer
    y_raw = pulseIn(y_accel, HIGH);
    y_adj = map(y_raw, accel_low, accel_high, -90, 90);
  }
}

void print_accel(){
  if(read_x){
    // print x values
    Serial.print("X: ");
    Serial.print(x_adj);
    Serial.print("     "); 
  }
  if(read_y){
    // print y values
    Serial.print("Y: ");
    Serial.print(y_adj);
    Serial.print("     ");
  }
  // end of line
  Serial.println("");
}

void read_gyroscope(){
  // Get new values
  getGyroValues();              
  // In following Dividing by 114 reduces noise
  Serial.print("Raw X:");  Serial.print(x / 114);  
  Serial.print(" Raw Y:"); Serial.print(y / 114);
  Serial.print(" Raw Z:"); Serial.print(z / 114);
}

void getGyroValues () {
  byte MSB, LSB;

  MSB = readI2C(0x29);
  LSB = readI2C(0x28);
  x = ((MSB << 8) | LSB);

  MSB = readI2C(0x2B);
  LSB = readI2C(0x2A);
  y = ((MSB << 8) | LSB);

  MSB = readI2C(0x2D);
  LSB = readI2C(0x2C);
  z = ((MSB << 8) | LSB);
}

int readI2C (byte regAddr) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);                // Register address to read
    Wire.endTransmission();             // Terminate request
    Wire.requestFrom(Addr, 1);          // Read a byte
    while(!Wire.available()) { };       // Wait for receipt
    return(Wire.read());                // Get result
}

void writeI2C (byte regAddr, byte val) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);
    Wire.write(val);
    Wire.endTransmission();
}


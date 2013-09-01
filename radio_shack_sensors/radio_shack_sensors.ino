// www.prototyperobotics.com  -  JD Warren 2013
// Arduino Uno
// This code is for using the accelerometer and gyroscope sensors sold at Radio Shack
// Gyroscope - Parallax 27911-RT 3-Axis Gyroscope Module
// Accelerometer - Memsic 2125 Dual-axis Accelerometer
// This code is a work-in-progress and will be labeled as version 4 (v4), once completed
//
// Even though I do my best to use the correct axises... every time I end up using one of the wrong ones. This time, the gyroscope incenuated Y but instead I needed X.
// So... I am using the Y axis from the Accelerometer and the X axis from the Gyroscope, as they area aligned. I would change it, but it is already soldered and I am lazy.

// declare input pins for x and y accelerometer
int accel = 7;

// raw values for x and y readings
int accel_raw = 0;

// values for min/max accelerometer readings
int accel_sample;
int accel_n = 100;
int accel_range = 1250;
int accel_low;
int accel_high;

// Floats for angles
float angle = 0.0;
float gyro_angle = 0.0;
float accel_angle = 0.0;

// scale variable for Gyro
float gyro_scale = 0.1;

// set variable weights
float gyro_weight = 0.02;
float accel_weight = 0.98;

// I2C setup for gyroscope
#include <Wire.h>
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
int Addr = 105;                 // I2C address of gyro
int gyro_rate;

// end of variable declaration

void setup(){
  Serial.begin(9600);
  Wire.begin();
  // setting up inputs for x and y accelerometer
  pinMode(accel, INPUT);
  writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
  writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
  writeI2C(CTRL_REG4, 0x80);    // Set scale (500 deg/sec)
  delay(100);                   // Wait to synchronize 
  sample_accel();
  delay(1000);
}

void sample_accel(){
  long accel_total = 0;
  for (int i = 0; i < accel_n; i++){
    read_accel();
    accel_total += accel_raw;
  }
  accel_sample = (accel_total / accel_n) - 100;
  Serial.print("Total: ");
  Serial.println(accel_total);
  Serial.print("AVG: ");
  Serial.println(accel_sample);
  accel_low = accel_sample - accel_range;
  accel_high = accel_sample + accel_range;
  Serial.print("Accel Low:  ");
  Serial.println(accel_low);
  Serial.print("Accel High:  ");
  Serial.println(accel_high);
}

void loop(){
  // read accelerometer values
  read_accel();
  // read gyroscope values
  read_gyroscope();
  // calculate angle
  calculate_angle();
  // print values
  print_stuff();
  // delay
  delay(50);
}

void calculate_angle(){
  angle = (float)(gyro_weight * gyro_angle) + (accel_weight * accel_angle);
}

void read_accel(){
  // read the y axis of the accelerometer
  accel_raw = pulseIn(accel, HIGH);
  accel_angle = map(accel_raw, accel_low, accel_high, -90, 90);
}

void print_stuff(){
    
  // print accel angle
  Serial.print("Accel: ");
  Serial.print(accel_angle);
  Serial.print("     ");
  
  // print gyro angle
  Serial.print("Gyro:"); 
  Serial.print(gyro_angle);
  Serial.print("    ");
  
  // print filtered angle
  Serial.print("Angle:"); 
  Serial.print(angle);
  Serial.print("    ");
  
  // end of line
  Serial.println("");  
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



// found at: http://forum.arduino.cc/index.php/topic,147351.0.html
//
// This is a simple program for testing the STMicroelectrics 3-axis gyroscope sold by Parallax.
//
// If you are new to the sensor you'll find all you need here.  Just rip out what you don't want to save RAM.
//
// If you are using an Arduino Uno R3 like me, connect the SCL line to pin A5.  Connect the SDA line to pin A4.
// I don't know anything about the other boards.  I'm new to Arduino.
//
// The sample code at Parallax is incorrect in a couple of places.  This code follows the application notes from STM.  In
// particular I want to note that I found it is very important to check status register 3 to make sure there is new
// data before trying to request it, or else you will get very noisy data.
//
// You can try modifying NUM_GYRO_SAMPLES and GYRO_SIGMA_MULTIPLE.
// These control the statistical averaging that is used to discard bad values.
// The NUM_GYRO_SAMPLES can be reduced for faster calibration, but accuracy will start to suffer if you go below 50 samples.
// Increase the GYRO_SIGMA_MULTIPLE if you have an application that doesn't need to keep track of small rotations.
// Reduce GYRO_SIGMA_MULTIPLE if small rotations are important and you don't mind more noise in the data.
//
// If you are trying to create a compass from the gyro data then use the heading array I added.  I found it pretty useful.
// If you set the sensor on your desk and only rotate it in yaw it works great.  Turn it upside down and it's a mess, naturally.
// 
// If you are putting the sensor in a robot hoping for dead reckoning it might work, but it may be necessary to recalibrate once 
// in awhile with the robot stationary.  It depends on how long your robot will be driving around and also how bumpy the
// surface is.
//
// Note that if you do not call updateGyro() often enough then you will miss data.  You can check for this by looking at
// the status register's 7th bit.  If it's on you've missed some data.  It's possible to set up an interrupt to catch it.  If you
// miss data then the report from the sensor isn't going to work as well for you.
//
// Use share and enjoy this however you like!
//
// Jim Bourke 2/6/2013 (RCGroups.com)
//
#include <Wire.h>

#define  CTRL_REG1  0x20
#define  CTRL_REG2  0x21
#define  CTRL_REG3  0x22
#define  CTRL_REG4  0x23
#define  CTRL_REG5  0x24
#define  CTRL_REG6  0x25

int gyroI2CAddr=105;

int gyroRaw[3];                         // raw sensor data, each axis, pretty useless really but here it is.
double gyroDPS[3];                      // gyro degrees per second, each axis

float heading[3]={0.0f};                // heading[x], heading[y], heading [z]

int gyroZeroRate[3];                    // Calibration data.  Needed because the sensor does center at zero, but rather always reports a small amount of rotation on each axis.
int gyroThreshold[3];                   // Raw rate change data less than the statistically derived threshold is discarded.

#define  NUM_GYRO_SAMPLES  50           // As recommended in STMicro doc
#define  GYRO_SIGMA_MULTIPLE  3         // As recommended 

float dpsPerDigit=.00875f;              // for conversion to degrees per second

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setupGyro();
  calibrateGyro();
}

void loop() {
  updateGyroValues();
  updateHeadings();
  //testCalibration();

  printDPS();
  Serial.print("   -->   ");
  printHeadings();
  Serial.println();
}

void printDPS()
{
  Serial.print("DPS X: ");
  Serial.print(gyroDPS[0]);
  Serial.print("  Y: ");
  Serial.print(gyroDPS[1]);
  Serial.print("  Z: ");
  Serial.print(gyroDPS[2]);
}

void printHeadings()
{
  Serial.print("Heading X: ");
  Serial.print(heading[0]);
  Serial.print("  Y: ");
  Serial.print(heading[1]);
  Serial.print("  Z: ");
  Serial.print(heading[2]);
}

void updateHeadings()
{
    
  float deltaT=getDeltaTMicros();

  for (int j=0;j<3;j++)
    heading[j] -= (gyroDPS[j]*deltaT)/1000000.0f;
}

// this simply returns the elapsed time since the last call.
unsigned long getDeltaTMicros()
{
  static unsigned long lastTime=0;
  
  unsigned long currentTime=micros();
  
  unsigned long deltaT=currentTime-lastTime;
  if (deltaT < 0.0)
     deltaT=currentTime+(4294967295-lastTime);
   
  lastTime=currentTime;
  
  return deltaT;
}

// I called this from the loop function to see what the right values were for the calibration constants.
// If you are trying to reduce the amount of time needed for calibration just try not to go so low that consecutive
// calibration calls give you completely unrelated data.  Some sensors are probably better than others.
void testCalibration()
{
  calibrateGyro();
  for (int j=0;j<3;j++)
  {
    Serial.print(gyroZeroRate[j]);
    Serial.print("  ");
    Serial.print(gyroThreshold[j]);
    Serial.print("  ");  
  }
  Serial.println();
  return; 
}

// The settings here will suffice unless you want to use the interrupt feature.
void setupGyro()
{
  gyroWriteI2C(CTRL_REG1, 0x1F);        // Turn on all axes, disable power down
  gyroWriteI2C(CTRL_REG3, 0x08);        // Enable control ready signal
  setGyroSensitivity500();

  delay(100);
}

// Call this at start up.  It's critical that your device is completely stationary during calibration.
// The sensor needs recalibration after lots of movement, lots of idle time, temperature changes, etc, so try to work that in to your design.
void calibrateGyro()
{
  long int gyroSums[3]={0};
  long int gyroSigma[3]={0};
 
  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
  {
    updateGyroValues();
    for (int j=0;j<3;j++)
    {
      gyroSums[j]+=gyroRaw[j];
      gyroSigma[j]+=gyroRaw[j]*gyroRaw[j];
    }
  }
  for (int j=0;j<3;j++)
  {
    int averageRate=gyroSums[j]/NUM_GYRO_SAMPLES;
    
    // Per STM docs, we store the average of the samples for each axis and subtract them when we use the data.
    gyroZeroRate[j]=averageRate;
    
    // Per STM docs, we create a threshold for each axis based on the standard deviation of the samples times 3.
    gyroThreshold[j]=sqrt((double(gyroSigma[j]) / NUM_GYRO_SAMPLES) - (averageRate * averageRate)) * GYRO_SIGMA_MULTIPLE;    
  }
}

void updateGyroValues() {

  while (!(gyroReadI2C(0x27) & B00001000)){}      // Without this line you will get bad data occasionally
  
  //if (gyroReadI2C(0x27) & B01000000)
  //  Serial.println("Data missed!  Consider using an interrupt");
    
  int reg=0x28;
  for (int j=0;j<3;j++)
  {
    gyroRaw[j]=(gyroReadI2C(reg) | (gyroReadI2C(reg+1)<<8));
    reg+=2;
  }
 
  int deltaGyro[3];
  for (int j=0;j<3;j++)
  {
    deltaGyro[j]=gyroRaw[j]-gyroZeroRate[j];      // Use the calibration data to modify the sensor value.
    if (abs(deltaGyro[j]) < gyroThreshold[j])
      deltaGyro[j]=0;
    gyroDPS[j]= dpsPerDigit * deltaGyro[j];      // Multiply the sensor value by the sensitivity factor to get degrees per second.
  }
}

void setGyroSensitivity250(void)
{
  dpsPerDigit=.00875f;
  gyroWriteI2C(CTRL_REG4, 0x80);        // Set scale (250 deg/sec)
}

void setGyroSensitivity500(void)
{
  dpsPerDigit=.0175f;
  gyroWriteI2C(CTRL_REG4, 0x90);        // Set scale (500 deg/sec)
}

void setGyroSensitivity2000(void)
{
  dpsPerDigit=.07f;
  gyroWriteI2C(CTRL_REG4,0xA0); 
}

int gyroReadI2C (byte regAddr) {
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.requestFrom(gyroI2CAddr, 1);
  while(!Wire.available()) {};
  return (Wire.read());
}

int gyroWriteI2C( byte regAddr, byte val){
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}

// www.prototyperobotics.com  -  JD Warren 2013
// Arduino Uno
// This code is for using the accelerometer and gyroscope sensors sold at Radio Shack
// Gyroscope - Parallax 27911-RT 3-Axis Gyroscope Module
// Accelerometer - Memsic 2125 Dual-axis Accelerometer
// This code is a work-in-progress and will be labeled as version 4 (v4), once completed
//
// Even though I do my best to use the correct axises... every time I end up using one of the wrong ones. This time, the gyroscope incenuated Y but instead I needed X.
// So... I am using the Y axis from the Accelerometer and the X axis from the Gyroscope, as they area aligned. I would change it, but it is already soldered and I am lazy.

// Pin 12 decides whether the Arduino goes into debug mode or not. if pin 12 is not connected to anything, the segbot will operate normally. if pin 12 is grounded, the segbot will boot into debug mode and will print sensor values to the serial monitor instead of motor values to the sabertooth.
boolean debug = true;
int debug_pin = 12;
int debug_led = 13;

// declare input pins
int accel = 7;
int engage_switch = 4;
int steeringPot = 0; // connect the steering potentiometer to Analog input 3
int positionPot = 1; // connect the gain potentiometer to Analog input 2

// variables for accelerometer
int accel_raw = 0;
int accel_low = 3000;
int accel_high = 7000;
int accel_avg = 0;
int accel_offset = 0;

// Floats for angles
float angle = 0.0;
float gyro_angle = 0.0;
float accel_angle = 0.0;
float accel_adj_angle = 0.0;

// variables for Gyro
float gyro_scale = 0.025;
int gyro_avg = 0;
int gyro_offset = 0;
float drift_correction_speed = 0.5;

// set variable weights
float gyro_weight = 0.95;
float accel_weight = 0.05;

// I2C setup for gyroscope
#include <Wire.h>
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
int Addr = 105;  // I2C address of gyro
int gyro_rate;

// engage button variables
int engage = false;
int engage_state = 1;
float allowance = 1.0;

// timer variables
int last_update;
int cycle_time;
long last_cycle = 0;
long kill_time = 0;

// motor speed variables
int motor_out = 0;
int motor_1_out = 0;
int motor_2_out = 0;
int m1_speed = 0;
int m2_speed = 0;
int output;

// potentiometer variables
int steer_val;
int steer_range = 7;
int steer_reading;
int position_reading;
int angle_offset = 0;
int position_range = 10;
int stop_increment = 1;

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
  // create inputs for engage_switch and debug_pin to enable user to boot into debug mode if needed by grounding pin 12
  pinMode(engage_switch, INPUT);
  pinMode(debug_pin, INPUT);
  // enable the Arduino internal pull-up resistor on the engage_switch and debug pins.
  digitalWrite(engage_switch, HIGH);
  digitalWrite(debug_pin, HIGH);
  // declare led as output
  pinMode(debug_led, OUTPUT);
  delay(100);
  set_accel_offset();
  // check pin 12 state: if left alone (not connected to anything), the Seg-bot will operate normally and the motor output values will be sent to the Sabertooth
  if (digitalRead(debug_pin) == LOW){
    debug = true;
    digitalWrite(debug_led, HIGH);
  }
  // if pin 12 is connected to GND while the Seg-bot is turned On, it will boot into Debug mode and the sensor values will be sent to the serial monitor
  else{
    debug = false;
    digitalWrite(debug_led, LOW);
  }
}

void loop(){
  // make sure button is pushed before engaging
  read_button();
  // read the values of each potentiometer
  read_pots();
  // read accelerometer values
  read_accel();
  // read gyroscope values
  read_gyroscope();
  // calculate angle
  calculate_angle();
  // fix drift by slowly returning the gyro angle reading to match the accelerometer reading
  fix_drift();

  if (debug){
    // Debug with the Serial monitor
    serial_print_stuff();
  }
  else {
    // update the motors with the new values
    update_motor_speed();
  }
  // check the loop cycle time and add a delay as necessary
  time_stamp();
}


void read_button(){
  if (digitalRead(engage_switch) == 1){
    delay(100);
    if (digitalRead(engage_switch) == 1){
      engage = false;
    }
  }
  else {
    if (engage == false){
      if (angle < allowance && angle > -allowance)
        engage = true;
      else {
        engage = false;
      }
    }
    else {
      engage = true;
    }
  }
}


void read_accel(){
  // read the y axis of the accelerometer
  accel_raw = pulseIn(accel, HIGH);
  accel_angle = map(accel_raw, accel_low, accel_high, -90, 90);
}

void read_gyroscope(){
  // Get new values
  getGyroValues();
  // sum gyro angle
  gyro_angle += gyro_rate * gyro_scale;
}

void getGyroValues() {
  byte MSB, LSB;
  MSB = readI2C(0x29);
  LSB = readI2C(0x28);
  // for some reason we have to divide the gyro_rate by 114? Thanks Parallax
  gyro_rate = ((MSB << 8) | LSB) / 114;
}

void calculate_angle(){
  // calculate angle using weighted average (complementary filter)
  angle = (float)(gyro_weight * gyro_angle) + (accel_weight * (accel_angle + angle_offset));
}

void fix_drift(){
  if (gyro_angle > accel_angle){
    gyro_angle -= drift_correction_speed;
  }
  else if (gyro_angle < accel_angle){
    gyro_angle += drift_correction_speed;
  }
}

void set_accel_offset(){
  int accel_sum = 0;
  for (int x = 0; x < 10; x++){
    read_accel();
    accel_sum += accel_angle;
  }
  accel_offset = accel_sum / 10;
  read_accel();
  read_gyroscope();
  calculate_angle();
}

void read_pots(){
  // Read and convert potentiometer values
  // Steering potentiometer
  steer_reading = analogRead(steeringPot); // We want to map this into a range between -1 and 1, and set that to steer_val
  steer_val = map(steer_reading, 0, 1023, steer_range, -steer_range);
  // Angle position potentiometer
  position_reading = map(analogRead(positionPot), 0, 1023, -position_range, position_range);
  angle_offset = accel_offset + position_reading;
}

void update_motor_speed(){
  // Update the motors
  if (engage == true){
      output = (angle * 10); // convert float angle back to integer format
      motor_out = map(output, -250, 250, -64, 64); // map the angle

      // assign steering bias
      motor_1_out = motor_out + (steer_val);
      motor_2_out = motor_out - (steer_val);
      // test for and correct invalid values
      if(motor_1_out > 64){
        motor_1_out = 64;
      }
      if(motor_1_out < -64){
        motor_1_out = -64;
      }
      if(motor_2_out > 64){
        motor_2_out = 64;
      }
      if(motor_2_out < -64){
        motor_2_out = -64;
      }
      // assign final motor output values
      m1_speed = 64 + motor_1_out;
      m2_speed = 192 + motor_2_out;

      if (m1_speed < 1){
        m1_speed = 1;
      }
      else if (m1_speed > 127){
        m1_speed = 127;
      }
      if (m2_speed < 128){
        m2_speed = 128;
      }
      else if (m2_speed > 255){
        m2_speed = 255;
      }
  }
  else{
    stop_motors();
  }
  // Serial speed values write here:
  Serial.write(m1_speed);
  Serial.write(m2_speed);

}

void stop_motors(){
  if (m1_speed > stop_increment){
    m1_speed -= stop_increment;
  }
  else if (m1_speed < -stop_increment){
    m1_speed += stop_increment;
  }
  else {
    m1_speed = 0;
  }
  if (m2_speed > stop_increment){
    m2_speed -= stop_increment;
  }
  else if (m2_speed < -stop_increment){
    m2_speed += stop_increment;
  }
  else {
    m2_speed = 0;
  }
}

void time_stamp(){
  // check to make sure it has been exactly 50 milliseconds since the last recorded time-stamp
  while((millis() - last_cycle) < 25){
    delay(1);
  }
  // once the loop cycle reaches 50 mS, reset timer value and proceed
  cycle_time = millis() - last_cycle;
  last_cycle = millis();
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

void sample_accel(){
  accel_raw = pulseIn(accel, HIGH);
  if (accel_raw < accel_low){
    accel_low = accel_raw;
  }
  else if (accel_raw > accel_high){
    accel_high = accel_raw;
  }
  Serial.print("Accel Low:  ");
  Serial.print(accel_low);
  Serial.print("   Accel High:  ");
  Serial.println(accel_high);
}


void serial_print_stuff(){
  // print accelerometer angle
  //Serial.print("A: ");
  Serial.print(accel_angle);
  // print gyro angle
  Serial.print("  G:");
  Serial.print(gyro_angle);
  // print filtered angle
  Serial.print("  F:");
  Serial.print(angle);

  /*
  // print cycle time
  Serial.print(" time: ");
  Serial.print(cycle_time); // print the loop cycle time

  // these values are commented out, unless testing
  Serial.print("o/m: ");
  Serial.print(output);
  Serial.print("/");
  Serial.print(motor_out);
  Serial.println(" ");
  Serial.print("steer_val: ");
  Serial.print(steer_val);
  Serial.print(" ");
  Serial.print("steer_reading: ");
  Serial.print(steer_reading);
  Serial.print(" ");
  Serial.print("m1/m2: ");
  Serial.print(m1_speed);
  Serial.print("/");
  Serial.println(m2_speed);
  */
  Serial.println(" ");
}

// JD Warren 2012
// Arduino Uno R2

// Use this code for CALIBRATING YOUR SENSORS!!!!

// In no way to I take responisibility for what you may do with this code! Use at your own risk.
// Test thoroughly with wheels off the ground before attempting to ride - Wear a helmet!

// 1. make sure the sensor pins are assigned correctly below.
// 2. upload code to Arduino, connect Sensor board, and turn on Serial monitor to 9600
// 3. record value of each sensor when sitting still on a flat table (0 degrees, level)
// 4. replace the default values in the REAL code for "int gyro_offset" and "int accel_offset"
// 5. CHANGE VALUES IN REAL CODE, NOT THIS TEST CODE.

// Name Analog input pins
int accel_pin = 0;
int gyro_pin = 1;

// accelerometer values
int accel_reading;

// gyroscope values
int gyro_reading;

// timer variables
int last_update;
int cycle_time;
long last_cycle = 0;
int timerVal = 70;
float loop_time = 0.07;

// end of Variables

void setup(){
  // Start the Serial monitor at 9600bps
  Serial.begin(9600);
  // Tell Arduino to use the Aref pin for the Analog voltage, don't forget to connect 3.3v to Aref!
  analogReference(EXTERNAL);
}

void loop(){
  // Start the loop by getting a reading from the Accelerometer and coverting it to an angle
  sample_accel();
  // now read the gyroscope to estimate the angle change
  sample_gyro();
  // check the loop cycle time and add a delay as necessary
  time_stamp();
  // Debug with the Serial monitor
  serial_print_stuff();
}

void sample_accel(){
  // Read and convert accelerometer value
  accel_reading = analogRead(accel_pin);
}

void sample_gyro(){
  // Read and convert gyro value
  gyro_reading = analogRead(gyro_pin);
}

void time_stamp(){
  // check to make sure it has been exactly 50 milliseconds since the last recorded time-stamp 
  while((millis() - last_cycle) < timerVal){
    delay(1);
  }
  // once the loop cycle reaches 50 mS, reset timer value and proceed
  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}

void serial_print_stuff(){
  // Debug with the Serial monitor

  Serial.print("gyro_offset: ");
  Serial.print(gyro_reading); // print the actual reading from the accelerometer sensor
  Serial.print(" ");

  Serial.print("accel_offset: ");
  Serial.print(accel_reading); // print the actual reading from the accelerometer sensor
  Serial.println(" ");

}


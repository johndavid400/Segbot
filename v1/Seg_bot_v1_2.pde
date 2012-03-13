// DIY Seg-bot
// JD Warren 2010
// Arduino Duemilanove (tested)
// Sparkfun Razor 6 DOF IMU - only using X axis from accelerometer and gyroscope 
// Steering potentiometer used to steer bot
// Gain potentiometer used to set max speed (sensitivity) 
// Engage switch (button) used to enable motors
// 
// In no way to I take responisibility for what you may do with this code! Use at your own risk.
// Test thoroughly with wheels off the ground before attempting to ride - Wear a helmet!


// use SoftwareSerial library to communicate with the Sabertooth motor-controller
#include <SoftwareSerial.h> 
// define pins used for SoftwareSerial communication
#define rxPin 2
#define txPin 3
// set up a new SoftwareSerial port, named "mySerial" or whatever you want to call it.
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);


// Name Analog input pins
int gyro_pin = 1;
int accel_pin = 5;
int steeringPot = 3;
int gainPot = 2;

// Name Digital I/O pins
int engage_switch = 4;
int ledPin = 13;

// value to hold the final angle 
float angle = 0.00;
// the following 2 values should add together to equal 1.0
float gyro_weight = 0.98;
float accel_weight = 0.02;

// accelerometer values
int accel_reading;
int accel_raw;
int accel_offset = 511;
float accel_angle;
float accel_scale = 0.01;

// gyroscope values
int gyro_offset = 391;
int gyro_raw; 
int gyro_reading;
float gyro_rate;
float gyro_scale = 0.025; // 0.01 by default
float gyro_angle;
float loop_time = 0.05;

// engage button variables
int engage = false;
int engage_state = 1;

// timer variables
int last_update;
int cycle_time;
long last_cycle = 0;

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
int gain_reading;
int gain_val;

// end of Variables


void setup(){

  // Start the Serial monitor at 9600bps
  Serial.begin(9600); 
  // define pinModes for tx and rx:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  // set the engage_switch pin as an Input
  pinMode(engage_switch, INPUT);

  // enable the Arduino internal pull-up resistor on the engage_switch pin.
  digitalWrite(engage_switch, HIGH);
  // Tell Arduino to use the Aref pin for the Analog voltage, don't forget to connect 3.3v to Aref!
  analogReference(EXTERNAL);

}

void loop(){
  // Start the loop by getting a reading from the Accelerometer and coverting it to an angle
  sample_accel(); 
  // now read the gyroscope to estimate the angle change
  sample_gyro();
  // combine the accel and gyro readings to come up with a "filtered" angle reading
  calculate_angle();
  // read the values of each potentiomoeter
  read_pots(); 
  // make sure bot is level before activating the motors
  auto_level();
  // update the motors with the new values
  update_motor_speed();
  // check the loop cycle time and add a delay as necessary
  time_stamp();
  // Debug with the Serial monitor
  serial_print_stuff();

}




void sample_accel(){
  // Read and convert accelerometer value

  accel_reading = analogRead(accel_pin);
  accel_raw = accel_reading - accel_offset;
  accel_raw = constrain(accel_raw, -90, 90);
  accel_angle = (float)(accel_raw * accel_scale);
}

void sample_gyro(){
  // Read and convert gyro value

  gyro_reading = analogRead(gyro_pin);
  gyro_raw = gyro_reading - gyro_offset;
  gyro_raw = constrain(gyro_raw, -391, 391);
  gyro_rate = (float)(gyro_raw * gyro_scale) * -loop_time; 
  gyro_angle = angle + gyro_rate;
}

void calculate_angle(){
  angle = (float)(gyro_weight * gyro_angle) + (accel_weight * accel_angle);
}

void read_pots(){
  // Read and convert potentiometer values
  // Steering potentiometer
  steer_reading = analogRead(steeringPot); // We want to coerce this into a range between -1 and 1, and set that to steer_val
  steer_val = map(steer_reading, 0, 1023, steer_range, -steer_range); 
  if (angle == 0.00){
    gain_reading = 0;
  }
  // Gain potentiometer
  gain_reading = analogRead(gainPot);
  gain_val = map(gain_reading, 0, 1023, 32, 64);
}

void auto_level(){
  // enable auto-level turn On
  //engage_state = digitalRead(engage_switch);

  if (digitalRead(engage_switch) == 1){
    delay(10);
    if (digitalRead(engage_switch) == 1){
      engage = false;
    }
  }
  else { 
    if (engage == false){
      if (angle < 0.02 && angle > -0.02)
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

void update_motor_speed(){
  // Update the motors

  if (engage == true){
    if (angle < -0.4 || angle > 0.4){
      motor_out = 0;
    } 
    else {
      output = (angle * -1000); // convert float angle back to integer format
      motor_out = map(output, -250, 250, -gain_val, gain_val); // map the angle
    }

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
  }

  else{
    m1_speed = 0;
    m2_speed = 0;
  }

  // write the final output values to the Sabertooth via SoftwareSerial
  mySerial.print(m1_speed, BYTE);
  mySerial.print(m2_speed, BYTE);
}

void time_stamp(){
  // check to make sure it has been exactly 50 milliseconds since the last recorded time-stamp 
  while((millis() - last_cycle) < 50){
    delay(1);
  }
  // once the loop cycle reaches 50 mS, reset timer value and proceed
  cycle_time = millis() - last_cycle;
  last_cycle = millis(); 

}


void serial_print_stuff(){
  // Debug with the Serial monitor

  Serial.print("Accel: ");  
  Serial.print(accel_angle);  // print the accelerometer angle
  Serial.print("  ");   

  Serial.print("Gyro: ");  
  Serial.print(gyro_angle);  // print the gyro angle
  Serial.print("  ");    

  Serial.print("Filtered: ");  
  Serial.print(angle);   // print the filtered angle
  Serial.print("  "); 

  Serial.print(" time: ");
  Serial.print(cycle_time); // print the loop cycle time
  Serial.println("    "); 

  /*
  
   Serial.print("o/m: ");
   Serial.print(output);
   Serial.print("/");
   Serial.print(motor_out);
   Serial.println("  "); 
   
   Serial.print("steer_val: ");
   Serial.print(steer_val);
   Serial.print("  "); 
   
   Serial.print("steer_reading: ");
   Serial.print(steer_reading);
   Serial.print("  "); 
   
   Serial.print("m1/m2: ");
   Serial.print(m1_speed);
   Serial.print("/");
   Serial.println(m2_speed);
   */
}


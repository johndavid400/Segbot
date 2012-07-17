// Modified Code for the Seg-bot
// by JD Warren
// Arduino Uno (tested)
// Sparkfun SEN-11072 IMU - using X axis from accelerometer and Y axis from gyroscope 4.5x
// Steering potentiometer used to steer bot
// Gain potentiometer used to set max speed (sensitivity)
// Engage switch (button) used to enable motors
//
// By loading this code, you are taking full responsibility for what you may do it!
// Use at your own risk!!!
// If you are concerned with the safety of this project, it may not be for you.
// Test thoroughly with wheels off the ground before attempting to ride - Wear a helmet!


// Pin 12 decides whether the Arduino goes into debug mode or not. if pin 12 is not connected to anything, the segbot will operate normally. if pin 12 is grounded, the segbot will boot into debug mode and will print sensor values to the serial monitor instead of motor values to the sabertooth.
boolean debug = false;
int debug_pin = 12;
int debug_led = 13;

// Name Analog input pins
int gyro_pin = 2; // connect the gyro Y axis (4.5x output) to Analog input 1
int accel_pin = 0; // connect the accelerometer X axis to Analog input 5
int steeringPot = 3; // connect the steering potentiometer to Analog input 3
int gainPot = 4; // connect the gain potentiometer to Analog input 2
// Name Digital I/O pins

// value to hold the final angle
float angle = 0.00;
// the following 2 values should add together to equal 1.0
float gyro_weight = 0.98;
float accel_weight = 0.02;
// accelerometer values
int accel_reading;
int accel_raw;
int accel_avg = 0;
int accel_offset = 428;
float accel_angle;
float accel_scale = 0.01;

//gyroscope values
int gyro_avg = 0;
int gyro_offset = 402;
int gyro_raw;
int gyro_reading;
float gyro_rate;
float gyro_scale = 0.01;
// 01 by default
float gyro_angle;
float loop_time = -0.05;

// engage button variables
int engage_switch = 7;
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
int segInitialize=0;

void calibrate(){
  // setup loop to read gyro 10 times
  for (int i = 0; i < 10; i++){
    // read gyro and each time, add the value to the running total
    gyro_avg = gyro_avg + analogRead(gyro_pin); 
    accel_avg = accel_avg + analogRead(accel_pin); 
  }
  // with a sum of 10 readings, divide by 10 to get the average
  gyro_offset = gyro_avg / 10;
  accel_offset = accel_avg / 10;
}

// end of Variables
void setup(){
  // Start the Serial monitor at 9600bps
  Serial.begin(9600);
  // set the engage_switch pin as an Input
  pinMode(engage_switch, INPUT);
  // enable the Arduino internal pull-up resistor on the engage_switch pin.
  digitalWrite(engage_switch, HIGH);
  // Tell Arduino to use the Aref pin for the Analog voltage, don't forget to connect 3.3v to Aref!
  analogReference(EXTERNAL);
  // calibrate gyro and accelerometer, you should keep the Seg-bot still and level when turning on so calibration will be accurate
  calibrate();
  // create input for debug_pin to enable user to boot into debug mode if needed by grounding pin 12
  pinMode(debug_pin, INPUT);
  pinMode(debug_led, OUTPUT);
  // enable the Arduino's internal pull-up resistor on pin D12
  digitalWrite(debug_pin, HIGH);
  // let pin voltage settle
  delay(100);
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
  // Start the loop by getting a reading from the Accelerometer and converting it to an angle
  sample_accel();
  // now read the gyroscope to estimate the angle change
  sample_gyro();
  // combine the accel and gyro readings to come up with a "filtered" angle reading
  calculate_angle();

  if (debug == false){
    // read the values of each potentiometer
    read_pots();
    // make sure bot is level before activating the motors
    auto_level();
    // update the motors with the new values
    update_motor_speed();
    // check the loop cycle time and add a delay as necessary
    time_stamp();
  }
  else {
    // check the loop cycle time and add a delay as necessary
    time_stamp();
    // Debug with the Serial monitor
    serial_print_stuff();
  }
}

void sample_accel(){
  // Read and convert accelerometer value
  accel_reading = analogRead(accel_pin);
  accel_raw = accel_reading - accel_offset;
  accel_raw = map(accel_raw, -130, 130, -90, 90);
  accel_angle = (float)(accel_raw * accel_scale);
}

void sample_gyro(){
  // Read and convert gyro value
  gyro_reading = analogRead(gyro_pin);
  gyro_raw = gyro_reading - gyro_offset;
  gyro_rate = (float)(gyro_raw * gyro_scale) * -loop_time;
  gyro_angle = angle + gyro_rate;
}

void calculate_angle(){
  angle = (float)(gyro_weight * gyro_angle) + (accel_weight * accel_angle);
}

void read_pots(){
  // Read and convert potentiometer values
  // Steering potentiometer
  steer_reading = analogRead(steeringPot); // We want to map this into a range between -1 and 1, and set that to steer_val
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
    if (angle < -0.45 || angle > 0.45){
      m1_speed = 0;
      m2_speed = 0;
    }
    else {
      output = (angle * -1000); // convert float angle back to integer format
      motor_out = map(output, -250, 250, -gain_val, gain_val); // map the angle
    
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
  }
  else{
    m1_speed = 0;
    m2_speed = 0;
  }
  
  // Serial speed values write here:
  Serial.print(m1_speed);
  Serial.print("   ");
  Serial.println(m2_speed);
  
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
  Serial.print(accel_angle); // print the accelerometer angle

  Serial.print(" ");
  Serial.print("Gyro: ");
  Serial.print(gyro_angle); // print the gyro angle

  Serial.print(" ");
  Serial.print("Filtered: ");
  Serial.print(angle); // print the filtered angle
  Serial.print(" ");
  Serial.print(" time: ");
  Serial.print(cycle_time); // print the loop cycle time
  Serial.println(" ");
    /*
   these values are commented out, unless testing
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
}


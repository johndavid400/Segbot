// www.prototyperobotics.com  -  JD Warren 2013
// Arduino Uno
// This code is for using the accelerometer and gyroscope sensors sold at Radio Shack
// Gyroscope - Parallax 27911-RT 3-Axis Gyroscope Module
// Accelerometer - Memsic 2125 Dual-axis Accelerometer
// This code is a work-in-progress and will be labeled as version 4 (v4), once completed

// declare input pins for x and y accelerometer
int accel = 7;

// Floats for angles
float angle = 0.0;
float gyro_angle = 0.0;
float accel_angle = 0.0;

// scale variable for Gyro
float gyro_scale = 0.1;

// set variable weights
float gyro_weight = 0.02;
float accel_weight = 0.98;

// end of variable declaration

void setup(){
  Serial.begin(9600);
  accel_setup();
  gyro_setup();
  sample_accel();
  delay(1000);
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





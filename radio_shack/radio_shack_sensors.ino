// This code is for using the accelerometer and gyroscope sensors sold at Radio Shack
// Gyroscope - Parallax 27911-RT 3-Axis Gyroscope Module
// Accelerometer - Memsic 2125 Dual-axis Accelerometer

// This code is a work-in-progress and will be labeled as version 4 (v4), once completed

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

void setup(){
  Serial.begin(9600);
  // setting up inputs for x and y accelerometer
  pinMode(x_accel, INPUT);
  pinMode(y_accel, INPUT);
}

void loop(){
  // read accelerometer values
  read_accel();
  // print values
  print_values();
  // delay
  delay(50);
}

void read_accel(){
  // read the x axis of the accelerometer
  x_raw = pulseIn(x_accel, HIGH);
  x_adj = map(x_raw, accel_low, accel_high, -90, 90);
  // read the y axis of the accelerometer
  y_raw = pulseIn(y_accel, HIGH);
  y_adj = map(y_raw, accel_low, accel_high, -90, 90);
}

void print_values(){
  // print x values
  Serial.print("X: ");
  Serial.print(x_raw);
  Serial.print("     "); 
  // print y values
  Serial.print("Y: ");
  Serial.print(y_raw);
  Serial.print("     ");
  // end of line
  Serial.println("");
}



// Code for Memsic 2125 accelerometer from Radio Shack
// Accelerometer is made by Parallax
// Model: 28017
// Catalog #: 276-029 


// raw values for x and y readings
int accel_raw = 0;

// values for min/max accelerometer readings
int accel_sample;
int accel_n = 100;
int accel_range = 1250;
int accel_low;
int accel_high;

// variables for accelerometer
//int accel_low = 3000;
//int accel_high = 7000;
int accel_avg = 0;
int accel_offset = 0;


void accel_setup(){
  pinMode(accel, INPUT);
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

void read_accel(){
  // read the y axis of the accelerometer
  accel_raw = pulseIn(accel, HIGH);
  accel_angle = map(accel_raw, accel_low, accel_high, -55, 50);
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

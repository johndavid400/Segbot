
// motor speed variables
int motor_out = 0;
int m1_out = 0;
int m2_out = 0;
//int m1_speed = 0;
//int m2_speed = 0;
int output;

void update_motor_speed(){
  /*
  motor_out = map(angle, -10, 10, -255, 255);
  
  // TODO: do stuff with steering here
  m1_out = motor_out;
  m2_out = motor_out;
  
  if (m1_out > 255){ m1_out = 255; }
  else if (m1_out < -255){ m1_out = -255; }
  if (m2_out > 255){ m2_out = 255; }
  else if (m2_out < -255){ m2_out = -255; }  
  
  // Update the motors
  if (m1_out < 0) {
    m1_forward(-m1_out);
  }
  else if (m1_out > 0) {
    m1_reverse(m1_out);
  }
  
  if (m2_out < 0) {
    m2_forward(-m2_out);
  }
  else if (m2_out > 0) {
    m2_reverse(m2_out);
  }  
  */
  if (angle < 0) {
    m1_reverse(255);
    m2_forward(255);
  }
  else if (angle > 0) {
    m1_forward(255);
    m2_reverse(255);
  } 
//  // print filtered angle
//  Serial.print("motors:"); 
//  Serial.print(m1_out);
//  Serial.print("/");
//  Serial.print(m2_out);
//  Serial.print("    ");
  
}

// motor functions for Arduino Motor-shield Motor R3
void m1_forward(int pwm_speed){
  digitalWrite(m1_dir, HIGH); // set the direction pin HIGH
  analogWrite(m1_pwm, pwm_speed); // set the speed using the variable passed in from m1_val as the pwm speed: m1_forward(m1_val);
}
void m1_reverse(int pwm_speed){
  digitalWrite(m1_dir, LOW);
  analogWrite(m1_pwm, pwm_speed); // set the speed using the variable passed in from m1_val as the pwm speed: m1_reverse(m1_val);
}
void m1_stop(){
  // stop this motor by writing the Enable pin LOW
  digitalWrite(m1_pwm, LOW);
}
void m2_forward(int pwm_speed){
  digitalWrite(m2_dir, HIGH);
  analogWrite(m2_pwm, pwm_speed); // set the speed using the variable passed in from m1_val as the pwm speed: m2_forward(m2_val);
}
void m2_reverse(int pwm_speed){
  digitalWrite(m2_dir, LOW);
  analogWrite(m2_pwm, pwm_speed); // set the speed using the variable passed in from m1_val as the pwm speed: m2_reverse(m2_val);
}
void m2_stop(){
  // stop this motor by writing the Enable pin LOW
  digitalWrite(m2_pwm, LOW);
}




// motor speed variables
int motor_out = 0;
int motor_1_out = 0;
int motor_2_out = 0;
//int m1_speed = 0;
//int m2_speed = 0;
int output;

void update_motor_speed(){
  // Update the motors

  output = (angle * 100); // convert float angle back to integer format
  if (output > 255) {
    output = 255;
  }
  else if (output < -255) {
    output = -255;
  }

  // assign steering bias
  motor_1_out = motor_out; // + (steer_val);
  motor_2_out = motor_out; // - (steer_val);
  // test for and correct invalid values
  if(motor_1_out > 255){
    motor_1_out = 255;
  }
  if(motor_1_out < -255){
    motor_1_out = -255;
  }
  if(motor_2_out > 255){
    motor_2_out = 255;
  }
  if(motor_2_out < -255){
    motor_2_out = -255;
  }
  
  if (motor_1_out > 0) {
    m1_forward(motor_1_out);
  }
  else if (motor_1_out < 0) {
    m1_reverse(-motor_1_out);
  }
  
  if (motor_2_out > 0) {
    m2_forward(motor_2_out);
  }
  else if (motor_2_out < 0) {
    m2_reverse(-motor_2_out);
  }

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


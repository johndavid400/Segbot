
// motor speed variables
int motor_out = 0;
int motor_1_out = 0;
int motor_2_out = 0;
int m1_speed = 0;
int m2_speed = 0;
int output;

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

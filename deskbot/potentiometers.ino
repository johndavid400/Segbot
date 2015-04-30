

void read_pots(){
  // Read and convert potentiometer values
  // Steering potentiometer
  steer_reading = analogRead(steeringPot); // We want to map this into a range between -1 and 1, and set that to steer_val
  steer_val = map(steer_reading, 0, 1023, steer_range, -steer_range);
  // Angle position potentiometer
  position_reading = map(analogRead(positionPot), 0, 1023, -position_range, position_range);
  angle_offset = accel_offset + position_reading;
}

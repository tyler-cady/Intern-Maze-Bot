float CORRIDOR_THRESHOLD = 8; //should hopefully be enough but not too much
void turn_90(bool R_L){
  motors_stop(2);
  delay(500);
  Turn(R_L,90,83);
  while(true){
    read_front_ultra();
    if (cm[1] > CORRIDOR_THRESHOLD) {  // CORRIDOR_THRESHOLD is the distance that indicates a clear path
        break;
    }
  }
  motors_stop(2);
}

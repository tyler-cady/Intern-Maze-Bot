#include <cmath>

int IR1 = 11;

const int SENSOR_ANGLE = 45; // degrees
const float READ_DIST = 11.8; //inches

float get_side_dist( ){
    // returns a constant value for now but read ultrasonic sensor
    return 3.0; //inches
}

float calc_gap_distance(){
    float b = get_side_dist();
    float c = b/cos(SENSOR_ANGLE);
    return sqrt((b*b)+(c*c)); 
}
void setup(){
  Serial.begin(115200); // Init Serial at 115200 Baud Rate.
  Serial.println("Serial Working"); // Test to check if serial is working or not
  pinMode(IRSensor, INPUT); // IR Sensor pin INPUT
  pinMode(LED, OUTPUT); // LED Pin Output
}


void loop(){
  int sensorStatus = digitalRead(IRSensor); // Set the GPIO as Input
  if (sensorStatus == 1) // Check if the pin high or not
  {
    // if the pin is high turn off the onboard Led
    digitalWrite(LED, LOW); // LED LOW
    Serial.println("WALL"); // print Motion Detected! on the serial monitor window
  }
  else  {
    //else turn on the onboard LED
    digitalWrite(LED, HIGH); // LED High
    Serial.println("NO WALL"); // print Motion Ended! on the serial monitor window
  }
}

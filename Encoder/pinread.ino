#include "Particle.h"

void setup() {
  Serial.begin(9600);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
}

void loop() {
  int sensorValueD3 = digitalRead(D3);
  int sensorValueD4 = digitalRead(D4);
  
  Serial.print("Digital Value of D3: ");
  Serial.println(sensorValueD3);
  
  Serial.print("Digital Value of D4: ");
  Serial.println(sensorValueD4);
  
  delay(1000); // Adjust delay as needed
}
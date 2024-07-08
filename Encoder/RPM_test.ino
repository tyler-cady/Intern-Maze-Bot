#include <EncoderRPM.h>

EncoderRPM encoder(2, 3);

void setup() {
  Serial.begin(9600);
  encoder.begin();
}

void loop() {
  encoder.update();
  Serial.print("RPM: ");
  Serial.println(encoder.getRPM());
  delay(1000);
}

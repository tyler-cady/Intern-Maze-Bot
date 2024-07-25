#include "pid.h"

#include "PinChangeInterrupt.h"

/////////////////////ENCODER MOTOR VARS/////////////////////////////////////
#define encoderPin1 11 // Encoder Output 'A' must connected with interrupt pin of arduino.
#define encoderPin2 12 // Encoder Output 'B' must connected with interrupt pin of arduino.
#define encoder2Pin1 7
#define encoder2Pin2 8
#define PPR 7

volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
volatile int lastEncoded2 = 0; // Here updated value of encoder store.
volatile long encoderValue2 = 0; // Raw encoder value

#define PWMright1 5
#define PWMright2 6
#define PWMleft1 9
#define PWMleft2 10

pid leftMotorPID(1.0, 0.5, 0.1, 100, 0);
pid rightMotorPID(1.0, 0.5, 0.1, 100, 0);


// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store


// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)
const bool restart = true;
const unsigned long previousMillis = 0;        // will store last time LED was updated
const int encoderValueSTART1 = 0, encoderValueSTART2 = 0, encoderValueEND1,encoderValueEND2; 
/////////////////////////////////////////////MOTORS////////////////////////////////////////////////////
// speed in percentage of duty cycle. i.e speed = 50 => 50% duty cycle
void setSpeed(float speed, int pin) {
  // analogWriteResolution(8);
  float percent = float(speed / 100);
  float dutycycle = float(255 * percent); // 255 is max of 8-bit resolution
  analogWrite(pin, dutycycle); // send PWM to H-bridge
}

// R_L determines turn right or left. R_L = true : right turn; R_L = false : left turn;
void turn(bool R_L, int degree, int turn_speed) {
  // while(MPU_getX()!=degree){
    if (R_L) {
      setSpeed(turn_speed, PWMright1);
      setSpeed(0, PWMright2);
      setSpeed(turn_speed, PWMleft1);
      setSpeed(0, PWMleft2);
    } else {
      setSpeed(turn_speed, PWMright1);
      setSpeed(0, PWMright2);
      setSpeed(turn_speed, PWMleft1);
      setSpeed(0, PWMleft2);
    }
  // }
  motors_stop(2);    // stop turning 
  resetEncoders();  // reset encoders to count for next turn
}

// R_L_BOTH: 0=right motor only; 1=left motor only; 2=both motors
void motors_stop(int R_L_BOTH) {
  if (R_L_BOTH == 0) { // stop Right motor
    setSpeed(0, PWMright1);
    setSpeed(0, PWMright2);
  } else if (R_L_BOTH == 1) { // stop left motor
    setSpeed(0, PWMleft1);
    setSpeed(0, PWMleft2);
  } else if (R_L_BOTH == 2) { // stop both motors
    setSpeed(0, PWMright1);
    setSpeed(0, PWMright2);
    setSpeed(0, PWMleft1);
    setSpeed(0, PWMleft2);
  }
}

// direction: true=forward; false=backwards. NEED TO BE CALIBRATED BASED ON ORIENTATION OF MOTORS IN CHASSIS
void motors_straight(bool direction, int speed) {
  if (direction) { // forward direction
    setSpeed(0, PWMright1);
    setSpeed(speed, PWMright2);
    setSpeed(speed, PWMleft1);
    setSpeed(0, PWMleft2);
  } else { // backwards
    setSpeed(speed, PWMright1);
    setSpeed(0, PWMright2);
    setSpeed(0, PWMleft1);
    setSpeed(speed, PWMleft2);
  }
}

void updateMotorSpeeds(double desiredSpeed, double dt) {
  float currentSpeedRight = getspeed(1);
  float currentSpeedLeft = getspeed(2);
  Serial.println(currentSpeedRight);
  Serial.println(currentSpeedLeft);

  double leftMotorOutput = leftMotorPID.tick(currentSpeedLeft, desiredSpeed, dt);
  double rightMotorOutput = rightMotorPID.tick(currentSpeedRight, desiredSpeed, dt);

  //Tests 
  Serial.println(leftMotorOutput);
  Serial.println(rightMotorOutput);


  setSpeed(0, PWMleft1);
  setSpeed(leftMotorOutput, PWMleft2);
  setSpeed(0, PWMright1);
  setSpeed(rightMotorOutput,PWMright2);
}

void setup() {
  pinMode(PWMright1, OUTPUT); 
  pinMode(PWMright2, OUTPUT); 
  pinMode(PWMleft1, OUTPUT); 
  pinMode(PWMleft2, OUTPUT);
  Serial.begin(9600); // initialize serial communication

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  pinMode(encoder2Pin1, INPUT_PULLUP); 
  pinMode(encoder2Pin2, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(encoderPin1), updateEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin2), updateEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2Pin1), updateEncoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2Pin2), updateEncoder2, CHANGE);
}
void readEncoders(){
 Serial.print("Encoder1: ");
 Serial.print(encoderValue, DEC);
 Serial.println(" ");
 Serial.print("Encoder2: ");
 Serial.print(encoderValue2, DEC);
 Serial.println(" "); 
}
void loop() {
  // if(restart){
  //   restart = false;
  //   encoderValueSTART1=getLeftEncoderSpeed();
  //   encoderValueSTART2=getLeftEncoderSpeed();
  // }
  
  double desiredSpeed = 100; // Example desired speed
  double dt = 0.1; // Example time delta (should be calculated based on actual loop time)
  readEncoders();
  updateMotorSpeeds(desiredSpeed, dt);
  delay(100); // Example delay (should be adjusted based on actual loop requirements)
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1); // MSB = most significant bit
  int LSB = digitalRead(encoderPin2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; // store this value for next time
}

void updateEncoder2() {
  int MSB = digitalRead(encoder2Pin1); // MSB = most significant bit
  int LSB = digitalRead(encoder2Pin2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum  = (lastEncoded2 << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2++;

  lastEncoded2 = encoded; // store this value for next time
}

int getLeftEncoderSpeed() {
  // Implement encoder reading and speed calculation for leftmotor
return encoderValue2 / PPR; // Example calculation (replace with actual logic)
}

int getRightEncoderSpeed() {
// Implement encoder reading and speed calculation for right motor
return encoderValue / PPR; // Example calculation (replace with actual logic)
}

void resetEncoders() {
encoderValue = 0;
encoderValue2 = 0;
}

float getspeed(int R_L){ //right is 1 left is 2
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    restart = true;
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    if(R_L == 1){
      encoderValueEND1=getRightEncoderSpeed();
      speed = ((encoderValueEND1-encoderValueSTART1)/interval);
      encoderValueSTART1=encoderValueEND1;
    }
    else{
      encoderValueEND1=getLeftEncoderSpeed();
      speed = ((encoderValueEND2-encoderValueSTART2)/interval);
      encoderValueSTART2=encoderValueEND2; 
    }
    // set the LED with the ledState of the variable:
    return speed;
  }

}


#ifndef PID_HPP
#define PID_HPP

#include <stdio.h>
#include <algorithm>
#include <numeric>
#include <vector>
#include <functional>


class pid
{
private:
  const double _max;
  const double _min;
  const double _Kp;
  const double _Kd;
  const double _Ki;
  double _pre_error;
  double _integral;

public:
  pid(double Kp, double Ki, double Kd, double max = 1, double min = -1)
      : _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0)
  {
  }
  /**
   * @brief Run the PID control loop.
   *
   * @param
   */
  double tick(double current, double desired, double dt)
  {
    // Calculate error
    const double error = desired - current;

    // Proportional term
    const double Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    const double Iout = _Ki * _integral;
    // Also restrict integral to prevent growth after it saturates
    _integral = std::max(_integral, _min);
    _integral = std::min(_integral, _max);

    // Derivative term
    const double derivative = (error - _pre_error) / dt;
    const double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    output = std::max(output, _min);
    output = std::min(output, _max);

    // Save error to previous error
    _pre_error = error;

    return output;
  }
};

#endif // end of PID_HPP




#include "PinChangeInterrupt.h"

/////////////////////ENCODER MOTOR VARS/////////////////////////////////////
#define encoderPin1 11 //Encoder Output 'A' must connected with intreput pin of arduino.
#define encoderPin2 12 //Encoder Otput 'B' must connected with intreput pin of arduino.
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



/////////////////////////////////////////////MOTORS////////////////////////////////////////////////////
//speed in percentage of duty cycle. i.e speed = 50 => 50% duty cycle
void setSpeed(float speed, int pin){
  // analogWriteResolution(8);
  float percent = float(speed/100);
  float dutycycle = float(255*percent); //255 is max of 8 bit resolution
  analogWrite(pin, dutycycle); //send PWM to H bridge
}

// R_L determines turn right or left. R_L = true : right turn; R_L = false : left turn;
void turn(bool R_L, int degree, int turn_speed) 
{
    // while(MPU_getX()!=degree){
        if (R_L){
            setSpeed(turn_speed,PWMright1);
            setSpeed(0,PWMright2);
            setSpeed(turn_speed,PWMleft1);
            setSpeed(0,PWMleft2);
        }
        else{
            setSpeed(turn_speed,PWMright1);
            setSpeed(0,PWMright2);
            setSpeed(turn_speed,PWMleft1);
            setSpeed(0,PWMleft2);
        }
    // }
    motors_stop(2);    //stop turning 
    resetEncoders();  //reset encoders to count for next turn
}
//R_L_BOTH: 0=right motor only; 1=left motor only; 2=both motors
void motors_stop(int R_L_BOTH) 
{
    if (R_L_BOTH == 0){ //stop Right motor
        setSpeed(0,PWMright1);
        setSpeed(0,PWMright2);
    }
    else if (R_L_BOTH == 1){ //stop left motor
        setSpeed(0,PWMleft1);
        setSpeed(0,PWMleft2);
    }
    else if (R_L_BOTH == 2){ //stop both motors
        setSpeed(0,PWMright1);
        setSpeed(0,PWMright2);
        setSpeed(0,PWMleft1);
        setSpeed(0,PWMleft2);
    }
}
//direction: true=forward; false=backwards. NEED TO BE CALIBRATED BASED ON ORIENTATION OF MOTORS IN CHASSIS
void motors_straight(bool direction, int speed) 
{
    if (direction){ //forward direction
        setSpeed(0,PWMright1);
        setSpeed(speed,PWMright2);
        setSpeed(speed,PWMleft1);
        setSpeed(0,PWMleft2);
    }
    else{ //backwards
        setSpeed(speed,PWMright1);
        setSpeed(0,PWMright2);
        setSpeed(0,PWMleft1);
        setSpeed(speed,PWMleft2);
    }
}

void setup(){
    pinMode(PWMright1, OUTPUT); 
    pinMode(PWMright2, OUTPUT); 
    pinMode(PWMleft1, OUTPUT); 
    pinMode(PWMleft2, OUTPUT);
    Serial.begin(9600); //initialize serial comunication

    pinMode(encoderPin1, INPUT_PULLUP); 
    pinMode(encoderPin2, INPUT_PULLUP);

    pinMode(encoder2Pin1, INPUT_PULLUP); 
    pinMode(encoder2Pin2, INPUT_PULLUP);

    // digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
    // digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

    //call updateEncoder() when any high/low changed seen
    //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
    // attachInterrupt(0, updateEncoder, CHANGE); 
    // attachInterrupt(1, updateEncoder, CHANGE);
    attachPCINT(digitalPinToPCINT(encoderPin1),updateEncoder, CHANGE);
    attachPCINT(digitalPinToPCINT(encoderPin2),updateEncoder, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder2Pin1),updateEncoder2, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder2Pin2),updateEncoder2, CHANGE);
}
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}
void updateEncoder2(){
  int MSB = digitalRead(encoder2Pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder2Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded2 << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2 --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2 ++;

  lastEncoded2 = encoded; //store this value for next time
}
//R_L: true right(1), false left(2)
int getRotations(bool R_L){
  if(R_L){
    return int(encoderValue/PPR);  //divide by Pulses per rotation to get total number of rotations recorded
  } else{
    return int(encoderValue2/PPR); //divide by Pulses per rotation to get total number of rotations recorded
  }

}
//to be used after a turn is made to ensure proper count
void resetEncoders(){
  encoderValue=0;
  encoderValue2=0;
}

//GET RID OF AFTER TESTING FOR OUPTUT PURPOSES ONLY
void readEncoders(){
 Serial.print("Encoder1: ");
 Serial.print(encoderValue, DEC);
 Serial.println(" ");
 Serial.print("Encoder2: ");
 Serial.print(encoderValue2, DEC);
 Serial.println(" "); 
}

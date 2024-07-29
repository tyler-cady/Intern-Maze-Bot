#ifndef PID_HPP
#define PID_HPP

#include "Kalman.h"

class pid {
private:
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
  Kalman _kalman;

public:
  pid(double Kp, double Ki, double Kd, double max = 255, double min = 0)
      : _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0), _integral(0), _kalman() {}

  double tick(double current, double desired, double dt) {
    // Use the Kalman filter to smooth the current measurement
    double smoothed_current = _kalman.getAngle(current, 0, dt);

    // Calculate error
    double error = desired - smoothed_current;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    double Iout = _Ki * _integral;
    // Restrict integral to prevent growth after it saturates
    if (_integral > _max) _integral = _max;
    else if (_integral < _min) _integral = _min;

    // Derivative term
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max) output = _max;
    else if (output < _min) output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
  }
};

#endif // end of PID_HPP
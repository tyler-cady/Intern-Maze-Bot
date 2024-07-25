#ifndef PID_HPP
#define PID_HPP
#include <ArduinoSTL.h>
// #include <stdio.h>
// #include <algorithm> 
// #include <numeric>
// #include <vector>
// #include <functional>

class pid {
private:
  const double _max;
  const double _min;
  const double _Kp;
  const double _Kd;
  const double _Ki;
  double _pre_error;
  double _integral;

public:
  pid(double Kp, double Ki, double Kd, double max = 255, double min = 0)
      : _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0), _integral(0) {}

  double tick(double current, double desired, double dt) {
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

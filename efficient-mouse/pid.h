#ifndef PID_HPP
#define PID_HPP

#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>

class pid {
private:
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;

    std::ofstream _log_file;

    // Function to get current time as a string
    std::string current_time() {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto local_time = std::localtime(&time_t_now);
        
        std::stringstream ss;
        ss << std::put_time(local_time, "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

    // Function to log data to a CSV file
    void log(const std::string& time, double input, double output) {
        cout << time << "," << input << "," << output << "\n";
    }

public:
    pid(double Kp, double Ki, double Kd, double max = 255, double min = 0) 
        : _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0), _integral(0) {
        // Open the log file in append mode
        _log_file.open("pid_log.csv", std::ios::out | std::ios::app);
    }

    ~pid() {
        if (_log_file.is_open()) {
            _log_file.close();
        }
    }

    double tick(double current, double desired, double dt) {
        // Calculate error
        double error = desired - current;

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

        // Log the current time, input (current), and output
        log(current_time(), current, output);

        return output;
    }
};

#endif // end of PID_HPP

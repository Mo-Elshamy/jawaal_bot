#ifndef PID_H
#define PID_H

class PIDController {
private:
    float kp_;
    float ki_;
    float kd_;
    float integral_;
    float prev_error_;
    float max_out_;
    float min_out_;

public:
    // Constructor to initialize gains and anti-windup boundaries
    PIDController(float kp, float ki, float kd, float min_out, float max_out);

    // Overwrites coefficients safely on-the-fly for dynamic ROS 2 parameter updates
    void setGains(float kp, float ki, float kd);

    // Executes the standard discrete velocity control math equation
    float compute(float target, float actual, float dt);
    
    // Resets the error histories (useful if the robot is forcibly stopped)
    void reset();
};

#endif
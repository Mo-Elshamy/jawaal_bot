#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H

#include "L298n_motor_driver.h"
#include "WheelEncoder.h"
#include "pid.h"

class RobotJoint {
private:
    L298N& motor_;
    WheelEncoder& encoder_;
    PIDController pid_;
    float target_velocity_;
    int32_t last_ticks_;

public:
    // Constructor links the specific hardware peripherals to this joint
    RobotJoint(L298N& motor, WheelEncoder& encoder, PIDController pid);

    void init();
    void setTargetVelocity(float rad_s);
    void updatePIDGains(float kp, float ki, float kd);
    int32_t readAbsoluteTicks();
    
    // The core execution cycle that drives the wheel
    void executeControlLoop(float dt, float ticks_per_rad);
};

#endif
#include "RobotJoint.h"

RobotJoint::RobotJoint(L298N& motor, WheelEncoder& encoder, PIDController pid)
    : motor_(motor), encoder_(encoder), pid_(pid), target_velocity_(0.0f), last_ticks_(0) {}

void RobotJoint::init() {
    motor_.init();
    encoder_.init();
}

void RobotJoint::setTargetVelocity(float rad_s) { 
    target_velocity_ = rad_s; 
}

void RobotJoint::updatePIDGains(float kp, float ki, float kd) { 
    pid_.setGains(kp, ki, kd); 
}

int32_t RobotJoint::readAbsoluteTicks() { 
    return encoder_.getTicks(); 
}

void RobotJoint::executeControlLoop(float dt, float ticks_per_rad) {
    int32_t current_ticks = encoder_.getTicks();
    int32_t delta_ticks = current_ticks - last_ticks_;
    last_ticks_ = current_ticks;

    // Convert raw discrete ticks into physical angular velocity (Radians/Second)
    float actual_velocity = (delta_ticks / ticks_per_rad) / dt;
    
    // Calculate required motor PWM effort to reach the target speed
    float control_effort = pid_.compute(target_velocity_, actual_velocity, dt);
    
    // Command the L298N H-Bridge
    motor_.setSpeed(control_effort);
}
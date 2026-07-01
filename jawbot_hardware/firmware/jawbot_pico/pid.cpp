#include "pid.h"

// Constructor implementation initializing member variables
PIDController::PIDController(float kp, float ki, float kd, float min_out, float max_out)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), prev_error_(0.0f), min_out_(min_out), max_out_(max_out) {}

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float PIDController::compute(float target, float actual, float dt) {
    float error = target - actual;
    
    // Proportional & Integral Math
    integral_ += error * dt;
    
    // Anti-windup integration clamp to prevent control overshoots
    if (integral_ > max_out_) integral_ = max_out_;
    if (integral_ < min_out_) integral_ = min_out_;

    // Derivative Math
    float derivative = (error - prev_error_) / dt;
    
    // Compute total control effort
    float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
    
    // Store current error for the next cycle's derivative calculation
    prev_error_ = error;

    // Ultimate physical actuator safety clamp (-100% to 100%)
    if (output > max_out_) output = max_out_;
    if (output < min_out_) output = min_out_;

    return output;
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}
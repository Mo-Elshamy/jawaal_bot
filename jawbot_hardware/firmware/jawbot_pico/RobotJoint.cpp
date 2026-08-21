#include "RobotJoint.h"
#include <cmath>

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

    // 1. Calculate actual angular velocity (rad/s)
    float actual_velocity = (static_cast<float>(delta_ticks) / ticks_per_rad) / dt;

    // 2. HARD STOP: Silent brake when command is zero
    if (std::fabs(target_velocity_) < 0.01f) {
        pid_.reset();
        motor_.setSpeed(0.0f);
        return;
    }

    // 3. Compute active PID feedback
    float pid_effort = pid_.compute(target_velocity_, actual_velocity, dt);

    // =====================================================================
    // 4. DUAL-STAGE FRICTION COMPENSATION & FEEDFORWARD
    // =====================================================================
    const float STATIC_BREAKAWAY = 38.0f; // Initial kick to overcome stiction
    const float KINETIC_ASSIST   = 33.0f; // Running assist once rolling
    const float KV_FEEDFORWARD   = 3.3f;  // Baseline voltage scaling

    // Taper boost once the wheel is rolling
    float active_offset = (std::fabs(actual_velocity) < 0.25f) ? STATIC_BREAKAWAY : KINETIC_ASSIST;

    float total_effort = (target_velocity_ * KV_FEEDFORWARD) + pid_effort;

    if (target_velocity_ > 0.0f) {
        total_effort += active_offset;
    } else if (target_velocity_ < 0.0f) {
        total_effort -= active_offset;
    }

    // Actuator safety limits (-100% to +100%)
    if (total_effort > 100.0f) total_effort = 100.0f;
    if (total_effort < -100.0f) total_effort = -100.0f;

    // 5. Command the L298N H-Bridge
    motor_.setSpeed(total_effort);
}
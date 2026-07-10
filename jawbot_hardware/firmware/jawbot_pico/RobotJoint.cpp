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
    
    // =====================================================================
    // DEADBAND COMPENSATION INJECTION (The "Anti-Whine" Logic)
    // =====================================================================
    // Determine this value experimentally! (e.g., if motor whines up to 
    // PWM 40 out of 255, the percentage is roughly 15.0%)
    float deadband_offset = 23.0f; 

    // SAFETY CHECK: Only jump the deadband if we actually intend to move!
    // If target is 0, we don't want tiny encoder noise to trigger a 15% power jump.
    if (target_velocity_ == 0.0f) {
        control_effort = 0.0f;
        pid_.reset(); // <-- THE MAGIC FIX: Instantly erases Ki memory so it doesn't bounce!
    } 
    // 2. ACTIVE DRIVING: Apply the deadband skip
    else {
        // Only jump the deadband if the PID is actually asking for meaningful power
        if (control_effort > 0.5f) {
            control_effort += deadband_offset;
        } else if (control_effort < -0.5f) {
            control_effort -= deadband_offset;
        } else {
            control_effort = 0.0f; // Ignore micro-noise
        }
    }
    // =====================================================================

    // Command the L298N H-Bridge
    motor_.setSpeed(control_effort);
}
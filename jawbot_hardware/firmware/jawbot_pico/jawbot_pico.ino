#include "L298n_motor_driver.h"
#include "WheelEncoder.h"
#include "pid.h"
#include "RobotJoint.h"
#include "SerialProtocol.h"

// --- 1. HARDWARE MAPPING ---
L298N left_motor(6, 7, 13);
WheelEncoder left_enc(3, 2); 

L298N right_motor(12, 10, 11);
WheelEncoder right_enc(4, 5);

// --- 2. INTERRUPT ROUTINES ---
void left_isr() { left_enc.handleInterrupt(); }
void right_isr() { right_enc.handleInterrupt(); }

// --- 3. SYSTEM CONSTANTS ---
const float TICKS_PER_REV = 1440.0f;
const float TICKS_PER_RAD = TICKS_PER_REV / (2.0f * PI);
const float LOOP_HZ = 50.0f;
const float LOOP_DT = 1.0f / LOOP_HZ;

// --- 4. OBJECT INSTANTIATIONS ---
RobotJoint left_joint(left_motor, left_enc, PIDController(15.0, 5.0, 0.5, -100.0, 100.0));
RobotJoint right_joint(right_motor, right_enc, PIDController(15.0, 5.0, 0.5, -100.0, 100.0));
SerialProtocol ros2_comms(left_joint, right_joint);

// --- 5. TIMER SETUP ---
struct repeating_timer timer;
volatile bool run_control_cycle = false;

bool timer_callback(struct repeating_timer *t) {
    run_control_cycle = true;
    return true; 
}

// =========================================================

void setup() {
    Serial.begin(115200);
    
    left_joint.init();
    right_joint.init();

    attachInterrupt(digitalPinToInterrupt(3), left_isr, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(4), right_isr, CHANGE);

    add_repeating_timer_us(-20000, timer_callback, NULL, &timer);
}

void loop() {
    // 1. HARDWARE REAL-TIME LAYER (50Hz)
    if (run_control_cycle) {
        run_control_cycle = false;
        
        left_joint.executeControlLoop(LOOP_DT, TICKS_PER_RAD);
        right_joint.executeControlLoop(LOOP_DT, TICKS_PER_RAD);

        ros2_comms.sendTelemetry(micros());
    }

    // 2. BACKGROUND COMMUNICATION LAYER
    ros2_comms.processIncoming();
}
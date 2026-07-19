#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "RobotJoint.h"
#include "gy87_driver.h" // <-- NEW: Include the IMU structure

class SerialProtocol {
private:
    RobotJoint& left_joint_;
    RobotJoint& right_joint_;
    String input_buffer_;
    uint32_t last_command_time_;

    // Internal method to decode the strings
    void parseCommand(String command);

public:
    // Constructor requires links to the robot's physical joints
    SerialProtocol(RobotJoint& left_joint, RobotJoint& right_joint);

    // Reads the serial line and handles parsing if a newline is detected
    void processIncoming();

    // Original telemetry (Wheels only)
    void sendTelemetry(uint32_t time_micros);

    // --- NEW: Overloaded telemetry with IMU data ---
    void sendTelemetry(uint32_t time_micros, const IMUData& imu_data, bool imu_active);

    uint32_t getLastCommandTime() const;
};

#endif
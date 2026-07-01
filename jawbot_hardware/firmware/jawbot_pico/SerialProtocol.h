#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "RobotJoint.h"

class SerialProtocol {
private:
    RobotJoint& left_joint_;
    RobotJoint& right_joint_;
    String input_buffer_;

    // Internal method to decode the strings
    void parseCommand(String command);

public:
    // Constructor requires links to the robot's physical joints
    SerialProtocol(RobotJoint& left_joint, RobotJoint& right_joint);

    // Reads the serial line and handles parsing if a newline is detected
    void processIncoming();

    // Packages and fires the encoder data upstream
    void sendTelemetry(uint32_t time_micros);
};

#endif
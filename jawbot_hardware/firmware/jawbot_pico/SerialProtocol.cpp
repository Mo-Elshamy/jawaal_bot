#include "SerialProtocol.h"

SerialProtocol::SerialProtocol(RobotJoint& left_joint, RobotJoint& right_joint)
    : left_joint_(left_joint), right_joint_(right_joint), input_buffer_(""), last_command_time_(0) {}

void SerialProtocol::processIncoming() {
    while (Serial.available() > 0) {
        char in_char = (char)Serial.read();
        if (in_char == '\n') {
            parseCommand(input_buffer_);
            input_buffer_ = ""; // Flush buffer
        } else {
            input_buffer_ += in_char;
        }
    }
}

void SerialProtocol::parseCommand(String command) {
    if (command.length() == 0) return;
    
    last_command_time_ = millis();
    char type = command.charAt(0);
    String payload = command.substring(2);

    if (type == 'm') { 
        float left_vel = 0.0, right_vel = 0.0;
        if (sscanf(payload.c_str(), "%f %f", &left_vel, &right_vel) == 2) {
            left_joint_.setTargetVelocity(left_vel);
            right_joint_.setTargetVelocity(right_vel);
        }
    } 
    else if (type == 'p') { 
        float kp = 0.0, ki = 0.0, kd = 0.0;
        if (sscanf(payload.c_str(), "%f %f %f", &kp, &ki, &kd) == 3) {
            left_joint_.updatePIDGains(kp, ki, kd);
            right_joint_.updatePIDGains(kp, ki, kd);
        }
    }
}

void SerialProtocol::sendTelemetry(uint32_t time_micros) {
    Serial.print("e ");
    Serial.print(left_joint_.readAbsoluteTicks());
    Serial.print(" ");
    Serial.print(right_joint_.readAbsoluteTicks());
    Serial.print(" t ");
    Serial.print(time_micros);
    Serial.print("\n");
}

uint32_t SerialProtocol::getLastCommandTime() const {
    return last_command_time_;
}
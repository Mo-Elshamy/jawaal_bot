#include "L298n_motor_driver.h"

L298N::L298N(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin)
    : pwm_pin_(pwm_pin), in1_pin_(in1_pin), in2_pin_(in2_pin) {}

void L298N::init() {
    pinMode(pwm_pin_, OUTPUT);
    pinMode(in1_pin_, OUTPUT);
    pinMode(in2_pin_, OUTPUT);
    
    analogWrite(pwm_pin_, 0);
    digitalWrite(in1_pin_, LOW);
    digitalWrite(in2_pin_, LOW);
}

void L298N::setSpeed(float output_pct) {
    if (output_pct > 100.0f) output_pct = 100.0f;
    if (output_pct < -100.0f) output_pct = -100.0f;

    if (output_pct > 0.0f) {
        // Forward
        digitalWrite(in1_pin_, HIGH);
        digitalWrite(in2_pin_, LOW);
        analogWrite(pwm_pin_, static_cast<int>(output_pct * 2.55f)); // 0-100% to 0-255 PWM
    } 
    else if (output_pct < 0.0f) {
        // Reverse
        digitalWrite(in1_pin_, LOW);
        digitalWrite(in2_pin_, HIGH);
        analogWrite(pwm_pin_, static_cast<int>(-output_pct * 2.55f));
    } 
    else {
        // Brake
        digitalWrite(in1_pin_, LOW);
        digitalWrite(in2_pin_, LOW);
        analogWrite(pwm_pin_, 0);
    }
}
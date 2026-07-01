#ifndef L298N_H
#define L298N_H

#include <Arduino.h>

class L298N {
private:
    uint8_t pwm_pin_; // ENA or ENB pin
    uint8_t in1_pin_; // Direction Input 1
    uint8_t in2_pin_; // Direction Input 2

public:
    // Constructor initializes the structural pin mappings
    L298N(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin);

    // Configures physical GPIO hardware registers
    void init();

    // Accepts command constraints from -100.0% to +100.0%
    void setSpeed(float output_pct);
};

#endif
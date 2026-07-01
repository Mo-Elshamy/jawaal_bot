#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <Arduino.h>

class WheelEncoder {
private:
    uint8_t pin_a_;
    uint8_t pin_b_;
    volatile int32_t ticks_; 

public:
    // Constructor
    WheelEncoder(uint8_t pin_a, uint8_t pin_b);
    
    // Configures the pins with internal pull-ups
    void init();
    
    // The core mathematical logic for quadrature tracking
    void handleInterrupt();
    
    // Safely retrieves the current tick count
    int32_t getTicks() const;
    
    // Resets the counter to zero
    void reset();
};

#endif
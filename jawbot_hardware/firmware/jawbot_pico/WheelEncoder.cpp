#include "WheelEncoder.h"

WheelEncoder::WheelEncoder(uint8_t pin_a, uint8_t pin_b)
    : pin_a_(pin_a), pin_b_(pin_b), ticks_(0) {}

void WheelEncoder::init() {
    // INPUT_PULLUP ensures the signal doesn't float when the encoder is between clicks
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);
}

void WheelEncoder::handleInterrupt() {
    // Quadrature logic: compare the states of Phase A and Phase B
    if (digitalRead(pin_a_) == digitalRead(pin_b_)) {
        ticks_++;
    } else {
        ticks_--;
    }
}

int32_t WheelEncoder::getTicks() const {
    return ticks_;
}

void WheelEncoder::reset() {
    ticks_ = 0;
}
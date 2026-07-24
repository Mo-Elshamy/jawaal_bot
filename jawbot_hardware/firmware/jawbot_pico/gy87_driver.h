#ifndef GY87_DRIVER_H
#define GY87_DRIVER_H

#include <Wire.h>
#include <Arduino.h>

// Struct to hold ROS-ready data
struct IMUData {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
};

class GY87 {
public:
    GY87();
    bool init();
    bool update(IMUData &data);

private:
    const uint8_t MPU_ADDR = 0x68;
    const uint8_t HMC_ADDR = 0x1E;

    const float ACCEL_SCALE = 9.80665 / 16384.0;
    const float GYRO_SCALE = 0.0174533 / 131.0;
    const float MAG_SCALE = 0.0000001; 

    const float offset_accel_x = -0.46396;
    const float offset_accel_y = -0.24761;
    const float offset_accel_z = -0.33175;
    const float offset_gyro_x = 0.06721;
    const float offset_gyro_y = 0.02300;
    const float offset_gyro_z = 0.01733;
};

#endif
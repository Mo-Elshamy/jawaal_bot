#include "gy87_driver.h"

GY87::GY87() {}

bool GY87::init() {
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();

    // 1. Wake MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    if(Wire.endTransmission() != 0) return false; // Fail if not found
    delay(10);

    // 2. Enable I2C Bypass for Magnetometer
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission();
    delay(10);

    // 3. Set HMC5883L to Continuous Mode
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x02);
    Wire.write(0x00);
    if(Wire.endTransmission() != 0) return false; 
    delay(10);

    return true;
}

bool GY87::update(IMUData &data) {
    // Read MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 14, (int)true);

    if (Wire.available() == 14) {
        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // Skip Temp
        int16_t gx = Wire.read() << 8 | Wire.read();
        int16_t gy = Wire.read() << 8 | Wire.read();
        int16_t gz = Wire.read() << 8 | Wire.read();

        // Convert to ROS standard units
        data.accel_x = ax * ACCEL_SCALE;
        data.accel_y = ay * ACCEL_SCALE;
        data.accel_z = az * ACCEL_SCALE;
        data.gyro_x = gx * GYRO_SCALE;
        data.gyro_y = gy * GYRO_SCALE;
        data.gyro_z = gz * GYRO_SCALE;
    } else {
        return false;
    }

    // Read HMC5883L
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom((int)HMC_ADDR, 6, (int)true);

    if (Wire.available() == 6) {
        int16_t mx = Wire.read() << 8 | Wire.read();
        int16_t mz = Wire.read() << 8 | Wire.read();
        int16_t my = Wire.read() << 8 | Wire.read();
        
        data.mag_x = mx * MAG_SCALE;
        data.mag_y = my * MAG_SCALE;
        data.mag_z = mz * MAG_SCALE;
    }

    return true;
}
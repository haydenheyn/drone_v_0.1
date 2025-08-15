#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "mpu6050.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

class MPU6050Sensor {
public:

MPU6050Sensor();
~MPU6050Sensor();

bool init(i2c_port_t i2c_port, uint8_t address = MPU6050_I2C_ADDRESS);
bool readAccelerometer(mpu6050_acce_value_t& acce); 
bool readGyroscope(mpu6050_gyro_value_t& gyro);
bool readTemperature(mpu6050_temp_value_t& temp);
bool isInitialized() const {return initialized;};

private:
mpu6050_handle_t handle;
bool initialized;

};

#endif // SENSOR_HPP
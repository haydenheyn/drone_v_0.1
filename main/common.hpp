#ifndef COMMON_HPP
#define COMMON_HPP

#include "mpu6050.h"

typedef struct {
    float pitch;
    float roll;
    float yaw;
    mpu6050_acce_value_t acc;
    mpu6050_gyro_value_t gyro;
    uint32_t timestamp;
} imu_data_t;




typedef struct {
    float pitch;
    float roll;
    float yaw;
    float thrust;
} pid_data_t;

typedef struct {
    float motor_one;
    float motor_two;
    float motor_three;
    float motor_four;
}  motor_pwm_data_t;


#endif
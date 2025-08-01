#include "sensor.hpp"
#include "esp_log.h"
#include "esp_err.h"

static const char* TAG = "MPU6050Sensor";

MPU6050Sensor::MPU6050Sensor() : handle(nullptr), initialized(false) {}

MPU6050Sensor::~MPU6050Sensor() {
    if (handle) {
        mpu6050_delete(handle);
    }
}





bool MPU6050Sensor::init(i2c_port_t i2c_port, uint8_t address) {
    handle = mpu6050_create(i2c_port, address);
    
    if (handle == nullptr) {
        ESP_LOGE(TAG, "Failed to create MPU6050 handle");
        return false;
    }

    if (mpu6050_wake_up(handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
    }

    if (mpu6050_config(handle, ACCE_FS_8G, GYRO_FS_250DPS)!= ESP_OK) { 
        ESP_LOGE(TAG, "Failed to configure MPU6050");
    }

    initialized = true;
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return true;
}


bool MPU6050Sensor::readAccelerometer(mpu6050_acce_value_t& acce) {
    if(!initialized) return false;
    return mpu6050_get_acce(handle, &acce) == ESP_OK;
}


bool MPU6050Sensor::readGyroscope(mpu6050_gyro_value_t& gyro) {
    if(!initialized) return false;
    return mpu6050_get_gyro(handle, &gyro) == ESP_OK;
}


bool MPU6050Sensor::readTemperature(mpu6050_temp_value_t& temp) {
    if(!initialized) return false;
    return mpu6050_get_temp(handle, &temp) == ESP_OK;
}
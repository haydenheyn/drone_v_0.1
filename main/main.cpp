#include <cstdio>
#include <cmath>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include <esp_timer.h>
#include "driver/uart.h"
#include "madgwick_filter.hpp"


#define BLINK_GPIO static_cast<gpio_num_t>(2)

// I2C pins
#define I2C_MASTER_SCL_IO    static_cast<gpio_num_t>(22)
#define I2C_MASTER_SDA_IO    static_cast<gpio_num_t>(21)
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define G_TO_N  9.81f


static const char *TAG = "MPU6050_CPP_APP";
static mpu6050_handle_t mpu6050 = nullptr;




class MPU6050Sensor {
private:
    mpu6050_handle_t handle;
    bool initialized;

public:
    MPU6050Sensor() : handle(nullptr), initialized(false) {}
    
    ~MPU6050Sensor() {
        if (handle) {
            mpu6050_delete(handle);
        }
    }
    
    bool init(i2c_port_t i2c_port, uint8_t address = MPU6050_I2C_ADDRESS) {
        handle = mpu6050_create(i2c_port, address);
        if (handle == nullptr) {
            ESP_LOGE(TAG, "Failed to create MPU6050 handle");
            return false;
        }
        
        esp_err_t ret = mpu6050_wake_up(handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
            return false;
        }
        
        ret = mpu6050_config(handle, ACCE_FS_4G, GYRO_FS_500DPS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure MPU6050: %s", esp_err_to_name(ret));
            return false;
        }
        
        initialized = true;
        ESP_LOGI(TAG, "MPU6050 initialized successfully");
        return true;
    }
    
    bool readAccelerometer(mpu6050_acce_value_t& acce) {
        if (!initialized) return false;
        return mpu6050_get_acce(handle, &acce) == ESP_OK;
    }
    
    bool readGyroscope(mpu6050_gyro_value_t& gyro) {
        if (!initialized) return false;
        return mpu6050_get_gyro(handle, &gyro) == ESP_OK;
    }
    
    bool readTemperature(mpu6050_temp_value_t& temp) {
        if (!initialized) return false;
        return mpu6050_get_temp(handle, &temp) == ESP_OK;
    }

    bool isInitialized() const { return initialized; }
};

class I2CMaster {
public:
    static bool init(gpio_num_t sda_pin, gpio_num_t scl_pin, 
                    i2c_port_t port = I2C_NUM_0, uint32_t freq = 100000) {
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = sda_pin;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = scl_pin;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = freq;
        
        esp_err_t ret = i2c_param_config(port, &conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        ESP_LOGI(TAG, "I2C initialized successfully");
        return true;
    }
};

class LED {
private:
    gpio_num_t pin;
    bool state;

public:
    LED(gpio_num_t gpio_pin) : pin(gpio_pin), state(false) {
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);
    }
    
    void toggle() {
        state = !state;
        gpio_set_level(pin, state ? 1 : 0);
    }
    
    void setState(bool on) {
        state = on;
        gpio_set_level(pin, state ? 1 : 0);
    }
    
    bool getState() const { return state; }
};

// Global objects
static MPU6050Sensor sensor;
static LED statusLED(BLINK_GPIO);

extern "C" {
    void sensor_task(void *pvParameter) {
        mpu6050_acce_value_t acce;
        mpu6050_gyro_value_t gyro;
        mpu6050_temp_value_t temp;
        float pitch,roll,yaw;
        espp::MadgwickFilter filter(.2);
        int64_t last_update_time = esp_timer_get_time();

        
        
        
        ESP_LOGI(TAG, "Starting C++ sensor reading task");
        
        while (true) {
            // Read sensor data using C++ class methods
            if (!sensor.readAccelerometer(acce)) {
                ESP_LOGE(TAG, "Failed to read accelerometer");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            
            if (!sensor.readGyroscope(gyro)) {
                ESP_LOGE(TAG, "Failed to read gyroscope");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            
            if (!sensor.readTemperature(temp)) {
                ESP_LOGE(TAG, "Failed to read temperature");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            int64_t current_time = esp_timer_get_time();
            float dt = (current_time - last_update_time)/1e6f; // Microseconds
            last_update_time = current_time;

            filter.update(dt,-acce.acce_x,acce.acce_y,acce.acce_z,gyro.gyro_x*DEG_TO_RAD,gyro.gyro_y*DEG_TO_RAD,gyro.gyro_z*DEG_TO_RAD);
            filter.get_euler(pitch,roll,yaw);

            /*
            // Print sensor data using C++ iostream (optional, can still use ESP_LOGI)
            ESP_LOGI(TAG, "Accel: X=%.2f, Y=%.2f, Z=%.2f g", 
                     acce.acce_x, acce.acce_y, acce.acce_z);
            ESP_LOGI(TAG, "Gyro: X=%.2f, Y=%.2f, Z=%.2f °/s", 
                     gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            ESP_LOGI(TAG, "Temperature: %.2f°C", temp.temp);

            ESP_LOGI(TAG, "Filter: pitch=%.2f, Roll=%0.2f, Yaw=%.2f",pitch,roll,yaw);
            */
           printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            acce.acce_x, acce.acce_y, acce.acce_z,
            gyro.gyro_x*DEG_TO_RAD, gyro.gyro_y*DEG_TO_RAD, gyro.gyro_z*DEG_TO_RAD,
            pitch, roll, yaw, dt);


            
            vTaskDelay(pdMS_TO_TICKS(2)); // Read every 10ms
        }
    }
    
    void blinky_task(void *pvParameter) {
        ESP_LOGI(TAG, "Starting C++ LED blink task");
        
        while (true) {
            statusLED.setState(false);
            vTaskDelay(pdMS_TO_TICKS(500));
            statusLED.setState(true);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    
    void app_main(void) {
        ESP_LOGI(TAG, "Starting MPU6050 C++ Application");

        
        // Initialize I2C using C++ class
        if (!I2CMaster::init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 
                           I2C_MASTER_NUM, I2C_MASTER_FREQ_HZ)) {
            ESP_LOGE(TAG, "Failed to initialize I2C");
            return;
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Initialize MPU6050 using C++ class
        if (!sensor.init(I2C_MASTER_NUM)) {
            ESP_LOGE(TAG, "Failed to initialize MPU6050 sensor");
            return;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Create tasks
        xTaskCreate(&blinky_task, "blinky_cpp", 2048, nullptr, 5, nullptr);
        xTaskCreate(&sensor_task, "sensor_cpp", 4096, nullptr, 6, nullptr);
        
        ESP_LOGI(TAG, "All C++ tasks created successfully");
    }
}
#include <cstdio>
#include <cmath>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include <esp_timer.h>
#include "driver/uart.h"
#include "madgwick_filter.hpp"
#include "imu_task.hpp"
#include "i2c_master.hpp"
#include "sensor.hpp"



// I2C pins
#define I2C_MASTER_SCL_IO    static_cast<gpio_num_t>(22)
#define I2C_MASTER_SDA_IO    static_cast<gpio_num_t>(21)
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000


#define BLINK_GPIO static_cast<gpio_num_t>(2)
#define DEG_TO_RAD 0.017453292519943295769236907684886f



static const char *TAG = "MPU6050_CPP_APP";
static mpu6050_handle_t mpu6050 = nullptr;





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

class PIDController {
private:
    float kp,ki,kd;
    float integral = 0;
    float last_error = 0;
public:
    
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki) , kd(kd) {}
    
    float update(float setpoint, float measured, float dt) {
        float error = setpoint - measured;
        integral += error * dt;
        float derivative = (error - last_error)/dt;
        last_error = error;
        return kp*error + ki*integral + kd*derivative;
    }

};

extern "C" {

    // Adjusts the commanded PWM for the motors
    void pid_task(void *pvParameter) {
        // Eventually abstract PWM to N
        // Update PID Values based upon the message queue
        // Should Be in PWM
         float pitch_motor_commmad{0.0f};
         float roll_motor_command{0.0f};
         float yaw_motor_command{0.0f};
         float thrust_motor_command{0.0f};

         // Setpoint should be coming from the RC
         float pitch_setpoint{0.0f};
         float roll_setpoint{0.0f};
         float yaw_setpoint{0.0f};
         float thrust_setpoint{0.0f};

         float pitch_measured{0.0f};
         float roll_measured{0.0f};
         float yaw_measured{0.0f};
         float thrust_measured{0.0f};

         PIDController pitch_controller(0,0,0); // kp, ki, kd
         PIDController roll_controller(0,0,0); // kp, ki, kd
         PIDController yaw_controller(0,0,0); // kp, ki, kd
         PIDController thrust_controller(0,0,0); // kp, ki, kd

         //
         float dt  = 0.0f;

         while(1) {
            // Update Measured IMU Data
            // Update Setpoint Data;

             pitch_motor_commmad = pitch_controller.update(pitch_setpoint, pitch_measured, dt);
             roll_motor_command = roll_controller.update(roll_setpoint, roll_measured, dt);
             yaw_motor_command = yaw_controller.update(yaw_setpoint, yaw_measured, dt);


             // Send Information to Motor Command Task

             vTaskDelay(4);
         }
    }
    
   
    MPU6050Sensor sensor;  
    
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
        xTaskCreate(&imu_task, "sensor_cpp", 4096, nullptr, 6, nullptr);
        xTaskCreatePinnedToCore(&pid_task, "pid_cpp", 4096, nullptr, 5, nullptr,1);
        ESP_LOGI(TAG, "All C++ tasks created successfully");
    }
}
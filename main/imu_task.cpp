#include "imu_task.hpp"
#include "sensor.hpp"
#include "madgwick_filter.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_queue.cpp"
#include "common.hpp" 

extern MPU6050Sensor sensor;


// Define constants
#define DEG_TO_RAD 0.0174532925f 



extern "C" void imu_task(void *pvParameter) {
    imu_data_t imu_packet;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t imu_read_frequency = pdMS_TO_TICKS(4); // 250Hz = 4ms

    espp::MadgwickFilter filter(0.15f);  // beta value

    while (true) {
        sensor.readAccelerometer(imu_packet.acc);
        sensor.readGyroscope(imu_packet.gyro);

        TickType_t current_time = xTaskGetTickCount();
        float dt = (current_time - last_wake_time) * portTICK_PERIOD_MS / 1000.0f;
        last_wake_time = current_time;

        filter.update(dt,
                      -imu_packet.acc.acce_x, imu_packet.acc.acce_y, imu_packet.acc.acce_z,
                      imu_packet.gyro.gyro_x * DEG_TO_RAD, imu_packet.gyro.gyro_y * DEG_TO_RAD, imu_packet.gyro.gyro_z * DEG_TO_RAD);

        filter.get_euler(imu_packet.pitch, imu_packet.roll, imu_packet.yaw);
        imu_packet.timestamp = current_time;
        
        
        if (imu_data_queue != nullptr) {
            xQueueSend(imu_data_queue, &imu_packet,0);
        } 

        //printf("%.2f,%.2f,%.2f\n",imu_packet.pitch,imu_packet.roll,imu_packet.yaw);
        vTaskDelayUntil(&last_wake_time, imu_read_frequency);
    }
}

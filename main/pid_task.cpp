 #include "pid_controller.hpp"
 #include "common.hpp"
 #include "freeRtos/queue.h"
 #include "freertos/task.h"
 #include "esp_log.h"
 #include "motor_task.cpp"

QueueHandle_t imu_data_queue;

 void pid_task(void *pvParameter) {
        static const char *TAG2 = "PID_TASK.CPP";
        imu_data_t received_data;
        imu_data_queue = xQueueCreate(10,sizeof(imu_data_t));
        if (imu_data_queue == nullptr) {
            printf("failed\n");
        }

         // Setpoint should be coming from the RC
         float pitch_setpoint{0.0f};
         float roll_setpoint{0.0f};
         float yaw_setpoint{0.0f};
         float thrust_setpoint{0.0f};

         float pitch_measured{0.0f};
         float roll_measured{0.0f};
         float yaw_measured{0.0f};
         float thrust_measured{0.0f};

         PIDController pitch_controller(1,0,0); // kp, ki, kd
         PIDController roll_controller(1,0,0); // kp, ki, kd
         PIDController yaw_controller(1,0,0); // kp, ki, kd
         PIDController thrust_controller(0,0,0); // kp, ki, kd

         // Arbitrary dt
         TickType_t last_update = xTaskGetTickCount();
         float dt  = 0.1f;
        
         pid_data_t pid_packet;
         while(true) {
            if(xQueueReceive(imu_data_queue, &received_data, portMAX_DELAY)) {
                pitch_measured = received_data.pitch;
                roll_measured = received_data.roll;
                yaw_measured = received_data.yaw;
                //ESP_LOGI(TAG2, "Received: pitch=%.2f roll=%.2f yaw%.2f",pitch_measured,roll_measured,yaw_measured);
                TickType_t now = received_data.timestamp;
                dt = (now - last_update)/1000.0f;
                last_update = now;
            }

            
             pid_packet.pitch = pitch_controller.update(pitch_setpoint, pitch_measured, dt);
             pid_packet.roll = roll_controller.update(roll_setpoint, roll_measured, dt);
             pid_packet.yaw = yaw_controller.update(yaw_setpoint, yaw_measured, dt);
             pid_packet.thrust = thrust_controller.update(0,0,.1); // Generic thrust
             //ESP_LOGI(TAG2,"pitch=%.2f roll=%.2f yaw=%.2f thrust%.2f",pid_packet.pitch,pid_packet.roll,pid_packet.yaw,pid_packet.thrust);
             
             // Send Information to Motor Command Task
             if (pid_data_queue != nullptr) {
                xQueueSend(pid_data_queue, &pid_packet, 0);
             }

             vTaskDelay(4);
         }
    }
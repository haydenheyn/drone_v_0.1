 #include "pid_controller.hpp"
 #include "common.hpp"
 #include "freeRtos/queue.h"
 #include "freertos/task.h"
 #include "esp_log.h"

QueueHandle_t imu_data_queue; 


 void pid_task(void *pvParameter) {
        static const char *TAG2 = "PID_TASK.CPP";
        imu_data_t received_data;
        imu_data_queue = xQueueCreate(10,sizeof(imu_data_t));
        if (imu_data_queue == nullptr) {
            printf("failed\n");
        }
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

         PIDController pitch_controller(1,0,0); // kp, ki, kd
         PIDController roll_controller(1,0,0); // kp, ki, kd
         PIDController yaw_controller(1,0,0); // kp, ki, kd
         PIDController thrust_controller(0,0,0); // kp, ki, kd

         // Arbitrary dt
         TickType_t last_update = xTaskGetTickCount();
         float dt  = 0.1f;

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

            
             pitch_motor_commmad = pitch_controller.update(pitch_setpoint, pitch_measured, dt);
             roll_motor_command = roll_controller.update(roll_setpoint, roll_measured, dt);
             yaw_motor_command = yaw_controller.update(yaw_setpoint, yaw_measured, dt);
             ESP_LOGI(TAG2, "Received: pitch=%.2f roll=%.2f yaw%.2f",pitch_motor_commmad,roll_motor_command,yaw_motor_command);

             // Send Information to Motor Command Task

             vTaskDelay(4);
         }
    }
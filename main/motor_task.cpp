#include "freeRtos/queue.h"
#include "freertos/task.h"
#include "common.hpp"
#include "esp_log.h"
#include <algorithm>
#include <driver/mcpwm.h>


QueueHandle_t pid_data_queue;
// Sets the motor pwm for each motor
void motor_task(void * pvParameter) {

    static const char *MOTORTAG = "MOTOR_TASK.CPP";
    pid_data_t received_data;
    motor_pwm_data_t motor_pwm;
    pid_data_queue = xQueueCreate(5,sizeof(pid_data_t));
    
    if (pid_data_queue == nullptr) {
        ESP_LOGE(MOTORTAG, "Failed to Create PID Data Queue");
    }


    while (true) {
        if(xQueueReceive(pid_data_queue, &received_data, portMAX_DELAY)) {

            /* ISSUE WITH HEADING, Heading will continually rotate / float away.
            motor_pwm.motor_one = received_data.thrust  - received_data.roll + received_data.pitch + received_data.yaw; // Front Right
            motor_pwm.motor_two = received_data.thrust  + received_data.roll - received_data.pitch + received_data.yaw; // Back Left
            motor_pwm.motor_three = received_data.thrust  + received_data.roll + received_data.pitch - received_data.yaw; // Front Left
            motor_pwm.motor_four = received_data.thrust  - received_data.roll - received_data.pitch - received_data.yaw; // Back Right
            */
            motor_pwm.motor_one = received_data.thrust  - received_data.roll + received_data.pitch; // Front Right
            motor_pwm.motor_two = received_data.thrust  + received_data.roll - received_data.pitch; // Back Left
            motor_pwm.motor_three = received_data.thrust  + received_data.roll + received_data.pitch; // Front Left
            motor_pwm.motor_four = received_data.thrust  - received_data.roll - received_data.pitch; // Back Right
           
            motor_pwm.motor_one = std::clamp(motor_pwm.motor_one,0.0f,100.0f);
            motor_pwm.motor_two = std::clamp(motor_pwm.motor_two,0.0f,100.0f);
            motor_pwm.motor_three = std::clamp(motor_pwm.motor_three,0.0f,100.0f);
            motor_pwm.motor_four = std::clamp(motor_pwm.motor_four,0.0f,100.0f);

            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor_pwm.motor_one);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, motor_pwm.motor_two);
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, motor_pwm.motor_three);
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, motor_pwm.motor_four);
            
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

            ESP_LOGI(MOTORTAG, "one=%.2f two=%.2f three=%.2f four=%.2f",motor_pwm.motor_one,motor_pwm.motor_two,motor_pwm.motor_three,motor_pwm.motor_four);
        }

    }
}



/*
Suggestions
Fix the PID controller rather than disabling yaw - implement integral wind-up protection and proper reference handling
Move queue creation to system initialization
Add proper error handling for queue operations
Consider adding motor arming logic for safety
Add telemetry for debugging PID performance
*/
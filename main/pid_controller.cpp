#include "pid_controller.hpp"

PIDController::PIDController(float kp, float ki, float kd) : kp(kp), ki(ki) , kd(kd) {}

float PIDController::update(float setpoint, float measured, float dt) {
        float error = setpoint - measured;
        integral += error * dt;
        float derivative = (error - last_error)/dt;
        last_error = error;
        return kp*error + ki*integral + kd*derivative;
}



#ifndef PID_CONTROLLER
#define PID_CONTROLLER

class PIDController {

public:
    PIDController(float kp, float ki, float kd);
    float update(float setpoint, float measured, float dt);

private:
    float kp,ki,kd;
    float integral = 0;
    float last_error = 0;
};

#endif
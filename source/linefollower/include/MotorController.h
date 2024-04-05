#ifndef __MotorController_H__
#define __MotorController_H__

#include "Motor.h"

class MotorController {
public:

    MotorController();
    
    void enable();
    void disable();

    void set_standby_pin(gpio_num_t standby_pin);
    void set_right_motor(gpio_num_t a1, gpio_num_t a2, gpio_num_t pwm);
    void set_left_motor(gpio_num_t b1, gpio_num_t b2, gpio_num_t pwm);
    void set_speed(float speed);
    void stop();
    void forward();
    void backward();
    void right();
    void left();
    void maneuver(uint32_t speed_motor_right, uint32_t speed_motor_left);

private:

    void init_hw();

    Motor right_motor;
    Motor left_motor;
    gpio_config_t io_conf = {};
    gpio_num_t MOTOR_CONTROLLER_STANDBY;

};

#endif //__MotorController_H__

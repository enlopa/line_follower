#ifndef __Motor_H__
#define __Motor_H__

#include "driver/gpio.h"
#include <driver/ledc.h>

class Motor {
public:
    Motor(gpio_num_t a1, gpio_num_t a2, gpio_num_t pwm, ledc_channel_t channel);
    Motor();

    void set_speed(uint32_t speed);
    uint32_t get_speed();
    void stop();
    void forward();
    void backward();

private:

    void init_hw();

    gpio_num_t a1, a2, pwm;
    ledc_channel_t channel;
    uint32_t speed;

};


#endif //__Motor_H__
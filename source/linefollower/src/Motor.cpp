#include "Motor.h"

Motor::Motor() 
{

}

Motor::Motor(gpio_num_t a1_, gpio_num_t a2_, gpio_num_t pwm_, ledc_channel_t channel_) 
{
    a1 = a1_;
    a2 = a2_;
    pwm = pwm_;
    channel = channel_;

    init_hw();
}

void Motor::set_speed(uint32_t speed) 
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    speed = speed;
}

uint32_t Motor::get_speed(void) 
{
    return speed;
}

void Motor::stop(void) 
{
    gpio_set_level(a1, 1);
    gpio_set_level(a2, 1);    
}

void Motor::forward(void) 
{
    gpio_set_level(a1, 1);
    gpio_set_level(a2, 0);
}

void Motor::backward(void) 
{
    gpio_set_level(a1, 0);
    gpio_set_level(a2, 1);
}

void Motor::init_hw(void) 
{
    //Configure PINs     
    io_conf.mode = GPIO_MODE_OUTPUT,
    io_conf.pin_bit_mask = (1ULL << a1) | (1ULL << a2) | (1ULL << pwm), // | (1ULL << STBY),
    io_conf.intr_type = GPIO_INTR_DISABLE,
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE,
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE,
    gpio_config(&io_conf);

    //Definimos el timer
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
    ledc_timer.timer_num = LEDC_TIMER_0, // timer index
    //ledc_timer.freq_hz = 10000, // frequency of PWM signal
    ledc_timer.freq_hz = 500, // frequency of PWM signal
    ledc_timer.clk_cfg = LEDC_AUTO_CLK, // Auto select the source clock
    ledc_timer_config(&ledc_timer);

    // Definimos el canal
    ledc_channel.gpio_num = pwm;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = channel;
    //ledc_channel.intr_type = LEDC_INTR_FADE_END;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    //ledc_channel.flags.output_invert = 0;
    ledc_channel_config(&ledc_channel);
    // Creamos el motor parado
    //gpio_set_level(a1, 0);
    //gpio_set_level(a2, 0);
}
#include "MotorController.h"
#include "esp_err.h"

MotorController::MotorController(void) 
{
    
}

void MotorController::set_standby_pin(gpio_num_t standby_pin) 
{
    MOTOR_CONTROLLER_STANDBY = standby_pin;
    init_hw();
    gpio_set_level(MOTOR_CONTROLLER_STANDBY, 1);
}

void MotorController::enable(void) 
{
    gpio_set_level(MOTOR_CONTROLLER_STANDBY, 1);
}

void MotorController::disable(void) 
{
    gpio_set_level(MOTOR_CONTROLLER_STANDBY, 0);
}

void MotorController::set_right_motor(gpio_num_t a1, gpio_num_t a2, gpio_num_t pwm) 
{
    right_motor = Motor(a1, a2, pwm, LEDC_CHANNEL_0);
}

void MotorController::set_left_motor(gpio_num_t b1, gpio_num_t b2, gpio_num_t pwm) 
{
    left_motor = Motor(b1, b2, pwm, LEDC_CHANNEL_1);
}

void MotorController::set_speed(float speed) 
{
    right_motor.set_speed(speed);
    left_motor.set_speed(speed);
}

void MotorController::stop(void) 
{
    right_motor.stop();
    left_motor.stop();
}

void MotorController::forward(void) 
{
    right_motor.forward();
    left_motor.forward();
}

void MotorController::backward(void) 
{
    right_motor.backward();
    left_motor.backward();
}

void MotorController::right(void) 
{
    right_motor.forward();
    left_motor.backward();
}

void MotorController::left(void) 
{
    right_motor.backward();
    left_motor.forward();
}

void MotorController::init_hw(void) 
{
    gpio_config_t io_conf;

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_CONTROLLER_STANDBY);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void MotorController::maneuver(uint32_t speed_motor_right, uint32_t speed_motor_left)
{
    right_motor.set_speed(speed_motor_right);
    left_motor.set_speed(speed_motor_left);
    forward();
}
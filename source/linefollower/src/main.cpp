#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include <esp_timer.h>

#include "config.h"
#include "MotorController.h"
#include "LineSensorController.h"
#include "pid.h"


MotorController motor_controller;
LineSensorController line_sensor_controller;

extern "C" void app_main();

void init_motor_controller(void) {
    motor_controller.set_standby_pin(MOTOR_STBY_PIN);
    motor_controller.enable();
    motor_controller.set_right_motor(MOTOR_RIGHT_AI1, MOTOR_RIGHT_AI2, MOTOR_RIGHT_PWM);
    motor_controller.set_left_motor(MOTOR_LEFT_BI1, MOTOR_LEFT_BI2, MOTOR_LEFT_PWM);
    motor_controller.set_speed(DEFAULT_SPEED);
}

void init_line_sensor_controller(void)
{
    line_sensor_controller = LineSensorController(LSENSOR_ARRAY_NUM_SENSORS, LSENSOR_ARRAY_ENABLE, LSENSOR_CONTROLLER_SIGNAL, LSENSOR_CONTROLLER_ENABLE, LSENSOR_CONTROLLER_S0, LSENSOR_CONTROLLER_S1, LSENSOR_CONTROLLER_S2, LSENSOR_CONTROLLER_S3);
}

void init_system(void) {
    init_motor_controller();
    init_line_sensor_controller();
}

void app_main() {

    init_system();

    uint64_t last_call = esp_timer_get_time();
    uint64_t curr_time = esp_timer_get_time();
    PID mypid(1,0,0);

    while(1) {
        
        float error = 0;

        curr_time = esp_timer_get_time();

        error = line_sensor_controller.get_line_deviation();
        float current_pid = mypid.update(error, curr_time-last_call);
        motor_controller.maneuver(DEFAULT_SPEED - (current_pid*20), DEFAULT_SPEED + (current_pid*20));
        printf("Valor del pid %f \n", current_pid);
        
        last_call = curr_time;
    }
}
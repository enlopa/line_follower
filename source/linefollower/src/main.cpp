#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"

#include "config.h"
#include "MotorController.h"
#include "LineSensorController.h"
#include "pid.h"


MotorController motor_controller;
LineSensorController line_sensor_controller = LineSensorController(LSENSOR_ARRAY_NUM_SENSORS, LSENSOR_ARRAY_ENABLE, LSENSOR_CONTROLLER_SIGNAL, LSENSOR_CONTROLLER_ENABLE, LSENSOR_CONTROLLER_S0, LSENSOR_CONTROLLER_S1, LSENSOR_CONTROLLER_S2, LSENSOR_CONTROLLER_S3);

extern "C" void app_main();

void init_motor_controller(void) {
    motor_controller.set_standby_pin(MOTOR_STBY_PIN);
    motor_controller.enable();
    motor_controller.set_right_motor(MOTOR_RIGHT_AI1, MOTOR_RIGHT_AI2, MOTOR_RIGHT_PWM);
    motor_controller.set_left_motor(MOTOR_LEFT_BI1, MOTOR_LEFT_BI2, MOTOR_LEFT_PWM);

    motor_controller.set_speed(300);
}

//void init_line_sensor_controller()
//{
    //line_sensor_controller = LineSensorController(LSENSOR_ARRAY_NUM_SENSORS, LSENSOR_ARRAY_ENABLE, LSENSOR_CONTROLLER_SIGNAL, LSENSOR_CONTROLLER_ENABLE, LSENSOR_CONTROLLER_S0, LSENSOR_CONTROLLER_S1, LSENSOR_CONTROLLER_S2, LSENSOR_CONTROLLER_S3);
//}

void init_system(void) {
    init_motor_controller();
    //init_line_sensor_controller();
}

void app_main() {

    init_system();


    while(1) {
        
        float error = 0;
        
        error = line_sensor_controller.get_error_from_sensor_array();
        PID mypid(1,0,0);
  
        //Se debe calcular el tiempo que ha pasado
        float current_pid = mypid.update(error, 5);
        printf("Valor del pid %f \n", current_pid);

        if (current_pid > 0.5f ) {
            motor_controller.maneuver(300 + (current_pid*10), 300 - (current_pid*10));
            printf("PID positivo\n");
        } else if (current_pid < -0.5f) {       
            motor_controller.maneuver(300 + (current_pid*10), 300 - (current_pid*10));
            printf("PID negativo\n");
        } else {
            motor_controller.maneuver(300, 300);
            printf("PID neutro\n");
        }
    }


/*
    ESP_LOGI("main", "Starting Motor running at half speed for 5 seconds with ramp up and ramp down.");
  
    motor_controller.forward();
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    motor_controller.stop();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    motor_controller.backward();
    vTaskDelay(1200 / portTICK_PERIOD_MS);
    motor_controller.stop();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    motor_controller.right();
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    motor_controller.stop();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    motor_controller.left();
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    motor_controller.stop();
*/
}
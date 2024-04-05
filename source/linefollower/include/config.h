#include "driver/gpio.h"
#include "hal/adc_types.h"

// =================================================================================
// MOTOR CONFIGURATION
// =================================================================================

// Motor controller. PIN to enable/disable driver
const gpio_num_t MOTOR_STBY_PIN = GPIO_NUM_33;

// Right motor configuration: PINS for input signals and PWM
const gpio_num_t MOTOR_RIGHT_AI1 = GPIO_NUM_32;
const gpio_num_t MOTOR_RIGHT_AI2 = GPIO_NUM_26;
const gpio_num_t MOTOR_RIGHT_PWM = GPIO_NUM_27;

// Left motor configuration: PINS for input signals and PWM
const gpio_num_t MOTOR_LEFT_BI1 = GPIO_NUM_17;
const gpio_num_t MOTOR_LEFT_BI2 = GPIO_NUM_16;
const gpio_num_t MOTOR_LEFT_PWM = GPIO_NUM_5;

// =================================================================================
// LINE SENSOR CONTROLLER CONFIGURATION
// ==================================================================================

// Mux configuration: PINs defintion
// PIN to read the value of the sensor selected
const adc_channel_t LSENSOR_CONTROLLER_SIGNAL = ADC_CHANNEL_7; //ADC1-7 - PIN 35
// Mux PINs to select the sensor to read
const gpio_num_t LSENSOR_CONTROLLER_S0 = GPIO_NUM_23;
const gpio_num_t LSENSOR_CONTROLLER_S1 = GPIO_NUM_22;
const gpio_num_t LSENSOR_CONTROLLER_S2 = GPIO_NUM_21;
const gpio_num_t LSENSOR_CONTROLLER_S3 = GPIO_NUM_19;
// PIN to Enable/disable Mux
const gpio_num_t LSENSOR_CONTROLLER_ENABLE = GPIO_NUM_18;

// =================================================================================
// LINE SENSOR ARRAY CONFIGURATION
// ==================================================================================

// Number of sensors that the array has
const uint32_t LSENSOR_ARRAY_NUM_SENSORS = 8;
// PIN to Enable/disable sensor array
const gpio_num_t LSENSOR_ARRAY_ENABLE = GPIO_NUM_12;

// =================================================================================
// ROBOT DEFAULT SPEED (0 to 1024)
// ==================================================================================
const uint32_t DEFAULT_SPEED = 600;
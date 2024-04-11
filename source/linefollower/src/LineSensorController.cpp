#include "LineSensorController.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal/adc_types.h"
#include "esp_err.h"

LineSensorController::LineSensorController()
{

}

LineSensorController::LineSensorController(uint32_t lsensor_array_num_sensors_, gpio_num_t lsensor_array_enable_, adc_channel_t mux_signal_channel, gpio_num_t mux_enable_pin_, gpio_num_t mux_s0_pin_, gpio_num_t mux_s1_pin_, gpio_num_t mux_s2_pin_, gpio_num_t mux_s3_pin_) 
{
    lsensor_array_num_sensors = lsensor_array_num_sensors_;
    lsensor_array_enable = lsensor_array_enable_;
    mux_enable_pin = mux_enable_pin_;
    mux_s0_pin = mux_s0_pin_;
    mux_s1_pin = mux_s1_pin_;
    mux_s2_pin = mux_s2_pin_;
    mux_s3_pin = mux_s3_pin_;

    //channel to assign to the ADC Reader
    init_hw(mux_signal_channel);
}

void LineSensorController::init_hw(adc_channel_t mux_signal_channel) 
{
    gpio_config_t io_conf;

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<lsensor_array_enable) | (1ULL<<mux_enable_pin) | (1ULL<<mux_s0_pin) | (1ULL<<mux_s1_pin) | (1ULL<<mux_s2_pin) | (1ULL<<mux_s3_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(lsensor_array_enable, 0);
    gpio_set_level(mux_enable_pin, 0);
    //gpio_set_level(mux_s0_pin, 0);
    //gpio_set_level(mux_s1_pin, 0);
    //gpio_set_level(mux_s2_pin, 0);
    //gpio_set_level(mux_s3_pin, 0);

    adc_reader = ADC_Reader(mux_signal_channel);
}

float LineSensorController::get_line_deviation()
{
    
    ArraySensorData array_sensor_data = read_sensor_array();
    
    return get_error(array_sensor_data);
}

ArraySensorData LineSensorController::read_sensor_array()
{
    ArraySensorData array_sensor_data = ArraySensorData(lsensor_array_num_sensors);
    const uint32_t sensorOffset = 3;
    uint32_t currentSensor;
     
    gpio_set_level(lsensor_array_enable,1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    for (uint32_t i = 0; i < lsensor_array_num_sensors; i++) {
        
        currentSensor = sensorOffset + i;
        gpio_set_level(mux_s0_pin, currentSensor & 0x1U);
        gpio_set_level(mux_s1_pin, (currentSensor>>1) & 0x1U);
        gpio_set_level(mux_s2_pin, (currentSensor>>2) & 0x1U);
        gpio_set_level(mux_s3_pin, (currentSensor>>3) & 0x1U);

        int sensor_value = adc_reader.read_sensor();
        array_sensor_data.raw_values[i] = sensor_value;
        array_sensor_data.normalized_values[i] = map(sensor_value,30,2000,0,100);
    }
    
    gpio_set_level(lsensor_array_enable,0);
    
    return array_sensor_data;
}

float LineSensorController::get_error(ArraySensorData& array_data)
{
    float num = (-28 * (array_data.normalized_values[0] - array_data.normalized_values[7])) + (-20 * (array_data.normalized_values[1] - array_data.normalized_values[6])) + (-12 * (array_data.normalized_values[2] - array_data.normalized_values[5])) + (-4 * (array_data.normalized_values[3] - array_data.normalized_values[4]));
    float denom = array_data.normalized_values[0] + array_data.normalized_values[1] + array_data.normalized_values[2] + array_data.normalized_values[3] + array_data.normalized_values[4] + array_data.normalized_values[5] + array_data.normalized_values[6] + array_data.normalized_values[7];
    float error = num / denom;
    return error;
}

// TODO: this function must be in a helper class
long LineSensorController::map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
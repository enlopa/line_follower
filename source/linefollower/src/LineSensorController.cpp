#include "LineSensorController.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal/adc_types.h"
#include "esp_log.h"


LineSensorController::LineSensorController(uint32_t lsensor_array_num_sensors_, gpio_num_t lsensor_array_enable_, adc_channel_t mux_signal_channel, gpio_num_t mux_enable_pin_, gpio_num_t mux_s0_pin_, gpio_num_t mux_s1_pin_, gpio_num_t mux_s2_pin_, gpio_num_t mux_s3_pin_) 
{
    lsensor_array_num_sensors = lsensor_array_num_sensors_;
    lsensor_array_enable = lsensor_array_enable_;
    mux_enable_pin = mux_enable_pin_;
    mux_s0_pin = mux_s0_pin_;
    mux_s1_pin = mux_s1_pin_;
    mux_s2_pin = mux_s2_pin_;
    mux_s3_pin = mux_s3_pin_;

    //el signal se asigna al lector de adc
    init_hw(mux_signal_channel);
}

void LineSensorController::init_hw(adc_channel_t mux_signal_channel) 
{
    //Activamos todos los gpios
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<lsensor_array_enable) | (1ULL<<mux_enable_pin) | (1ULL<<mux_s0_pin) | (1ULL<<mux_s1_pin) | (1ULL<<mux_s2_pin) | (1ULL<<mux_s3_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(lsensor_array_enable, 0);
    gpio_set_level(mux_enable_pin, 0);
    gpio_set_level(mux_s0_pin, 0);
    gpio_set_level(mux_s1_pin, 0);
    gpio_set_level(mux_s2_pin, 0);
    gpio_set_level(mux_s3_pin, 0);

    adc_reader = ADC_Reader(mux_signal_channel);
}

int LineSensorController::read_sensor_of_array(int position) 
{
    //Configuramos el pin a leer en el mux 
        
    //y leemos el signal
    gpio_set_level(lsensor_array_enable,1);
    
    //TODO: Leer realmente el sensor que se pasa por parámetro
    //Activamos el pin S3 para leer el adc 4
    //gpio_set_level(mux_s2_pin,1);
    gpio_set_level(mux_s0_pin,1);
    gpio_set_level(mux_s1_pin,1);
    
    return adc_reader.read_sensor();
    gpio_set_level(lsensor_array_enable,0); 
}


float LineSensorController::get_error_from_sensor_array()
{
    
    ArraySensorData array_sensor_data = read_sensor_array();
    
    return getError(array_sensor_data);
}

ArraySensorData LineSensorController::read_sensor_array()
{
    ArraySensorData array_sensor_data = ArraySensorData(lsensor_array_num_sensors);
    const uint32_t sensorOffset = 3;
    uint32_t currentSensor;
    //int sensor_normalized_values[lsensor_array_num_sensors];
    
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

float LineSensorController::getError(ArraySensorData& array_data)
{
    float num = (-28 * (array_data.normalized_values[0] - array_data.normalized_values[7])) + (-20 * (array_data.normalized_values[1] - array_data.normalized_values[6])) + (-12 * (array_data.normalized_values[2] - array_data.normalized_values[5])) + (-0.4 * (array_data.normalized_values[3] - array_data.normalized_values[4]));
    float denom = array_data.normalized_values[0] + array_data.normalized_values[1] + array_data.normalized_values[2] + array_data.normalized_values[3] + array_data.normalized_values[4] + array_data.normalized_values[5] + array_data.normalized_values[6] + array_data.normalized_values[7];
    float error = num / denom;
    return error;
}

// TODO: this function must be in a helper class
long LineSensorController::map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



// https://theultimatelinefollower.blogspot.com/2015/12/interpolation.html
// https://theultimatelinefollower.blogspot.com/2015/12/reading-calibrating-and-normalizing.html

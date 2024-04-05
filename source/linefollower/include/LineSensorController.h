#ifndef __LineSensorController_H__
#define __LineSensorController_H__

#include "driver/gpio.h"
#include "ADC_Reader.h"
#include "ArraySensorData.h"


class LineSensorController {
public:
    LineSensorController();
    LineSensorController(uint32_t lsensor_array_num_sensors, gpio_num_t lsensor_array_enable, adc_channel_t mux_signal_channel, gpio_num_t mux_enable_pin, gpio_num_t mux_s0_pin, gpio_num_t mux_s1_pin, gpio_num_t mux_s2_pin, gpio_num_t mux_s3_pin);
    ArraySensorData read_sensor_array();
    float get_error_from_sensor_array();
    
private:
    void init_hw(adc_channel_t mux_signal_channel);
    long map(int x, int in_min, int in_max, int out_min, int out_max);
    float get_error(ArraySensorData& array_data);
    
    ADC_Reader adc_reader;
    adc_channel_t mux_signal_channel;
    gpio_config_t io_conf = {};
    gpio_num_t lsensor_array_enable, mux_enable_pin, mux_s0_pin, mux_s1_pin, mux_s2_pin, mux_s3_pin;
    uint32_t lsensor_array_num_sensors;
  
};

#endif //__LineSensorController_H__
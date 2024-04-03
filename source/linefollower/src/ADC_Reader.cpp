#include "ADC_Reader.h"

ADC_Reader::ADC_Reader() 
{

}

ADC_Reader::ADC_Reader(adc_channel_t adc_channel) 
{
    channel = adc_channel;
    init_hw();
}

void ADC_Reader::init_hw(void) 
{
    //Config ADC
    
    init_config1.unit_id = ADC_UNIT_1,
    init_config1.ulp_mode = ADC_ULP_MODE_DISABLE,
    //ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    
    config.atten = ADC_ATTEN_DB_11,
    config.bitwidth = ADC_BITWIDTH_11,
    //ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    adc_oneshot_config_channel(adc1_handle, channel, &config);
    
}

int ADC_Reader::read_sensor(void) 
{
    //return ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw[0][0]));
    static int adc_raw[2][10];
    //ESP_LOGE("main", "Valor del sensor: %d", sensor_value);
    adc_oneshot_read(adc1_handle, channel, &adc_raw[0][0]);
    return adc_raw[0][0];
    //return 1;
}

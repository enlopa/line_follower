#include "ADC_Reader.h"
#include "esp_err.h"

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
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    
}

int ADC_Reader::read_sensor(void) 
{
    static int adc_raw[2][10];
    adc_oneshot_read(adc1_handle, channel, &adc_raw[0][0]);
    return adc_raw[0][0];
}

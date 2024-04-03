#ifndef __ADC_Reader_H__
#define __ADC_Reader_H__

#include "hal/adc_types.h"
#include <esp_adc/adc_oneshot.h>

class ADC_Reader {
public:
    ADC_Reader();
    ADC_Reader(adc_channel_t adc_channel);

    int read_sensor();

private:

    void init_hw();

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {};
    adc_oneshot_chan_cfg_t config = {};
    adc_channel_t channel;
};

#endif //__ADC_Reader_H__
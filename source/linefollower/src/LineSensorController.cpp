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
    
    //TODO: BORRAR
    //Activamos el pin S3 para leer el adc 4
    //gpio_set_level(mux_s2_pin,1);
    gpio_set_level(mux_s0_pin,1);
    gpio_set_level(mux_s1_pin,1);
    
    return adc_reader.read_sensor();
    gpio_set_level(lsensor_array_enable,0); 
}

/*
void LineSensorController::read_sensor_array()
{
    int sensor_values[8];
    int indexed_array[8][4] = {
        {1,1,0,0},  //Sensor 1 is connected to the C3 PIN in the mux
        {0,0,1,0},  //Sensor 2 is connected to the C4 PIN in the mux
        {1,0,1,0},  //Sensor 3 is connected to the C5 PIN in the mux
        {0,1,1,0},  //Sensor 4 is connected to the C6 PIN in the mux
        {1,1,1,0},  //Sensor 5 is connected to the C7 PIN in the mux
        {0,0,0,1},  //Sensor 6 is connected to the C8 PIN in the mux
        {1,0,0,1},  //Sensor 7 is connected to the C9 PIN in the mux
        {0,1,0,1}   //Sensor 8 is connected to the C10 PIN in the mux
    };

    gpio_num_t array_mux_pines[4] = {mux_s0_pin, mux_s1_pin, mux_s2_pin, mux_s3_pin};

    gpio_set_level(lsensor_array_enable,1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // For each sensor: activate pins in the mux and read the signal PIN to get the value of each sensor
    int i,j;
    for (i=0; i<8; i++) {
        for (j=0;j<4;j++) {
            gpio_set_level(array_mux_pines[j],indexed_array[i][j]);
            //ESP_LOGE("LineSensorController", "Sensor activado %d: S%d: %d", i, j, indexed_array[i][j]); 
        }
        int sensor_value = adc_reader.read_sensor();
        sensor_values[i] = sensor_value;
        //ESP_LOGE("LineSensorController", "VAlor Sensor %d: %d", i, sensor_value); 
    }

    gpio_set_level(lsensor_array_enable,0);
    
    ESP_LOGI("LineSensorController", "Valores de los sensores %4d | %4d | %4d | %4d | %4d | %4d | %4d | %4d", sensor_values[0],sensor_values[1],sensor_values[2],sensor_values[3],sensor_values[4],sensor_values[5],sensor_values[6],sensor_values[7]); 
    
    //Normalizamos los valores
    for (i=0; i<8; i++) {
        sensor_values[i] = map(sensor_values[i],30,2000,0,100);
    }

    ESP_LOGI("LineSensorController", "Valores normalizados %4d | %4d | %4d | %4d | %4d | %4d | %4d | %4d", sensor_values[0],sensor_values[1],sensor_values[2],sensor_values[3],sensor_values[4],sensor_values[5],sensor_values[6],sensor_values[7]); 
}
*/

//void LineSensorController::maneuver()
//{
//  normalized_values = read_normalized_sensor_array();
//    error = getError(normalized_values);
//    PID mypid(1,0,0);
    //Se debe calcular el tiempo que ha pasado
//    float pid_value = mypid.update(error, 8);
//printf("Valor del pid %f \n", mypid.update(error, 8));




//}

float LineSensorController::get_error_from_sensor_array()
{
    /*
    ArraySensorData array_sensor_data = ArraySensorData(lsensor_array_num_sensors);
    const uint32_t sensorOffset = 3;
    uint32_t currentSensor;
    int sensor_normalized_values[lsensor_array_num_sensors];
    
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
        //Esta linea sobrará cuando pase al objeto y ya no se use el array.
        sensor_normalized_values[i] = map(sensor_value,30,2000,0,100);
    }
    
    gpio_set_level(lsensor_array_enable,0);
    return getError(sensor_normalized_values);
    */
    ArraySensorData array_sensor_data = read_sensor_array();
    //printf("=================================================================%d", array_sensor_data.normalized_values[7]);
    return getError(array_sensor_data);
    //return 0;

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
        
        //Esta linea sobrará cuando pase al objeto y ya no se use el array.
        //sensor_normalized_values[i] = map(sensor_value,30,2000,0,100);
    }
    
    gpio_set_level(lsensor_array_enable,0);
    
    return array_sensor_data;
}

/*
void LineSensorController::read_sensor_array()
{
    const uint32_t sensorOffset = 3;
    uint32_t currentSensor;
    int sensor_raw_values[lsensor_array_num_sensors];
    int sensor_normalized_values[lsensor_array_num_sensors];
    
    gpio_set_level(lsensor_array_enable,1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    float error = 0;

    for (uint32_t i = 0; i < lsensor_array_num_sensors; i++) {
        
        currentSensor = sensorOffset + i;
        gpio_set_level(mux_s0_pin, currentSensor & 0x1U);
        gpio_set_level(mux_s1_pin, (currentSensor>>1) & 0x1U);
        gpio_set_level(mux_s2_pin, (currentSensor>>2) & 0x1U);
        gpio_set_level(mux_s3_pin, (currentSensor>>3) & 0x1U);

        int sensor_value = adc_reader.read_sensor();
        sensor_raw_values[i] = sensor_value;
        sensor_normalized_values[i] = map(sensor_value,30,2000,0,100);

        printf("%d ", sensor_raw_values[i]);
    }
    printf("\n");
    error = getError(sensor_normalized_values);
    printf("Valor del error %f \n", error);
    gpio_set_level(lsensor_array_enable,0);
    
    //PID mypid(1,0,0);
    //Se debe calcular el tiempo que ha pasado
    //printf("Valor del pid %f \n", mypid.update(error, 8));
}
*/

float LineSensorController::getError(int sensor_normalized_values[])
{
    float num = (-28 * (sensor_normalized_values[0] - sensor_normalized_values[7])) + (-20 * (sensor_normalized_values[1] - sensor_normalized_values[6])) + (-12 * (sensor_normalized_values[2] - sensor_normalized_values[5])) + (-0.4 * (sensor_normalized_values[3] - sensor_normalized_values[4]));
    float denom = sensor_normalized_values[0] + sensor_normalized_values[1] + sensor_normalized_values[2] + sensor_normalized_values[3] + sensor_normalized_values[4] + sensor_normalized_values[5] + sensor_normalized_values[6] + sensor_normalized_values[7];
    float error = num / denom;

    return error;
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

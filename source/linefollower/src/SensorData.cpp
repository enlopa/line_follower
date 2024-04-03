#include "ArraySensorData.h"

ArraySensorData::ArraySensorData(int num_sensors) {
    num_sensors = num_sensors;
    raw_values.reserve(num_sensors);
    normalized_values.reserve(num_sensors);
}

void ArraySensorData::reset_sensor_data(void) {
    raw_values.clear();
    normalized_values.clear();
}
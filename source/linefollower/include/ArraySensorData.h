#ifndef __ARRAYSENSORDATA_H__
#define __ARRAYSENSORDATA_H__

#include "vector"

class ArraySensorData {
public:
    ArraySensorData(int num_sensors);

    void reset_sensor_data();

    std::vector<int> raw_values;
    std::vector<int> normalized_values;

private:
    
};

#endif //__ARRAYSENSORDATA_H__
//
// Created by pettinz on 31/03/19.
//

#ifndef TRYSPI_SENSOR_H
#define TRYSPI_SENSOR_H

#include "sensor_t.h"

template <class T>
class Sensor {
    sensor_t type;
    T value;

public:
    Sensor() = default;
    Sensor(sensor_t type, T value);

    void setValue(T value);
    T getValue();
    sensor_t getType();
};

#endif //TRYSPI_SENSOR_H

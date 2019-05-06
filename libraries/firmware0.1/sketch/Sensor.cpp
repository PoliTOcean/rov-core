//
// Created by pettinz on 31/03/19.
//

#include "Sensor.h"

template <class T>
Sensor<T>::Sensor(sensor_t type, T value) : type(type), value(value) {}

template <class T>
void Sensor<T>::setValue(T value) { this->value = value; }

template <class T>
T Sensor<T>::getValue() { return value; }

template <class T>
sensor_t Sensor<T>::getType() { return type; }

template class Sensor<unsigned char>;

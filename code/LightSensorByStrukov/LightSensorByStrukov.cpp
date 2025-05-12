#include "LightSensorByStrukov.h"

LightSensor::LightSensor();
LightSensor::setup(int pin) // Первичная настройка
{
    sensor_pin = pin;
    isCalb = false;
    pinMode(pin, INPUT)
}

LightSensor::readRawBright() // Необработанное значение с датчика
{
    return analogRead(pin);
}

LightSensor::calb(int b, int w) // Калибровка датчика
{
    isCalb = true;
    black = b;
    white = w;
}

LightSensor::readCalbBright(int max) // Значение с датчика с учетом калибровки
{
    if (isCalb)
    {
        return constrain(map(analogRead(sensor_pin), black, white, 0, max), 0, max)
    }
}
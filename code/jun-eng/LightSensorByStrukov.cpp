#include "LightSensorByStrukov.h"

LightSensor::LightSensor() {}
void LightSensor::setup(int _pin) // Первичная настройка
{
    sensor_pin = _pin;
    isCalb = false;
    pinMode(sensor_pin, INPUT);
}

int LightSensor::readRawBright() // Необработанное значение с датчика
{
    return analogRead(sensor_pin);
}

void LightSensor::calb(int b, int w) // Калибровка датчика
{
    isCalb = true;
    black = b;
    white = w;
}

int LightSensor::readCalbBright(int max) // Значение с датчика с учетом калибровки
{
    if (isCalb)
    {
        return constrain(map(analogRead(sensor_pin), black, white, 0, max), 0, max);
    }
}
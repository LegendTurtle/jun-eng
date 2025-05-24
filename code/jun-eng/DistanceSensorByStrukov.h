#include <Arduino.h>

class DistanceSensor
{
public:
    DistanceSensor();
    void setup(int _pin_trig, int _pin_echo); // Первичная настройка
    int readDistance(); // Чтение расстояния

private:
    int trig_pin;
    int echo_pin;
};
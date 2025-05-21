#include <Arduino.h>

class LightSensor
{
public:
    LightSensor();
    void setup(int _pin);
    int readRawBright();
    int readCalbBright(int max);
    void calb(int b, int w);

private:
    int sensor_pin;
    int black;
    int white;
    bool isCalb;
};
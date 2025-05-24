#include <Arduino.h>

#define FORWARD 0
#define BACKWARD 1
#define STOP 2

class MotorControl
{
public:
    MotorControl();
    void setup(int c_pin, int c_pin2, int sp_pin); // Первичная настройка
    void run(int sp, int orient); // Запуск моторов
    void release(); // Остановка без удерживания

private:
    int control_pin_a;
    int control_pin_b;
    int speed_pin;
};
#include "MotorControlByStrukov.h"

MotorControl::MotorControl() {}

void MotorControl::setup(int c_pin, int c_pin2, int sp_pin) // Первичная настройка
{
    control_pin_a = c_pin;
    control_pin_b = c_pin2;
    speed_pin = sp_pin;

    pinMode(control_pin_a, OUTPUT);
    pinMode(control_pin_b, OUTPUT);
    pinMode(speed_pin, OUTPUT);
}

void MotorControl::run(int sp, int orient) // Запуск моторов
{
    if (orient == FORWARD) // Проезд вперед
    {
        digitalWrite(control_pin_a, HIGH);
        digitalWrite(control_pin_b, LOW);
    }
    else if (orient == BACKWARD) // Проезд назад
    {
        digitalWrite(control_pin_a, LOW);
        digitalWrite(control_pin_b, HIGH);
    }
    else if (orient == STOP) // Остановка и удерживание
    {
        digitalWrite(control_pin_a, LOW);
        digitalWrite(control_pin_b, LOW);
    }
    analogWrite(speed_pin, sp);
}

void MotorControl::release() // Остановка без удерживания
{
    digitalWrite(speed_pin, LOW);
}
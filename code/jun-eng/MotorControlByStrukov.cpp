#include "MotorControlByStrukov.h"

MotorControl::MotorControl() {}

void MotorControl::setup(int c_pin, int c_pin2, int sp_pin)
{
    control_pin_a = c_pin;
    control_pin_b = c_pin2;
    speed_pin = sp_pin;

    pinMode(control_pin_a, OUTPUT);
    pinMode(control_pin_b, OUTPUT);
    pinMode(speed_pin, OUTPUT);
}

void MotorControl::run(int sp, int orient)
{
    if (orient == FORWARD)
    {
        digitalWrite(control_pin_a, HIGH);
        digitalWrite(control_pin_b, LOW);
    }
    else if (orient == BACKWARD)
    {
        digitalWrite(control_pin_a, LOW);
        digitalWrite(control_pin_b, HIGH);
    }
    else if (orient == STOP)
    {
        digitalWrite(control_pin_a, LOW);
        digitalWrite(control_pin_b, LOW);
    }
    analogWrite(speed_pin, sp);
}

void MotorControl::release()
{
    digitalWrite(speed_pin, LOW);
}
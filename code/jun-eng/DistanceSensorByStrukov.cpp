#include "DistanceSensorByStrukov.h"

DistanceSensor::DistanceSensor(){}

void DistanceSensor::setup(int _pin_trig, int _pin_echo) {
  trig_pin = _pin_trig;
  echo_pin = _pin_echo;
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
}

int DistanceSensor::readDistance(){
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(4);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  long t = pulseIn(echo_pin, HIGH);

  return t / 29 / 2;
}
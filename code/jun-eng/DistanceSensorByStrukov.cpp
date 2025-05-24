#include "DistanceSensorByStrukov.h"

DistanceSensor::DistanceSensor(){}

void DistanceSensor::setup(int _pin_trig, int _pin_echo) { // Первичная астройка
  trig_pin = _pin_trig;
  echo_pin = _pin_echo;
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
}

int DistanceSensor::readDistance(){ // Чтение расстояния
  // Подача сигнала для создания ультразвуковой волны
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(4);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  long t = pulseIn(echo_pin, HIGH); // Принятие сигнала

  return t / 29.1 / 2; // расчет
}
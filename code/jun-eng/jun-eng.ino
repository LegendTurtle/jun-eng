#include "LightSensorByStrukov.h"  // Подключение библиотек
#include "MotorControlByStrukov.h"
#include "DistanceSensorByStrukov.h"
#include <Servo.h>

// Константы
#define LEFT_SPEED 100  // Скорость езды по линии
#define RIGHT_SPEED 115 

#define EN1 3 // Регулировка скорости 1-го мотора
#define EN2 5 // Регулировка скорости 2-го мотора 

#define M1A 4 // Регулировка направления езды 1-го мотора
#define M1B 2

#define M2A 7 // Регулировка направления езды 2-го мотора
#define M2B 8

#define TRIG 9 // Пины для датчика расстояния
#define ECHO 10

#define SERVO 6 // Пин для сервопривода

#define BUT 13 // Пин, к которому подключена кнопка


DistanceSensor beater_sensor;  // Объявление датчика расстояния

Servo beater;  // Объявление сервопривода

LightSensor left_sensor;  // Объявление датчиков отраженного света
LightSensor right_sensor;

MotorControl left_motor;  // Объявление моторов
MotorControl right_motor;

void debug_sensors(){ // Отладка датчиков
  Serial.print(left_sensor.readRawBright());
  Serial.print(" ");
  Serial.println(right_sensor.readRawBright());
}

int PID(int sensor1_value, int sensor2_value, float kp, float kd, float ki, float dt, int minOut, int maxOut) { // ПИД регулятор
  static int prevErr = 0, integral = 0; // Единоразовое обнуление переменных
  int err = sensor1_value - sensor2_value; // P составляющая
  int D = (err - prevErr) / dt; // D составляющая
  integral = constrain(integral + (err * dt * ki), minOut, maxOut); // I составляющая с ограничением значений
  prevErr = err; // Запись предыдущей ошибки
  return (err * kp) + (integral) + (D * kd); // Возвращение результатов расчета с учетом коэфицентов
}

void line() { // Езда по линии
  int PID_res = PID(left_sensor.readCalbBright(100), right_sensor.readCalbBright(100), 0.72, 0.002, 0.0024, 0.001, -120, 120); // Расчет управляющего воздействия

  left_motor.run(LEFT_SPEED + PID_res, FORWARD); // Управление моторами 
  right_motor.run(RIGHT_SPEED - PID_res, FORWARD);
}

void to_the_next_bar() {
  while (beater_sensor.readDistance() > 12) { // Езда по линии до момента, когда обнаружит ворота
    line();
  }
  while (beater_sensor.readDistance() < 12) { // Езда по линии до потери видимости ворот
    line();
  }
  left_motor.run(0, STOP); // Остановка
  right_motor.run(0, STOP);
  delay(250);

  left_motor.run(LEFT_SPEED, FORWARD); // Проезд на достаточное расстояние от ворот
  right_motor.run(RIGHT_SPEED, FORWARD);
  delay(400);

  left_motor.run(0, STOP); // Остановка
  right_motor.run(0, STOP);
  delay(500);
}

void beat(){
  beater.write(0); // Подъем механизма сбития

  left_motor.run(LEFT_SPEED, BACKWARD); // Сбитие планки
  right_motor.run(RIGHT_SPEED, BACKWARD);
  delay(750);

  left_motor.run(0, STOP); // Остановка
  right_motor.run(0, STOP);

  while (beater_sensor.readDistance() < 12) { // Езда по линии до потери видимости ворот
    line();
  }

  left_motor.run(0, STOP); // Остановка
  right_motor.run(0, STOP);
  
  beater.write(55); // Деактивация механизма сбития
}

void setup() {
  Serial.begin(9600); // Запуск Serial-порта для отладки

  pinMode(BUT, INPUT_PULLUP); // Настройка пина, подключенного к кнопке на подтяжку к +

  beater.attach(SERVO);  // Настройка сервопривода

  beater_sensor.setup(TRIG, ECHO); // Настройка датчика расстояния

  left_sensor.setup(A4);  // Настройка датчиков отраженного света
  right_sensor.setup(A5);

  left_sensor.calb(375, 220);  // Калибровка датчиков отраженного света
  right_sensor.calb(330, 150);

  left_motor.setup(M1A, M1B, EN1);  // Настройка моторов
  right_motor.setup(M2A, M2B, EN2);

  beater.write(55); // Начальная деактивация механизма сбития
}

void loop() {
  if (!digitalRead(BUT)) { // Начало алгоритма
    program();
  }
}

void program() { // Алгоритм для выполнения регламента: Езда до 2-х по счету ворот, их сбитие, проезд до следующих и остановка
  to_the_next_bar();
  to_the_next_bar();
  beat();
  to_the_next_bar();
}
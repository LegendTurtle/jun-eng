#include "LightSensorByStrukov.h"  // Подключение библиотек
#include "MotorControlByStrukov.h"
#include "DistanceSensorByStrukov.h"
#include <Servo.h>

// Константы
#define LEFT_LINE_SPEED 100  // Скорость езды по линии
#define RIGHT_LINE_SPEED 115
#define BUT 13

DistanceSensor beater_sensor;  // Объявление датчика расстояния

Servo beater;  // Объявление сервопривода

LightSensor left_sensor;  // Объявление датчиков отраженного света
LightSensor right_sensor;

MotorControl left_motor;  // Объявление моторов
MotorControl right_motor;

int PID2(int sensor1_value, int sensor2_value, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  int err = sensor1_value - sensor2_value;
  static int integral = 0, prevErr = 0;
  integral = constrain(integral + (err * dt * ki), minOut, maxOut) * 0;
  int D = (err - prevErr) / dt;
  prevErr = err;
  return constrain((err * kp) + (integral) + (D * kd), minOut, maxOut);
}

int PID(int sensor1_value, int sensor2_value, float kp, float kd, float ki, float dt, int minOut, int maxOut) {
  static int prevErr = 0, integral = 0;
  int err = sensor1_value - sensor2_value;
  int D = (err - prevErr) / dt;
  integral = constrain(integral + (err * dt * ki), minOut, maxOut);
  prevErr = err;
  return (err * kp) + (integral) + (D * kd);
}

void line() {
  int PID_res = PID(left_sensor.readCalbBright(100), right_sensor.readCalbBright(100), 0.72, 0.002, 0.0024, 0.001, -120, 120);
  left_motor.run(LEFT_LINE_SPEED + PID_res, FORWARD);
  right_motor.run(RIGHT_LINE_SPEED - PID_res, FORWARD);
}

void to_the_next_bar() {
  while (beater_sensor.readDistance() > 12) {
    line();
  }
  while (beater_sensor.readDistance() < 12) {
    line();
  }
  left_motor.run(0, STOP);
  right_motor.run(0, STOP);
  left_motor.run(100, FORWARD);
  right_motor.run(115, FORWARD);
  delay(600);
  left_motor.run(0, STOP);
  right_motor.run(0, STOP);
}

void beat(){
  beater.write(0);
  left_motor.run(0, STOP);
  right_motor.run(0, STOP);
  delay(500);
  left_motor.run(100, BACKWARD);
  right_motor.run(115, BACKWARD);
  delay(750);
  left_motor.run(0, STOP);
  right_motor.run(0, STOP);
  while (beater_sensor.readDistance() < 12) {
    line();
  }
  left_motor.run(0, STOP);
  right_motor.run(0, STOP);
  beater.write(55);
}

void setup() {
  Serial.begin(9600);

  pinMode(BUT, INPUT_PULLUP);

  beater.attach(6);  // Настройка сервопривода

  beater_sensor.setup(9, 10);

  left_sensor.setup(A4);  // Настройка датчиков отраженного света
  right_sensor.setup(A5);

  left_sensor.calb(375, 220);  // Калибровка датчиков отраженного света
  right_sensor.calb(330, 150);

  left_motor.setup(4, 2, 3);  // Настройка моторов
  right_motor.setup(7, 8, 5);

  beater.write(55);
}

void loop() {
  if (!digitalRead(BUT)) {
    program();
  }
}

void program() {
  //left_motor.run(100, FORWARD);
  //right_motor.run(128, FORWAR,D);
  //Serial.print(left_sensor.readRawBright());
  //Serial.print(" ");
  //Serial.println(right_sensor.readRawBright());
  //Serial.println(beater_sensor.readDistance());
  to_the_next_bar();
  to_the_next_bar();
  beat();
  to_the_next_bar();
  //while (beater_sensor.readDistance() < 12) {
  //  line();
  //}
}
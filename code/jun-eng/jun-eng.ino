#include "LightSensorByStrukov.h"  // Подключение библиотек
#include "MotorControlByStrukov.h"

#define LINE_SPEED = 350
#define kP = 1
#define kI = 1
#define kD = 1

LightSensor left_sensor;  // Объявление датчиков отраженного света
LightSensor right_sensor;

MotorControl left_motor;  // Объявление моторов
MotorControl right_motor;

int PID(float sensor1_value, float sensor2_value, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = sensor1 – sensor2;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void line(){
  int PID_res = PID(left_sensor.readCalbValue(), right_sensor.readCalbValue(), kP, kI, kD, 0.01, 0, 350);
  left_motor.run(LINE_SPEED + PID_res, FORWARD);
  right_motor.run(LINE_SPEED - PID_res, FORWARD)
}

void setup() {
  Serial.begin(9600);

  left_sensor.setup(A6);  // Настройка датчиков отраженного света
  right_sensor.setup(A7);

  left_sensor.calb(740, 420);  // Калибровка датчиков отраженного света
  right_sensor.calb(660, 300);

  left_motor.setup(4, 2, 3); // Настройка моторов
  right_motor.setup(7, 8, 5);
}

void loop() {
  static int PID_res = PID(left_sensor.readCalbValue(), right_sensor.readCalbValue());
}
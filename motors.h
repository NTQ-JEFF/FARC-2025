#ifndef MOTORS_H
#define MOTORS_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==== Kênh PCA9685 ====
#define PWM_LINEAR1 14
#define PWM_LINEAR2 15
#define MOTOR1A 10
#define MOTOR1B 11
#define MOTOR2A 12
#define MOTOR2B 13
#define SERVO_1 6
#define SERVO_2 7
#define SERVO_3 5

#define SERVO_MIN 500  // microseconds cho góc 0°
#define SERVO_MAX 2500 // microseconds cho góc 180°

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void driveMotors(int m1a, int m1b, int m2a, int m2b, int turnL, int turnR) {
  int n1a = constrain(m1a - turnL + turnR, 0, 4095);
  int n1b = constrain(m1b - turnL + turnR, 0, 4095);
  int n2a = constrain(m2a + turnL - turnR, 0, 4095);
  int n2b = constrain(m2b + turnL - turnR, 0, 4095);

  pwm.setPin(MOTOR1A, n1a, 0);
  pwm.setPin(MOTOR1B, n1b, 0);
  pwm.setPin(MOTOR2A, n2a, 0);
  pwm.setPin(MOTOR2B, n2b, 0);
}

void driveLinear(int speed, bool forward) {
  pwm.setPin(PWM_LINEAR1, forward ? speed : 0, 0);
  pwm.setPin(PWM_LINEAR2, forward ? 0 : speed, 0);
}

void setServo(int channel, int angle) {
  int us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  int tick = us * 4096 / 20000;
  pwm.setPWM(channel, 0, tick);
}

void initMotors() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  driveMotors(0, 0, 0, 0, 0, 0);
  driveLinear(0, true);
  setServo(SERVO_1, 35);
  setServo(SERVO_2, 0);
  setServo(SERVO_3, 90);
}

#endif

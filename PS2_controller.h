#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <PS2X_lib.h>
#include "motors.h"

// ==== PS2 ====
PS2X ps2x;

// ==== Pin PS2 ====
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14

// ==== Joystick ====
#define JOY_CENTER_Y 128  // PSS_LY
#define JOY_CENTER_X 127  // PSS_RX

#define MAX_DRIVE 1250
#define MAX_LINEAR 4000
#define SMOOTH_STEP 125

#define TURN_SCALE 0.9

// ==== Biến điều khiển lái ====
int targetLeft = 0;
int targetRight = 0;
int driveLeft = 0;
int driveRight = 0;
float LEFT_SCALE;

// ==== Biến điều khiển Linear ====
int targetLinear = 0;   // có dấu
int linearActual = 0;

// ==== Prototype ====
int RL_CHECK();

// ==== Setup PS2 ====
void setupPS2controller() {
  int error = -1;
  while (error != 0) {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
    if (error == 0) {
      Serial.println("✅ PS2 Controller connected!");
    } else {
      Serial.println("❌ PS2 not connected. Retrying...");
      delay(500);
    }
  }
}

// ==== Smooth ====
int smooth(int now, int target) {
  if (now < target) {
    now += SMOOTH_STEP;
    if (now > target) now = target;
  } else if (now > target) {
    now -= SMOOTH_STEP;
    if (now < target) now = target;}
  return now;
}

// ==== Điều khiển ====
void PS2controlSmooth() {
  ps2x.read_gamepad(0, 0);

  // Neutral → Stop all
  if (ps2x.Analog(PSS_LY) == 128 && ps2x.Analog(PSS_RX) == 128) {
    driveLeft = driveRight = 0;
    linearActual = 0;
    driveMotors(0, 0, 0, 0, 0, 0);
    driveLinear(0, true);
    setServo(SERVO, 0);
    return;
  }

  // ===== Xử lý joystick =====

  // 1️⃣ Tốc độ tiến/lùi
  int speed = 0;
  if (JOY_CENTER_Y - ps2x.Analog(PSS_LY) > 0) {
    speed = map(ps2x.Analog(PSS_LY), JOY_CENTER_Y, 0, 0, MAX_DRIVE);
  } else {
    speed = -map(ps2x.Analog(PSS_LY), JOY_CENTER_Y, 255, 0, MAX_DRIVE);
  }

  // 2️⃣ Góc quẹo (giảm tốc độ xoay)
  int turn = 0;
  if (JOY_CENTER_X - ps2x.Analog(PSS_RX) > 0) {
    turn = map(ps2x.Analog(PSS_RX), JOY_CENTER_X, 0, 0, MAX_DRIVE * TURN_SCALE);
  } else {
    turn = -map(ps2x.Analog(PSS_RX), JOY_CENTER_X, 255, 0, MAX_DRIVE * TURN_SCALE);
  }

  bool Turning = (abs(turn) > 30);

  // 3️⃣ Tính tốc độ trái/phải
  targetLeft  = speed - turn;
  targetRight = speed + turn;
  
  if (!Turning) LEFT_SCALE = 0.98;
  else if (Turning) LEFT_SCALE = 1.0;

  // 4️⃣ Làm mượt trái/phải
  driveLeft  = smooth(driveLeft, targetLeft) * LEFT_SCALE;
  driveRight = smooth(driveRight, targetRight);

  // 5️⃣ Gửi ra motor lái
  int L_A = driveLeft >= 0 ? abs(driveLeft) : 0;
  int L_B = driveLeft <  0 ? abs(driveLeft) : 0;
  int R_A = driveRight >= 0 ? abs(driveRight) : 0;
  int R_B = driveRight <  0 ? abs(driveRight) : 0;

  driveMotors(L_A, L_B, R_A, R_B, 0, 0);

 // ===== XỬ LÝ RL + LINEAR + SERVO =====

int rlState = RL_CHECK();  // Gọi 1 lần duy nhất

// --- Điều khiển motor linear theo R1/R2 ---
switch (rlState) {
  case 1: targetLinear = MAX_LINEAR; break;
  case 2: targetLinear = -MAX_LINEAR; break;
  default: targetLinear = 0; break;
}
driveLinear(abs(targetLinear), targetLinear >= 0);

// --- Điều khiển servo mượt khi giữ L1/L2 ---
static int servoAngle = 0;
static unsigned long lastServoUpdate = 0;
const int SERVO_STEP = 2;        // Mỗi lần tăng/giảm 2 độ
const int SERVO_DELAY = 50;     // Cứ 100ms tăng/giảm 1 bước

unsigned long now = millis();
bool updated = false;

  if (ps2x.Button(PSB_L1) && now - lastServoUpdate >= SERVO_DELAY) {
    servoAngle += SERVO_STEP;
    if (servoAngle > 180) servoAngle = 180;
    updated = true;
  }

  if (ps2x.Button(PSB_L2) && now - lastServoUpdate >= SERVO_DELAY) {
    servoAngle -= SERVO_STEP;
    if (servoAngle < 0) servoAngle = 0;
    updated = true;
  }

  if (updated) {
    setServo(SERVO, servoAngle);
    Serial.print("Servo góc: "); Serial.println(servoAngle);
    lastServoUpdate = now;
  }
}

// ==== RL_CHECK có Serial Monitor ====
int RL_CHECK() {
  static int lastState = -1; // Lưu trạng thái trước đó

  int currentState = 0;
  if (ps2x.Button(PSB_R1)) currentState = 1;
  else if (ps2x.Button(PSB_R2)) currentState = 2;
  else if (ps2x.Button(PSB_L1)) currentState = 3;
  else if (ps2x.Button(PSB_L2)) currentState = 4;

  // Chỉ in khi thay đổi
  if (currentState != lastState) {
    switch (currentState) {
      case 0: Serial.println("RL: None"); break;
      case 1: Serial.println("RL: R1 pressed"); break;
      case 2: Serial.println("RL: R2 pressed"); break;
      case 3: Serial.println("RL: L1 pressed"); break;
      case 4: Serial.println("RL: L2 pressed"); break;
    }
    lastState = currentState;
  }

  return currentState;
}

#endif

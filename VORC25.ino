#include "motors.h"
#include "PS2_controller.h"

#define LOOP_INTERVAL 10  // ms, tốc độ lặp nhanh và ổn định

unsigned long lastLoopTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  initMotors();
  setupPS2controller();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = now;
    PS2controlSmooth();
  }
}

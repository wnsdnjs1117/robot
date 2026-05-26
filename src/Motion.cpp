/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 비례식 각도 제어 구현부
 * ============================================================ */
#include "Motion.h"

#include "Config.h"

void drive(int l, int r) {
  l = constrain(l, -100, 100);
  r = constrain(r, -100, 100);
  prizm.setMotorSpeeds(-(l * 7), r * 7);
}

void stopAll() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  delay(200);
}

// 매개변수로 각도와 방향을 받아 다이나믹하게 턴 수행
void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * degrees);

  if (isRight)
    drive(SPIN_SPEED, -SPIN_SPEED);
  else
    drive(-SPIN_SPEED, SPIN_SPEED);

  while (abs(prizm.readEncoderCount(1)) < targetCounts) {
    delay(5);
  }
  stopAll();
}

void reverseStraight(int counts) {
  prizm.resetEncoders();
  drive(-BACK_SPEED, -BACK_SPEED);
  while (abs(prizm.readEncoderCount(1)) < counts) {
    delay(5);
  }
  stopAll();
}

void readSensors(int& L, int& C, int& R) {
  L = digitalRead(SENSOR_LEFT);
  C = digitalRead(SENSOR_CENTER);
  R = digitalRead(SENSOR_RIGHT);
  if (INVERT_SENSORS) {
    L = !L;
    C = !C;
    R = !R;
  }
}

bool anyLine(int L, int C, int R) { return (L == 1 || C == 1 || R == 1); }

void lineFollowStep(int L, int C, int R) {
  if (L == 1 && C == 0 && R == 0) {
    drive(SPEED - 20, SPEED + 10);
    lastSensorState = 1;
  } else if (L == 1 && C == 1 && R == 0) {
    drive(SPEED - 10, SPEED + 5);
    lastSensorState = 1;
  } else if (L == 0 && C == 0 && R == 1) {
    drive(SPEED + 10, SPEED - 20);
    lastSensorState = 2;
  } else if (L == 0 && C == 1 && R == 1) {
    drive(SPEED + 5, SPEED - 10);
    lastSensorState = 2;
  } else if (C == 1) {
    if (lastSensorState == 1) {
      drive(SPEED + 4, SPEED - 4);
      lastSensorState = 0;
    } else if (lastSensorState == 2) {
      drive(SPEED - 4, SPEED + 4);
      lastSensorState = 0;
    } else {
      drive(SPEED, SPEED);
    }
  } else if (L == 0 && C == 0 && R == 0) {
    if (lastSensorState == 1)
      drive(SPEED - 16, SPEED + 6);
    else if (lastSensorState == 2)
      drive(SPEED + 6, SPEED - 16);
    else
      drive(SPEED, SPEED);
  } else {
    drive(SPEED, SPEED);
  }
}

void lineFollowStepReverse(int L, int C, int R) {
  drive(-BACK_SPEED, -BACK_SPEED);  // 조향 없는 일직선 후진
}

bool detectCrossing(int L, int C, int R) {
  bool isFull = (L == 1 && C == 1 && R == 1);
  bool isT    = !isFull && (C == 1) && (L == 1 || R == 1);

  if (isFull) crossingStable++;  else crossingStable  = 0;
  if (isT)    crossingStableT++; else crossingStableT = 0;

  if (!isFull && !isT) {
    crossingArmed = true;
    return false;
  }
  if (crossingArmed && crossingStable  >= CROSS_CONFIRM)   { crossingArmed = false; return true; }
  if (crossingArmed && crossingStableT >= T_CROSS_CONFIRM) { crossingArmed = false; return true; }
  return false;
}
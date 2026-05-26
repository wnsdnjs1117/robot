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

void readRearSensors(int& RL, int& RC, int& RR) {
  RL = (analogRead(SENSOR_REAR_LEFT)   >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RC = (analogRead(SENSOR_REAR_CENTER) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RR = (analogRead(SENSOR_REAR_RIGHT)  >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
}

bool anyRearLine(int RL, int RC, int RR) { return RL || RC || RR; }

void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  bool frontHasLine   = anyLine(FL, FC, FR);
  bool rearHasLine    = anyRearLine(RL, RC, RR);
  bool rearIsCrossing = (RL && RC && RR);

  // 전방 센서 기본 조향 (기존 lineFollowStep 동일)
  int lsp = SPEED, rsp = SPEED;
  if      (FL&&!FC&&!FR) { lsp=SPEED-20; rsp=SPEED+10; lastSensorState=1; }
  else if (FL&&FC&&!FR)  { lsp=SPEED-10; rsp=SPEED+5;  lastSensorState=1; }
  else if (!FL&&!FC&&FR) { lsp=SPEED+10; rsp=SPEED-20; lastSensorState=2; }
  else if (!FL&&FC&&FR)  { lsp=SPEED+5;  rsp=SPEED-10; lastSensorState=2; }
  else if (FC) {
    if      (lastSensorState==1) { lsp=SPEED+4; rsp=SPEED-4; lastSensorState=0; }
    else if (lastSensorState==2) { lsp=SPEED-4; rsp=SPEED+4; lastSensorState=0; }
    else                         { lsp=SPEED;   rsp=SPEED; }
  } else {
    if      (lastSensorState==1) { lsp=SPEED-16; rsp=SPEED+6; }
    else if (lastSensorState==2) { lsp=SPEED+6;  rsp=SPEED-16; }
    else                         { lsp=SPEED;    rsp=SPEED; }
  }

  // 후방 각도 교정: 전방 라인 있음 AND 후방 라인 있음 AND 후방이 교차로(111) 아님
  if (frontHasLine && rearHasLine && !rearIsCrossing) {
    int angErr  = (RR - RL) - (FR - FL);
    int angCorr = constrain(angErr * ANGULAR_GAIN, -5, 5);
    lsp += angCorr;
    rsp -= angCorr;
  }

  drive(constrain(lsp, -100, 100), constrain(rsp, -100, 100));
}

void alignHeadingOnLine() {
  // CROSS_ALIGN_COUNTS 과전진 후, 후방 센서는 출발 라인 REAR_TO_AXLE_COUNTS 뒤에 있음
  // 조금 더 전진해 후방 센서가 출발 라인을 감지하면 즉시 정지
  prizm.resetEncoders();
  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) break;
    if (abs(prizm.readEncoderCount(1)) > REAR_TO_AXLE_COUNTS + 100) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(50);

  // 미세 회전: RL과 RR 동시 감지 = 라인과 수직 정렬 (최대 60회)
  for (int t = 0; t < 60; t++) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    if (RL && RR) break;
    if (!RL && RR) drive(4, -4);   // RR만 감지 → 우회전(CW)으로 RL 끌어당김
    else           drive(-4, 4);   // RL만 감지 → 좌회전(CCW)으로 RR 끌어당김
    delay(10);
  }
  stopAll();
  delay(50);
}

bool detectCrossing(int L, int C, int R) {
  bool isCross = (L == 1 && C == 1 && R == 1);
  if (isCross)
    crossingStable++;
  else
    crossingStable = 0;

  if (!isCross) {
    crossingArmed = true;
    return false;
  }
  if (crossingArmed && crossingStable >= CROSS_CONFIRM) {
    crossingArmed = false;
    return true;
  }
  return false;
}
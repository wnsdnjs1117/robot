/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 비례식 각도 제어 구현부
 * ============================================================ */
#include "Motion.h"

#include "Config.h"

// ── [1] 모터 기본 제어 ────────────────────────────────────────

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

void reverseStraight(int counts) {
  prizm.resetEncoders();
  drive(-BACK_SPEED, -BACK_SPEED);
  while (abs(prizm.readEncoderCount(1)) < counts) {
    delay(5);
  }
  stopAll();
}

// ── [2] 회전 제어 ────────────────────────────────────────────
//
// 사다리꼴 속도 프로파일:
//   0 ~ 25%   : minSpd(10) → SPIN_SPEED 선형 가속
//   25 ~ 70%  : SPIN_SPEED 순항
//   70 ~ 90%  : SPIN_SPEED → minSpd 선형 감속
//   90% ~ 제동: minSpd 저속 유지
//   제동점 도달: stopAll() 즉시 브레이크 (관성 보정 SPIN_BRAKE_LEAD)

void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * degrees);
  long brakePoint   = targetCounts - SPIN_BRAKE_LEAD;
  const int minSpd  = 10;

  while (true) {
    long pos = abs(prizm.readEncoderCount(1));
    if (pos >= brakePoint) break;  // 제동 선행점 도달 → 즉시 정지

    // 사다리꼴 속도 계산
    float ratio = (float)pos / (float)targetCounts;
    int spd;
    if (ratio < 0.25f) {
      spd = minSpd + (int)((SPIN_SPEED - minSpd) * (ratio / 0.25f));
    } else if (ratio < 0.70f) {
      spd = SPIN_SPEED;
    } else if (ratio < 0.90f) {
      spd = minSpd + (int)((SPIN_SPEED - minSpd) * ((0.90f - ratio) / 0.20f));
    } else {
      spd = minSpd;
    }

    if (isRight)
      drive(spd, -spd);
    else
      drive(-spd, spd);

    delay(5);
  }
  stopAll();
}

// ── [3] 센서 읽기 ────────────────────────────────────────────

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

void readRearSensors(int& RL, int& RC, int& RR) {
  RL = (analogRead(SENSOR_REAR_LEFT)   >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RC = (analogRead(SENSOR_REAR_CENTER) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RR = (analogRead(SENSOR_REAR_RIGHT)  >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
}

bool anyLine(int L, int C, int R)          { return (L == 1 || C == 1 || R == 1); }
bool anyRearLine(int RL, int RC, int RR)   { return RL || RC || RR; }

// ── [4] 라인 트레이싱 ────────────────────────────────────────

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
  } else {  // 0,0,0
    if (lastSensorState == 1)
      drive(SPEED - 16, SPEED + 6);
    else if (lastSensorState == 2)
      drive(SPEED + 6, SPEED - 16);
    else
      drive(SPEED, SPEED);
  }
}

void lineFollowStepReverse(int L, int C, int R) {
  drive(-BACK_SPEED, -BACK_SPEED);  // 조향 없는 일직선 후진
}

// 전방 주도 라인트레이싱 + 후방 각도 교정
// 전방 0,0,0 → 후방 무시 / 후방 1,1,1 → 교차로이므로 후방 무시
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  bool frontHasLine   = anyLine(FL, FC, FR);
  bool rearHasLine    = anyRearLine(RL, RC, RR);
  bool rearIsCrossing = (RL && RC && RR);

  // 전방 센서 기본 조향
  int lsp = SPEED, rsp = SPEED;
  if      (FL && !FC && !FR) { lsp = SPEED - 20; rsp = SPEED + 10; lastSensorState = 1; }
  else if (FL &&  FC && !FR) { lsp = SPEED - 10; rsp = SPEED + 5;  lastSensorState = 1; }
  else if (!FL && !FC && FR) { lsp = SPEED + 10; rsp = SPEED - 20; lastSensorState = 2; }
  else if (!FL &&  FC && FR) { lsp = SPEED + 5;  rsp = SPEED - 10; lastSensorState = 2; }
  else if (FC) {
    if      (lastSensorState == 1) { lsp = SPEED + 4; rsp = SPEED - 4; lastSensorState = 0; }
    else if (lastSensorState == 2) { lsp = SPEED - 4; rsp = SPEED + 4; lastSensorState = 0; }
    else                           { lsp = SPEED;      rsp = SPEED; }
  } else {  // 0,0,0
    if      (lastSensorState == 1) { lsp = SPEED - 16; rsp = SPEED + 6; }
    else if (lastSensorState == 2) { lsp = SPEED + 6;  rsp = SPEED - 16; }
    else                           { lsp = SPEED;       rsp = SPEED; }
  }

  // 후방 각도 교정: 전방 라인 있음 AND 후방 라인 있음 AND 후방 교차로 아님
  if (frontHasLine && rearHasLine && !rearIsCrossing) {
    int angErr  = (RR - RL) - (FR - FL);
    int angCorr = constrain(angErr * ANGULAR_GAIN, -5, 5);
    lsp += angCorr;
    rsp -= angCorr;
  }

  drive(constrain(lsp, -100, 100), constrain(rsp, -100, 100));
}

// ── [5] 교차로 감지 ──────────────────────────────────────────

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

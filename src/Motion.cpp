/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 비례식 각도 제어 구현부
 * ============================================================ */
#include "Motion.h"

#include "Config.h"

// ── [1] 모터 기본 제어 ────────────────────────────────────────

void drive(int l, int r) {
  // 직진(같은 방향) 시에만 좌 모터 편향 보정 적용
  if ((l > 0 && r > 0) || (l < 0 && r < 0)) {
    l -= DRIVE_BIAS;
    r += DRIVE_BIAS;
  }
  l = constrain(l, -100, 100);
  r = constrain(r, -100, 100);
  prizm.setMotorSpeeds(-(l * 7), r * 7);
}

void stopAll() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  delay(0);
}

void softStop() {
  prizm.setMotorPower(1, 0);
  prizm.setMotorPower(2, 0);
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
  long brakePoint = targetCounts - SPIN_BRAKE_LEAD;
  const int minSpd = 10;

  while (true) {
    long pos = abs(prizm.readEncoderCount(1));
    if (pos >= brakePoint) break;

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

// 회전 방향의 새 라인에 정렬해 멈추는 제자리 회전
//   isRight    : true=우회전(CW), false=좌회전(CCW)
//   maxDegrees : 라인을 못 찾았을 때 무한 회전을 막는 한계각
//
// 동작:
//   1) 시작 시 밟고 있는 라인은 무시한다.
//      (TURN_LINE_ARM_DEG 이상 회전 + 전방 센서가 라인을 완전히 벗어나면 감지 arming)
//   2) arming 후 중앙 센서(C)가 새 라인에 닿으면 정렬된 것으로 보고 정지 → true 반환
//   3) maxDegrees를 넘도록 회전해도 못 찾으면 정지 → false 반환
bool turnToLine(bool isRight, int maxDegrees) {
  prizm.resetEncoders();
  long armCounts = (long)((SPIN_90_COUNTS / 90.0) * TURN_LINE_ARM_DEG);
  long maxCounts = (long)((SPIN_90_COUNTS / 90.0) * maxDegrees);
  bool armed = false;  // 시작 라인을 벗어나 새 라인 감지 준비됨

  while (abs(prizm.readEncoderCount(1)) < maxCounts) {
    int L, C, R;
    readSensors(L, C, R);

    if (!armed) {
      // 최소 각도 이상 회전 + 시작 라인 완전 이탈 → 감지 시작
      if (abs(prizm.readEncoderCount(1)) >= armCounts && !anyLine(L, C, R)) armed = true;
    } else if (C == 1) {
      stopAll();  // 새 라인 중앙 정렬 → 정지
      return true;
    }

    if (isRight)
      drive(SPIN_SPEED, -SPIN_SPEED);
    else
      drive(-SPIN_SPEED, SPIN_SPEED);
    delay(5);
  }

  stopAll();  // 한계각 초과 — 라인 미발견
  return false;
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
  RL = (analogRead(SENSOR_REAR_LEFT) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RC = (analogRead(SENSOR_REAR_CENTER) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RR = (analogRead(SENSOR_REAR_RIGHT) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
}

bool anyLine(int L, int C, int R) { return (L == 1 || C == 1 || R == 1); }
bool anyRearLine(int RL, int RC, int RR) { return RL || RC || RR; }

// ── [4] 라인 트레이싱 ────────────────────────────────────────
//
// 전방 주도 라인트레이싱 + 후방 각도 교정
// 전방 0,0,0 → 후방 무시 / 후방 1,1,1 → 교차로이므로 후방 무시

void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  bool frontHasLine = anyLine(FL, FC, FR);
  bool rearHasLine = anyRearLine(RL, RC, RR);
  bool rearIsCrossing = (RL && RC && RR);

  int lsp = SPEED, rsp = SPEED;
  if (FL && !FC && !FR) {
    lsp = SPEED - 20;
    rsp = SPEED + 10;
    lastSensorState = 1;
  } else if (FL && FC && !FR) {
    lsp = SPEED - 10;
    rsp = SPEED + 5;
    lastSensorState = 1;
  } else if (!FL && !FC && FR) {
    lsp = SPEED + 10;
    rsp = SPEED - 20;
    lastSensorState = 2;
  } else if (!FL && FC && FR) {
    lsp = SPEED + 5;
    rsp = SPEED - 10;
    lastSensorState = 2;
  } else if (FC) {
    if (lastSensorState == 1) {
      lsp = SPEED + 4;
      rsp = SPEED - 4;
      lastSensorState = 0;
    } else if (lastSensorState == 2) {
      lsp = SPEED - 4;
      rsp = SPEED + 4;
      lastSensorState = 0;
    } else {
      lsp = SPEED;
      rsp = SPEED;
    }
  } else {
    if (lastSensorState == 1) {
      lsp = SPEED - 20;   // 엣지 감지 수준의 좌회전으로 선 재탐색
      rsp = SPEED +  6;
    } else if (lastSensorState == 2) {
      lsp = SPEED +  6;
      rsp = SPEED - 20;
    } else {
      lsp = SPEED / 2;    // 방향 기억 없음 → 감속 직진 (탈출 최소화)
      rsp = SPEED / 2;
    }
  }

  bool frontIsCrossing = (FL && FC && FR);
  if (frontHasLine && rearHasLine && !frontIsCrossing && !rearIsCrossing) {
    int angCorr = constrain((RL - RR) * ANGULAR_GAIN, -5, 5);
    lsp += angCorr;
    rsp -= angCorr;
  }

  drive(constrain(lsp, -100, 100), constrain(rsp, -100, 100));
}

// ── [5] 교차로 감지 ──────────────────────────────────────────

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

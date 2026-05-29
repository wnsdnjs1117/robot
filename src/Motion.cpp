/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 비례식 각도 제어 구현부
 * ============================================================ */
#include "Motion.h"

#include "Config.h"

// ── [1] 모터 기본 제어 ────────────────────────────────────────

void drive(int l, int r) {
  // 전진: 좌↓ 우↑ / 후진: 부호 반전 (좌↑ 우↓) — DRIVE_BIAS > 0 이면 좌 감속
  if (l > 0 && r > 0) {
    l -= DRIVE_BIAS;
    r += DRIVE_BIAS;
  } else if (l < 0 && r < 0) {
    l += DRIVE_BIAS;
    r -= DRIVE_BIAS;
  }
  l = constrain(l, -100, 100);
  r = constrain(r, -100, 100);
  prizm.setMotorSpeeds(-(l * 7), r * 7);
}

void stopAll() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  delay(50);
}

// ── [2] 회전 제어 ────────────────────────────────────────────

void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * degrees);
  long brakePoint = targetCounts - SPIN_BRAKE_LEAD;
  const int minSpd = 10;

  while (true) {
    long pos = (abs(prizm.readEncoderCount(1)) + abs(prizm.readEncoderCount(2))) / 2;
    if (pos >= brakePoint) break;

    // ★ 90도 회전 오버슈팅 방지 로직 (회전의 60% 이상 진행된 상태에서만 작동)
    if (degrees == 90 && pos > (targetCounts * 0.6f)) {
      int L, C, R;
      readSensors(L, C, R);

      if (isRight && L == 1) {
        // 우회전 시: 좌측 센서가 선을 밟으면 오버슈팅으로 간주하고 즉시 정지
        Serial.println(F(">> [TURN] 우회전 오버슈트 방지! (좌센서 감지)"));
        break;
      }
      if (!isRight && R == 1) {
        // 좌회전 시: 우측 센서가 선을 밟으면 오버슈팅으로 간주하고 즉시 정지
        Serial.println(F(">> [TURN] 좌회전 오버슈트 방지! (우센서 감지)"));
        break;
      }
    }

    // 사다리꼴 속도 프로파일
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

// 회전 방향의 새 라인에 정렬해 멈추는 제자리 회전 (현재 맵 구조상 예비용으로 보존)
bool turnToLine(bool isRight, int maxDegrees) {
  prizm.resetEncoders();
  long armCounts = (long)((SPIN_90_COUNTS / 90.0) * TURN_LINE_ARM_DEG);
  long maxCounts = (long)((SPIN_90_COUNTS / 90.0) * maxDegrees);
  bool armed = false;

  while (abs(prizm.readEncoderCount(1)) < maxCounts) {
    int L, C, R;
    readSensors(L, C, R);

    if (!armed) {
      if (abs(prizm.readEncoderCount(1)) >= armCounts && !anyLine(L, C, R)) armed = true;
    } else if (C == 1) {
      stopAll();
      return true;
    }

    if (isRight)
      drive(SPIN_SPEED, -SPIN_SPEED);
    else
      drive(-SPIN_SPEED, SPIN_SPEED);
    delay(5);
  }

  stopAll();
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
      lsp = SPEED - 20;
      rsp = SPEED + 6;
    } else if (lastSensorState == 2) {
      lsp = SPEED + 6;
      rsp = SPEED - 20;
    } else {
      lsp = SPEED / 2;
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
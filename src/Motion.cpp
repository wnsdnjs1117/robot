/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 비례식 각도 제어 구현부
 * ============================================================ */
#include "Motion.h"

#include "Config.h"

// ── [1] 모터 구동 및 정지 ────────────────────────────────────────

void drive(int l, int r) {
  // 모터 편차 교정: 로봇이 똑바로 가지 않고 휘어진다면 DRIVE_BIAS 값을 통해 좌우 힘을 조절합니다.
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
  // 전력을 차단하는 것이 아니라 모터 단자를 쇼트시켜 강력한 물리적 브레이크를 겁니다.
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  delay(50);
}

// ── [2] 거리 기반 제자리 회전 (오버슈팅 방지 탑재) ────────────────

void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * degrees);
  long brakePoint = targetCounts - SPIN_BRAKE_LEAD;
  const int minSpd = 10;

  while (true) {
    long pos = (abs(prizm.readEncoderCount(1)) + abs(prizm.readEncoderCount(2))) / 2;
    if (pos >= brakePoint) break;

    // [90도 오버슈트 방지]
    // 센서가 바퀴 축보다 앞에 있기 때문에, 바닥이 미끄러워 회전을 너무 많이 해버리면 꼬리 쪽 센서가 선을 밟게 됩니다.
    // 회전의 80%가 지난 시점부터 이를 감시하다가, 꼬리 센서가 선을 밟으면 즉시 강제 종료합니다.
    if (degrees == 90 && pos > (targetCounts * 0.8f)) {
      int L, C, R;
      readSensors(L, C, R);

      if (isRight && L == 1) {
        Serial.println(F(">> [TURN] 우회전 오버슈트 방지! (좌센서 감지)"));
        break;
      }
      if (!isRight && R == 1) {
        Serial.println(F(">> [TURN] 좌회전 오버슈트 방지! (우센서 감지)"));
        break;
      }
    }

    // [사다리꼴 속도 프로파일] 서서히 가속하고 부드럽게 감속하여 바퀴 미끄러짐 방지
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

    // 센서 처리를 안정화하는 기본 주행 루프 딜레이
    delay(5);
  }

  stopAll();
}

// ── [3] 라인 센서 읽기 ──────────────────────────────────────

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

// ── [4] 다중 센서 라인 트레이싱 제어 ────────────────────────────

void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  bool frontHasLine = anyLine(FL, FC, FR);
  bool rearHasLine = anyRearLine(RL, RC, RR);
  bool rearIsCrossing = (RL && RC && RR);

  int lsp = SPEED, rsp = SPEED;

  // 전방 조향: 선이 왼쪽에 있으면 우측 모터 속도를 높여 추종
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
    // 선을 벗어났을 경우 마지막으로 선을 밟았던 방향으로 강하게 조향
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

  // 전후방 융합: 후방 센서가 삐뚤어져 있다면 차체가 대각선이라는 뜻이므로 이를 교정
  if (frontHasLine && rearHasLine && !frontIsCrossing && !rearIsCrossing) {
    int angCorr = constrain((RL - RR) * ANGULAR_GAIN, -5, 5);
    lsp += angCorr;
    rsp -= angCorr;
  }

  drive(constrain(lsp, -100, 100), constrain(rsp, -100, 100));
}

// ── [5] 교차로 필터링 감지 ─────────────────────

bool detectCrossing(int L, int C, int R) {
  bool isCross = (L == 1 && C == 1 && R == 1);

  // 노이즈 필터링: 지정된 횟수 연속으로 3개 센서가 모두 1이어야 진짜 교차로로 인정
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
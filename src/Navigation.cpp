/* ============================================================
 * Navigation.cpp - 거리/각도 기반 진입 및 탈출 탐색 알고리즘
 * ============================================================ */
#include "Navigation.h"

#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

// ── [1] 라인 추종 → 교차로 ───────────────────────────────────

void followToCrossing(bool stopAtEnd) {
  {
    int L, C, R;
    readSensors(L, C, R);
    if (L == 1 && C == 1 && R == 1) {
      while (true) {
        readSensors(L, C, R);
        if (!(L == 1 && C == 1 && R == 1)) break;
        drive(SPEED, SPEED);
        delay(5);
      }
      for (int i = 0; i < 10; i++) {
        drive(SPEED, SPEED);
        delay(5);
      }
    }
  }

  crossingArmed = true;
  crossingStable = 0;
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (detectCrossing(L, C, R)) {
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
        drive(SPEED, SPEED);
        delay(5);
      }
      if (stopAtEnd) {
        stopAll();
      }
      return;
    }
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR);
    delay(5);
  }
}

void followToCrossing() { followToCrossing(true); }

// ── [2] 블라인드 구간 출발 정렬 ──────────────────────────────
void alignHeadingOnLine() {
  prizm.resetEncoders();
  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) break;
    if (abs(prizm.readEncoderCount(1)) > DIST_REAR_CROSS_ALIGN_COUNTS + 50) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();

  prizm.resetEncoders();
  for (int t = 0; t < 40; t++) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);

    if (abs(prizm.readEncoderCount(1)) >= ALIGN_MAX_COUNTS) break;

    if (RL && RR) break;
    if (!RL && RC && !RR) break;
    if (RL && RC && RR) break;
    if (!RL && !RC && !RR) break;

    if (RL && !RC && !RR)
      drive(-4, 4);
    else if (!RL && !RC && RR)
      drive(4, -4);
    else if (RL && RC && !RR)
      drive(4, -4);
    else if (!RL && RC && RR)
      drive(-4, 4);

    delay(10);
  }
  stopAll();
}

// ── [3] 존 진입/탈출 (거리 기반으로 단순화) ─────────────────────────

void enterZone() {
  Serial.println(F(">> [NAV] 전진으로 존 진입 시작"));
  prizm.resetEncoders();

  while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_ENTER_FWD_COUNTS) {
    drive(SPEED, SPEED);
    delay(5);
  }
  stopAll();
}

void reverseEnterZone() {
  Serial.println(F(">> [NAV] 후진으로 존 진입 시작"));
  prizm.resetEncoders();

  while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_ENTER_REV_COUNTS) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(5);
  }
  stopAll();
}

void reverseAcrossToOppositeZone() {
  lastSensorState = 0;
  prizm.resetEncoders();
  bool rearCrossFound = false;
  int crossCount = 0;

  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    int L, C, R;
    readSensors(L, C, R);

    bool rearHasLine = anyRearLine(RL, RC, RR);
    bool frontHasLine = anyLine(L, C, R);

    bool rearIsCrossing = ((RL && RC) || (RC && RR) || (RL && RR));
    bool frontIsCrossing = ((L && C) || (C && R) || (L && R));

    if (rearIsCrossing && !rearCrossFound) {
      crossCount++;
      if (crossCount >= 1) {
        rearCrossFound = true;
        prizm.resetEncoders();
      }
    } else if (!rearIsCrossing && !rearCrossFound) {
      crossCount = 0;
    }

    if (rearCrossFound && abs(prizm.readEncoderCount(1)) >= DIST_REAR_CROSS_ALIGN_COUNTS) {
      break;
    }

    if (!rearCrossFound && abs(prizm.readEncoderCount(1)) > 6000) {
      break;
    }

    int lsp = -BACK_SPEED;
    int rsp = -BACK_SPEED;

    if (rearHasLine && !rearIsCrossing) {
      if (RL && !RC && !RR) {
        lsp = -(BACK_SPEED - BACK_STEER_STRONG);
        rsp = -(BACK_SPEED + BACK_STEER_STRONG);
      } else if (!RL && !RC && RR) {
        lsp = -(BACK_SPEED + BACK_STEER_STRONG);
        rsp = -(BACK_SPEED - BACK_STEER_STRONG);
      } else if (RL && RC && !RR) {
        lsp = -(BACK_SPEED - BACK_STEER_WEAK);
        rsp = -(BACK_SPEED + BACK_STEER_WEAK);
      } else if (!RL && RC && RR) {
        lsp = -(BACK_SPEED + BACK_STEER_WEAK);
        rsp = -(BACK_SPEED - BACK_STEER_WEAK);
      }
    }

    if (rearHasLine && frontHasLine && !rearIsCrossing && !frontIsCrossing) {
      int angCorr = constrain((L - R) * ANGULAR_GAIN, -5, 5);
      lsp -= angCorr;
      rsp += angCorr;
    }

    drive(lsp, rsp);
    delay(5);
  }
  stopAll();

  // 반대편으로 넘어가서 진입 수행
  reverseEnterZone();
}

// ── [4] 특수 경로 ────────────────────────────────────────────

void goToMainLine() {
  Serial.println(F(">>> [START-RUN] 서향 출발 -> 12번 노드(빈 공간) -> 9-2 노드 -> 8번 노드"));

  robotHeading = 270;

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_START_TO_12_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(HEADING_12_TO_9);

  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(270);
  followToCrossing(true);

  currentNode = 8;
  Serial.println(F(">> [START] 8번 노드 안착 완료"));
}

void returnToFinish() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [FINISH] 복귀 기동: 9-3 -> 12 -> START"));

  moveToNode(9);

  turnToHeading(90);
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_9_TO_9_3_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(HEADING_9_3_TO_12);

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_9_3_TO_12_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(HEADING_12_TO_START);

  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_FINISH_ENTRY_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();

  tone(BUZZER_PIN, 1000);
  delay(1500);
  noTone(BUZZER_PIN);
  prizm.setGreenLED(HIGH);
  Serial.println(F(">> [FINISH] 경기 종료."));
  Serial.println(F("========================================\n"));
}

int qrSearchStage() {
  int randomFound = 0;

  Serial.println(F("\n--- [2구역 탐색] ---"));
  enterZone();
  lastEntryWasForward = true;
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 2;
  }

  Serial.println(F("\n--- [4구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 4;
  }

  Serial.println(F("\n--- [1구역 탐색] ---"));
  followToCrossing();
  turnAngle(90, false);
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  lastEntryWasForward = true;
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 1;
  }

  Serial.println(F("\n--- [3구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;
  scanZone(3);
  stopAll();
  printSearchResult();
  return 3;
}
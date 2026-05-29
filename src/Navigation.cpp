/* ============================================================
 * Navigation.cpp - 즉시 종료 및 구역 번호 반환형 탐색 알고리즘
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
      while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
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
    if (abs(prizm.readEncoderCount(1)) > REAR_TO_AXLE_COUNTS + 100) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();

  prizm.resetEncoders();
  for (int t = 0; t < 60; t++) {
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

// ── [3] 존 진입/탈출 ─────────────────────────────────────────

void enterZone(int zone) {
  lastSensorState = 0;
  int enterExtra = (zone == 1 || zone == 2) ? ZONE12_ENTER_EXTRA : ZONE_ENTER_EXTRA;

  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (!anyLine(L, C, R)) break;
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR);
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < enterExtra) {
    drive(SPEED, SPEED);
    delay(5);
  }
  softStop();
}

void reverseEnterZone(int zone) {
  lastSensorState = 0;
  int depthExtra = (zone == 1 || zone == 2) ? ZONE12_DEPTH_EXTRA : ZONE_DEPTH_EXTRA;
  bool lineWasFound = false;

  prizm.resetEncoders();
  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);

    bool rearHasLine = anyRearLine(RL, RC, RR);
    if (rearHasLine) lineWasFound = true;
    if (lineWasFound && !rearHasLine) break;
    if (abs(prizm.readEncoderCount(1)) >= ZONE_FOLLOW_MAX) break;

    int lsp = -BACK_SPEED, rsp = -BACK_SPEED;
    bool rearIsCrossing = ((RL && RC) || (RC && RR) || (RL && RR));
    if (rearHasLine && !rearIsCrossing) {
      applyRearLineSteering(RL, RC, RR, lsp, rsp);
    }
    {
      int fL, fC, fR;
      readSensors(fL, fC, fR);
      bool frontHasLine = anyLine(fL, fC, fR);
      bool frontIsCrossing = ((fL && fC) || (fC && fR) || (fL && fR));
      if (rearHasLine && frontHasLine && !rearIsCrossing && !frontIsCrossing) {
        int angCorr = constrain((fL - fR) * ANGULAR_GAIN, -5, 5);
        lsp -= angCorr;
        rsp += angCorr;
      }
    }
    drive(lsp, rsp);
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < depthExtra) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(5);
  }
  softStop();
}

void reverseAcrossToOppositeZone(int targetZone) {
  lastSensorState = 0;
  prizm.resetEncoders();
  bool rearCrossFound = false;

  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    int L, C, R;
    readSensors(L, C, R);

    bool rearHasLine    = anyRearLine(RL, RC, RR);
    bool frontHasLine   = anyLine(L, C, R);
    bool rearIsCrossing = ((RL && RC) || (RC && RR) || (RL && RR));
    bool frontIsCrossing = ((L && C) || (C && R) || (L && R));

    if (rearIsCrossing && !rearCrossFound) {
      rearCrossFound = true;
      prizm.resetEncoders();
    }

    if (rearCrossFound && abs(prizm.readEncoderCount(1)) >= REAR_TO_AXLE_COUNTS) break;
    if (!rearCrossFound && abs(prizm.readEncoderCount(1)) > EXIT_SAFETY_COUNTS) break;

    int lsp = -BACK_SPEED, rsp = -BACK_SPEED;

    if (!rearCrossFound) {
      if (rearHasLine && !rearIsCrossing) {
        applyRearLineSteering(RL, RC, RR, lsp, rsp);
      } else if (!rearHasLine) {
        long d1 = abs(prizm.readEncoderCount(1));
        long d2 = abs(prizm.readEncoderCount(2));
        long diff = d1 - d2;
        int corr = (abs(diff) <= BLIND_CORR_DEADZONE) ? 0
                 : constrain((int)(diff / BLIND_CORR_GAIN), -BLIND_CORR_CAP, BLIND_CORR_CAP);
        lsp = -BACK_SPEED + corr;
        rsp = -BACK_SPEED - corr;
      }

      if (rearHasLine && frontHasLine && !rearIsCrossing && !frontIsCrossing) {
        int angCorr = constrain((L - R) * ANGULAR_GAIN, -5, 5);
        lsp -= angCorr;
        rsp += angCorr;
      }
    }

    drive(lsp, rsp);
    delay(5);
  }

  reverseEnterZone(targetZone);
}

// ── [4] 특수 경로 ────────────────────────────────────────────

void goToMainLine() {
  DPRINTLNF(">>> [START-RUN] 스타트 박스 탈출");
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < START_ESCAPE_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }
  stopAll();

  DPRINTLNF(">>> [START-RUN] 서쪽(좌측) 방향 전환");
  if (WEST_IS_LEFT)
    turnAngle(90, false);
  else
    turnAngle(90, true);

  DPRINTLNF(">>> [START-RUN] 11번, 10번 노드 라인 패스 (무정차 직진)");
  int passedLines = 0;
  bool lineArmed = true;
  int lineStable = 0;

  while (passedLines < 2) {
    int L, C, R;
    readSensors(L, C, R);
    bool onLine = anyLine(L, C, R);
    if (onLine)
      lineStable++;
    else
      lineStable = 0;

    if (onLine && lineArmed && lineStable >= CROSS_CONFIRM) {
      passedLines++;
      lineArmed = false;
    }
    if (!onLine) lineArmed = true;

    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }

  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (!anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }

  DPRINTLNF(">>> [START-RUN] 메인라인/세로선 진입 탐색");

  while (true) {
    int L, C, R;
    readSensors(L, C, R);

    if (anyLine(L, C, R)) {
      bool isVertical = false;
      bool hitCrossing = false;
      long startEnc = abs(prizm.readEncoderCount(1));

      while (abs(prizm.readEncoderCount(1)) - startEnc < 400) {
        int cL, cC, cR;
        readSensors(cL, cC, cR);

        if (!anyLine(cL, cC, cR)) {
          isVertical = true;
          break;
        }

        if (cL == 1 && cC == 1 && cR == 1) {
          hitCrossing = true;
          break;
        }

        drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
        liftDownTick();
        delay(5);
      }

      if (isVertical) {
        DPRINTLNF(">> [START] 8번 세로선 관통 확인 -> 교차로 정렬");
        long traveled = abs(prizm.readEncoderCount(1)) - startEnc;
        long remainingCounts = CROSS_ALIGN_COUNTS - traveled;

        prizm.resetEncoders();
        if (remainingCounts > 0) {
          while (abs(prizm.readEncoderCount(1)) < remainingCounts) {
            drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
            liftDownTick();
            delay(5);
          }
        }
        stopAll();
      } else if (hitCrossing) {
        DPRINTLNF(">> [START] 직진 테스트 중 8번 교차로 직접 도달 -> 즉시 정렬");
        prizm.resetEncoders();
        while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
          drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
          liftDownTick();
          delay(5);
        }
        stopAll();
      } else {
        DPRINTLNF(">> [START] 9번 가로선 연속 감지 -> 8번 교차로로 라인 트레이싱");
        followToCrossing(true);
      }
      break;
    }

    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }

  DPRINTLNF(">> [START] 8번 노드 안착 -> 북향(2구역 방향) 회전");
  turnAngle(90, true);
  robotHeading = HDG_N;
}

void returnToFinish() {
  DPRINTLNF("\n========================================");
  DPRINTLNF(">> [FINISH] FINISH 구역 복귀 기동");

  // 노드12(START 바로 위 가상 노드)까지 이동 후 남향 직진으로 진입
  moveToNode(12);
  turnAngle(90, true);   // CW 90°: 동향 → 남향 (노드12에 선 없으므로 엔코더 기반)
  robotHeading = HDG_S;

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < FINISH_ENTRY_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  softStop();
  DPRINTLNF(">> [FINISH] FINISH 구역 진입 완료!");

  tone(BUZZER_PIN, 1000);
  delay(1500);
  noTone(BUZZER_PIN);

  prizm.setGreenLED(HIGH);
  DPRINTLNF(">> [FINISH] 부저 완료. 경기 종료.");
  DPRINTLNF("========================================\n");
}

int qrSearchStage() {
  int randomFound = 0;

  DPRINTLNF("\n--- [2구역 탐색] ---");
  enterZone(2);
  lastEntryWasForward = true;
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    softStop();
    printSearchResult();
    return 2;
  }

  DPRINTLNF("\n--- [4구역 탐색] ---");
  reverseAcrossToOppositeZone(4);
  lastEntryWasForward = false;
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    softStop();
    printSearchResult();
    return 4;
  }

  DPRINTLNF("\n--- [1구역 탐색] ---");
  followToCrossing();
  turnAngle(90, false);
  followToCrossing();
  turnAngle(90, true);
  enterZone(1);
  lastEntryWasForward = true;
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) {
    softStop();
    printSearchResult();
    return 1;
  }

  DPRINTLNF("\n--- [3구역 탐색] ---");
  reverseAcrossToOppositeZone(3);
  lastEntryWasForward = false;
  scanZone(3);
  softStop();
  printSearchResult();
  return 3;
}
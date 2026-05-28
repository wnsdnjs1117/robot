/* ============================================================
 * MapRouter.cpp - 헤딩 완전 추적 / 진입방식 기반 탈출
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "Motion.h"

// Navigation.cpp와 순환 include 방지 — 필요한 함수만 전방 선언
void followToCrossing();
void followToCrossing(bool stopAtEnd);  // 연속 주행용 오버로딩
void reverseAcrossToOppositeZone();
void enterZone();
void reverseEnterZone();
void alignHeadingOnLine();

int robotHeading = HDG_N;
int currentNode = 8;
bool lastEntryWasForward = true;

static bool isTiltedNorthAt10 = false;

// ── [내부] 헬퍼 ──────────────────────────────────────────────

static int zoneToNode(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5) return 10;
  if (zone == 6) return 11;
  return 8;
}

static int nodeIndex(int n) {
  if (n == 7) return 0;
  if (n == 8) return 1;
  if (n == 9) return 2;
  if (n == 10) return 3;
  if (n == 11) return 4;
  return 1;
}

void turnToHeading(int target) {
  int diff = (target - robotHeading + 4) % 4;
  if (diff == 0) return;
  if (diff == 1) {
    turnAngle(90, true);
  }
  if (diff == 3) {
    turnAngle(90, false);
  }
  if (diff == 2) {
    turnAngle(90, true);
    turnAngle(90, true);
  }
  robotHeading = target;
}

static void blindDriveUntilLine() {
  prizm.resetEncoders();
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) {
      break;
    }
    long diff = abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
    int corr = (abs(diff) <= 5) ? 0 : constrain((int)(diff / 12), -4, 4);
    drive(BLIND_SPEED - corr, BLIND_SPEED + corr);
    delay(5);
  }
}

// ── [내부] 노드 간 단일 구간 이동 ────────────────────────────
static void stepNode(int from, int to, bool stopAtEnd) {
  if (from == 10 && isTiltedNorthAt10) {
    stopAll();
    delay(100);
    turnAngle(5, true);
    isTiltedNorthAt10 = false;
  }

  int dir = (to > from) ? HDG_E : HDG_W;
  turnToHeading(dir);

  if (from == 8 && to == 9) {
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (!anyLine(L, C, R)) {
        if (stopAtEnd) stopAll();
        break;
      }
      int RL, RC, RR;
      readRearSensors(RL, RC, RR);
      lineFollowStepFull(L, C, R, RL, RC, RR);
      delay(5);
    }

  } else if (from == 9 && to == 8) {
    prizm.resetEncoders();
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      long diff =
          abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
      int corr = (abs(diff) <= 5) ? 0 : constrain((int)(diff / 12), -4, 4);
      drive(STRAIGHT_SPEED - corr, STRAIGHT_SPEED + corr);
      delay(5);
    }
    followToCrossing(stopAtEnd);

  } else if (from == 9 && to == 10) {
    alignHeadingOnLine();
    turnAngle(5, false);
    isTiltedNorthAt10 = true;
    prizm.resetEncoders();
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) {
        prizm.resetEncoders();
        while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
          drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
          delay(5);
        }
        if (stopAtEnd) stopAll();
        robotHeading = HDG_E;
        break;
      }
      long diff =
          abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
      int corr = (abs(diff) <= 5) ? 0 : constrain((int)(diff / 12), -4, 4);
      drive(BLIND_SPEED - corr, BLIND_SPEED + corr);
      delay(5);
    }

  } else if (from == 10 && to == 9) {
    blindDriveUntilLine();
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();
    robotHeading = HDG_W;

  } else if (from == 10 && to == 11) {
    blindDriveUntilLine();
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();
    robotHeading = HDG_E;

  } else if (from == 11 && to == 10) {
    blindDriveUntilLine();
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();
    robotHeading = HDG_W;

  } else {
    followToCrossing(stopAtEnd);
  }

  currentNode = to;
  Serial.print(F(">> [NAV] "));
  Serial.print(from);
  Serial.print(F("->"));
  Serial.println(to);
}

// ── [공개] 라우팅 핵심 3대 함수 ───────────────────────────────

void moveToNode(int toNode) {
  static const int nodes[] = {7, 8, 9, 10, 11};
  int cur = nodeIndex(currentNode);
  int tgt = nodeIndex(toNode);
  if (cur == tgt) return;
  int step = (cur < tgt) ? 1 : -1;

  while (cur != tgt) {
    int next = cur + step;
    bool isFinal = (next == tgt);
    stepNode(nodes[cur], nodes[next], isFinal);
    cur = next;
  }
}

void exitZone(int zone) {
  if (lastEntryWasForward) {
    // [핵심 수정] 고정 거리 후진을 버리고, 메인 라인 교차로를 동적으로 찾음
    prizm.resetEncoders();
    bool rearCrossFound = false;
    int crossCount = 0;

    while (true) {
      int RL, RC, RR;
      readRearSensors(RL, RC, RR);
      int L, C, R;
      readSensors(L, C, R);

      bool rearHasLine = anyRearLine(RL, RC, RR);
      bool rearIsCrossing = (RL && RC && RR);
      bool frontHasLine = anyLine(L, C, R);
      bool frontIsCrossing = (L && C && R);

      // 교차로 감지 시
      if (rearIsCrossing && !rearCrossFound) {
        crossCount++;
        if (crossCount >= 2) {
          rearCrossFound = true;
          prizm.resetEncoders();  // 여기서부터 바퀴를 교차로에 올리기 위해
                                  // 카운트 리셋
        }
      } else if (!rearIsCrossing && !rearCrossFound) {
        crossCount = 0;
      }

      // 후방 센서가 교차로를 밟은 직후, 바퀴가 교차로 중심에 올 때까지(1000
      // 카운트) 더 후진
      if (rearCrossFound &&
          abs(prizm.readEncoderCount(1)) >= REAR_TO_AXLE_COUNTS) {
        break;
      }

      // 무한루프 방지 (안전망)
      if (!rearCrossFound && abs(prizm.readEncoderCount(1)) > 6000) {
        break;
      }

      int lsp = -BACK_SPEED, rsp = -BACK_SPEED;

      if (rearHasLine && !rearIsCrossing) {
        if (RL && !RC && !RR) {
          lsp = -(BACK_SPEED - 10);
          rsp = -(BACK_SPEED + 10);
        } else if (!RL && !RC && RR) {
          lsp = -(BACK_SPEED + 10);
          rsp = -(BACK_SPEED - 10);
        } else if (RL && RC && !RR) {
          lsp = -(BACK_SPEED - 5);
          rsp = -(BACK_SPEED + 5);
        } else if (!RL && RC && RR) {
          lsp = -(BACK_SPEED + 5);
          rsp = -(BACK_SPEED - 5);
        }
      } else if (!rearHasLine) {
        long diff =
            abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
        int corr = (abs(diff) <= 5) ? 0 : constrain((int)(diff / 12), -4, 4);
        lsp = -BACK_SPEED + corr;
        rsp = -BACK_SPEED - corr;
      }

      if (frontHasLine && rearHasLine && !frontIsCrossing && !rearIsCrossing) {
        int angCorr = constrain((L - R) * ANGULAR_GAIN, -5, 5);
        lsp -= angCorr;
        rsp += angCorr;
      }

      drive(lsp, rsp);
      delay(5);
    }
    stopAll();
  } else {
    if (zoneToNode(zone) == 7) {
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < NODE7_EXIT_COUNTS) {
        drive(SPEED, SPEED);
        delay(5);
      }
      stopAll();
    } else {
      followToCrossing(true);
    }
  }
  currentNode = zoneToNode(zone);
  Serial.print(F(">> [NAV] exitZone "));
  Serial.print(zone);
  Serial.print(F(" -> node "));
  Serial.println(currentNode);
}

void goToZoneDirect(int zone) {
  int targetNode = zoneToNode(zone);
  Serial.print(F(">> [NAV] zone "));
  Serial.print(zone);
  Serial.print(F(" (node "));
  Serial.print(currentNode);
  Serial.print(F("->"));
  Serial.print(targetNode);
  Serial.println(F(")"));

  moveToNode(targetNode);

  int zoneSide = (zone == 3 || zone == 4) ? HDG_S : HDG_N;

  if (robotHeading == HDG_E || robotHeading == HDG_W) {
    turnToHeading(zoneSide);
  }

  bool enterForward = (robotHeading == zoneSide);
  if (enterForward) {
    enterZone();
    lastEntryWasForward = true;
  } else {
    reverseEnterZone();
    lastEntryWasForward = false;
  }
}
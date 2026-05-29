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

// 9->10 이동 시 북쪽으로 2도 틀어진 상태를 추적하는 변수
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
  if (n == 12) return 5;
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

  // 회전 관성 안정화를 위한 대기
  stopAll();
  delay(200);
}

static void blindDriveUntilLine() {
  prizm.resetEncoders();
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) {
      break;
    }
    // 차동 보정 완전 제거: 순수 정속 주행
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }
}

// ── [내부] 노드 간 단일 구간 이동 ────────────────────────────
static void stepNode(int from, int to, bool stopAtEnd) {
  // 10번 노드 출발 전 미세 보정
  if (from == 10 && isTiltedNorthAt10) {
    stopAll();
    delay(100);
    turnAngle(2, true);  // 2도 시계방향 복구
    isTiltedNorthAt10 = false;
    stopAll();
    delay(100);
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

      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    followToCrossing(stopAtEnd);

  } else if (from == 9 && to == 10) {
    // ★ [버그 해결] 출발하자마자 좌우로 30도씩 꺾던 alignHeadingOnLine() 주석 처리(제거)!
    // 이제 선을 물고 비비지 않고 곧바로 2도 미세 튜닝만 한 뒤 정속 직진합니다.
    // alignHeadingOnLine();

    turnAngle(2, false);
    stopAll();
    delay(100);

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

      drive(BLIND_SPEED, BLIND_SPEED);
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

  } else if (from == 11 && to == 12) {
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < NODE_11_12_COUNTS) {
      drive(BLIND_SPEED, BLIND_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();
    robotHeading = HDG_E;

  } else if (from == 12 && to == 11) {
    blindDriveUntilLine();
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
  static const int nodes[] = {7, 8, 9, 10, 11, 12};
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
    prizm.resetEncoders();
    bool rearCrossFound = false;
    int crossCount = 0;
    bool lineWasSeen = false;

    while (true) {
      int RL, RC, RR;
      readRearSensors(RL, RC, RR);
      int L, C, R;
      readSensors(L, C, R);

      bool rearHasLine = anyRearLine(RL, RC, RR);
      bool frontHasLine = anyLine(L, C, R);
      bool rearIsCrossing = ((RL && RC) || (RC && RR) || (RL && RR));
      bool frontIsCrossing = ((L && C) || (C && R) || (L && R));

      if (rearHasLine) lineWasSeen = true;

      bool distanceQualified = (abs(prizm.readEncoderCount(1)) >= 1200);

      if (zone == 5 || zone == 6) {
        if (distanceQualified && lineWasSeen && !rearHasLine && !rearCrossFound) {
          crossCount++;
          if (crossCount >= 3) {
            rearCrossFound = true;
            prizm.resetEncoders();
          }
        } else if (rearHasLine && !rearCrossFound) {
          crossCount = 0;
        }
      } else {
        if (distanceQualified && rearIsCrossing && !rearCrossFound) {
          crossCount++;
          if (crossCount >= 1) {
            rearCrossFound = true;
            prizm.resetEncoders();
          }
        } else if (!rearIsCrossing && !rearCrossFound) {
          crossCount = 0;
        }
      }

      if (rearCrossFound && abs(prizm.readEncoderCount(1)) >= REAR_TO_AXLE_COUNTS) {
        break;
      }

      if (!rearCrossFound && abs(prizm.readEncoderCount(1)) > 6000) {
        break;
      }

      int lsp = -BACK_SPEED, rsp = -BACK_SPEED;

      if (!rearCrossFound) {
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
          lsp = -BACK_SPEED;
          rsp = -BACK_SPEED;
        }

        if (frontHasLine && rearHasLine && !rearIsCrossing && !frontIsCrossing) {
          int angCorr = constrain((L - R) * ANGULAR_GAIN, -5, 5);
          lsp -= angCorr;
          rsp += angCorr;
        }
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

  if (zone == 5) {
    isTiltedNorthAt10 = false;
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
/* ============================================================
 * MapRouter.cpp - 헤딩 완전 추적 / 진입방식 기반 탈출
 *
 *  경기장 레이아웃 (위 = 북)
 *
 *   [1구역]  [2구역]             [5입고]   [6출고]
 *      │        │                   │        │
 *     [7]──────[8]──────[9]  ···  [10]  ··· [11] ··[12]
 *      │        │                                    ·
 *   [3구역]  [4구역]                                  ·
 *                                              ┌─────┴──────┐
 *                                              │  출발/도착  │
 *                                              │   START    │
 *                                              └────────────┘
 *
 *   ─── : 검은 라인 있음      ··· : 블라인드 구간 (선 없음)
 *   [N] : 교차로 노드         │   : 구역 연결선
 *
 *   구역-노드: 1,3→[7] / 2,4→[8] / 5→[10] / 6→[11]
 *   START/FINISH: [12] 아래 (출발·도착 겸용 박스, [11] 동쪽 가상 노드)
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "Lift.h"
#include "Motion.h"

// Navigation.cpp와 순환 include 방지 — 필요한 함수만 전방 선언
void followToCrossing();
void followToCrossing(bool stopAtEnd);  // 연속 주행용 오버로딩
void reverseAcrossToOppositeZone(int targetZone);
void enterZone(int zone);
void reverseEnterZone(int zone);
void alignHeadingOnLine();

int robotHeading = HDG_N;
int currentNode = 8;
bool lastEntryWasForward = true;

// ── [내부] 헬퍼 ──────────────────────────────────────────────

static int zoneToNode(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5) return 10;
  if (zone == 6) return 11;
  return 8;
}

static int nodeIndex(int n) {
  if (n == 7)  return 0;
  if (n == 8)  return 1;
  if (n == 9)  return 2;
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
}

static void blindPostAlign() {
  for (int i = 0; i < 20; i++) {
    int L, C, R;
    readSensors(L, C, R);
    if (!anyLine(L, C, R) || (L && C && R)) break;
    if      ( L && !R) { drive(BLIND_SPEED - 6, BLIND_SPEED + 6); }
    else if (!L &&  R) { drive(BLIND_SPEED + 6, BLIND_SPEED - 6); }
    else               break;
    delay(10);
  }
  softStop();
}

static void advanceToCrossing(bool stopAtEnd, int newHeading) {
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  if (stopAtEnd) stopAll();
  robotHeading = newHeading;
}

static void blindDriveUntilLine(long maxCounts = 0) {
  prizm.resetEncoders();
  while (true) {
    if (maxCounts > 0 && abs(prizm.readEncoderCount(1)) >= maxCounts) break;
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    long diff = abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
    int corr = (abs(diff) <= BLIND_CORR_DEADZONE) ? 0
             : constrain((int)(diff / BLIND_CORR_GAIN), -BLIND_CORR_CAP, BLIND_CORR_CAP);
    drive(BLIND_SPEED - corr, BLIND_SPEED + corr);
    delay(5);
  }
  softStop();
  blindPostAlign();
}

// ── [내부] 노드 간 단일 구간 이동 ────────────────────────────
static void stepNode(int from, int to, bool stopAtEnd) {
  int dir = (to > from) ? HDG_E : HDG_W;
  turnToHeading(dir);

  if (from == 8 && to == 9) {
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (!anyLine(L, C, R)) {
        if (stopAtEnd) softStop();
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
      int corr = (abs(diff) <= BLIND_CORR_DEADZONE) ? 0
               : constrain((int)(diff / BLIND_CORR_GAIN), -BLIND_CORR_CAP, BLIND_CORR_CAP);
      drive(STRAIGHT_SPEED - corr, STRAIGHT_SPEED + corr);
      delay(5);
    }
    followToCrossing(stopAtEnd);

  } else if (from == 9 && to == 10) {
    alignHeadingOnLine();
    prizm.resetEncoders();
    while (true) {
      if (abs(prizm.readEncoderCount(1)) >= BLIND_NODE_MAX) {
        if (stopAtEnd) stopAll();
        robotHeading = HDG_E;
        break;
      }
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) {
        softStop();
        blindPostAlign();
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
      int corr = (abs(diff) <= BLIND_CORR_DEADZONE) ? 0
               : constrain((int)(diff / BLIND_CORR_GAIN), -BLIND_CORR_CAP, BLIND_CORR_CAP);
      drive(BLIND_SPEED - corr, BLIND_SPEED + corr);
      delay(5);
    }

  } else if (from == 10 && to == 9) {
    snapToLine();
    blindDriveUntilLine(BLIND_NODE_MAX);
    advanceToCrossing(stopAtEnd, HDG_W);

  } else if (from == 10 && to == 11) {
    snapToLine();
    blindDriveUntilLine(BLIND_NODE_MAX);
    advanceToCrossing(stopAtEnd, HDG_E);

  } else if (from == 11 && to == 10) {
    snapToLine();
    blindDriveUntilLine(BLIND_NODE_MAX);
    advanceToCrossing(stopAtEnd, HDG_W);

  } else if (from == 11 && to == 12) {
    // 선 없는 블라인드 구간 — 엔코더 기반 고정 거리 동향 이동
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < NODE_11_12_COUNTS) {
      long diff = abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
      int corr = (abs(diff) <= BLIND_CORR_DEADZONE) ? 0
               : constrain((int)(diff / BLIND_CORR_GAIN), -BLIND_CORR_CAP, BLIND_CORR_CAP);
      drive(BLIND_SPEED - corr, BLIND_SPEED + corr);
      delay(5);
    }
    if (stopAtEnd) stopAll();
    robotHeading = HDG_E;

  } else if (from == 12 && to == 11) {
    // 서향 blindDriveUntilLine → 6-11 세로선 감지 → 노드11
    blindDriveUntilLine();
    robotHeading = HDG_W;

  } else {
    followToCrossing(stopAtEnd);
  }

  currentNode = to;
  DPRINTF(">> [NAV] ");
  DPRINT(from);
  DPRINTF("->");
  DPRINTLN(to);
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
    int node = zoneToNode(zone);
    if (node == 8) {
      // 8번 노드(존2·4)만 센서로 교차로 감지 후 후진
      prizm.resetEncoders();
      bool rearCrossFound = false;

      while (true) {
        int RL, RC, RR;
        readRearSensors(RL, RC, RR);
        int L, C, R;
        readSensors(L, C, R);

        bool rearIsCrossing = ((RL && RC) || (RC && RR) || (RL && RR));
        bool frontHasLine = anyLine(L, C, R);
        bool frontIsCrossing = ((L && C) || (C && R) || (L && R));
        bool distanceQualified = (abs(prizm.readEncoderCount(1)) >= NODE8_EXIT_QUAL);

        if (distanceQualified && rearIsCrossing && !rearCrossFound) {
          rearCrossFound = true;
          prizm.resetEncoders();
        }

        if (rearCrossFound && abs(prizm.readEncoderCount(1)) >= REAR_TO_AXLE_COUNTS) break;
        if (!rearCrossFound && abs(prizm.readEncoderCount(1)) > EXIT_SAFETY_COUNTS) break;

        int lsp = -BACK_SPEED, rsp = -BACK_SPEED;
        if (!rearCrossFound) {
          bool rearHasLine = anyRearLine(RL, RC, RR);
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
          if (frontHasLine && anyRearLine(RL,RC,RR) && !rearIsCrossing && !frontIsCrossing) {
            int angCorr = constrain((L - R) * ANGULAR_GAIN, -5, 5);
            lsp -= angCorr;
            rsp += angCorr;
          }
        }
        drive(lsp, rsp);
        liftUpTick();
        liftDownTick();
        delay(5);
      }
      softStop();
    } else {
      // 나머지 노드(7·10·11): 후방 센서 라인 추종 + 엔코더 거리 컷오프
      long exitCounts = (node == 7)  ? NODE7_REV_EXIT_COUNTS
                      : (zone == 5)  ? ZONE5_EXIT_COUNTS
                                     : ZONE6_EXIT_COUNTS;
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < exitCounts) {
        int RL, RC, RR;
        readRearSensors(RL, RC, RR);
        int lsp = -BACK_SPEED, rsp = -BACK_SPEED;
        bool rearHasLine    = anyRearLine(RL, RC, RR);
        bool rearIsCrossing = ((RL && RC) || (RC && RR) || (RL && RR));
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
        drive(lsp, rsp);
        liftUpTick();
        liftDownTick();
        delay(5);
      }
      softStop();
    }
  } else {
    if (zoneToNode(zone) == 7) {
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < NODE7_EXIT_COUNTS) {
        drive(SPEED, SPEED);
        delay(5);
      }
      softStop();
    } else {
      followToCrossing(true);
    }
  }

  currentNode = zoneToNode(zone);
  DPRINTF(">> [NAV] exitZone ");
  DPRINT(zone);
  DPRINTF(" -> node ");
  DPRINTLN(currentNode);
}

void goToZoneDirect(int zone) {
  int targetNode = zoneToNode(zone);
  DPRINTF(">> [NAV] zone ");
  DPRINT(zone);
  DPRINTF(" (node ");
  DPRINT(currentNode);
  DPRINTF("->");
  DPRINT(targetNode);
  DPRINTLNF(")");

  moveToNode(targetNode);

  int zoneSide = (zone == 3 || zone == 4) ? HDG_S : HDG_N;

  if (robotHeading != zoneSide) {
    turnToHeading(zoneSide);
  }

  bool enterForward = (robotHeading == zoneSide);
  if (enterForward) {
    enterZone(zone);
    lastEntryWasForward = true;
  } else {
    reverseEnterZone(zone);
    lastEntryWasForward = false;
  }
}
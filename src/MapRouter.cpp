/* ============================================================
 * MapRouter.cpp - 절대 방위각(Degree) 기반 최단 경로 라우터
 * ============================================================ */
#include "MapRouter.h"
#include "Config.h"
#include "Motion.h"
#include "Navigation.h"

int robotHeading = 0;  // 0=북, 90=동, 180=남, 270=서
int currentNode = 11;
bool lastEntryWasForward = true;

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

void turnToHeading(int targetAngle) {
  targetAngle = (targetAngle % 360 + 360) % 360;
  int diff = targetAngle - robotHeading;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  if (diff == 0) return;

  if (diff > 0) turnAngle(diff, true);
  else turnAngle(-diff, false);

  robotHeading = targetAngle;
  stopAll();
  delay(200);  
}

static void blindDriveUntilLine() {
  prizm.resetEncoders();
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }
}

static void executeBlindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd) {
  turnToHeading(targetHeading);
  blindDriveUntilLine();
  prizm.resetEncoders();
  // ★ 실시간 카운트 변환 적용
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  if (stopAtEnd) stopAll();
  // 다음 구간이 있을 때만 수평 방향 정렬
  // (최종 목적지라면 goToZoneDirect가 존 방향으로 회전하므로 불필요)
  if (alignHeading != -1 && !stopAtEnd) {
    turnToHeading(alignHeading);
  }
}

static void stepNode(int from, int to, bool stopAtEnd) {
  if (from == 8 && to == 9) {
    turnToHeading(90);
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
    turnToHeading(270);
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
    executeBlindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd);
  } else if (from == 10 && to == 9) {
    executeBlindDriveAndAlign(HEADING_10_TO_9, 270, stopAtEnd);
  } else if (from == 10 && to == 11) {
    executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd);
  } else if (from == 11 && to == 10) {
    executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd);
  } else {
    int dir = (to > from) ? 90 : 270;
    turnToHeading(dir);
    followToCrossing(stopAtEnd);
  }

  currentNode = to;
  DPRINTF(">> [NAV] "); DPRINT(from); DPRINTF("->"); DPRINTLN(to);
}

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

// ── [스마트 탈출 로직: 구역에 따른 탈출 거리 분기] ─────────────────────
void exitZone(int zone) {
  int targetNode = zoneToNode(zone); 
  
  // ★ 거리를 불러와 여기서 직접 CM()으로 변환합니다.
  long targetRevDist = CM((zone == 5 || zone == 6) ? DIST_ZONE56_EXIT_REV_CM : DIST_ZONE_EXIT_REV_CM);
  long targetFwdDist = CM((zone == 5 || zone == 6) ? DIST_ZONE56_EXIT_FWD_CM : DIST_ZONE_EXIT_FWD_CM);

  if (lastEntryWasForward) {
    DPRINTLNF(">> [NAV] 후진으로 존 탈출 시작 (상수 거리 + 라인 유지)");
    prizm.resetEncoders();
    
    while (abs(prizm.readEncoderCount(1)) < targetRevDist) {
      int RL, RC, RR;
      readRearSensors(RL, RC, RR);
      reverseLineFollowStep(RL, RC, RR); 
      delay(5);
    }
    stopAll();
  } else {
    DPRINTLNF(">> [NAV] 전진으로 존 탈출 시작 (상수 거리 + 라인 유지)");
    prizm.resetEncoders();
    
    while (abs(prizm.readEncoderCount(1)) < targetFwdDist) {
      int L, C, R, RL, RC, RR;
      readSensors(L, C, R);
      readRearSensors(RL, RC, RR);
      lineFollowStepFull(L, C, R, RL, RC, RR); 
      delay(5);
    }
    stopAll();
  }

  currentNode = targetNode;
  DPRINTF(">> [NAV] exitZone "); DPRINT(zone); DPRINTF(" -> node "); DPRINTLN(currentNode);
}

void goToZoneDirect(int zone) {
  int targetNode = zoneToNode(zone);
  DPRINTF(">> [NAV] zone "); DPRINT(zone); DPRINTF(" (node "); DPRINT(currentNode); DPRINTF("->"); DPRINT(targetNode); DPRINTLNF(")");

  moveToNode(targetNode);

  int zoneSide = (zone == 3 || zone == 4) ? 180 : 0;
  if (robotHeading == 90 || robotHeading == 270) {
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
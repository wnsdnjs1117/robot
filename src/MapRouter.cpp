/* ============================================================
 * MapRouter.cpp - 절대 방위각(Degree) 기반 최단 경로 라우터
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "Motion.h"
#include "Navigation.h"

// 전역 로봇 위치 상태
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

// 스마트 최단 거리 회전 (ex: 270도에서 0도로 갈때 우측으로 90도만 돎)
void turnToHeading(int targetAngle) {
  targetAngle = (targetAngle % 360 + 360) % 360;
  int diff = targetAngle - robotHeading;

  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  if (diff == 0) return;

  if (diff > 0)
    turnAngle(diff, true);
  else
    turnAngle(-diff, false);

  robotHeading = targetAngle;

  stopAll();
  delay(200);  // 회전 후 차체 진동 대기
}

// 센서를 끄고 바닥에 선이 보일 때까지 직진 밀어붙이기
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

// 대각선 맹주행 + 교차로 안착 통합 헬퍼 함수
static void executeBlindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd) {
  turnToHeading(targetHeading);
  blindDriveUntilLine();

  // 선을 만나면 바퀴 축이 교차점에 맞도록 지정된 거리만큼 더 전진
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  if (stopAtEnd) stopAll();

  // 도착 후 다음 이동을 위해 맵 수평 방향으로 재정렬
  if (alignHeading != -1) {
    turnToHeading(alignHeading);
  }
}

// 1칸(Node) 단위 이동 로직 (노드 간 예외 상황 분기)
static void stepNode(int from, int to, bool stopAtEnd) {
  // 8 -> 9 : 십자가가 없는 끊긴 선 구간 돌파
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
    // 9 -> 8 : 끊긴 선에서 출발하여 8번 노드를 찾음
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

    // 대각선 및 가상 라인 돌파 구간
  } else if (from == 9 && to == 10) {
    executeBlindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd);
  } else if (from == 10 && to == 9) {
    executeBlindDriveAndAlign(HEADING_10_TO_9, 270, stopAtEnd);
  } else if (from == 10 && to == 11) {
    executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd);
  } else if (from == 11 && to == 10) {
    executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd);

    // 일반 십자가 라인 구간
  } else {
    int dir = (to > from) ? 90 : 270;
    turnToHeading(dir);
    followToCrossing(stopAtEnd);
  }

  currentNode = to;
  DPRINTF(">> [NAV] ");
  DPRINT(from);
  DPRINTF("->");
  DPRINTLN(to);
}

// 징검다리 횡단 메인 함수 (시작 노드에서 끝 노드까지 stepNode를 반복 호출)
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
  // 로봇이 진입했던 방향을 기억해 뒀다가 반대로 빠져나옵니다
  if (lastEntryWasForward) {
    DPRINTLNF(">> [NAV] 후진으로 존 탈출 시작");
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_EXIT_REV_COUNTS) {
      drive(-BACK_SPEED, -BACK_SPEED);
      delay(5);
    }
    stopAll();
  } else {
    DPRINTLNF(">> [NAV] 전진으로 존 탈출 시작");
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_EXIT_FWD_COUNTS) {
      drive(SPEED, SPEED);
      delay(5);
    }
    stopAll();
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

  // 존 앞에 있는 교차로 노드로 라우팅
  moveToNode(targetNode);

  int zoneSide = (zone == 3 || zone == 4) ? 180 : 0;
  if (robotHeading == 90 || robotHeading == 270) {
    turnToHeading(zoneSide);
  }

  // 회전하는 시간을 아끼기 위해 등지고 있으면 후진으로 진입
  bool enterForward = (robotHeading == zoneSide);
  if (enterForward) {
    enterZone();
    lastEntryWasForward = true;
  } else {
    reverseEnterZone();
    lastEntryWasForward = false;
  }
}
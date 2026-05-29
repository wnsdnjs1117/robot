/* ============================================================
 * MapRouter.cpp - 절대 각도(Degree) 기반 추적 / 대각선 진입 / 거리 탈출
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "Motion.h"

// Navigation.cpp와 순환 include 방지
void followToCrossing();
void followToCrossing(bool stopAtEnd);
void reverseAcrossToOppositeZone();
void enterZone();
void reverseEnterZone();
void alignHeadingOnLine();

// 방위 표현 대신 0~360도 절대 각도 사용 (0: 북, 90: 동, 180: 남, 270: 서)
int robotHeading = 0;
int currentNode = 11;
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
  if (n == 7) return 0;
  if (n == 8) return 1;
  if (n == 9) return 2;
  if (n == 10) return 3;
  if (n == 11) return 4;
  return 1;
}

// 목표 각도로 회전 (최단 거리 방향으로 스마트 회전)
void turnToHeading(int targetAngle) {
  targetAngle = (targetAngle % 360 + 360) % 360;
  int diff = targetAngle - robotHeading;

  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  if (diff == 0) return;

  if (diff > 0) {
    turnAngle(diff, true);
  } else {
    turnAngle(-diff, false);
  }
  robotHeading = targetAngle;

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
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }
}

// ── [내부] 노드 간 단일 구간 이동 (대각선 로직 추가) ────────────
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
    turnToHeading(HEADING_9_TO_10);
    prizm.resetEncoders();
    blindDriveUntilLine();
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();
    turnToHeading(90);

  } else if (from == 10 && to == 9) {
    turnToHeading(HEADING_10_TO_9);
    blindDriveUntilLine();
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();
    turnToHeading(270);

  } else if (from == 10 && to == 11) {
    turnToHeading(HEADING_10_TO_11);
    blindDriveUntilLine();
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();

  } else if (from == 11 && to == 10) {
    turnToHeading(HEADING_11_TO_10);
    blindDriveUntilLine();
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    if (stopAtEnd) stopAll();

  } else {
    int dir = (to > from) ? 90 : 270;
    turnToHeading(dir);
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
    Serial.println(F(">> [NAV] 후진으로 존 탈출 시작"));
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_EXIT_REV_COUNTS) {
      drive(-BACK_SPEED, -BACK_SPEED);
      delay(5);
    }
    stopAll();
  } else {
    Serial.println(F(">> [NAV] 전진으로 존 탈출 시작"));
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_EXIT_FWD_COUNTS) {
      drive(SPEED, SPEED);
      delay(5);
    }
    stopAll();
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
/* ============================================================
 * MapRouter.cpp - 방위각 기반 최단 경로 및 복귀 스마트 탈출
 * ============================================================ */
#include "MapRouter.h"
#include "Config.h"
#include "Motion.h"
#include "Navigation.h"
#include "Lift.h" 

int robotHeading = 0;  
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
  stopAll(); delay(200);  
}

static void blindDriveUntilLine() {
  prizm.resetEncoders(); safeDelay(40);
  lastSensorState = 0; 
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
}

static void executeBlindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd) {
  turnToHeading(targetHeading);
  blindDriveUntilLine();
  prizm.resetEncoders(); safeDelay(40);
  
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  if (stopAtEnd) stopAll();
  if (alignHeading != -1 && !stopAtEnd) turnToHeading(alignHeading);
}

static void stepNode(int from, int to, bool stopAtEnd) {
  if (from == 8 && to == 9) {
    turnToHeading(90);
    while (true) {
      int L, C, R, RL, RC, RR;
      readSensors(L, C, R); readRearSensors(RL, RC, RR);
      if (!anyLine(L, C, R)) { if (stopAtEnd) stopAll(); break; }
      lineFollowStepFull(L, C, R, RL, RC, RR);
      liftUpTick(); liftDownTick(); delay(5);
    }
  } else if (from == 9 && to == 8) {
    turnToHeading(270);
    prizm.resetEncoders(); safeDelay(40);
    lastSensorState = 0;
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    followToCrossing(stopAtEnd);
  } else if (from == 9 && to == 10) {
    executeBlindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd);
  } else if (from == 10 && to == 11) {
    executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd);
  } else if (from == 11 && to == 10) {
    executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd);
  } else if (from == 10 && to == 9) {
    turnToHeading(270);
    prizm.resetEncoders(); safeDelay(40);
    while(abs(prizm.readEncoderCount(1)) < CM(DIST_10_TO_13_CM)) {
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_13_TO_9_2); 
    blindDriveUntilLine();
    prizm.resetEncoders(); safeDelay(40);
    
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    if (stopAtEnd) stopAll();
    else turnToHeading(270); 
  } else {
    int dir = (to > from) ? 90 : 270;
    turnToHeading(dir);
    followToCrossing(stopAtEnd);
  }
  currentNode = to;
}

void moveToNode(int toNode) {
  if (currentNode == toNode) return;
  static const int nodes[] = {7, 8, 9, 10, 11};
  int cur = nodeIndex(currentNode);
  int tgt = nodeIndex(toNode);
  int step = (cur < tgt) ? 1 : -1;
  while (cur != tgt) {
    int next = cur + step;
    bool isFinal = (next == tgt);
    stepNode(nodes[cur], nodes[next], isFinal);
    cur = next;
  }
}

// ── [스마트 탈출 로직 (★탈출 시 절대 조향 없이 완벽한 맹목적 주행 적용)] ──
void exitZone(int zone) {
  int targetNode = zoneToNode(zone); 
  if (targetNode == 7) enableEdgeSteering = true;

  lastSensorState = 0; 

  // 1. 공통: 라인을 찾을 때까지 절대 조향 없이 '맹목적 직진/후진' 구동
  if (lastEntryWasForward) {
    while (true) {
       int L, C, R, RL, RC, RR;
       readSensors(L, C, R); readRearSensors(RL, RC, RR);
       if (anyRearLine(RL, RC, RR)) break;
       drive(-BACK_SPEED, -BACK_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
  } else {
    while (true) {
       int L, C, R, RL, RC, RR;
       readSensors(L, C, R); readRearSensors(RL, RC, RR);
       if (anyLine(L, C, R)) break;
       drive(SPEED, SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
  }

  prizm.resetEncoders(); safeDelay(40); 

  // 2. 탈출 및 바퀴축 정렬 주행
  if (lastEntryWasForward) {
    if (targetNode == 8) {
       while (true) {
          int L, C, R, RL, RC, RR;
          readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (RL && RC && RR) break;
          reverseLineFollowStep(RL, RC, RR, L, C, R);
          liftUpTick(); liftDownTick(); delay(5);
       }
       prizm.resetEncoders(); safeDelay(40);
       
       // ★ 사용자 수치(4.0) 적용
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_REAR_CM)) {
          drive(-BACK_SPEED, -BACK_SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       long targetEscapeDist;
       if (zone == 5 || zone == 6) targetEscapeDist = CM(EXIT_REV_SPECIAL_56_CM); // 30.0
       else if (zone == 1 || zone == 2) targetEscapeDist = CM(EXIT_REV_EXTRA_12_CM); // 33.0
       else targetEscapeDist = CM(EXIT_REV_EXTRA_3456_CM); // 35.0
       
       while (abs(prizm.readEncoderCount(1)) < targetEscapeDist) {
          int L, C, R, RL, RC, RR;
          readSensors(L, C, R); readRearSensors(RL, RC, RR);
          reverseLineFollowStep(RL, RC, RR, L, C, R); 
          liftUpTick(); liftDownTick(); delay(5);
       }
    }
  } else {
    if (targetNode == 8) {
       while (true) {
          int L, C, R, RL, RC, RR;
          readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (L && C && R) break;
          lineFollowStepFull(L, C, R, RL, RC, RR);
          liftUpTick(); liftDownTick(); delay(5);
       }
       prizm.resetEncoders(); safeDelay(40);
       
       // ★ 사용자 수치(6.0) 적용
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_FRONT_CM)) {
          drive(SPEED, SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       long targetEscapeDist;
       if (zone == 1 || zone == 2) targetEscapeDist = CM(EXIT_FWD_EXTRA_12_CM); // 35.0
       else targetEscapeDist = CM(EXIT_FWD_EXTRA_3456_CM); // 37.0

       while (abs(prizm.readEncoderCount(1)) < targetEscapeDist) {
          int L, C, R, RL, RC, RR;
          readSensors(L, C, R); readRearSensors(RL, RC, RR);
          lineFollowStepFull(L, C, R, RL, RC, RR); 
          liftUpTick(); liftDownTick(); delay(5);
       }
    }
  }

  stopAll();
  if (targetNode == 7) enableEdgeSteering = false;
  currentNode = targetNode;
}

void goToZoneDirect(int zone) {
  int targetNode = zoneToNode(zone);
  moveToNode(targetNode);

  int zoneSide = (zone == 3 || zone == 4) ? 180 : 0;
  turnToHeading(zoneSide);

  if (targetNode == 7) enableEdgeSteering = true;

  bool enterForward = (robotHeading == zoneSide);
  if (enterForward) { enterZone(); lastEntryWasForward = true; } 
  else { reverseEnterZone(); lastEntryWasForward = false; }

  if (targetNode == 7) enableEdgeSteering = false;
}
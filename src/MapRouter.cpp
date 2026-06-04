/* ============================================================
 * MapRouter.cpp - 방위각 기반 최단 경로 및 복귀 스마트 탈출 (멈칫거림 완벽 제거)
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

void turnToHeading(int targetAngle) {
  targetAngle = (targetAngle % 360 + 360) % 360;
  int diff = targetAngle - robotHeading;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  
  // 이미 목표 각도면 즉시 그대로 패스
  if (diff == 0) return;

  if (diff > 0) turnAngle(diff, true);
  else turnAngle(-diff, false);

  robotHeading = targetAngle;
}

// ★ 리셋을 없애고 시작값(startEnc) 차이로 거리를 재서 멈칫거림 차단
static void ignoreNodeBlind() {
  long startEnc = abs(prizm.readEncoderCount(1));
  while (abs(abs(prizm.readEncoderCount(1)) - startEnc) < CM(DIST_IGNORE_NODE_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick();
  }
}

static void ignoreNodeTrace() {
  long startEnc = abs(prizm.readEncoderCount(1));
  while (abs(abs(prizm.readEncoderCount(1)) - startEnc) < CM(DIST_IGNORE_NODE_CM)) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR);
    liftUpTick(); liftDownTick();
  }
}

static void blindDriveUntilLine() {
  lastSensorState = 0; 
  // 리셋 완전 삭제
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    liftUpTick(); liftDownTick();
  }
}

static void executeBlindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd) {
  turnToHeading(targetHeading);
  
  ignoreNodeBlind(); 
  blindDriveUntilLine(); 
  
  if (stopAtEnd) {
    // 멈출 때는 바퀴축까지 스무스하게 감속 후 정지
    driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
  } else {
    // 그냥 지나가는 노드면 멈추거나 리셋하지 않고 그대로 돌진!
    long alignEnc = abs(prizm.readEncoderCount(1));
    while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick();
    }
  }
  
  if (alignHeading != -1 && !stopAtEnd) turnToHeading(alignHeading);
}

static void stepNode(int from, int to, bool stopAtEnd) {
  if (from == 8 && to == 9) {
    turnToHeading(90);
    ignoreNodeTrace(); 
    while (true) {
      int L, C, R, RL, RC, RR;
      readSensors(L, C, R); readRearSensors(RL, RC, RR);
      if (!anyLine(L, C, R)) { if (stopAtEnd) stopAll(); break; }
      lineFollowStepFull(L, C, R, RL, RC, RR);
      liftUpTick(); liftDownTick();
    }
  } 
  else if (from == 9 && to == 8) {
    turnToHeading(270);
    ignoreNodeBlind(); 
    lastSensorState = 0;
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick();
    }
    followToCrossing(stopAtEnd);
  } 
  else if (from == 9 && to == 10) { 
    executeBlindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd);
  } 
  else if (from == 9 && to == 11) { 
    turnToHeading(HEADING_9_TO_11); 
    ignoreNodeBlind();     
    blindDriveUntilLine(); 
    ignoreNodeBlind();     
    blindDriveUntilLine(); 
    
    if (stopAtEnd) {
      driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
    } else {
      long alignEnc = abs(prizm.readEncoderCount(1));
      while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) {
        drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
        liftUpTick(); liftDownTick();
      }
    }
  } 
  else if (from == 10 && to == 11) { 
    executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd); 
  } 
  else if (from == 11 && to == 10) { 
    executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd); 
  } 
  else if (from == 10 && to == 9) { 
    turnToHeading(HEADING_10_TO_12); 
    long startEnc = abs(prizm.readEncoderCount(1));
    while(abs(abs(prizm.readEncoderCount(1)) - startEnc) < CM(DIST_10_TO_12_CM)) { 
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick();
    }
    turnToHeading(HEADING_12_TO_9_2); 
    blindDriveUntilLine(); 
    
    if (stopAtEnd) {
      driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
    } else {
      long alignEnc = abs(prizm.readEncoderCount(1));
      while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) {
        drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
        liftUpTick(); liftDownTick();
      }
      turnToHeading(270); 
    }
  } 
  else if (from == 11 && to == 9) { 
    turnToHeading(HEADING_11_TO_12); 
    long startEnc = abs(prizm.readEncoderCount(1));
    while(abs(abs(prizm.readEncoderCount(1)) - startEnc) < CM(DIST_11_TO_12_CM)) { 
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick();
    }
    turnToHeading(HEADING_12_TO_9_2); 
    blindDriveUntilLine(); 
    
    if (stopAtEnd) {
      driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
    } else {
      long alignEnc = abs(prizm.readEncoderCount(1));
      while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) {
        drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
        liftUpTick(); liftDownTick();
      }
      turnToHeading(270); 
    }
  } 
  else {
    int dir = (to > from) ? 90 : 270;
    turnToHeading(dir);
    followToCrossing(stopAtEnd);
  }
  currentNode = to;
}

void moveToNode(int toNode) {
  if (currentNode == toNode) return;
  while (currentNode != toNode) {
    int nextNode = toNode; 
    if (currentNode < 9 && toNode >= 9) nextNode = currentNode + 1; 
    else if (currentNode > 9 && toNode <= 9) nextNode = 9; 
    else if (currentNode == 9 && toNode == 11) nextNode = 11; 
    else if (currentNode == 9 && toNode == 10) nextNode = 10;
    else if (currentNode == 10 && toNode == 11) nextNode = 11;
    else if (currentNode == 11 && toNode == 10) nextNode = 10;
    else if (currentNode == 9 && toNode == 8) nextNode = 8;
    else if (currentNode == 8 && toNode == 7) nextNode = 7;
    else nextNode = (currentNode < toNode) ? currentNode + 1 : currentNode - 1; 
    
    bool isFinal = (nextNode == toNode);
    stepNode(currentNode, nextNode, isFinal);
  }
}

static void exitTraceDist(float totalCm, float offAfterCm, bool forward) {
  long startEnc = abs(prizm.readEncoderCount(1));
  long total   = CM(totalCm);
  long offStart = (offAfterCm > 0.0f) ? CM(offAfterCm) : -1;

  while (abs(abs(prizm.readEncoderCount(1)) - startEnc) < total) {
    int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long d = abs(abs(prizm.readEncoderCount(1)) - startEnc);
    bool mask = (offStart >= 0 && d >= offStart);
    if (forward) {
      if (mask) { L = C = R = 0; }
      lineFollowStepFull(L, C, R, RL, RC, RR);
    } else {
      if (mask) { RL = RC = RR = 0; }
      reverseLineFollowStep(RL, RC, RR, L, C, R);
    }
    liftUpTick(); liftDownTick();
  }
}

static void exitRev56() {
  long startEnc = abs(prizm.readEncoderCount(1));
  while (true) {
    int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (abs(abs(prizm.readEncoderCount(1)) - startEnc) > CM(EXIT_REV_56_ARM_CM) && !anyRearLine(RL, RC, RR)) break;
    reverseLineFollowStep(RL, RC, RR, L, C, R);
    liftUpTick(); liftDownTick();
  }
}

void exitZone(int zone) {
  int targetNode = zoneToNode(zone); 
  if (targetNode == 7) enableEdgeSteering = true;

  lastSensorState = 0; 
  if (lastEntryWasForward) {
    while (true) {
       int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
       if (anyRearLine(RL, RC, RR)) break; 
       drive(-ZONE_EXIT_BLIND_BACK_SPEED, -ZONE_EXIT_BLIND_BACK_SPEED);
       liftUpTick(); liftDownTick();
    }
  } else {
    while (true) {
       int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
       if (anyLine(L, C, R)) break; 
       drive(ZONE_EXIT_BLIND_SPEED, ZONE_EXIT_BLIND_SPEED);
       liftUpTick(); liftDownTick();
    }
  }

  // ★ 존 탈출 후 바퀴 정렬 시에도 리셋 없이 스무스하게 연결
  if (lastEntryWasForward) { 
    if (targetNode == 8) { 
       while (true) {
          int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (RL && RC && RR) break; 
          reverseLineFollowStep(RL, RC, RR, L, C, R); liftUpTick(); liftDownTick();
       }
       long alignEnc = abs(prizm.readEncoderCount(1));
       while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(ALIGN_AXIS_REAR_CM)) {
          drive(-ZONE_EXIT_BLIND_BACK_SPEED, -ZONE_EXIT_BLIND_BACK_SPEED);
          liftUpTick(); liftDownTick();
       }
    } else {
       if (zone == 1)      exitTraceDist(EXIT_REV_EXTRA_1_CM, EXIT1_SENSOR_OFF_AFTER_CM, false);
       else if (zone == 3) exitTraceDist(EXIT_REV_EXTRA_3_CM, EXIT3_SENSOR_OFF_AFTER_CM, false);
       else                exitRev56();
    }
  } else { 
    if (targetNode == 8) { 
       while (true) {
          int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (L && C && R) break; 
          lineFollowStepFull(L, C, R, RL, RC, RR); liftUpTick(); liftDownTick();
       }
       long alignEnc = abs(prizm.readEncoderCount(1));
       while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(ALIGN_AXIS_FRONT_CM)) {
          drive(ZONE_EXIT_BLIND_SPEED, ZONE_EXIT_BLIND_SPEED);
          liftUpTick(); liftDownTick();
       }
    } else {
       if (zone == 1)      exitTraceDist(EXIT_FWD_EXTRA_1_CM, EXIT1_SENSOR_OFF_AFTER_CM, true);
       else if (zone == 3) exitTraceDist(EXIT_FWD_EXTRA_3_CM, EXIT3_SENSOR_OFF_AFTER_CM, true);
       else                exitTraceDist(EXIT_FWD_EXTRA_3_CM, 0.0f, true);
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
  
  if (robotHeading != 0 && robotHeading != 180) {
    turnToHeading(zoneSide);
  }

  if (targetNode == 7) enableEdgeSteering = true;

  bool enterForward = (robotHeading == zoneSide);
  if (enterForward) { 
    enterZone(); 
    lastEntryWasForward = true; 
  } else { 
    reverseEnterZone(); 
    lastEntryWasForward = false; 
  }

  if (targetNode == 7) enableEdgeSteering = false;
}
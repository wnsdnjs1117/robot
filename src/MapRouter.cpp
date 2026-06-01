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

// ★ 새로운 맞춤형 노드 간 이동 로직
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
  } 
  else if (from == 9 && to == 8) {
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
  } 
  else if (from == 9 && to == 10) { // 9-3 -> 10-2
    executeBlindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd);
  } 
  else if (from == 9 && to == 11) { // 9-3 -> 11-2 (10번선 무시)
    turnToHeading(HEADING_9_TO_10); // 동일 축으로 가정
    
    // 1. 첫 번째 선(10-2) 만날 때까지 주행
    blindDriveUntilLine();
    
    // 2. 10번 선을 무시하고 강제로 일정 거리 돌파
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(15.0f)) { 
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    
    // 3. 두 번째 선(11-2) 만날 때까지 주행
    blindDriveUntilLine();
    
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    if (stopAtEnd) stopAll();
  } 
  else if (from == 10 && to == 11) { // 10-2 -> 11-2
    executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd);
  } 
  else if (from == 11 && to == 10) { // 11-2 -> 10-2
    executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd);
  } 
  else if (from == 10 && to == 9) { // 10-2 -> 12 -> 9-2
    turnToHeading(HEADING_10_TO_12); // ★ 180도(아래)로 회전
    prizm.resetEncoders(); safeDelay(40);
    while(abs(prizm.readEncoderCount(1)) < CM(DIST_10_TO_12_CM)) {
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
    
    turnToHeading(HEADING_12_TO_9_2); // ★ 12번 도착 후 9-2를 향해 회전
    blindDriveUntilLine();
    
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    if (stopAtEnd) stopAll();
    else turnToHeading(270); 
  } 
  else if (from == 11 && to == 9) { // 11-2 -> 12 -> 9-2
    turnToHeading(HEADING_11_TO_12); // ★ 대각선(왼쪽 아래) 방향으로 정밀 회전
    prizm.resetEncoders(); safeDelay(40);
    while(abs(prizm.readEncoderCount(1)) < CM(DIST_11_TO_12_CM)) {
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
    
    turnToHeading(HEADING_12_TO_9_2); // ★ 12번 도착 후 9-2를 향해 회전
    blindDriveUntilLine();
    
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    if (stopAtEnd) stopAll();
    else turnToHeading(270); 
  } 
  else {
    int dir = (to > from) ? 90 : 270;
    turnToHeading(dir);
    followToCrossing(stopAtEnd);
  }
  currentNode = to;
}

// ★ 규칙에 따른 다이렉트 이동 판단기
void moveToNode(int toNode) {
  if (currentNode == toNode) return;
  
  while (currentNode != toNode) {
    int nextNode = toNode; 
    
    // 특정 다이렉트 규칙 조건 성립 시 점프
    if (currentNode < 9 && toNode >= 9) nextNode = currentNode + 1; // 7->8, 8->9
    else if (currentNode > 9 && toNode <= 9) nextNode = 9; // 10->9 (12경유), 11->9 (12경유) 모두 한방에 처리
    else if (currentNode == 9 && toNode == 11) nextNode = 11; // 9->11 10번 무시 다이렉트
    else if (currentNode == 9 && toNode == 10) nextNode = 10;
    else if (currentNode == 10 && toNode == 11) nextNode = 11;
    else if (currentNode == 11 && toNode == 10) nextNode = 10;
    else if (currentNode == 9 && toNode == 8) nextNode = 8;
    else if (currentNode == 8 && toNode == 7) nextNode = 7;
    else nextNode = (currentNode < toNode) ? currentNode + 1 : currentNode - 1; // 예외 폴백
    
    bool isFinal = (nextNode == toNode);
    stepNode(currentNode, nextNode, isFinal);
  }
}

// ── [스마트 탈출 로직] ──
void exitZone(int zone) {
  int targetNode = zoneToNode(zone); 
  if (targetNode == 7) enableEdgeSteering = true;

  lastSensorState = 0; 
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
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_REAR_CM)) {
          drive(-BACK_SPEED, -BACK_SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       long targetEscapeDist;
       if (zone == 5 || zone == 6) targetEscapeDist = CM(EXIT_REV_SPECIAL_56_CM); 
       else if (zone == 1 || zone == 2) targetEscapeDist = CM(EXIT_REV_EXTRA_12_CM); 
       else targetEscapeDist = CM(EXIT_REV_EXTRA_3456_CM); 
       
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
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_FRONT_CM)) {
          drive(SPEED, SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       long targetEscapeDist;
       if (zone == 1 || zone == 2) targetEscapeDist = CM(EXIT_FWD_EXTRA_12_CM); 
       else targetEscapeDist = CM(EXIT_FWD_EXTRA_3456_CM); 

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
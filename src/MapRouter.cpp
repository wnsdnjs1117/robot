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
  
  // 이미 목표 각도면 멈추지 않고(stopAll 없이) 즉시 그대로 패스!
  if (diff == 0) return;

  if (diff > 0) turnAngle(diff, true);
  else turnAngle(-diff, false);

  robotHeading = targetAngle;
  // ★ 기존에 있던 중복 stopAll(); delay(200); 완전 삭제
  // turnAngle 내부에서 이미 깔끔하게 브레이크 잡고 해제했음
}

static void ignoreNodeBlind() {
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_IGNORE_NODE_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
}

static void ignoreNodeTrace() {
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_IGNORE_NODE_CM)) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR);
    liftUpTick(); liftDownTick(); delay(5);
  }
}

static void blindDriveUntilLine() {
  lastSensorState = 0; 
  prizm.resetEncoders(); safeDelay(40);
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
}

static void executeBlindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd) {
  turnToHeading(targetHeading);
  
  ignoreNodeBlind(); 
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
    ignoreNodeTrace(); 
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
    ignoreNodeBlind(); 
    lastSensorState = 0;
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
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
    
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    if (stopAtEnd) stopAll();
  } 
  else if (from == 10 && to == 11) { 
    executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd); 
  } 
  else if (from == 11 && to == 10) { 
    executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd); 
  } 
  else if (from == 10 && to == 9) { 
    turnToHeading(HEADING_10_TO_12); 
    prizm.resetEncoders(); safeDelay(40);
    while(abs(prizm.readEncoderCount(1)) < CM(DIST_10_TO_12_CM)) { 
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_12_TO_9_2); 
    blindDriveUntilLine(); 
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    if (stopAtEnd) stopAll();
    else turnToHeading(270); 
  } 
  else if (from == 11 && to == 9) { 
    turnToHeading(HEADING_11_TO_12); 
    prizm.resetEncoders(); safeDelay(40);
    while(abs(prizm.readEncoderCount(1)) < CM(DIST_11_TO_12_CM)) { 
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_12_TO_9_2); 
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
    ignoreNodeTrace(); 
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

// ── [탈출 보조] 라인 추종으로 지정 거리 탈출 (7번 가로선 오판 방지 센서 끄기 포함) ──
//   totalCm: 라인 닿은 지점부터 이동할 총 거리 (바퀴축이 코너에 닿을 때까지)
//   offAfterCm > 0 이면 offAfterCm 지점부터 탈출 끝까지 이동방향 센서를 꺼서,
//   7번 노드 가로선을 밟아도 오판/헛조향하지 않게 한다.
//   forward=true → 전방센서로 추종(전방센서 끔), false → 후방센서로 추종(후방센서 끔)
static void exitTraceDist(float totalCm, float offAfterCm, bool forward) {
  prizm.resetEncoders(); safeDelay(40);
  long total   = CM(totalCm);
  long offStart = (offAfterCm > 0.0f) ? CM(offAfterCm) : -1;

  while (abs(prizm.readEncoderCount(1)) < total) {
    int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long d = abs(prizm.readEncoderCount(1));
    bool mask = (offStart >= 0 && d >= offStart);   // 끄기 시작 후 끝까지 무시
    if (forward) {
      if (mask) { L = C = R = 0; }          // 이동방향(전방) 센서 끔
      lineFollowStepFull(L, C, R, RL, RC, RR);
    } else {
      if (mask) { RL = RC = RR = 0; }       // 이동방향(후방) 센서 끔
      reverseLineFollowStep(RL, RC, RR, L, C, R);
    }
    liftUpTick(); liftDownTick(); delay(5);
  }
}

// ── [탈출 보조] 5·6번 후진 탈출: 후방센서 라인 끊김 감지 시 정착(10-2 / 11-2) ──
//   오감지 방지로 EXIT_REV_56_ARM_CM(23cm) 이후부터 라인 끊김 감지 시작.
static void exitRev56() {
  prizm.resetEncoders(); safeDelay(40);
  while (true) {
    int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (abs(prizm.readEncoderCount(1)) > CM(EXIT_REV_56_ARM_CM) && !anyRearLine(RL, RC, RR)) break;
    reverseLineFollowStep(RL, RC, RR, L, C, R);
    liftUpTick(); liftDownTick(); delay(5);
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
       liftUpTick(); liftDownTick(); delay(5);
    }
  } else {
    while (true) {
       int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
       if (anyLine(L, C, R)) break; 
       drive(ZONE_EXIT_BLIND_SPEED, ZONE_EXIT_BLIND_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
  }

  prizm.resetEncoders(); safeDelay(40); 

  if (lastEntryWasForward) { 
    if (targetNode == 8) { 
       while (true) {
          int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (RL && RC && RR) break; 
          reverseLineFollowStep(RL, RC, RR, L, C, R); liftUpTick(); liftDownTick(); delay(5);
       }
       prizm.resetEncoders(); safeDelay(40);
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_REAR_CM)) {
          drive(-ZONE_EXIT_BLIND_BACK_SPEED, -ZONE_EXIT_BLIND_BACK_SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       // 1·3번(7번 노드): 라인 닿은 후 바퀴축이 코너에 닿을 때까지 후진 (가로선 통과 센서 끔)
       if (zone == 1)      exitTraceDist(EXIT_REV_EXTRA_1_CM, EXIT1_SENSOR_OFF_AFTER_CM, false);
       else if (zone == 3) exitTraceDist(EXIT_REV_EXTRA_3_CM, EXIT3_SENSOR_OFF_AFTER_CM, false);
       // 5·6번: 라인 끊김 감지로 10-2 / 11-2에 정착
       else                exitRev56();
    }
  } else { 
    if (targetNode == 8) { 
       while (true) {
          int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (L && C && R) break; 
          lineFollowStepFull(L, C, R, RL, RC, RR); liftUpTick(); liftDownTick(); delay(5);
       }
       prizm.resetEncoders(); safeDelay(40);
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_FRONT_CM)) {
          drive(ZONE_EXIT_BLIND_SPEED, ZONE_EXIT_BLIND_SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       // 1·3번(7번 노드): 라인 닿은 후 바퀴축이 코너에 닿을 때까지 전진 (가로선 통과 센서 끔)
       if (zone == 1)      exitTraceDist(EXIT_FWD_EXTRA_1_CM, EXIT1_SENSOR_OFF_AFTER_CM, true);
       else if (zone == 3) exitTraceDist(EXIT_FWD_EXTRA_3_CM, EXIT3_SENSOR_OFF_AFTER_CM, true);
       else                exitTraceDist(EXIT_FWD_EXTRA_3_CM, 0.0f, true); // 5·6번 전진 탈출(30cm 기준)
    }
  }
  // 존 탈출 완료 시 정지 (존에 들어갈 때, 나올 때 허용)
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
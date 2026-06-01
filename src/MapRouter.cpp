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

// ★ [추가] 맹목 이동 시: 이전 노드 출발점 선을 완벽히 벗어나기 위해 무시거리를 눈감고 밀고 나감
static void ignoreNodeBlind() {
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_IGNORE_NODE_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
}

// ★ [추가] 라인트레이싱 이동 시: 이전 노드 교차로를 벗어나기 위해 무시거리 동안 교차로 감지 무시
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
  
  ignoreNodeBlind(); // ★ 방향 전환 후 무조건 10cm 강제 탈출
  blindDriveUntilLine(); // 이후 새로운 선 탐색
  
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  if (stopAtEnd) stopAll();
  if (alignHeading != -1 && !stopAtEnd) turnToHeading(alignHeading);
}

// ── [모든 경우 공통 무시 규칙이 적용된 이동 로직] ──
static void stepNode(int from, int to, bool stopAtEnd) {
  if (from == 8 && to == 9) {
    turnToHeading(90);
    ignoreNodeTrace(); // ★ 8번에서 출발할 때 10cm 동안 교차로 무시하며 라인트레이싱

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
    ignoreNodeBlind(); // ★ 9번에서 출발할 때 10cm 동안 선 감지 무시 (완벽 이탈)
    
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
    executeBlindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd); // (내부에 10cm 이탈 로직 포함)
  } 
  else if (from == 9 && to == 11) { 
    turnToHeading(HEADING_9_TO_11); 
    
    ignoreNodeBlind();     // ★ 9번에서 10cm 탈출
    blindDriveUntilLine(); // 10번 만남
    
    ignoreNodeBlind();     // ★ 10번 선을 밟은 상태에서 10cm 다시 탈출 (10번 무시)
    blindDriveUntilLine(); // 비로소 11번 만남
    
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    if (stopAtEnd) stopAll();
  } 
  else if (from == 10 && to == 11) { 
    executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd); // (내부에 10cm 이탈 포함)
  } 
  else if (from == 11 && to == 10) { 
    executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd); // (내부에 10cm 이탈 포함)
  } 
  else if (from == 10 && to == 9) { 
    turnToHeading(HEADING_10_TO_12); 
    prizm.resetEncoders(); safeDelay(40);
    // [예외적 기하학] 10번 -> 12번 이동. 
    while(abs(prizm.readEncoderCount(1)) < CM(DIST_10_TO_12_CM)) { 
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_12_TO_9_2); 
    // 12번은 허공 경유지이므로 빠져나올 선이 없음. 바로 선 탐색 개시!
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
    // [예외적 기하학] 11번 -> 12번.
    while(abs(prizm.readEncoderCount(1)) < CM(DIST_11_TO_12_CM)) { 
       drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_12_TO_9_2); 
    blindDriveUntilLine(); // 허공이므로 바로 탐색 개시
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
    ignoreNodeTrace(); // ★ 7, 8번 등 메인 가로선 이동 시 출발 직후 10cm 강제 주행 (교차로 무시)
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

// ── [스마트 탈출 로직] ──
void exitZone(int zone) {
  int targetNode = zoneToNode(zone); 
  if (targetNode == 7) enableEdgeSteering = true;

  lastSensorState = 0; 
  // 1단계: 라인을 만날 때까지 직진 또는 후진 (맹목적 구동)
  if (lastEntryWasForward) {
    while (true) {
       int L, C, R, RL, RC, RR;
       readSensors(L, C, R); readRearSensors(RL, RC, RR);
       if (anyRearLine(RL, RC, RR)) break; // 후진하다 라인을 만남!
       drive(-ZONE_EXIT_BLIND_BACK_SPEED, -ZONE_EXIT_BLIND_BACK_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
  } else {
    while (true) {
       int L, C, R, RL, RC, RR;
       readSensors(L, C, R); readRearSensors(RL, RC, RR);
       if (anyLine(L, C, R)) break; // 전진하다 라인을 만남!
       drive(ZONE_EXIT_BLIND_SPEED, ZONE_EXIT_BLIND_SPEED);
       liftUpTick(); liftDownTick(); delay(5);
    }
  }

  prizm.resetEncoders(); safeDelay(40); 

  // 2단계: 라인을 만난 직후부터 요구된 '추가 거리' 주행
  if (lastEntryWasForward) { // 후진으로 탈출 중
    if (targetNode == 8) { // 2번, 4번 구역 (교차로 감지)
       while (true) {
          int L, C, R, RL, RC, RR;
          readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (RL && RC && RR) break; // 후면 센서 1.1.1 교차로 감지
          reverseLineFollowStep(RL, RC, RR, L, C, R);
          liftUpTick(); liftDownTick(); delay(5);
       }
       prizm.resetEncoders(); safeDelay(40);
       // 후진 시 (4+1 = 5cm) 주행
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_REAR_CM)) {
          drive(-ZONE_EXIT_BLIND_BACK_SPEED, -ZONE_EXIT_BLIND_BACK_SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       // 1, 3, 5, 6번 구역 맹목적 추가 주행
       long targetEscapeDist;
       if (zone == 1) targetEscapeDist = CM(EXIT_REV_EXTRA_1_CM); // 33cm (28+4+1)
       else if (zone == 3) targetEscapeDist = CM(EXIT_REV_EXTRA_3_CM); // 35cm (30+4+1)
       else targetEscapeDist = CM(EXIT_REV_SPECIAL_56_CM); // 30cm (5, 6번 구역)
       
       while (abs(prizm.readEncoderCount(1)) < targetEscapeDist) {
          int L, C, R, RL, RC, RR;
          readSensors(L, C, R); readRearSensors(RL, RC, RR);
          reverseLineFollowStep(RL, RC, RR, L, C, R); 
          liftUpTick(); liftDownTick(); delay(5);
       }
    }
  } else { // 전진으로 탈출 중
    if (targetNode == 8) { // 2번, 4번 구역 (교차로 감지)
       while (true) {
          int L, C, R, RL, RC, RR;
          readSensors(L, C, R); readRearSensors(RL, RC, RR);
          if (L && C && R) break; // 전면 센서 1.1.1 교차로 감지
          lineFollowStepFull(L, C, R, RL, RC, RR);
          liftUpTick(); liftDownTick(); delay(5);
       }
       prizm.resetEncoders(); safeDelay(40);
       // 전진 시 (6+1 = 7cm) 주행
       while (abs(prizm.readEncoderCount(1)) < CM(ALIGN_AXIS_FRONT_CM)) {
          drive(ZONE_EXIT_BLIND_SPEED, ZONE_EXIT_BLIND_SPEED);
          liftUpTick(); liftDownTick(); delay(5);
       }
    } else {
       // 1, 3번 구역 맹목적 추가 주행 (5, 6번은 항상 후진 탈출이므로 여기에 도달하지 않음)
       long targetEscapeDist;
       if (zone == 1) targetEscapeDist = CM(EXIT_FWD_EXTRA_1_CM); // 35cm (28+6+1)
       else targetEscapeDist = CM(EXIT_FWD_EXTRA_3_CM); // 37cm (30+6+1)

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
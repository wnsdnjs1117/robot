/* ============================================================
 * Navigation.cpp - 거리/각도 기반 진입 및 탈출 탐색 시나리오
 * ============================================================ */
#include "Navigation.h"
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

// ── [1] 일반 수평선 라인 추종 ───────────────────────────────────

void followToCrossing(bool stopAtEnd) {
  {
    int L, C, R;
    readSensors(L, C, R);
    if (L == 1 && C == 1 && R == 1) {
      while (true) {
        readSensors(L, C, R);
        if (!(L == 1 && C == 1 && R == 1)) break;
        drive(SPEED, SPEED);
        delay(5);
      }
      for (int i = 0; i < 10; i++) {
        drive(SPEED, SPEED);
        delay(5);
      }
    }
  }

  crossingArmed = true;
  crossingStable = 0;
  while (true) {
    int L, C, R;
    readSensors(L, C, R);

    if (detectCrossing(L, C, R)) {
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
        drive(SPEED, SPEED);
        delay(5);
      }
      if (stopAtEnd) stopAll();
      return;
    }

    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR);
    delay(5);  
  }
}

void followToCrossing() { followToCrossing(true); }

// ── [2] 존(구역) 진입 및 횡단 ─────────────────────────────────────

void enterZone() {
  DPRINTLNF(">> [NAV] 전진으로 존 진입 시작 (선 끊김 감지 대기)");
  
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R);
    if (!anyLine(L, C, R)) break; 
    
    readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR);
    delay(5);
  }

  DPRINTLNF(">> [NAV] 선 끊김 확인. 바퀴축 기준으로 리프트를 목표 깊이에 배치합니다.");
  prizm.resetEncoders();
  long extraDist = CM(DIST_ZONE_DEPTH_CM + DIST_AXIS_TO_LIFT_CM + DIST_AXIS_TO_FRONT_SENSOR_CM);
  
  while (abs(prizm.readEncoderCount(1)) < extraDist) {
    drive(SPEED / 2, SPEED / 2); 
    delay(5);
  }
  stopAll();
}

void reverseEnterZone() {
  DPRINTLNF(">> [NAV] 후진으로 존 진입 시작 (후방 선 끊김 감지 대기)");
  
  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    if (!anyRearLine(RL, RC, RR)) break; 
    
    reverseLineFollowStep(RL, RC, RR); 
    delay(5);
  }

  DPRINTLNF(">> [NAV] 선 끊김 확인. 바퀴축 기준으로 리프트를 목표 깊이에 배치합니다.");
  prizm.resetEncoders();
  long extraDist = CM(DIST_ZONE_DEPTH_CM - DIST_AXIS_TO_LIFT_CM + DIST_AXIS_TO_REAR_SENSOR_CM);
  
  while (abs(prizm.readEncoderCount(1)) < extraDist) {
    drive(-BACK_SPEED / 2, -BACK_SPEED / 2); 
    delay(5);
  }
  stopAll();
}

void reverseAcrossToOppositeZone() {
  DPRINTLNF(">> [NAV] 반대편 존으로 횡단 (탈출 후 반대편 존 진입)");
  prizm.resetEncoders();
  
  while (abs(prizm.readEncoderCount(1)) < CM(75.0f)) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    reverseLineFollowStep(RL, RC, RR);
    delay(5);
  }
  stopAll();
  delay(100);

  reverseEnterZone();
}

// ── [3] 특수 시나리오: 출발 및 복귀 기동 ──────────────────────────

void goToMainLine() {
  DPRINTLNF(">>> [START-RUN] 서향 출발 -> 12번 노드(빈 공간) -> 9-2 노드 -> 8번 노드");
  robotHeading = 270;
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_START_TO_12_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();  
    delay(5);
  }
  stopAll();
  delay(100);
  turnToHeading(HEADING_12_TO_9);
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);
  turnToHeading(270);
  followToCrossing(true);
  currentNode = 8;
  DPRINTLNF(">> [START] 8번 노드 안착 완료");
}

void returnToFinish() {
  DPRINTLNF("\n========================================");
  
  if (currentNode == 10 || currentNode == 11) {
    DPRINTLNF(">> [FINISH] 복귀 기동 (상단): 11 -> START 다이렉트");
    moveToNode(11);
    delay(100);
    
    turnToHeading(HEADING_11_TO_START); 
    
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(BLIND_SPEED, BLIND_SPEED);
      delay(5);
    }
    
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_FINISH_ENTRY_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    stopAll();
    delay(200);

    DPRINTLNF(">> [FINISH] 박스 내 동쪽(90도) 정렬 기동");
    turnToHeading(90);
  } 
  else {
    DPRINTLNF(">> [FINISH] 복귀 기동 (하단): 9-3 -> 12 -> START");
    moveToNode(9);
    turnToHeading(90);
    
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (!anyLine(L, C, R)) break; 
      int RL, RC, RR;
      readRearSensors(RL, RC, RR);
      lineFollowStepFull(L, C, R, RL, RC, RR);
      delay(5);
    }
    stopAll();
    delay(100);
    
    turnToHeading(HEADING_9_3_TO_12);
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_9_3_TO_12_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    stopAll();
    delay(100);
    
    turnToHeading(HEADING_12_TO_START);
    
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(BLIND_SPEED, BLIND_SPEED);
      delay(5);
    }
    
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_FINISH_ENTRY_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      delay(5);
    }
    stopAll();
    delay(200);

    DPRINTLNF(">> [FINISH] 박스 내 동쪽(90도) 정렬 확인 기동");
    turnToHeading(90);
  }

  stopAll();
  tone(BUZZER_PIN, 1000);
  delay(1500);
  noTone(BUZZER_PIN);
  prizm.setGreenLED(HIGH);
  DPRINTLNF(">> [FINISH] 경기 종료. 완벽하게 안착했습니다!");
  DPRINTLNF("========================================\n");
}

// ── [4] 탐색 스테이지 (1~4구역 순회) ──────────────────────────────────

int qrSearchStage() {
  int randomFound = 0;
  DPRINTLNF("\n--- [2구역 탐색] ---");
  turnToHeading(0); 
  enterZone();
  lastEntryWasForward = true;
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 2;  
  }
  DPRINTLNF("\n--- [4구역 탐색] ---");
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 4;
  }
  DPRINTLNF("\n--- [1구역 탐색] ---");
  followToCrossing();
  turnAngle(90, false);
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  lastEntryWasForward = true;
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 1;
  }
  DPRINTLNF("\n--- [3구역 탐색] ---");
  
  enableEdgeSteering = true;
  reverseAcrossToOppositeZone();
  enableEdgeSteering = false;

  lastEntryWasForward = false;
  scanZone(3);
  stopAll();
  printSearchResult();
  return 3;
}

// ── [5] 일반 메인 복도 교차로 필터링 감지 ─────────────────────

bool detectCrossing(int L, int C, int R) {
  bool isCross = (L == 1 && R == 1);
  if (isCross) crossingStable++; else crossingStable = 0;
  if (!isCross) { crossingArmed = true; return false; }
  
  if (crossingArmed && crossingStable >= 2) {
    crossingArmed = false;
    return true;
  }
  return false;
}
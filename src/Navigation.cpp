/* ============================================================
 * Navigation.cpp - 거리/각도 진입 및 가로 라인 추종 탐색
 * ============================================================ */
#include "Navigation.h"
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

// ── [1] 가로축 교차로 추종 ─────
void followToCrossing(bool stopAtEnd) {
  {
    int L, C, R;
    readSensors(L, C, R);
    if (L == 1 && C == 1 && R == 1) {
      while (true) {
        readSensors(L, C, R);
        if (!(L == 1 && C == 1 && R == 1)) break;
        drive(SPEED, SPEED);
        liftUpTick(); liftDownTick(); delay(5);
      }
      for (int i = 0; i < 10; i++) {
        drive(SPEED, SPEED);
        liftUpTick(); liftDownTick(); delay(5);
      }
    }
  }

  crossingArmed = true; crossingStable = 0;
  
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);

    bool isCross = (L == 1 && C == 1 && R == 1);
    if (isCross) crossingStable++; else crossingStable = 0;
    if (!isCross) crossingArmed = true;

    if (crossingArmed && crossingStable >= CROSS_CONFIRM) {
      crossingArmed = false;
      prizm.resetEncoders(); safeDelay(40); 
      
      while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
        drive(SPEED, SPEED); 
        liftUpTick(); liftDownTick(); delay(5);
      }
      if (stopAtEnd) stopAll();
      return;
    }
    lineFollowStepFull(L, C, R, RL, RC, RR);
    liftUpTick(); liftDownTick(); delay(5);  
  }
}
void followToCrossing() { followToCrossing(true); }

// ── [2] 존(구역) 진입 ────────────────
void enterZone() {
  lastSensorState = 0;
  
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (!anyLine(L, C, R)) break; 
    lineFollowStepFull(L, C, R, RL, RC, RR); 
    liftUpTick(); liftDownTick(); delay(5);
  }

  prizm.resetEncoders(); safeDelay(40); 
  long extraDist = CM(ENTRY_FWD_EXTRA_CM); 
  
  while (abs(prizm.readEncoderCount(1)) < extraDist) {
    drive(SPEED, SPEED); 
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll();
}

void reverseEnterZone() {
  lastSensorState = 0;
  
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (!anyRearLine(RL, RC, RR)) break; 
    reverseLineFollowStep(RL, RC, RR, L, C, R); 
    liftUpTick(); liftDownTick(); delay(5);
  }

  prizm.resetEncoders(); safeDelay(40); 
  long extraDist = CM(ENTRY_REV_EXTRA_CM);
  
  while (abs(prizm.readEncoderCount(1)) < extraDist) {
    drive(-BACK_SPEED, -BACK_SPEED); 
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll();
}

void reverseAcrossToOppositeZone() {
  lastSensorState = 0;
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) break;
    drive(-BACK_SPEED, -BACK_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  prizm.resetEncoders(); safeDelay(40);
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (abs(prizm.readEncoderCount(1)) > CM(40.0f) && !anyRearLine(RL, RC, RR)) break;
    reverseLineFollowStep(RL, RC, RR, L, C, R);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  prizm.resetEncoders(); safeDelay(40);
  long extraDist = CM(ENTRY_REV_EXTRA_CM); 
  
  while (abs(prizm.readEncoderCount(1)) < extraDist) {
    drive(-BACK_SPEED, -BACK_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll();
}

// ── [3] 탐색 및 시작/종료 처리 (★ 새로 수정된 규칙 적용) ────────────────
void goToMainLine() {
  robotHeading = 270; 

  // 1. START의 감싸는 네모 선을 빠져나갈 때까지 전진
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }

  // 2. START -> 13 이동
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_START_TO_13_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 3. 13에서 9-2 방향 턴
  stopAll(); delay(100); turnToHeading(HEADING_13_TO_9);
  
  // 4. 9-2 선(검은선)을 만날 때까지 주행
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 5. 선 넘어가서 십자 교차로 정렬
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll(); delay(100); turnToHeading(270);
  followToCrossing(true); // 9-2에서 8로 이동
  currentNode = 8;
}

void returnToFinish() {
  if (currentNode == 11) {
    // [규칙] 11번노드에서 START노드로 이동 : 11-3 에서 160도 방향 바라보고 이동 -> START
    turnToHeading(160);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; // START 네모 만남
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
  } 
  else if (currentNode == 10) {
    // [규칙] 10번노드에서 START노드로 이동 : 10-3 -> 13 -> START
    turnToHeading(HEADING_10_TO_13); 
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_10_TO_13_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_13_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; // START 네모 만남
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
  } 
  else { 
    // 7, 8, 9번에 있을 경우 먼저 9번으로 복귀
    moveToNode(9); 
    // [규칙] 9번노드에서 START노드로 이동 : 9-3 -> 13 -> START
    turnToHeading(HEADING_9_TO_13); 
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_9_TO_13_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_13_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; // START 네모 만남
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
  }

  // [공통] 네모 라인 안쪽으로 살짝 진입
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_FINISH_ENTRY_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 정면(90도)을 보고 세리머니
  stopAll(); delay(200); turnToHeading(90); 
  stopAll(); tone(BUZZER_PIN, 1000); delay(1500); noTone(BUZZER_PIN); prizm.setGreenLED(HIGH);
}

int qrSearchStage() {
  int randomFound = 0;
  turnToHeading(0); enterZone(); lastEntryWasForward = true;
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) { stopAll(); printSearchResult(); return 2; }
  
  reverseAcrossToOppositeZone(); lastEntryWasForward = false;
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) { stopAll(); printSearchResult(); return 4; }
  
  followToCrossing(); turnAngle(90, false); followToCrossing(); turnAngle(90, true);
  enterZone(); lastEntryWasForward = true;
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) { stopAll(); printSearchResult(); return 1; }
  
  enableEdgeSteering = true; reverseAcrossToOppositeZone(); enableEdgeSteering = false;
  lastEntryWasForward = false; scanZone(3); stopAll(); printSearchResult();
  return 3;
}
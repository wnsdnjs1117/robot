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
    drive(ZONE_ENTRY_BLIND_SPEED, ZONE_ENTRY_BLIND_SPEED); 
    liftUpTick(); liftDownTick(); delay(5);
  }
  // 존 진입 완료 후 멈춤 (허용 구간)
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
    drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED); 
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
    drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
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
    drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll();
}

// ── [3] 탐색 및 시작/종료 처리 ────────────────
void goToMainLine() {
  robotHeading = 270; 

  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }

  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_START_TO_13_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 회전 전 관성만 제어, 무의미한 delay 삭제
  stopAll(); turnToHeading(HEADING_13_TO_9);
  
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 회전 전 관성 제어, 딜레이 삭제
  stopAll(); turnToHeading(270);
  followToCrossing(true); 
  currentNode = 8;
}

void returnToFinish() {
  if (currentNode == 11) {
    turnToHeading(160);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; 
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
  } 
  else if (currentNode == 10) {
    turnToHeading(HEADING_10_TO_13); 
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_10_TO_13_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_13_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; 
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
  } 
  else { 
    moveToNode(9); 
    turnToHeading(HEADING_9_TO_13); 
    prizm.resetEncoders(); safeDelay(40);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_9_TO_13_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    turnToHeading(HEADING_13_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; 
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
  }

  prizm.resetEncoders(); safeDelay(40);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_FINISH_ENTRY_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 종료 지점 세레모니
  stopAll(); turnToHeading(90); 
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
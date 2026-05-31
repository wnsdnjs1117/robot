/* ============================================================
 * Navigation.cpp - 거리/각도 진입 및 가로 라인 추종 탐색
 * ============================================================ */
#include "Navigation.h"
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

// ── [1] 가로축 교차로 추종 (뒤뚱거림 완벽 차단: 축정렬 시 오직 직진) ─────
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

    // 교차로 감지는 무조건 앞 센서 기준
    bool isCross = (L == 1 && C == 1 && R == 1);
    if (isCross) crossingStable++; else crossingStable = 0;
    if (!isCross) crossingArmed = true;

    if (crossingArmed && crossingStable >= CROSS_CONFIRM) {
      crossingArmed = false;
      prizm.resetEncoders(); safeDelay(30); 
      
      // ★ 라인트레이싱(조향)을 끄고 '오직 직진'만 해서 축 거리를 채웁니다. 후진/뒤뚱거림 절대 발생 안함.
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

// ── [2] 존(구역) 진입 ─────────────────────────────────────
void enterZone() {
  lastSensorState = 0;
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (!anyLine(L, C, R)) break; 
    lineFollowStepFull(L, C, R, RL, RC, RR);
    liftUpTick(); liftDownTick(); delay(5);
  }

  prizm.resetEncoders(); safeDelay(30); 
  long extraDist = CM(ENTRY_FWD_EXTRA_CM); // 37.5cm 
  
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

  prizm.resetEncoders(); safeDelay(30); 
  long extraDist = CM(ENTRY_REV_EXTRA_CM); // 27.5cm
  
  while (abs(prizm.readEncoderCount(1)) < extraDist) {
    drive(-BACK_SPEED, -BACK_SPEED); 
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll();
}

void reverseAcrossToOppositeZone() {
  lastSensorState = 0;
  // 빈 땅 맹목적 직진 (후진)
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) break;
    drive(-BACK_SPEED, -BACK_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 반대편 빈땅이 나올때까지 라인 추종
  prizm.resetEncoders(); safeDelay(30);
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (abs(prizm.readEncoderCount(1)) > CM(40.0f) && !anyRearLine(RL, RC, RR)) break;
    reverseLineFollowStep(RL, RC, RR, L, C, R);
    liftUpTick(); liftDownTick(); delay(5);
  }
  
  // 반대편 존 정착
  prizm.resetEncoders(); safeDelay(30);
  long extraDist = CM(ENTRY_REV_EXTRA_CM); // 27.5cm
  
  while (abs(prizm.readEncoderCount(1)) < extraDist) {
    drive(-BACK_SPEED, -BACK_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll();
}

// ── [3] 탐색 및 시작/종료 처리 ───────────────────────────────────
void goToMainLine() {
  robotHeading = 270; 

  // ★ 1. 출발 후 START 박스의 검은 선을 밟을 때까지 전진
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }

  // ★ 2. 선을 밟은 시점부터 12번 노드까지 거리 측정 시작
  prizm.resetEncoders(); safeDelay(30);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_START_TO_12_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll(); delay(100); turnToHeading(HEADING_12_TO_9);
  
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  prizm.resetEncoders(); safeDelay(30);
  while (abs(prizm.readEncoderCount(1)) < CM(DIST_CROSS_ALIGN_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick(); delay(5);
  }
  stopAll(); delay(100); turnToHeading(270);
  followToCrossing(true);
  currentNode = 8;
}

void returnToFinish() {
  if (currentNode == 10 || currentNode == 11) {
    moveToNode(11); delay(100); turnToHeading(HEADING_11_TO_START); 
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    prizm.resetEncoders(); safeDelay(30);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_FINISH_ENTRY_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    stopAll(); delay(200); turnToHeading(90);
  } else {
    moveToNode(9); turnToHeading(90);
    while (true) {
      int L, C, R, RL, RC, RR;
      readSensors(L, C, R); readRearSensors(RL, RC, RR);
      if (!anyLine(L, C, R)) break; 
      lineFollowStepFull(L, C, R, RL, RC, RR);
      liftUpTick(); liftDownTick(); delay(5);
    }
    stopAll(); delay(100); turnToHeading(HEADING_9_3_TO_12);
    prizm.resetEncoders(); safeDelay(30);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_9_3_TO_12_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    stopAll(); delay(100); turnToHeading(HEADING_12_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    prizm.resetEncoders(); safeDelay(30);
    while (abs(prizm.readEncoderCount(1)) < CM(DIST_FINISH_ENTRY_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      liftUpTick(); liftDownTick(); delay(5);
    }
    stopAll(); delay(200); turnToHeading(90);
  }
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
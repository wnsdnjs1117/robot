/* ============================================================
 * Navigation.cpp - 무중단 교차로 통과 및 스무스 감속 정지 로직 적용
 * ============================================================ */
#include "Navigation.h"
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

// ── [1] 가로축 교차로 추종 ─────
void followToCrossing(bool stopAtEnd) {
  // 이미 교차로 위에 있다면 교차로를 빠져나갈 때까지 부드럽게 라인트레이싱 유지
  {
    int L, C, R;
    readSensors(L, C, R);
    if (L == 1 && C == 1 && R == 1) {
      while (true) {
        readSensors(L, C, R);
        if (!(L == 1 && C == 1 && R == 1)) break;
        lineFollowStepFull(L, C, R, 0, 0, 0, SPEED);
        liftUpTick(); liftDownTick();
      }
      // 교차로 이탈 후 3cm 정도만 더 부드럽게 전진 (엔코더 리셋 없이)
      long clearEnc = abs(prizm.readEncoderCount(1));
      while(abs(abs(prizm.readEncoderCount(1)) - clearEnc) < CM(3.0)) {
         drive(SPEED, SPEED);
         liftUpTick(); liftDownTick();
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
      
      if (stopAtEnd) {
         // ★ 멈춰야 할 경우: 교차로 감지 후 바퀴 축까지 부드럽게 감속(20)하다가 정지!
         driveExtraDecel(DIST_CROSS_ALIGN_CM, SPEED);
         return;
      } else {
         // ★ 안 멈추는 경우 (9->10 등): 엔코더 리셋을 없애서 '멈칫거림(Hesitation)' 완벽 해결!
         long alignEnc = abs(prizm.readEncoderCount(1));
         while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) {
           drive(SPEED, SPEED); 
           liftUpTick(); liftDownTick();
         }
         return;
      }
    }
    lineFollowStepFull(L, C, R, RL, RC, RR, SPEED);
    liftUpTick(); liftDownTick(); 
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
    lineFollowStepFull(L, C, R, RL, RC, RR, ZONE_ENTRY_BLIND_SPEED); 
    liftUpTick(); liftDownTick(); 
  }

  // ★ 블라인드 탐지 후 바퀴 축까지 부드럽게 감속하여 완벽하게 안착
  driveExtraDecel(ENTRY_FWD_EXTRA_CM, ZONE_ENTRY_BLIND_SPEED);
}

void reverseEnterZone() {
  lastSensorState = 0;
  
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (!anyRearLine(RL, RC, RR)) break; 
    reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_ENTRY_BLIND_BACK_SPEED); 
    liftUpTick(); liftDownTick(); 
  }

  // ★ 후진 존 진입 시에도 부드럽게 감속하여 안착
  driveExtraDecel(ENTRY_REV_EXTRA_CM, -ZONE_ENTRY_BLIND_BACK_SPEED);
}

void reverseAcrossToOppositeZone() {
  lastSensorState = 0;
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) break;
    drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick();
  }
  
  long startEnc = abs(prizm.readEncoderCount(1));
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (abs(abs(prizm.readEncoderCount(1)) - startEnc) > CM(40.0f) && !anyRearLine(RL, RC, RR)) break;
    reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick();
  }
  
  driveExtraDecel(ENTRY_REV_EXTRA_CM, -ZONE_ENTRY_BLIND_BACK_SPEED);
}

// ── [3] 탐색 및 시작/종료 처리 ────────────────
void goToMainLine() {
  robotHeading = 270; 

  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick();
  }

  // ★ 13번 노드 도착 직전! 선을 발견하자마자 축까지 부드럽게 감속하면서 정지(요청하신 기능!)
  driveExtraDecel(DIST_START_TO_13_CM, STRAIGHT_SPEED);
  
  turnToHeading(HEADING_13_TO_9);
  
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    liftUpTick(); liftDownTick();
  }
  
  driveExtraDecel(DIST_CROSS_ALIGN_CM, BLIND_SPEED);
  
  turnToHeading(270);
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
      liftUpTick(); liftDownTick();
    }
  } 
  else if (currentNode == 10) {
    turnToHeading(HEADING_10_TO_13); 
    driveStraightSmooth(DIST_10_TO_13_CM, STRAIGHT_SPEED); // S-커브 주행 적용
    turnToHeading(HEADING_13_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; 
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick();
    }
  } 
  else { 
    moveToNode(9); 
    turnToHeading(HEADING_9_TO_13); 
    driveStraightSmooth(DIST_9_TO_13_CM, STRAIGHT_SPEED); // S-커브 주행 적용
    turnToHeading(HEADING_13_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; 
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick();
    }
  }

  // ★ 마지막 종료 지점 진입 시에도 부드럽게 감속하여 정확한 위치에 안착
  driveExtraDecel(DIST_FINISH_ENTRY_CM, BLIND_SPEED);
  
  // 종료 지점 세레모니
  stopAll(); turnToHeading(90); 
  stopAll();
  
  pinMode(BUZZER_PIN, OUTPUT);
  unsigned long _beepEnd = millis() + 1500;
  while (millis() < _beepEnd) {
    digitalWrite(BUZZER_PIN, HIGH); delayMicroseconds(500);
    digitalWrite(BUZZER_PIN, LOW);  delayMicroseconds(500);
  }
  digitalWrite(BUZZER_PIN, LOW);
  prizm.setGreenLED(HIGH);
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
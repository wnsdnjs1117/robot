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
// 가로(7~9) 이동 시 출발 노드의 교차선을 목적지로 오판하지 않도록:
//   1) 출발 직후 DIST_IGNORE_NODE_CM(5cm) 동안은 교차로 감지를 금지(라인만 추종).
//      이 구간의 1.1.1은 출발 노드의 세로선일 뿐이므로 조향 없이 직진 처리됨.
//   2) 무시 구간을 지난 뒤, "비교차(빈 라인) 상태를 한 번 본 다음"에만 무장(armed)하여
//      다음에 만나는 1.1.1을 목적지 교차로로 확정한다.
void followToCrossing(bool stopAtEnd) {
  lastSensorState = 0;
  prizm.resetEncoders(); safeDelay(40);
  crossingArmed = false; crossingStable = 0;
  long ignore = CM(DIST_IGNORE_NODE_CM);

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long d = abs(prizm.readEncoderCount(1));
    bool isCross = (L == 1 && C == 1 && R == 1);

    // [1] 출발 노드 무시 구간: 감지 금지, 라인만 추종 (1.1.1=세로선 → 직진/무조향)
    if (d < ignore) {
      lineFollowStepFull(L, C, R, RL, RC, RR);
      liftUpTick(); liftDownTick(); delay(5);
      continue;
    }

    // [2] 무시 구간 통과 후: 빈 라인을 한 번 본 뒤에만 무장
    if (isCross) crossingStable++; else { crossingStable = 0; crossingArmed = true; }

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
// 전진 진입: 라인을 따라가다 ★후방센서★가 존 라인을 벗어나면, 리프트가 존 중앙에 올 때까지
//            ENTRY_FWD_EXTRA_CM(26cm) 더 직진 후 멈춤.
//            진입 초반 ENTRY_FWD_REAR_OFF_CM(6.5cm) 동안은 후방센서가 7/8번 가로선 위에
//            있으므로 후방센서를 무시한다(조향·끊김판정 모두).
void enterZone() {
  lastSensorState = 0;
  prizm.resetEncoders(); safeDelay(40);

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    bool early = abs(prizm.readEncoderCount(1)) < CM(ENTRY_FWD_REAR_OFF_CM);
    if (!early && !anyRearLine(RL, RC, RR)) break;   // 후방센서가 존 라인을 벗어남
    if (early) { RL = RC = RR = 0; }                 // 초반: 후방센서 무시
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

// 후진 진입: 라인을 따라가다 ★전방센서★가 존 라인을 벗어나면 ENTRY_REV_EXTRA_CM(3cm) 더 후진.
//            진입 초반 ENTRY_REV_FRONT_OFF_CM(8.5cm) 동안은 전방센서가 7/8번 가로선 위에
//            있으므로 전방센서를 무시한다(조향·끊김판정 모두).
void reverseEnterZone() {
  lastSensorState = 0;
  prizm.resetEncoders(); safeDelay(40);

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    bool early = abs(prizm.readEncoderCount(1)) < CM(ENTRY_REV_FRONT_OFF_CM);
    if (!early && !anyLine(L, C, R)) break;          // 전방센서가 존 라인을 벗어남
    if (early) { L = C = R = 0; }                    // 초반: 전방센서 무시
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
    // 반대편 존에 깊이 들어가 ★전방센서★가 존 라인을 벗어나면 종료 (40cm 가드로 조기탈출 방지)
    if (abs(prizm.readEncoderCount(1)) > CM(40.0f) && !anyLine(L, C, R)) break;
    // 후진 진입 초반: 전방센서 무시 (7/8번 가로선 오판 방지)
    if (abs(prizm.readEncoderCount(1)) < CM(ENTRY_REV_FRONT_OFF_CM)) { L = C = R = 0; }
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
  stopAll();
  // 부저 비프 (tone()/noTone() 미링크 환경 대비 — 1kHz 사각파 직접 구동, 1.5초)
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
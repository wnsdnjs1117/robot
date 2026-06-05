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
// ★ 전진 진입: 전방센서로 조향. 초반 후방센서 무시(교차로 선 오감지 방지) 후,
//   후방(후행)센서가 라인을 봤다가 끊기는 지점 기준으로 추가 전진 → 리프트를 존 중앙에 안착.
void enterZone(int zone) {
  ZoneCfg c = zoneCfg(zone);
  lastSensorState = 0;
  long s = labs(prizm.readEncoderCount(1));
  bool armed = false;

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long d = labs(labs(prizm.readEncoderCount(1)) - s);

    if (d < CM(c.entryFwdRearMute)) { RL = RC = RR = 0; }   // 초반 후방센서 무시
    else if (anyRearLine(RL, RC, RR)) armed = true;          // 후방이 존 라인을 잡음
    if (armed && !anyRearLine(RL, RC, RR)) break;            // 후방 라인 끊김 = 기준점

    lineFollowStepFull(L, C, R, RL, RC, RR, ZONE_ENTRY_BLIND_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  driveExtraDecel(c.entryFwdExtra, ZONE_ENTRY_BLIND_SPEED);
}

// ★ 후진 진입: 후방센서로 조향. 초반 전방센서 무시 후,
//   전방(후행)센서가 라인을 봤다가 끊기는 지점 기준으로 추가 후진.
void reverseEnterZone(int zone) {
  ZoneCfg c = zoneCfg(zone);
  lastSensorState = 0;
  long s = labs(prizm.readEncoderCount(1));
  bool armed = false;

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long d = labs(labs(prizm.readEncoderCount(1)) - s);

    if (d < CM(c.entryRevFrontMute)) { L = C = R = 0; }      // 초반 전방센서 무시
    else if (anyLine(L, C, R)) armed = true;                 // 전방이 존 라인을 잡음
    if (armed && !anyLine(L, C, R)) break;                   // 전방 라인 끊김 = 기준점

    reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  driveExtraDecel(c.entryRevExtra, -ZONE_ENTRY_BLIND_BACK_SPEED);
}

// ★ 반대편 존으로 후진 횡단: ① 후방(선행)센서가 반대편 라인을 잡을 때까지 블라인드 후진,
//   ② 반대편 라인을 후진 추종하다 전방(후행)센서 끊김에서 정지(최소 횡단거리 가드 적용).
void reverseAcrossToOppositeZone(int zone) {
  ZoneCfg c = zoneCfg(zone);
  lastSensorState = 0;

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) break;
    drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  long s = labs(prizm.readEncoderCount(1));
  bool armed = false;
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long d = labs(labs(prizm.readEncoderCount(1)) - s);

    if (d < CM(c.entryRevFrontMute)) { L = C = R = 0; }
    else if (anyLine(L, C, R)) armed = true;
    if (d > CM(ZONE_CROSS_MIN_CM) && armed && !anyLine(L, C, R)) break;

    reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  driveExtraDecel(c.entryRevExtra, -ZONE_ENTRY_BLIND_BACK_SPEED);
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

// 1~4구역 중 박스를 인식한 개수
static int countFound1to4() {
  int c = 0;
  for (int z = 1; z <= 4; z++) if (boxes[z].found) c++;
  return c;
}

// 1패스: 2→4→1→3 순으로 진입+대기 스캔. 박스는 정확히 2개이므로 2개 찾으면 조기 종료.
// 각 존 진입 직전 arm, 다음 존으로 횡단하기 직전 disarm(융합 역방향 횡단에서 오귀속 방지).
int qrSearchStage() {
  scanArm(2); turnToHeading(0); enterZone(2); lastEntryWasForward = true;
  scanZone(2);
  if (countFound1to4() >= 2) { scanDisarm(); stopAll(); printSearchResult(); return 2; }
  scanDisarm();

  scanArm(4); reverseAcrossToOppositeZone(4); lastEntryWasForward = false;
  scanZone(4);
  if (countFound1to4() >= 2) { scanDisarm(); stopAll(); printSearchResult(); return 4; }
  scanDisarm();

  followToCrossing(); turnAngle(90, false); followToCrossing(); turnAngle(90, true);
  scanArm(1); enterZone(1); lastEntryWasForward = true;
  scanZone(1);
  if (countFound1to4() >= 2) { scanDisarm(); stopAll(); printSearchResult(); return 1; }
  scanDisarm();

  scanArm(3); enableEdgeSteering = true; reverseAcrossToOppositeZone(3); enableEdgeSteering = false;
  lastEntryWasForward = false; scanZone(3); scanDisarm();
  stopAll(); printSearchResult();
  return 3;
}

// 1패스 후 인식<2면, 미발견 존(빈 존 포함)을 재진입해 2개 찾을 때까지 재스캔.
// (빈 존 2개가 있으므로 종료 조건은 '인식수==2'. MAX_RESCAN_TRIES로 무한루프 방지)
void rescanZones1to4() {
  int tries = 0;
  while (countFound1to4() < 2 && tries < MAX_RESCAN_TRIES) {
    for (int z = 1; z <= 4 && countFound1to4() < 2; z++) {
      if (boxes[z].found) continue;
      scanArm(z);
      goToZoneDirect(z);   // 진입 스캔
      scanZone(z);         // 대기 스캔
      exitZone(z);         // 탈출 스캔 후 노드 복귀
      scanDisarm();
    }
    tries++;
  }
}
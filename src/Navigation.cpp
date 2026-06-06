/* ============================================================
 * Navigation.cpp - 무중단 교차로 통과 및 스무스 감속 정지 로직 적용
 * ============================================================ */
#include "Navigation.h"
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

void followToCrossing(bool stopAtEnd) {
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
         driveExtraDecel(DIST_CROSS_ALIGN_CM, SPEED);
         return;
      } else {
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

void enterZone(int zone) {
  ZoneCfg c = zoneCfg(zone);
  lastSensorState = 0;
  long s = labs(prizm.readEncoderCount(1));
  bool armed = false;

  DPRINTF("\n+Fwd Z:"); DPRINT(zone);

  // 1. Min/Max 삭제, 선 만날 때까지만 이동
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long currentDist = labs(labs(prizm.readEncoderCount(1)) - s);

    if (currentDist > CM(8.0f)) { 
        if (anyLine(L, C, R)) { 
            if(!armed) DPRINTF(" L1");
            armed = true; 
        }
    }
    if (armed && !anyLine(L, C, R)) { DPRINTF(" L0"); break; }

    lineFollowStepFull(L, C, R, RL, RC, RR, ZONE_ENTRY_BLIND_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  // 2. Extra 거리만큼만 추가 이동
  long remainCounts = CM(c.entryFwdExtra);
  if (remainCounts > 0) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    while (labs(labs(prizm.readEncoderCount(1)) - currentEnc) < remainCounts) {
        drive(ZONE_ENTRY_BLIND_SPEED, ZONE_ENTRY_BLIND_SPEED);
        liftUpTick(); liftDownTick(); scanTick();
    }
  }
  
  stopAll(); 
  float actualCm = (float)(labs(labs(prizm.readEncoderCount(1)) - s)) / COUNTS_PER_CM;
  DPRINTF(" Done (실제 이동: "); DPRINT(actualCm); DPRINTLNF(" cm)");
}

void reverseEnterZone(int zone) {
  ZoneCfg c = zoneCfg(zone);
  lastSensorState = 0;
  long s = labs(prizm.readEncoderCount(1));
  bool armed = false;

  DPRINTF("\n+Rev Z:"); DPRINT(zone);

  // 1. Min/Max 삭제, 선 만날 때까지만 이동
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long currentDist = labs(labs(prizm.readEncoderCount(1)) - s);

    if (currentDist > CM(8.0f)) { 
        if (anyRearLine(RL, RC, RR)) { 
            if(!armed) DPRINTF(" L1");
            armed = true; 
        }
    }
    if (armed && !anyRearLine(RL, RC, RR)) { DPRINTF(" L0"); break; }

    reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  // 2. Extra 거리만큼만 추가 이동
  long remainCounts = CM(c.entryRevExtra);
  if (remainCounts > 0) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    while (labs(labs(prizm.readEncoderCount(1)) - currentEnc) < remainCounts) {
        drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
        liftUpTick(); liftDownTick(); scanTick();
    }
  }
  
  stopAll(); 
  float actualCm = (float)(labs(labs(prizm.readEncoderCount(1)) - s)) / COUNTS_PER_CM;
  DPRINTF(" Done (실제 이동: "); DPRINT(actualCm); DPRINTLNF(" cm)");
}

void reverseAcrossToOppositeZone(int zone) {
  ZoneCfg c = zoneCfg(zone);
  lastSensorState = 0;

  DPRINTF("\n+Cross Z:"); DPRINT(zone);

  // 첫번째 선 넘기
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) { DPRINTF(" FindLine"); break; }
    drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  long s = labs(prizm.readEncoderCount(1));
  bool armed = false;
  
  // 두번째 선 찾기 (Min/Max 삭제)
  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long currentDist = labs(labs(prizm.readEncoderCount(1)) - s);

    if (currentDist > CM(8.0f)) { 
        if (anyRearLine(RL, RC, RR)) {
            if(!armed) DPRINTF(" L1");
            armed = true;
        }
    }
    if (armed && !anyRearLine(RL, RC, RR)) { DPRINTF(" L0"); break; }

    reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_ENTRY_BLIND_BACK_SPEED);
    liftUpTick(); liftDownTick(); scanTick();
  }

  // Extra 거리만큼 추가 이동
  long remainCounts = CM(c.entryRevExtra);
  if (remainCounts > 0) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    while (labs(labs(prizm.readEncoderCount(1)) - currentEnc) < remainCounts) {
        drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
        liftUpTick(); liftDownTick(); scanTick();
    }
  }
  
  stopAll(); 
  float actualCm = (float)(labs(labs(prizm.readEncoderCount(1)) - s)) / COUNTS_PER_CM;
  DPRINTF(" Done (실제 이동: "); DPRINT(actualCm); DPRINTLNF(" cm)");
}

void goToMainLine() {
  robotHeading = 270; 
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftUpTick(); liftDownTick();
  }
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
    driveStraightSmooth(DIST_10_TO_13_CM, STRAIGHT_SPEED); 
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
    driveStraightSmooth(DIST_9_TO_13_CM, STRAIGHT_SPEED);
    turnToHeading(HEADING_13_TO_START);
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break; 
      drive(BLIND_SPEED, BLIND_SPEED);
      liftUpTick(); liftDownTick();
    }
  }
  driveExtraDecel(DIST_FINISH_ENTRY_CM, BLIND_SPEED);
  stopAll(); turnToHeading(90); stopAll();
  
  pinMode(BUZZER_PIN, OUTPUT);
  unsigned long _beepEnd = millis() + 1500;
  while (millis() < _beepEnd) {
    digitalWrite(BUZZER_PIN, HIGH); delayMicroseconds(500);
    digitalWrite(BUZZER_PIN, LOW);  delayMicroseconds(500);
  }
  digitalWrite(BUZZER_PIN, LOW);
  prizm.setGreenLED(HIGH);
}

static int countFound1to4() {
  int c = 0;
  for (int z = 1; z <= 4; z++) if (boxes[z].found) c++;
  return c;
}

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

void rescanZones1to4() {
  int tries = 0;
  while (countFound1to4() < 2 && tries < MAX_RESCAN_TRIES) {
    for (int z = 1; z <= 4 && countFound1to4() < 2; z++) {
      if (boxes[z].found) continue;
      scanArm(z); goToZoneDirect(z); scanZone(z); exitZone(z); scanDisarm();
    }
    tries++;
  }
}
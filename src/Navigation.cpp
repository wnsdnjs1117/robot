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

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long currentDist = labs(labs(prizm.readEncoderCount(1)) - s);

    if (currentDist > CM(c.entryFwdMax)) { DPRINTF(" !MAX"); break; }

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

  long distSoFar = labs(labs(prizm.readEncoderCount(1)) - s);
  long targetTotalDist = distSoFar + CM(c.entryFwdExtra);

  if (targetTotalDist < CM(c.entryFwdMin)) { targetTotalDist = CM(c.entryFwdMin); DPRINTF(" <MIN"); }
  else if (targetTotalDist > CM(c.entryFwdMax)) { targetTotalDist = CM(c.entryFwdMax); DPRINTF(" >MAX"); }
  else { DPRINTF(" OK"); }

  long remainCounts = targetTotalDist - distSoFar;
  if (remainCounts > 0) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    while (labs(labs(prizm.readEncoderCount(1)) - currentEnc) < remainCounts) {
        drive(ZONE_ENTRY_BLIND_SPEED, ZONE_ENTRY_BLIND_SPEED);
        liftUpTick(); liftDownTick(); scanTick();
    }
  }
  stopAll(); DPRINTLNF(" Done");
}

void reverseEnterZone(int zone) {
  ZoneCfg c = zoneCfg(zone);
  lastSensorState = 0;
  long s = labs(prizm.readEncoderCount(1));
  bool armed = false;

  DPRINTF("\n+Rev Z:"); DPRINT(zone);

  while (true) {
    int L, C, R, RL, RC, RR;
    readSensors(L, C, R); readRearSensors(RL, RC, RR);
    long currentDist = labs(labs(prizm.readEncoderCount(1)) - s);

    if (currentDist > CM(c.entryRevMax)) { DPRINTF(" !MAX"); break; }

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

  long distSoFar = labs(labs(prizm.readEncoderCount(1)) - s);
  long targetTotalDist = distSoFar + CM(c.entryRevExtra);

  if (targetTotalDist < CM(c.entryRevMin)) { targetTotalDist = CM(c.entryRevMin); DPRINTF(" <MIN"); }
  else if (targetTotalDist > CM(c.entryRevMax)) { targetTotalDist = CM(c.entryRevMax); DPRINTF(" >MAX"); }
  else { DPRINTF(" OK"); }

  long remainCounts = targetTotalDist - distSoFar;
  if (remainCounts > 0) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    while (labs(labs(prizm.readEncoderCount(1)) - currentEnc) < remainCounts) {
        drive(-ZONE_ENTRY_BLIND_BACK_SPEED, -ZONE_ENTRY_BLIND_BACK_SPEED);
        liftUpTick(); liftDownTick(); scanTick();
    }
  }
  stopAll(); DPRINTLNF(" Done");
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
  // --- 2번 구역 ---
  scanArm(2); turnToHeading(0); enterZone(2); lastEntryWasForward = true;
  scanZone(2);
  if (countFound1to4() >= 2) { scanDisarm(); stopAll(); printSearchResult(); return 2; }
  scanDisarm();

  // ★ 2번 구역을 완전히 빠져나온 후 -> 4번 구역으로 후진 진입
  exitZone(2);
  
  scanArm(4); 
  reverseEnterZone(4); 
  lastEntryWasForward = false;
  scanZone(4);
  if (countFound1to4() >= 2) { scanDisarm(); stopAll(); printSearchResult(); return 4; }
  scanDisarm();

  // ★ 4번 구역을 완전히 빠져나온 후 -> 1번 구역 앞(7번 노드)으로 이동
  exitZone(4);
  turnToHeading(270); 
  followToCrossing(true); // 8번 노드에서 7번 노드로 이동
  turnToHeading(0);

  // --- 1번 구역 ---
  scanArm(1); enterZone(1); lastEntryWasForward = true;
  scanZone(1);
  if (countFound1to4() >= 2) { scanDisarm(); stopAll(); printSearchResult(); return 1; }
  scanDisarm();

  // ★ 1번 구역을 완전히 빠져나온 후 -> 3번 구역으로 후진 진입
  exitZone(1);
  
  scanArm(3); 
  enableEdgeSteering = true; 
  reverseEnterZone(3); 
  enableEdgeSteering = false;
  lastEntryWasForward = false; 
  scanZone(3); 
  scanDisarm();
  
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
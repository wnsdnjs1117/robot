/* ============================================================
 * Navigation.cpp - 교차로·구역 진입·QR 탐색 시나리오
 * ============================================================ */
#include "Navigation.h"
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

namespace {

// 전진/후진 구역 진입: 맹목 가속 → 입구선 라인추종 → 엑스트라(일정속 맹목)
void runZoneEntry(bool reverse, int zone, int openSpeed) {
  ZoneMotionProfile profile = getZoneProfile(zone);
  float extraDistance = reverse ? profile.entryReverseExtra : profile.entryForwardExtra;
  int lineSpeed = reverse ? SPEED_LINE_FOLLOW_REV : SPEED_LINE_FOLLOW_FWD;

  lineTraceLastEdge = 0;
  resetLineTracePid();
  DriveEncMark motionStart = captureDriveEnc();
  long extraSpan = toEncoderCounts(extraDistance);

  bool sawLine = false;
  bool pastLine = false;
  DriveEncMark pastLineMark = {0, 0};

  if (reverse) DPRINTF("\n+Rev Z:");
  else          DPRINTF("\n+Fwd Z:");
  DPRINT(zone);

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);
    bool onLine = reverse ? rearOnLine(rl, rc, rr) : frontOnLine(fl, fc, fr);

    if (!pastLine) {
      if (onLine) {
        if (!sawLine) { DPRINTF(" L1"); sawLine = true; }
        if (reverse) traceLineReverse(rl, rc, rr, fl, fc, fr, lineSpeed);
        else         traceLineForward(fl, fc, fr, rl, rc, rr, lineSpeed);
      } else if (sawLine) {
        DPRINTF(" L0");
        pastLine = true;
        pastLineMark = captureDriveEnc();
        if (extraSpan <= 0) break;
      } else {
        int speed = blindRampSpeed(motionStart, openSpeed);
        int dir = reverse ? -1 : 1;
        setWheelSpeeds(dir * speed, dir * speed);
      }
    } else {
      long traveled = encoderTraveledSince(pastLineMark);
      if (traveled >= extraSpan) break;
      int speed = blindDecelSpeed(pastLineMark, extraSpan, openSpeed);
      if (speed <= 0) break;
      int dir = reverse ? -1 : 1;
      setWheelSpeeds(dir * speed, dir * speed);
    }
    liftUpTick(); liftDownTick(); pollZoneScan();
  }

  stopMotors();
  DPRINTF(" Done (실제 이동: ");
  DPRINT((float)encoderTraveledSince(motionStart) / COUNTS_PER_CM);
  DPRINTLNF(" cm)");
}

int countScannedBoxesInZones1to4() {
  int count = 0;
  for (int z = 1; z <= 4; z++) if (boxes[z].found) count++;
  return count;
}

} // namespace

void traceUntilIntersection(bool stopAtEnd) {
  resetLineTracePid();
  int fl, fc, fr;
  readFrontLineSensors(fl, fc, fr);
  if (fl == 1 && fc == 1 && fr == 1) {
    long clearEnc = labs(prizm.readEncoderCount(1));
    while (true) {
      readFrontLineSensors(fl, fc, fr);
      if (!(fl == 1 && fc == 1 && fr == 1)) break;
      traceLineForward(fl, fc, fr, 0, 0, 0, RAMP_MIN_SPEED);
      liftUpTick(); liftDownTick();
    }
    while (labs(labs(prizm.readEncoderCount(1)) - clearEnc) < toEncoderCounts(3.0f)) {
      setWheelSpeeds(RAMP_MIN_SPEED, RAMP_MIN_SPEED);
      liftUpTick(); liftDownTick();
    }
  }

  long startEnc = labs(prizm.readEncoderCount(1));
  long accelSpan = toEncoderCounts(RAMP_ACCEL_CM);
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);

  intersectionArmed = true;
  intersectionHitCount = 0;
  bool crossFound = false;
  long crossEnc = 0;
  int speedAtCross = SPEED_LINE_FOLLOW_FWD;

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long traveled = currentEnc - startEnc;
    int fl2, fc2, fr2, rl, rc, rr;
    readFrontLineSensors(fl2, fc2, fr2);
    readRearLineSensors(rl, rc, rr);

    if (!crossFound) {
      bool isCross = (fl2 == 1 && fc2 == 1 && fr2 == 1);
      if (isCross) intersectionHitCount++;
      else intersectionHitCount = 0;
      if (!isCross) intersectionArmed = true;

      if (intersectionArmed && intersectionHitCount >= CROSS_CONFIRM) {
        if (!stopAtEnd) break;
        crossFound = true;
        crossEnc = currentEnc;
        speedAtCross = calcRampUpSpeed(traveled, accelSpan, SPEED_LINE_FOLLOW_FWD);
        if (alignSpan <= 0) break;
      } else {
        int speed = calcRampUpSpeed(traveled, accelSpan, SPEED_LINE_FOLLOW_FWD);
        traceLineForward(fl2, fc2, fr2, rl, rc, rr, speed);
      }
    }

    if (crossFound) {
      long remaining = alignSpan - (currentEnc - crossEnc);
      if (remaining <= 0) break;
      int speed = stopAtEnd
          ? calcRampDownSpeed(remaining, alignSpan, speedAtCross)
          : speedAtCross;
      setWheelSpeeds(speed, speed);
    }
    liftUpTick(); liftDownTick();
  }
  if (stopAtEnd) stopMotors();
}

void traceUntilIntersection() { traceUntilIntersection(true); }

void enterZoneForward(int zone) {
  runZoneEntry(false, zone, SPEED_OPEN_DRIVE_FWD);
}

void enterZoneReverse(int zone) {
  runZoneEntry(true, zone, SPEED_OPEN_DRIVE_REV);
}

void crossToOppositeZone(int targetZone, int fromZone, bool enableScan) {
  ZoneMotionProfile targetProfile = getZoneProfile(targetZone);
  lineTraceLastEdge = 0;
  resetLineTracePid();
  DPRINTF("\n+Cross Z:"); DPRINT(targetZone);

  if (enableScan && fromZone > 0) beginZoneScan(fromZone);

  DriveEncMark motionStart = captureDriveEnc();
  long extraSpan = toEncoderCounts(targetProfile.entryReverseExtra);

  // 1) 탈출선까지 맹목 후진 (가속 유지)
  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);
    if (rearOnLine(rl, rc, rr)) { DPRINTF(" FindLine"); break; }
    int speed = blindRampSpeed(motionStart, SPEED_OPEN_DRIVE_REV);
    setWheelSpeeds(-speed, -speed);
    liftUpTick(); liftDownTick(); pollZoneScan();
  }

  if (enableScan) beginZoneScan(targetZone);

  // 2) 탈출선 따라 후진 라인 추종 (일정 속도, startEnc 리셋 없음)
  bool pastLine = false;
  DriveEncMark pastLineMark = {0, 0};

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);
    bool onLine = rearOnLine(rl, rc, rr);

    if (!pastLine) {
      if (!onLine) {
        DPRINTF(" L0");
        pastLine = true;
        pastLineMark = captureDriveEnc();
        if (extraSpan <= 0) break;
      } else {
        traceLineReverse(rl, rc, rr, fl, fc, fr, SPEED_LINE_FOLLOW_REV);
      }
    } else {
      long traveled = encoderTraveledSince(pastLineMark);
      if (traveled >= extraSpan) break;
      int speed = blindDecelSpeed(pastLineMark, extraSpan, SPEED_OPEN_DRIVE_REV);
      if (speed <= 0) break;
      setWheelSpeeds(-speed, -speed);
    }
    liftUpTick(); liftDownTick(); pollZoneScan();
  }

  stopMotors();
  DPRINTLNF(" Cross Done");
}

void driveOntoMainTrack() {
  headingDeg = 270;
  long startEnc = labs(prizm.readEncoderCount(1));
  long accelSpan = toEncoderCounts(RAMP_ACCEL_CM);

  while (true) {
    long traveled = labs(prizm.readEncoderCount(1)) - startEnc;
    int fl, fc, fr;
    readFrontLineSensors(fl, fc, fr);
    if (frontOnLine(fl, fc, fr)) break;
    int speed = calcRampUpSpeed(traveled, accelSpan, START_LINE_SEARCH_SPEED);
    setWheelSpeeds(speed, speed);
    liftUpTick(); liftDownTick();
  }

  driveDistanceCm(DIST_START_TO_13_CM, SPEED_OPEN_DRIVE_FWD, true);
  rotateToHeading(HEADING_13_TO_9);
  driveOverLinesAndAlign(1, DIST_CROSS_ALIGN_CM, SPEED_OPEN_DRIVE_FWD, true);
  rotateToHeading(270);
  traceUntilIntersection(true);
  intersectionNode = 8;
}

void driveToFinishArea() {
  if (intersectionNode == 11) {
    rotateToHeading(160);
    driveOverLinesAndAlign(1, DIST_FINISH_ENTRY_CM, SPEED_OPEN_DRIVE_FWD, true);
  }
  else if (intersectionNode == 10) {
    rotateToHeading(HEADING_10_TO_13);
    driveDistanceCm(DIST_10_TO_13_CM, SPEED_OPEN_DRIVE_FWD, true);
    rotateToHeading(HEADING_13_TO_START);
    driveOverLinesAndAlign(1, DIST_FINISH_ENTRY_CM, SPEED_OPEN_DRIVE_FWD, true);
  }
  else {
    driveToIntersectionNode(9);
    rotateToHeading(HEADING_9_TO_13);
    driveDistanceCm(DIST_9_TO_13_CM, SPEED_OPEN_DRIVE_FWD, true);
    rotateToHeading(HEADING_13_TO_START);
    driveOverLinesAndAlign(1, DIST_FINISH_ENTRY_CM, SPEED_OPEN_DRIVE_FWD, true);
  }

  stopMotors();
  rotateToHeading(90);
  stopMotors();

  pinMode(PIN_BUZZER, OUTPUT);
  unsigned long beepEnd = millis() + 1500;
  while (millis() < beepEnd) {
    digitalWrite(PIN_BUZZER, HIGH); delayMicroseconds(500);
    digitalWrite(PIN_BUZZER, LOW);  delayMicroseconds(500);
  }
  digitalWrite(PIN_BUZZER, LOW);
  prizm.setGreenLED(HIGH);
}

int searchQrInZones1to4() {
  beginZoneScan(2);
  rotateToHeading(0);
  enterZoneForward(2);
  enteredZoneForward = true;
  waitForZoneScan(2);
  if (countScannedBoxesInZones1to4() >= 2) {
    endZoneScan(); stopMotors(); printSearchResult(); return 2;
  }

  crossToOppositeZone(4, 2);
  enteredZoneForward = false;
  waitForZoneScan(4);
  if (countScannedBoxesInZones1to4() >= 2) {
    endZoneScan(); stopMotors(); printSearchResult(); return 4;
  }
  endZoneScan();

  leaveZone(4);
  rotateByDegrees(90, false);
  traceUntilIntersection();
  rotateByDegrees(90, true);

  beginZoneScan(1);
  enterZoneForward(1);
  enteredZoneForward = true;
  waitForZoneScan(1);
  if (countScannedBoxesInZones1to4() >= 2) {
    endZoneScan(); stopMotors(); printSearchResult(); return 1;
  }

  enableTeeZoneSteering = true;
  crossToOppositeZone(3, 1);
  enableTeeZoneSteering = false;
  enteredZoneForward = false;
  waitForZoneScan(3);
  endZoneScan();
  stopMotors();
  printSearchResult();
  return 3;
}

void rescanMissingQrZones1to4() {
  int tries = 0;
  while (countScannedBoxesInZones1to4() < 2 && tries < MAX_RESCAN_TRIES) {
    for (int z = 1; z <= 4 && countScannedBoxesInZones1to4() < 2; z++) {
      if (boxes[z].found) continue;
      beginZoneScan(z);
      navigateToZone(z);
      waitForZoneScan(z);
      leaveZone(z);
      endZoneScan();
    }
    tries++;
  }
}

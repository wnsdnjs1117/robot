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

// 전진/후진 구역 진입: 맹목 가속 → 라인 추종(cruise) → 입구선 이후 맹목 감속
void runZoneEntry(bool reverse, int zone, int openSpeed) {
  ZoneMotionProfile profile = getZoneProfile(zone);
  float extraDistance = reverse ? profile.entryReverseExtra : profile.entryForwardExtra;
  lineTraceLastEdge = 0;
  resetLineTracePid();
  DriveEncMark motionStart = captureDriveEnc();
  long extraSpan = toEncoderCounts(extraDistance);

  bool sawLine = false;
  bool pastLine = false;
  DriveEncMark lineStartMark = {0, 0};
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
        if (!sawLine) {
          DPRINTF(" L1");
          sawLine = true;
          lineStartMark = captureDriveEnc();
        }
        int speed = rampMarkSpeed(lineStartMark, openSpeed);
        if (reverse) traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
        else         traceLineForward(fl, fc, fr, rl, rc, rr, speed);
      } else if (sawLine) {
        DPRINTF(" L0");
        pastLine = true;
        pastLineMark = captureDriveEnc();
        if (extraSpan <= 0) break;
      } else {
        int speed = rampMarkSpeed(motionStart, openSpeed);
        int dir = reverse ? -1 : 1;
        setWheelSpeeds(dir * speed, dir * speed);
      }
    } else {
      int speed = decelMarkSpeed(pastLineMark, extraSpan, openSpeed);
      if (finishEncoderSpan(pastLineMark, extraSpan, speed)) break;
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

void alignOnTrackHeading(int openSpeed, float alignCm) {
  resetLineTracePid();
  lineTraceLastEdge = 0;
  DriveEncMark motionStart = captureDriveEnc();
  long alignSpan = toEncoderCounts(alignCm);
  bool onLine = false;

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);

    if (!onLine) {
      if (frontOnLine(fl, fc, fr)) onLine = true;
      else {
        int speed = rampMarkSpeed(motionStart, openSpeed);
        setWheelSpeeds(speed, speed);
        liftUpTick(); liftDownTick();
        continue;
      }
    }

    int speed = decelMarkSpeed(motionStart, alignSpan, openSpeed);
    if (finishEncoderSpan(motionStart, alignSpan, speed)) break;
    traceLineForward(fl, fc, fr, rl, rc, rr, speed);
    liftUpTick(); liftDownTick();
  }
}

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
  long accelSpan = rampCmCountsAtSpeed(RAMP_ACCEL_CM, SPEED_LINE_FOLLOW_FWD);
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);

  intersectionArmed = true;
  intersectionHitCount = 0;
  bool crossFound = false;
  DriveEncMark crossMark = {0, 0};

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
        crossMark = captureDriveEnc();
        if (alignSpan <= 0) break;
      } else {
        int speed = calcRampUpSpeed(traveled, accelSpan, SPEED_LINE_FOLLOW_FWD);
        traceLineForward(fl2, fc2, fr2, rl, rc, rr, speed);
        liftUpTick(); liftDownTick();
        continue;
      }
    }

    int speed = crossAlignSpeed(crossMark, alignSpan);
    if (finishEncoderSpan(crossMark, alignSpan, speed)) break;
    traceLineForward(fl2, fc2, fr2, rl, rc, rr, speed);
    liftUpTick(); liftDownTick();
  }
}

void traceUntilIntersection() { traceUntilIntersection(true); }

void enterZoneForward(int zone) {
  runZoneEntry(false, zone, SPEED_OPEN_ZONE_FWD);
}

void enterZoneReverse(int zone) {
  runZoneEntry(true, zone, SPEED_OPEN_ZONE_REV);
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
    int speed = rampMarkSpeed(motionStart, SPEED_OPEN_ZONE_REV);
    setWheelSpeeds(-speed, -speed);
    liftUpTick(); liftDownTick(); pollZoneScan();
  }

  if (enableScan) beginZoneScan(targetZone);

  // 2) 탈출선 따라 후진 — 라인 cruise → 입구선 이후 맹목 감속
  DriveEncMark lineStartMark = captureDriveEnc();

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);
    bool onLine = rearOnLine(rl, rc, rr);

    if (onLine) {
      int speed = rampMarkSpeed(lineStartMark, SPEED_OPEN_ZONE_REV);
      traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
    } else {
      DPRINTF(" L0");
      DriveEncMark pastLineMark = captureDriveEnc();
      if (extraSpan <= 0) break;
      while (true) {
        int speed = decelMarkSpeed(pastLineMark, extraSpan, SPEED_OPEN_ZONE_REV);
        if (finishEncoderSpan(pastLineMark, extraSpan, speed)) break;
        setWheelSpeeds(-speed, -speed);
        liftUpTick(); liftDownTick(); pollZoneScan();
      }
      break;
    }
    liftUpTick(); liftDownTick(); pollZoneScan();
  }

  stopMotors();
  DPRINTLNF(" Cross Done");
}

void driveOntoMainTrack() {
  headingDeg = 270;
  long startEnc = labs(prizm.readEncoderCount(1));
  long accelSpan = rampCmCountsAtSpeed(RAMP_ACCEL_CM, START_LINE_SEARCH_SPEED);

  while (true) {
    long traveled = labs(prizm.readEncoderCount(1)) - startEnc;
    int fl, fc, fr;
    readFrontLineSensors(fl, fc, fr);
    if (frontOnLine(fl, fc, fr)) break;
    int speed = calcRampUpSpeed(traveled, accelSpan, START_LINE_SEARCH_SPEED);
    setWheelSpeeds(speed, speed);
    liftUpTick(); liftDownTick();
  }

  driveDistanceCm(DIST_START_TO_13_CM, SPEED_OPEN_TRACK_FWD, true);
  rotateToHeading(HEADING_13_TO_9);
  driveOverLinesAndAlign(1, DIST_CROSS_ALIGN_CM, SPEED_OPEN_TRACK_FWD, true);
  rotateToHeading(270);
  traceUntilIntersection(true);
  intersectionNode = 8;
}

void driveToFinishArea() {
  if (intersectionNode == 11) {
    rotateToHeading(160);
    driveOverLinesAndAlign(1, DIST_FINISH_ENTRY_CM, SPEED_OPEN_TRACK_FWD, true);
  }
  else if (intersectionNode == 10) {
    rotateToHeading(HEADING_10_TO_13);
    driveDistanceCm(DIST_10_TO_13_CM, SPEED_OPEN_TRACK_FWD, true);
    rotateToHeading(HEADING_13_TO_START);
    driveOverLinesAndAlign(1, DIST_FINISH_ENTRY_CM, SPEED_OPEN_TRACK_FWD, true);
  }
  else {
    driveToIntersectionNode(9);
    rotateToHeading(HEADING_9_TO_13);
    driveDistanceCm(DIST_9_TO_13_CM, SPEED_OPEN_TRACK_FWD, true);
    rotateToHeading(HEADING_13_TO_START);
    driveOverLinesAndAlign(1, DIST_FINISH_ENTRY_CM, SPEED_OPEN_TRACK_FWD, true);
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
  enterZoneAt(2);
  waitForZoneScan(2);
  if (countScannedBoxesInZones1to4() >= 2) {
    endZoneScan(); stopMotors(); printSearchResult(); return 2;
  }

  moveBetweenZones(2, 4, zoneMoveOpts(true, true));
  waitForZoneScan(4);
  if (countScannedBoxesInZones1to4() >= 2) {
    endZoneScan(); stopMotors(); printSearchResult(); return 4;
  }

  moveBetweenZones(4, 1, zoneMoveOpts(true, true));
  waitForZoneScan(1);
  if (countScannedBoxesInZones1to4() >= 2) {
    endZoneScan(); stopMotors(); printSearchResult(); return 1;
  }

  moveBetweenZones(1, 3, zoneMoveOpts(true, true));
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

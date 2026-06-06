/* ============================================================
 * MapRouter.cpp - 교차로 노드 간 경로 및 구역 탈출
 * ============================================================ */
#include "MapRouter.h"
#include "Config.h"
#include "Motion.h"
#include "Navigation.h"
#include "Lift.h"
#include "BoxMap.h"

int  headingDeg = 0;
int  intersectionNode = 11;
bool enteredZoneForward = true;

static int zoneToIntersection(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5) return 10;
  if (zone == 6) return 11;
  return 8;
}

void rotateToHeading(int targetDeg) {
  targetDeg = (targetDeg % 360 + 360) % 360;
  int diff = targetDeg - headingDeg;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  if (diff == 0) return;
  if (diff > 0) rotateByDegrees(diff, true);
  else          rotateByDegrees(-diff, false);
  headingDeg = targetDeg;
}

static int trackLegSpeed(DriveEncMark motionStart, long ignoreSpan, long approachStart,
    bool& approachDecel, DriveEncMark& approachMark, int cruiseSpeed) {
  long traveled = encoderTraveledSince(motionStart);
  if (traveled < ignoreSpan)
    return RAMP_MIN_SPEED;
  if (traveled < approachStart)
    return rampMarkSpeed(motionStart, cruiseSpeed);
  if (!approachDecel) {
    approachDecel = true;
    approachMark = captureDriveEnc();
  }
  return decelMarkSpeed(approachMark, rampDecelSpanCounts(cruiseSpeed), cruiseSpeed);
}

static void blindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd,
    float legSpanCm, int lineCount = 1) {
  rotateToHeading(targetHeading);
  resetLineTracePid();
  resetRampSpeedLimiter(RAMP_MIN_SPEED);
  const int cruiseSpeed = SPEED_OPEN_TRACK_FWD;
  DriveEncMark motionStart = captureDriveEnc();
  long ignoreSpan = toEncoderCounts(DIST_IGNORE_NODE_CM);
  long approachStart = trackLegApproachStartCounts(legSpanCm, cruiseSpeed);
  long approachDecelSpan = rampDecelSpanCounts(cruiseSpeed);
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);
  bool approachDecel = false;
  DriveEncMark approachMark = {0, 0};
  int linesPassed = 0;
  bool lineFound = false;
  bool ignoreAfterLine = false;
  DriveEncMark ignoreMark = {0, 0};
  DriveEncMark lineMark = {0, 0};

  while (true) {
    if (!lineFound) {
      if (ignoreAfterLine) {
        if (encoderTraveledSince(ignoreMark) >= ignoreSpan)
          ignoreAfterLine = false;
      } else {
        int fl, fc, fr;
        readFrontLineSensors(fl, fc, fr);
        if (frontOnLine(fl, fc, fr)) {
          linesPassed++;
          if (linesPassed >= lineCount) {
            lineFound = true;
            lineMark = captureDriveEnc();
            if (alignSpan <= 0) {
              if (stopAtEnd) stopMotors();
              break;
            }
          } else {
            ignoreAfterLine = true;
            ignoreMark = captureDriveEnc();
          }
        }
      }
      if (!lineFound) {
        int speed = trackLegSpeed(motionStart, ignoreSpan, approachStart, approachDecel,
            approachMark, cruiseSpeed);
        setWheelSpeeds(speed, speed);
        driveLoopTick();
        continue;
      }
    }

    int alignStart = approachDecel
        ? decelMarkSpeed(approachMark, approachDecelSpan, cruiseSpeed)
        : cruiseSpeed;
    int speed = crossAlignSpeed(lineMark, alignSpan, alignStart);
    if (stopAtEnd && finishAlignSpan(lineMark, alignSpan)) break;
    if (!stopAtEnd && encoderTraveledSince(lineMark) >= alignSpan) break;
    setWheelSpeeds(speed, speed);
    driveLoopTick();
  }

  if (alignHeading != -1 && !stopAtEnd) rotateToHeading(alignHeading);
}

void driveTrackLegBlind(int targetHeading, int alignHeading, bool stopAtEnd,
    float legSpanCm, int lineCount) {
  blindDriveAndAlign(targetHeading, alignHeading, stopAtEnd, legSpanCm, lineCount);
}

// 7↔8 직선 — RAMP_DECEL 접근 감속, stopAtEnd=false면 교차 통과(정지 없음)
static void stepTrackLeg78(int heading, bool stopAtEnd) {
  rotateToHeading(heading);
  resetLineTracePid();
  clearIntersectionCross();
  DriveEncMark motionStart = captureDriveEnc();
  long ignoreSpan = toEncoderCounts(DIST_IGNORE_NODE_CM);
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);
  long approachStart = trackNodeApproachStartCounts(SPEED_TRACK_7_9_LINE);
  long approachDecelSpan = rampDecelSpanCounts(SPEED_TRACK_7_9_LINE);
  intersectionArmed = true;
  intersectionHitCount = 0;
  bool crossFound = false;
  bool approachDecel = false;
  DriveEncMark crossMark = {0, 0};
  DriveEncMark approachMark = {0, 0};

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readLineSensors(fl, fc, fr, rl, rc, rr);

    if (!crossFound) {
      if (encoderTraveledSince(motionStart) < ignoreSpan) {
        traceLineForward(fl, fc, fr, rl, rc, rr, RAMP_MIN_SPEED,
            LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
      } else {
        bool isCross = frontCrossFull(fl, fc, fr);
        if (isCross) intersectionHitCount++;
        else { intersectionHitCount = 0; intersectionArmed = true; }
        if (intersectionArmed && intersectionHitCount >= CROSS_CONFIRM) {
          if (!stopAtEnd) break;
          crossFound = true;
          crossMark = captureDriveEnc();
          if (alignSpan <= 0) break;
        } else {
          int speed = trackLegSpeed(motionStart, ignoreSpan, approachStart, approachDecel,
              approachMark, SPEED_TRACK_7_9_LINE);
          traceLineForward(fl, fc, fr, rl, rc, rr, speed,
              LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
          driveLoopTick();
          continue;
        }
      }
      if (!crossFound) {
        driveLoopTick();
        continue;
      }
    }

    int alignStart = approachDecel
        ? decelMarkSpeed(approachMark, approachDecelSpan, SPEED_TRACK_7_9_LINE)
        : SPEED_TRACK_7_9_LINE;
    int speed = crossAlignSpeed(crossMark, alignSpan, alignStart);
    if (finishAlignSpan(crossMark, alignSpan)) break;
    traceLineForward(fl, fc, fr, rl, rc, rr, speed,
        LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
    driveLoopTick();
  }
  if (stopAtEnd)
    correctTrackLegOvershoot(motionStart, DIST_TRACK_NODE_SPAN_CM);
}

// 8→9 / 7→9 — 라인 끝에서 정지. legSpanCm=전체 직선(7→9는 140cm)
static void stepTrackLegToLineEnd(float legSpanCm, bool stopAtEnd) {
  rotateToHeading(90);
  resetLineTracePid();
  bool longLeg = (legSpanCm >= DIST_TRACK_7_TO_9_CM - 0.01f);
  if (stopAtEnd && !longLeg)
    clearIntersectionCross();
  else if (longLeg && stopAtEnd) {
    int fl0, fc0, fr0, rl0, rc0, rr0;
    readLineSensors(fl0, fc0, fr0, rl0, rc0, rr0);
    if (frontCrossFull(fl0, fc0, fr0))
      clearIntersectionCross();
  }
  DriveEncMark motionStart = captureDriveEnc();
  long ignoreSpan = toEncoderCounts(DIST_IGNORE_NODE_CM);
  long approachStart = toEncoderCounts(legSpanCm
      - RAMP_DECEL_CM * rampCruiseFactor(SPEED_TRACK_7_9_LINE));
  long node8Center = toEncoderCounts(DIST_TRACK_NODE_SPAN_CM);
  long node8PassHalf = toEncoderCounts(DIST_TRACK_NODE8_PASS_HALF_CM);
  bool approachDecel = false;
  bool lineSeen = false;
  DriveEncMark approachMark = {0, 0};

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readLineSensors(fl, fc, fr, rl, rc, rr);
    if (frontOnLine(fl, fc, fr)) lineSeen = true;

    long traveled = encoderTraveledSince(motionStart);
    int speed = trackLegSpeed(motionStart, ignoreSpan, approachStart, approachDecel,
        approachMark, SPEED_TRACK_7_9_LINE);

    if (longLeg && traveled >= node8Center - node8PassHalf
        && traveled <= node8Center + node8PassHalf
        && frontCrossFull(fl, fc, fr)) {
      resetRampSpeedLimiter(speed);
      setWheelSpeeds(speed, speed);
      liftUpTick(); liftDownTick();
      continue;
    }

    if (!frontOnLine(fl, fc, fr)) {
      if (lineSeen && traveled >= approachStart) {
        if (stopAtEnd) stopMotors();
        break;
      }
      resetRampSpeedLimiter(speed);
      speed = smoothRampSpeed(speed);
      setWheelSpeeds(speed, speed);
    } else {
      traceLineForward(fl, fc, fr, rl, rc, rr, speed,
          LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
    }
    driveLoopTick();
  }
  if (stopAtEnd)
    correctTrackLegOvershoot(motionStart, legSpanCm);
}

static void stepBetweenNodes(int fromNode, int toNode, bool stopAtEnd) {
  if (fromNode == 8 && toNode == 9) {
    stepTrackLegToLineEnd(DIST_TRACK_NODE_SPAN_CM, stopAtEnd);
  }
  else if (fromNode == 7 && toNode == 8) {
    stepTrackLeg78(90, stopAtEnd);
  }
  else if (fromNode == 7 && toNode == 9) {
    stepTrackLegToLineEnd(DIST_TRACK_7_TO_9_CM, stopAtEnd);
  }
  else if (fromNode == 8 && toNode == 7) {
    stepTrackLeg78(270, stopAtEnd);
  }
  else if (fromNode == 9 && toNode == 8) {
    rotateToHeading(270);
    driveOverLinesAndAlign(1, 0, SPEED_9_TO_8, false);
    setWheelSpeeds(0, 0);
    delayWithTicks(40);
    traceUntilIntersection(stopAtEnd, SPEED_9_TO_8);
  }
  else if (fromNode == 9 && toNode == 10) {
    blindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd, DIST_TRACK_9_TO_10_CM);
  }
  else if (fromNode == 9 && toNode == 11) {
    blindDriveAndAlign(HEADING_9_TO_11, -1, stopAtEnd, DIST_TRACK_9_TO_11_CM, 2);
  }
  else if (fromNode == 10 && toNode == 11) {
    blindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd, DIST_TRACK_10_TO_11_CM);
  }
  else if (fromNode == 11 && toNode == 10) {
    blindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd, DIST_TRACK_10_TO_11_CM);
  }
  else if (fromNode == 10 && toNode == 9) {
    rotateToHeading(HEADING_10_TO_12);
    driveDistanceCm(DIST_10_TO_12_CM, SPEED_OPEN_TRACK_FWD, true);
    blindDriveAndAlign(HEADING_12_TO_9_2, -1, stopAtEnd, DIST_TRACK_12_TO_9_CM);
    if (!stopAtEnd) rotateToHeading(270);
  }
  else if (fromNode == 11 && toNode == 9) {
    rotateToHeading(HEADING_11_TO_12);
    driveDistanceCm(DIST_11_TO_12_CM, SPEED_OPEN_TRACK_FWD, true);
    blindDriveAndAlign(HEADING_12_TO_9_2, -1, stopAtEnd, DIST_TRACK_12_TO_9_CM);
    if (!stopAtEnd) rotateToHeading(270);
  }
  else {
    int dir = (toNode > fromNode) ? 90 : 270;
    rotateToHeading(dir);
    traceUntilIntersection(stopAtEnd);
  }
  intersectionNode = toNode;
}

void driveToIntersectionNode(int targetNode) {
  if (intersectionNode == targetNode) return;
  if (intersectionNode == 7 && targetNode >= 9) {
    stepTrackLegToLineEnd(DIST_TRACK_7_TO_9_CM, targetNode == 9);
    intersectionNode = 9;
    if (targetNode == 9) return;
  }
  while (intersectionNode != targetNode) {
    int nextNode = targetNode;
    if (intersectionNode < 9 && targetNode >= 9) nextNode = intersectionNode + 1;
    else if (intersectionNode > 9 && targetNode <= 9) nextNode = 9;
    else if (intersectionNode == 9 && targetNode == 11) nextNode = 11;
    else if (intersectionNode == 9 && targetNode == 10) nextNode = 10;
    else if (intersectionNode == 10 && targetNode == 11) nextNode = 11;
    else if (intersectionNode == 11 && targetNode == 10) nextNode = 10;
    else if (intersectionNode == 9 && targetNode == 8) nextNode = 8;
    else if (intersectionNode == 8 && targetNode == 7) nextNode = 7;
    else nextNode = (intersectionNode < targetNode) ? intersectionNode + 1 : intersectionNode - 1;
    stepBetweenNodes(intersectionNode, nextNode, nextNode == targetNode);
  }
}

static bool exitLineDetected(int zone, bool reverse,
    int fl, int fc, int fr, int rl, int rc, int rr, int& confirmCount) {
  if (zone == 2 || zone == 4) {
    return reverse ? (rl == 1 && rc == 1 && rr == 1)
                   : frontCrossFull(fl, fc, fr);
  }
  if (reverse) {
    if (rearOnLine(rl, rc, rr)) {
      if (++confirmCount >= EXIT_LINE_CONFIRM) return true;
    } else {
      confirmCount = 0;
    }
  } else {
    if (frontOnLine(fl, fc, fr)) {
      if (++confirmCount >= EXIT_LINE_CONFIRM) return true;
    } else {
      confirmCount = 0;
    }
  }
  return false;
}

void leaveZone(int zone) {
  int targetNode = zoneToIntersection(zone);
  ZoneMotionProfile profile = getZoneProfile(zone);
  if (targetNode == 7) enableTeeZoneSteering = true;

  lineTraceLastEdge = 0;
  resetLineTracePid();
  DriveEncMark motionStart = captureDriveEnc();

  DPRINTF("\n-Exit Z:"); DPRINT(zone);

  if (enteredZoneForward) {
    DPRINTF(" Rev");
    const bool crossZone = (zone == 2 || zone == 4);
    long extraSpan = crossZone ? toEncoderCounts(DIST_AXIS_TO_REAR_SENSOR_CM)
        : toEncoderCounts(profile.exitReverseExtra);
    if (extraSpan <= 0)
      extraSpan = toEncoderCounts(DIST_AXIS_TO_REAR_SENSOR_CM);
    long approachDecelSpan = zoneCrossApproachDecelSpan(zone, true);
    int openSpeed = SPEED_OPEN_ZONE_REV;

    bool lineDetected = false;
    DriveEncMark lineMark = {0, 0};
    DriveEncMark approachMark = {0, 0};
    bool approachDecel = false;
    int confirmCount = 0;

    while (true) {
      int fl, fc, fr, rl, rc, rr;
      readLineSensors(fl, fc, fr, rl, rc, rr);

      if (!lineDetected) {
        if (exitLineDetected(zone, true, fl, fc, fr, rl, rc, rr, confirmCount)) {
          lineDetected = true;
          lineMark = captureDriveEnc();
          DPRINTF(" L_ON");
          if (extraSpan <= 0) break;
        } else {
          if (crossZone && rearOnLine(rl, rc, rr) && !approachDecel) {
            approachDecel = true;
            approachMark = captureDriveEnc();
          }
          int speed = (crossZone && approachDecel)
              ? decelMarkSpeed(approachMark, approachDecelSpan, openSpeed)
              : rampMarkSpeed(motionStart, openSpeed);
          bool traceRev = false;
          if (rearOnLine(rl, rc, rr)) {
            if (zone == 5 || zone == 6) traceRev = true;
            else if (crossZone && !(rl && rc && rr)) traceRev = true;
          }
          if (traceRev) {
            traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
          } else {
            setWheelSpeeds(-speed, -speed);
          }
        }
      } else {
        int speed = decelMarkSpeed(lineMark, extraSpan, openSpeed);
        if (finishEncoderSpan(lineMark, extraSpan, speed)) break;
        if (zone == 2 || zone == 4) {
          setWheelSpeeds(-speed, -speed);
        } else {
          traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
        }
      }
      driveLoopTick();
    }
  } else {
    DPRINTF(" Fwd");
    const bool crossZone = (zone == 2 || zone == 4);
    long extraSpan = crossZone ? toEncoderCounts(DIST_AXIS_TO_FRONT_SENSOR_CM)
        : toEncoderCounts(profile.exitForwardExtra);
    if (extraSpan <= 0)
      extraSpan = toEncoderCounts(DIST_AXIS_TO_FRONT_SENSOR_CM);
    long approachDecelSpan = zoneCrossApproachDecelSpan(zone, false);
    int openSpeed = SPEED_OPEN_ZONE_FWD;

    bool lineDetected = false;
    DriveEncMark lineMark = {0, 0};
    DriveEncMark approachMark = {0, 0};
    bool approachDecel = false;
    int confirmCount = 0;

    while (true) {
      int fl, fc, fr, rl, rc, rr;
      readLineSensors(fl, fc, fr, rl, rc, rr);

      if (!lineDetected) {
        if (exitLineDetected(zone, false, fl, fc, fr, rl, rc, rr, confirmCount)) {
          lineDetected = true;
          lineMark = captureDriveEnc();
          DPRINTF(" L_ON");
          if (extraSpan <= 0) break;
        } else {
          if (crossZone && frontOnLine(fl, fc, fr) && !frontCrossFull(fl, fc, fr) && !approachDecel) {
            approachDecel = true;
            approachMark = captureDriveEnc();
          }
          int speed = (crossZone && approachDecel)
              ? decelMarkSpeed(approachMark, approachDecelSpan, openSpeed)
              : rampMarkSpeed(motionStart, openSpeed);
          if (crossZone && frontOnLine(fl, fc, fr) && !frontCrossFull(fl, fc, fr)) {
            traceLineForward(fl, fc, fr, rl, rc, rr, speed,
              LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
          } else {
            speed = smoothRampSpeed(speed);
            setWheelSpeeds(speed, speed);
          }
        }
      } else {
        int speed = decelMarkSpeed(lineMark, extraSpan, openSpeed);
        if (finishEncoderSpan(lineMark, extraSpan, speed)) break;
        if (zone == 2 || zone == 4 || zone == 5 || zone == 6) {
          speed = smoothRampSpeed(speed);
          setWheelSpeeds(speed, speed);
        } else {
          traceLineForward(fl, fc, fr, rl, rc, rr, speed,
              LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
        }
      }
      driveLoopTick();
    }
  }

  if (zone == 2 || zone == 4) setWheelSpeeds(0, 0);
  else stopMotors();
  endZoneScan();
  DPRINTF(" Done (실제 이동: ");
  DPRINT((float)encoderTraveledSince(motionStart) / COUNTS_PER_CM);
  DPRINTLNF(" cm)");

  if (targetNode == 7) enableTeeZoneSteering = false;
  intersectionNode = targetNode;
}

bool zonesAreVerticalOpposites(int zoneA, int zoneB) {
  return (zoneA == 1 && zoneB == 3) || (zoneA == 3 && zoneB == 1)
      || (zoneA == 2 && zoneB == 4) || (zoneA == 4 && zoneB == 2);
}

void moveBetweenOppositeZones(int fromZone, int toZone) {
  moveBetweenZones(fromZone, toZone, zoneMoveOpts(false, true));
}

void enterZoneAt(int zone) {
  int targetNode = zoneToIntersection(zone);
  int zoneHeading = (zone == 3 || zone == 4) ? 180 : 0;
  if (headingDeg != 0 && headingDeg != 180) rotateToHeading(zoneHeading);

  if (targetNode == 7) enableTeeZoneSteering = true;
  if (headingDeg == zoneHeading) {
    enterZoneForward(zone);
    enteredZoneForward = true;
  } else {
    enterZoneReverse(zone);
    enteredZoneForward = false;
  }
  if (targetNode == 7) enableTeeZoneSteering = false;
}

void moveBetweenZones(int fromZone, int toZone, ZoneMoveOptions opts) {
  if (fromZone == toZone) return;

  if (!opts.alreadyInFromZone) {
    navigateToZone(fromZone);
  }

  DPRINTF("\n[MOVE] ");
  DPRINT(fromZone);
  DPRINTF(" -> ");
  DPRINTLN(toZone);

  if (zonesAreVerticalOpposites(fromZone, toZone)) {
    bool teeSteer = (fromZone == 1 || fromZone == 3 || toZone == 1 || toZone == 3);
    if (teeSteer) enableTeeZoneSteering = true;
    crossToOppositeZone(toZone, fromZone, opts.scanQr);
    if (teeSteer) enableTeeZoneSteering = false;
    enteredZoneForward = false;
    intersectionNode = zoneToIntersection(toZone);
    return;
  }

  leaveZone(fromZone);

  int fromNode = zoneToIntersection(fromZone);
  int toNode = zoneToIntersection(toZone);
  if (opts.scanQr) endZoneScan();
  driveToIntersectionNode(toNode);
  if (opts.scanQr) beginZoneScan(toZone);
  enterZoneAt(toZone);
}

void navigateToZone(int zone) {
  int targetNode = zoneToIntersection(zone);
  int savedScanZone = activeScanZone;
  endZoneScan();
  driveToIntersectionNode(targetNode);
  if (savedScanZone) beginZoneScan(savedScanZone);
  enterZoneAt(zone);
}

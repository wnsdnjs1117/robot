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

static void blindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd) {
  rotateToHeading(targetHeading);
  driveOverLinesAndAlign(1, DIST_CROSS_ALIGN_CM, SPEED_OPEN_TRACK_FWD, stopAtEnd);
  if (alignHeading != -1 && !stopAtEnd) rotateToHeading(alignHeading);
}

static void stepBetweenNodes(int fromNode, int toNode, bool stopAtEnd) {
  if (fromNode == 8 && toNode == 9) {
    rotateToHeading(90);
    while (true) {
      int fl, fc, fr, rl, rc, rr;
      readFrontLineSensors(fl, fc, fr);
      readRearLineSensors(rl, rc, rr);
      if (!frontOnLine(fl, fc, fr)) {
        if (stopAtEnd) stopMotors();
        break;
      }
      traceLineForward(fl, fc, fr, rl, rc, rr, SPEED_LINE_FOLLOW_FWD);
      liftUpTick(); liftDownTick();
    }
  }
  else if (fromNode == 8 && toNode == 7) {
    rotateToHeading(270);
    resetLineTracePid();
    DriveEncMark motionStart = captureDriveEnc();
    long ignoreSpan = toEncoderCounts(DIST_IGNORE_NODE_CM);
    long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);
    intersectionArmed = true;
    intersectionHitCount = 0;
    bool crossFound = false;
    DriveEncMark crossMark = {0, 0};

    while (true) {
      int fl, fc, fr, rl, rc, rr;
      readFrontLineSensors(fl, fc, fr);
      readRearLineSensors(rl, rc, rr);

      if (!crossFound) {
        if (encoderTraveledSince(motionStart) < ignoreSpan) {
          int speed = rampMarkSpeed(motionStart, SPEED_LINE_FOLLOW_FWD);
          traceLineForward(fl, fc, fr, rl, rc, rr, speed);
        } else {
          bool isCross = (fl == 1 && fc == 1 && fr == 1);
          if (isCross) intersectionHitCount++;
          else { intersectionHitCount = 0; intersectionArmed = true; }
          if (intersectionArmed && intersectionHitCount >= CROSS_CONFIRM) {
            if (!stopAtEnd) break;
            crossFound = true;
            crossMark = captureDriveEnc();
            if (alignSpan <= 0) break;
          } else {
            traceLineForward(fl, fc, fr, rl, rc, rr, SPEED_LINE_FOLLOW_FWD);
            liftUpTick(); liftDownTick();
            continue;
          }
        }
        if (!crossFound) {
          liftUpTick(); liftDownTick();
          continue;
        }
      }

      int speed = crossAlignSpeed(crossMark, alignSpan);
      if (finishEncoderSpan(crossMark, alignSpan, speed)) break;
      traceLineForward(fl, fc, fr, rl, rc, rr, speed);
      liftUpTick(); liftDownTick();
    }
  }
  else if (fromNode == 9 && toNode == 8) {
    rotateToHeading(270);
    driveOverLinesAndAlign(1, 0, SPEED_OPEN_TRACK_FWD, false);
    traceUntilIntersection(stopAtEnd);
  }
  else if (fromNode == 9 && toNode == 10) {
    blindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd);
  }
  else if (fromNode == 9 && toNode == 11) {
    rotateToHeading(HEADING_9_TO_11);
    driveOverLinesAndAlign(2, DIST_CROSS_ALIGN_CM, SPEED_OPEN_TRACK_FWD, stopAtEnd);
  }
  else if (fromNode == 10 && toNode == 11) {
    blindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd);
  }
  else if (fromNode == 11 && toNode == 10) {
    blindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd);
  }
  else if (fromNode == 10 && toNode == 9) {
    rotateToHeading(HEADING_10_TO_12);
    driveDistanceCm(DIST_10_TO_12_CM, SPEED_OPEN_TRACK_FWD, true);
    rotateToHeading(HEADING_12_TO_9_2);
    driveOverLinesAndAlign(1, DIST_CROSS_ALIGN_CM, SPEED_OPEN_TRACK_FWD, stopAtEnd);
    if (!stopAtEnd) rotateToHeading(270);
  }
  else if (fromNode == 11 && toNode == 9) {
    rotateToHeading(HEADING_11_TO_12);
    driveDistanceCm(DIST_11_TO_12_CM, SPEED_OPEN_TRACK_FWD, true);
    rotateToHeading(HEADING_12_TO_9_2);
    driveOverLinesAndAlign(1, DIST_CROSS_ALIGN_CM, SPEED_OPEN_TRACK_FWD, stopAtEnd);
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
                   : (fl == 1 && fc == 1 && fr == 1);
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
    long extraSpan = (zone == 5 || zone == 6 || zone == 1 || zone == 3)
        ? toEncoderCounts(profile.exitReverseExtra)
        : toEncoderCounts(DIST_AXIS_TO_REAR_SENSOR_CM);
    int openSpeed = SPEED_OPEN_ZONE_REV;

    bool lineDetected = false;
    DriveEncMark lineMark = {0, 0};
    int confirmCount = 0;

    while (true) {
      int fl, fc, fr, rl, rc, rr;
      readFrontLineSensors(fl, fc, fr);
      readRearLineSensors(rl, rc, rr);

      if (!lineDetected) {
        if (exitLineDetected(zone, true, fl, fc, fr, rl, rc, rr, confirmCount)) {
          lineDetected = true;
          lineMark = captureDriveEnc();
          DPRINTF(" L_ON");
          if (extraSpan <= 0) break;
        } else {
          int speed = rampMarkSpeed(motionStart, openSpeed);
          bool traceRev = false;
          if (rearOnLine(rl, rc, rr)) {
            if (zone == 5 || zone == 6) traceRev = true;
            else if ((zone == 2 || zone == 4) && !(rl && rc && rr)) traceRev = true;
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
      liftUpTick(); liftDownTick(); pollZoneScan();
    }
  } else {
    DPRINTF(" Fwd");
    long extraSpan = (zone == 5 || zone == 6 || zone == 1 || zone == 3)
        ? toEncoderCounts(profile.exitForwardExtra)
        : toEncoderCounts(DIST_AXIS_TO_FRONT_SENSOR_CM);
    int openSpeed = SPEED_OPEN_ZONE_FWD;

    bool lineDetected = false;
    DriveEncMark lineMark = {0, 0};
    int confirmCount = 0;

    while (true) {
      int fl, fc, fr, rl, rc, rr;
      readFrontLineSensors(fl, fc, fr);
      readRearLineSensors(rl, rc, rr);

      if (!lineDetected) {
        if (exitLineDetected(zone, false, fl, fc, fr, rl, rc, rr, confirmCount)) {
          lineDetected = true;
          lineMark = captureDriveEnc();
          DPRINTF(" L_ON");
          if (extraSpan <= 0) break;
        } else {
          if ((zone == 2 || zone == 4) && frontOnLine(fl, fc, fr) && !(fl && fc && fr)) {
            int speed = rampMarkSpeed(motionStart, openSpeed);
            traceLineForward(fl, fc, fr, rl, rc, rr, speed);
          } else {
            int speed = rampMarkSpeed(motionStart, openSpeed);
            setWheelSpeeds(speed, speed);
          }
        }
      } else {
        int speed = decelMarkSpeed(lineMark, extraSpan, openSpeed);
        if (finishEncoderSpan(lineMark, extraSpan, speed)) break;
        if (zone == 2 || zone == 4 || zone == 5 || zone == 6) {
          setWheelSpeeds(speed, speed);
        } else {
          traceLineForward(fl, fc, fr, rl, rc, rr, speed);
        }
      }
      liftUpTick(); liftDownTick(); pollZoneScan();
    }
  }

  stopMotors();
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
  if (fromNode == 8 && toNode == 7 && headingDeg == 0) {
    rotateToHeading(270);
    alignOnTrackHeading(SPEED_OPEN_TRACK_FWD, DIST_CROSS_ALIGN_CM);
    intersectionNode = 8;
  }

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

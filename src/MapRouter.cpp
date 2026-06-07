/* ============================================================
 * MapRouter.cpp - 교차로 노드 간 경로 및 구역 탈출
 * ============================================================ */
#include "MapRouter.h"
#include "Config.h"
#include "Lift.h"
#include "Motion.h"
#include "Navigation.h"
#include "BoxMap.h"

float headingDeg = 0.0f;
int   intersectionNode = 11;
bool  enteredZoneForward = true;
bool  finishFromZone6Exit = false;
static int g_seamlessTo = 0;  // 1↔3 배송: leaveZone 직후 무정지 진입 대상 존

static int zoneToIntersection(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5) return 10;
  if (zone == 6) return 11;
  return 8;
}

static float normDeg(float d) {
  while (d >= 360.0f) d -= 360.0f;
  while (d < 0.0f) d += 360.0f;
  return d;
}

void rotateToHeading(float targetDeg) {
  targetDeg = normDeg(targetDeg);

  float diff = targetDeg - headingDeg;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  
  if (diff > -0.01f && diff < 0.01f) return;
  if (diff > 0.0f) rotateByDegrees(diff, true);
  else             rotateByDegrees(-diff, false);
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

static bool registerNodeLineHit(int lineCount, bool anyFrontLine, int& linesPassed,
    int& lineConfirmCount, bool& ignoreAfterLine, DriveEncMark& ignoreMark,
    DriveEncMark& lineMark) {
  if (!(anyFrontLine || (++lineConfirmCount >= EXIT_LINE_CONFIRM))) return false;
  linesPassed++;
  lineConfirmCount = 0;
  if (linesPassed >= lineCount) {
    lineMark = captureDriveEnc();
    return true;
  }
  ignoreAfterLine = true;
  ignoreMark = captureDriveEnc();
  return false;
}

static void blindDriveAndAlign(float targetHeading, float alignHeading, bool stopAtEnd,
    float legSpanCm, int lineCount = 1, bool anyFrontLine = false) {
  rotateToHeading(targetHeading);
  resetLineTracePid();
  resetRampSpeedLimiter(RAMP_MIN_SPEED);
  const int cruiseSpeed = SPEED_OPEN_TRACK_FWD;
  DriveEncMark motionStart = captureDriveEnc();

  long ignoreSpan = toEncoderCounts(DIST_IGNORE_NODE_CM);
  float detectTargetCm = legSpanCm - DIST_NODE_DETECT_CRAWL_CM;
  if (detectTargetCm < DIST_IGNORE_NODE_CM) detectTargetCm = legSpanCm;
  long approachStart = trackLegApproachStartCounts(detectTargetCm, cruiseSpeed);
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);

  bool approachDecel = false;
  DriveEncMark approachMark = {0, 0};
  int linesPassed = 0;
  bool lineFound = false;

  bool initialIgnore = true;
  bool ignoreAfterLine = false;
  DriveEncMark ignoreMark = {0, 0};
  DriveEncMark lineMark = {0, 0};
  int lastCurSpeed = RAMP_MIN_SPEED;
  int lineConfirmCount = 0;

  while (true) {
    int fl, fc, fr;
    readFrontLineSensors(fl, fc, fr);

    if (!lineFound) {
      if (initialIgnore) {
        if (encoderTraveledSince(motionStart) >= ignoreSpan)
          initialIgnore = false;
      } else if (ignoreAfterLine) {
        if (encoderTraveledSince(ignoreMark) >= ignoreSpan)
          ignoreAfterLine = false;
      } else if (frontOnLine(fl, fc, fr)) {
        if (registerNodeLineHit(lineCount, anyFrontLine, linesPassed, lineConfirmCount,
                ignoreAfterLine, ignoreMark, lineMark)) {
          lineFound = true;
          if (alignSpan <= 0) {
            if (stopAtEnd) stopMotors();
            break;
          }
        }
      } else {
        lineConfirmCount = 0;
      }

      if (!lineFound) {
        int speed = trackLegSpeed(motionStart, ignoreSpan, approachStart, approachDecel,
            approachMark, cruiseSpeed);
        speed = smoothRampSpeed(speed);
        lastCurSpeed = speed;
        if (!initialIgnore && !ignoreAfterLine && frontOnLine(fl, fc, fr))
          traceLineForward(fl, fc, fr, 0, 0, 0, speed);
        else
          setWheelSpeeds(speed, speed);
        driveLoopTick();

        if (!initialIgnore && !ignoreAfterLine) {
          readFrontLineSensors(fl, fc, fr);
          if (frontOnLine(fl, fc, fr)
              && registerNodeLineHit(lineCount, anyFrontLine, linesPassed,
                     lineConfirmCount, ignoreAfterLine, ignoreMark, lineMark)) {
            lineFound = true;
            if (alignSpan <= 0) {
              if (stopAtEnd) stopMotors();
              break;
            }
          }
        }
        if (!lineFound) continue;
      }
    }

    int speed = crossAlignSpeed(lineMark, alignSpan, lastCurSpeed);
    speed = smoothRampSpeed(speed);
    if (stopAtEnd && finishAlignSpan(lineMark, alignSpan, speed)) break;
    if (!stopAtEnd && encoderTraveledSince(lineMark) >= alignSpan) break;
    setWheelSpeeds(speed, speed);
    driveLoopTick();
  }

  if (alignHeading >= 0.0f && !stopAtEnd) rotateToHeading(alignHeading);
}

void driveTrackLegBlind(float targetHeading, float alignHeading, bool stopAtEnd,
    float legSpanCm, int lineCount, bool anyFrontLine) {
  blindDriveAndAlign(targetHeading, alignHeading, stopAtEnd, legSpanCm, lineCount,
      anyFrontLine);
}

static void stepTrackLeg78(float heading, bool stopAtEnd) {
  rotateToHeading(heading);
  resetLineTracePid();
  clearIntersectionCross(); 
  DriveEncMark motionStart = captureDriveEnc();
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);
  long approachStart = trackNodeApproachStartCounts(SPEED_TRACK_7_9_LINE);
  
  bool crossFound = false;
  bool approachDecel = false;
  DriveEncMark crossMark = {0, 0};
  DriveEncMark approachMark = {0, 0};
  
  long ignoreSpan = toEncoderCounts(DIST_IGNORE_NODE_CM);
  int lastCurSpeed = RAMP_MIN_SPEED;

  intersectionArmed = true;
  intersectionHitCount = 0;

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);

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
          speed = smoothRampSpeed(speed);
          lastCurSpeed = speed;
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

    int speed = crossAlignSpeed(crossMark, alignSpan, lastCurSpeed);
    speed = smoothRampSpeed(speed);
    if (finishAlignSpan(crossMark, alignSpan, speed)) break;
    traceLineForward(fl, fc, fr, rl, rc, rr, speed,
        LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
    driveLoopTick();
  }
}

static void stepTrackLegToLineEnd(float legSpanCm, bool stopAtEnd) {
  rotateToHeading(90.0f);
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
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);
    if (frontOnLine(fl, fc, fr)) lineSeen = true;

    long traveled = encoderTraveledSince(motionStart);
    int speed = trackLegSpeed(motionStart, ignoreSpan, approachStart, approachDecel,
        approachMark, SPEED_TRACK_7_9_LINE);

    if (longLeg && traveled >= node8Center - node8PassHalf
        && traveled <= node8Center + node8PassHalf
        && frontCrossFull(fl, fc, fr)) {
      resetRampSpeedLimiter(speed);
      speed = smoothRampSpeed(speed);
      traceLineForward(fl, fc, fr, rl, rc, rr, speed,
          LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
      driveLoopTick();
      continue;
    }

    if (!frontOnLine(fl, fc, fr)) {
      if (lineSeen && traveled >= approachStart) {
        if (stopAtEnd) stopMotors();
        break;
      }
    }
    resetRampSpeedLimiter(speed);
    speed = smoothRampSpeed(speed);
    traceLineForward(fl, fc, fr, rl, rc, rr, speed,
        LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
    driveLoopTick();
  }
}

static void stepBetweenNodes(int fromNode, int toNode, bool stopAtEnd) {
  if (fromNode == 8 && toNode == 9) {
    stepTrackLegToLineEnd(DIST_TRACK_NODE_SPAN_CM, stopAtEnd);
  }
  else if (fromNode == 7 && toNode == 8) {
    stepTrackLeg78(90.0f, stopAtEnd);
  }
  else if (fromNode == 7 && toNode == 9) {
    stepTrackLegToLineEnd(DIST_TRACK_7_TO_9_CM, stopAtEnd);
  }
  else if (fromNode == 8 && toNode == 7) {
    stepTrackLeg78(270.0f, stopAtEnd);
  }
  else if (fromNode == 9 && toNode == 8) {
    rotateToHeading(270.0f);
    DriveEncMark legStart = captureDriveEnc();
    driveOverLinesAndAlign(1, 0.0f, SPEED_9_TO_8, false);
    setWheelSpeeds(0, 0);
    delayWithTicks(40);
    // 9->8 은 약 35cm 로 8->9(70cm)보다 짧다. legStart 기준 누적 거리로
    // 노드 8 도달 전에 미리 감속한다(cruise 속도는 SPEED_9_TO_8 유지).
    traceUntilIntersection(stopAtEnd, SPEED_9_TO_8, legStart, DIST_9_TO_8_CM);
  }
  else if (fromNode == 9 && toNode == 10) {
    blindDriveAndAlign(HEADING_9_TO_10, 90.0f, stopAtEnd, DIST_TRACK_9_TO_10_CM, 1, true);
  }
  else if (fromNode == 9 && toNode == 11) {
    blindDriveAndAlign(HEADING_9_TO_11, -1.0f, stopAtEnd, DIST_TRACK_9_TO_11_CM, 2, true);
  }
  else if (fromNode == 10 && toNode == 11) {
    blindDriveAndAlign(HEADING_10_TO_11, -1.0f, stopAtEnd, DIST_TRACK_10_TO_11_CM, 1, true);
  }
  else if (fromNode == 11 && toNode == 10) {
    blindDriveAndAlign(HEADING_11_TO_10, -1.0f, stopAtEnd, DIST_TRACK_10_TO_11_CM, 1, true);
  }
  else if (fromNode == 10 && toNode == 9) {
    rotateToHeading(HEADING_10_TO_12);
    driveDistanceCm(DIST_10_TO_12_CM, SPEED_OPEN_TRACK_FWD, true);
    blindDriveAndAlign(HEADING_12_TO_9_2, -1.0f, stopAtEnd, DIST_TRACK_12_TO_9_CM, 1, true);
    if (!stopAtEnd) rotateToHeading(270.0f);
  }
  else if (fromNode == 11 && toNode == 9) {
    rotateToHeading(HEADING_11_TO_12);
    driveDistanceCm(DIST_11_TO_12_CM, SPEED_OPEN_TRACK_FWD, true);
    blindDriveAndAlign(HEADING_12_TO_9_2, -1.0f, stopAtEnd, DIST_TRACK_12_TO_9_CM, 1, true);
    if (!stopAtEnd) rotateToHeading(270.0f);
  }
  else {
    float dir = (toNode > fromNode) ? 90.0f : 270.0f;
    rotateToHeading(dir);
    traceUntilIntersection(stopAtEnd);
  }
  intersectionNode = toNode;
}

static int nextIntersectionNode(int cur, int target) {
  if (cur < 9 && target >= 9) return cur + 1;
  if (cur > 9 && target <= 9) return 9;
  if (cur == 9 && target == 11) return 11;
  if (cur == 9 && target == 10) return 10;
  if (cur == 10 && target == 11) return 11;
  if (cur == 11 && target == 10) return 10;
  if (cur == 9 && target == 8) return 8;
  if (cur == 8 && target == 7) return 7;
  return (cur < target) ? cur + 1 : cur - 1;
}

void driveToIntersectionNode(int targetNode) {
  if (intersectionNode == targetNode) return;
  finishFromZone6Exit = false;
  if (intersectionNode == 7 && targetNode >= 9) {
    stepTrackLegToLineEnd(DIST_TRACK_7_TO_9_CM, targetNode == 9);
    intersectionNode = 9;
    if (targetNode == 9) return;
  }
  while (intersectionNode != targetNode) {
    int next = nextIntersectionNode(intersectionNode, targetNode);
    stepBetweenNodes(intersectionNode, next, next == targetNode);
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
  int lastExitSp = RAMP_MIN_SPEED;

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
    int lastCurSpeed = RAMP_MIN_SPEED;

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
          
          speed = smoothRampSpeed(speed);
          lastCurSpeed = speed;
          lastExitSp = speed;

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
        int speed = decelMarkSpeed(lineMark, extraSpan, lastCurSpeed);
        speed = smoothRampSpeed(speed);
        lastExitSp = speed;
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
          
          speed = smoothRampSpeed(speed);
          lastExitSp = speed;

          if (crossZone && frontOnLine(fl, fc, fr) && !frontCrossFull(fl, fc, fr)) {
            traceLineForward(fl, fc, fr, rl, rc, rr, speed,
              LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
          } else {
            setWheelSpeeds(speed, speed);
          }
        }
      } else {
        int up = rampMarkSpeed(motionStart, openSpeed);
        int down = decelMarkSpeed(lineMark, extraSpan, openSpeed);
        int speed = (up < down) ? up : down;
        speed = smoothRampSpeed(speed);
        lastExitSp = speed;
        if (finishEncoderSpan(lineMark, extraSpan, speed)) break;
        if (zone == 2 || zone == 4 || zone == 5 || zone == 6) {
          setWheelSpeeds(speed, speed);
        } else {
          traceLineForward(fl, fc, fr, rl, rc, rr, speed,
              LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
        }
      }
      driveLoopTick();
    }
  }

  if (g_seamlessTo && (zone == 1 || zone == 3)) {
    ZoneMotionProfile tp = getZoneProfile(g_seamlessTo);
    bool eFwd = (g_seamlessTo == 1);
    int dir = eFwd ? 1 : -1;
    int sp = eFwd ? SPEED_OPEN_ZONE_FWD : SPEED_OPEN_ZONE_REV;
    long entSp = toEncoderCounts(eFwd ? tp.entryForwardExtra : tp.entryReverseExtra);
    DriveEncMark pm = {0, 0};
    bool sawL, past = false;
    int fl, fc, fr, rl, rc, rr;
    readLineSensors(fl, fc, fr, rl, rc, rr);
    sawL = eFwd ? frontOnLine(fl, fc, fr) : rearOnLine(rl, rc, rr);
    while (true) {
      readLineSensors(fl, fc, fr, rl, rc, rr);
      bool on = eFwd ? frontOnLine(fl, fc, fr) : rearOnLine(rl, rc, rr);
      if (!past) {
        if (on) {
          sawL = true;
          lastExitSp = smoothRampSpeed(rampMarkSpeed(motionStart, sp));
          if (eFwd) traceLineForward(fl, fc, fr, rl, rc, rr, lastExitSp);
          else traceLineReverse(rl, rc, rr, fl, fc, fr, lastExitSp);
        } else if (sawL) {
          pm = captureDriveEnc();
          if (entSp <= 0) break;
          past = true;
        } else {
          lastExitSp = smoothRampSpeed(rampMarkSpeed(motionStart, sp));
          setWheelSpeeds(dir * lastExitSp, dir * lastExitSp);
        }
      } else {
        lastExitSp = smoothRampSpeed(decelMarkSpeed(pm, entSp, lastExitSp));
        if (finishEncoderSpan(pm, entSp, lastExitSp)) break;
        setWheelSpeeds(dir * lastExitSp, dir * lastExitSp);
      }
      driveLoopTick();
    }
    endZoneScan();
    enableTeeZoneSteering = false;
    intersectionNode = 7;
    enteredZoneForward = eFwd;
    finishFromZone6Exit = false;
    stopMotors();
    return;
  }

  if (zone == 2 || zone == 4) setWheelSpeeds(0, 0);
  else stopMotors();
  endZoneScan();
  DPRINTF(" Done (실제 이동: ");
  DPRINT((float)encoderTraveledSince(motionStart) / COUNTS_PER_CM);
  DPRINTLNF(" cm)");

  if (targetNode == 7) enableTeeZoneSteering = false;
  intersectionNode = targetNode;
  finishFromZone6Exit = (zone == 6);
}

void enterZoneAt(int zone) {
  finishFromZone6Exit = false;
  int targetNode = zoneToIntersection(zone);
  float zoneHeading = (zone == 3 || zone == 4) ? 180.0f : 0.0f;
  
  float modH = normDeg(headingDeg);
  bool isNorth = (modH < 0.1f || modH > 359.9f);
  bool isSouth = (modH > 179.9f && modH < 180.1f);
  if (!isNorth && !isSouth) rotateToHeading(zoneHeading);

  if (targetNode == 7) enableTeeZoneSteering = true;
  
  modH = normDeg(headingDeg);
  bool isSame = (modH > zoneHeading - 0.1f && modH < zoneHeading + 0.1f) ||
                (modH > zoneHeading + 359.9f && modH < zoneHeading + 360.1f);
                
  if (isSame) {
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

  if (!opts.scanQr && ((fromZone == 1 && toZone == 3) || (fromZone == 3 && toZone == 1))) {
    rotateToHeading(0.0f);
    enteredZoneForward = (fromZone == 1);
    g_seamlessTo = toZone;
    leaveZone(fromZone);
    g_seamlessTo = 0;
    return;
  }

  if (opts.scanQr
      && ((fromZone == 1 && toZone == 3) || (fromZone == 3 && toZone == 1)
       || (fromZone == 2 && toZone == 4) || (fromZone == 4 && toZone == 2))) {
    bool teeSteer = (fromZone == 1 || fromZone == 3 || toZone == 1 || toZone == 3);
    if (teeSteer) enableTeeZoneSteering = true;
    crossToOppositeZone(toZone, fromZone, opts.scanQr);
    if (teeSteer) enableTeeZoneSteering = false;
    enteredZoneForward = false;
    intersectionNode = zoneToIntersection(toZone);
    return;
  }

  leaveZone(fromZone);

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
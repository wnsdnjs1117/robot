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

// 24cm(HIGH)로 든 박스를 장애물 노드(8/9) 통과 후 이동 중 14cm로 미리 내릴지 여부.
// deliverBoxBetweenZones 가 HIGH 운반일 때만 arm 하고, 장애물 통과 지점에서 1회 트리거된다.
bool g_carryPreLowerArmed = false;

// 장애물 노드를 지난 지점에서 호출: armed 면 'afterCm 더 주행한 뒤' 14cm로 내리도록 예약.
// 리프트가 차체 중앙에 있어 센서가 노드를 감지한 순간엔 박스가 아직 노드를 못 지났으므로,
// afterCm 여유(기본 20cm)를 둬서 박스가 장애물을 확실히 지난 뒤 하강하게 한다.
static void scheduleCarryPreLowerIfArmed(float afterCm) {
  if (!g_carryPreLowerArmed) return;
  g_carryPreLowerArmed = false;
  scheduleCarryPreLower(afterCm);
}

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
    float legSpanCm, int lineCount = 1, bool anyFrontLine = false,
    float preLowerCm = -1.0f) {  // >=0 이면 leg 시작 후 그 거리에서 운반 박스 14cm 미리 내리기
  rotateToHeading(targetHeading);
  resetLineTracePid();
  resetRampSpeedLimiter(RAMP_MIN_SPEED);
  const int cruiseSpeed = SPEED_OPEN_TRACK_FWD;
  DriveEncMark motionStart = captureDriveEnc();

  // 9->10/11 처럼 노드 9를 떠난 직후(회전 후) 기준으로 preLowerCm 주행 뒤 운반 박스 14cm 내리기 예약.
  if (preLowerCm >= 0.0f) scheduleCarryPreLowerIfArmed(preLowerCm);

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

// 메인 가로 트랙의 빠른 라인트레이싱(앞+뒤 센서 + LINE_KP_TRACK_7_9 게인)으로
// heading 방향으로 달려 다음 십자선(앞 1.1.1)까지 간다. 7-8, 8-7 뿐 아니라
// 9->8 도 이걸로 통일해서 사용한다.
//  - approachDistCm: 십자선 도달 전 미리 감속을 시작할 기준 거리
//  - cruiseSpeed: 순항 속도
static void stepMainTrackToCross(float heading, bool stopAtEnd,
    float approachDistCm = DIST_TRACK_NODE_SPAN_CM,
    int cruiseSpeed = SPEED_TRACK_7_9_LINE,
    float preLowerCm = -1.0f) {  // >=0(8->7)이면 회전 후 그 거리 주행 뒤 운반 박스 14cm 내리기
  rotateToHeading(heading);
  resetLineTracePid();
  clearIntersectionCross();
  DriveEncMark motionStart = captureDriveEnc();
  if (preLowerCm >= 0.0f) scheduleCarryPreLowerIfArmed(preLowerCm);
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);
  long approachStart = trackLegApproachStartCounts(approachDistCm, cruiseSpeed);

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
              approachMark, cruiseSpeed);
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

// 메인 가로 트랙을 빠른 라인트레이싱으로 달리며, 중간 십자선 passCount 개를
// "감속 없이 그대로 통과"한 뒤 마지막 목표 십자선에서 정지/정렬한다.
// (예: 9->7 은 노드 8 을 통과(passCount=1)하고 노드 7 에서 정지 — 7->9 처럼 연속 주행)
// 마지막 구간 감속은 직전 통과 십자선 기준 finalApproachDistCm 로 계산하므로
// 시작점(노드 9)이 매번 달라도 안정적이다.
static void stepMainTrackMultiCross(float heading, bool stopAtEnd,
    int passCount, float finalApproachDistCm, int cruiseSpeed) {
  rotateToHeading(heading);
  resetLineTracePid();
  clearIntersectionCross();

  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);
  long ignoreSpan = toEncoderCounts(DIST_IGNORE_NODE_CM);
  long finalApproachStart = trackLegApproachStartCounts(finalApproachDistCm, cruiseSpeed);
  long decelSpan = rampDecelSpanCounts(cruiseSpeed);

  DriveEncMark segStart = captureDriveEnc();   // 현재 구간 시작(시작점 또는 직전 통과 십자선)
  int crossesLeft = passCount;                 // 더 통과해야 할 중간 십자선 수
  bool approachDecel = false;
  DriveEncMark approachMark = {0, 0};
  bool crossFound = false;                      // 최종 십자선 도달
  DriveEncMark crossMark = {0, 0};
  int lastCurSpeed = RAMP_MIN_SPEED;

  intersectionArmed = true;
  intersectionHitCount = 0;

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readFrontLineSensors(fl, fc, fr);
    readRearLineSensors(rl, rc, rr);

    if (!crossFound) {
      // 구간 시작 직후 ignoreSpan 동안은 십자선 감지를 끔(직전 십자선/노드 9 stub 회피).
      if (encoderTraveledSince(segStart) >= ignoreSpan) {
        bool isCross = frontCrossFull(fl, fc, fr);
        if (isCross) intersectionHitCount++;
        else { intersectionHitCount = 0; intersectionArmed = true; }
        if (intersectionArmed && intersectionHitCount >= CROSS_CONFIRM) {
          if (crossesLeft > 0) {
            // 중간 십자선(예: 노드 8): 감속 없이 통과, 다음 구간 시작
            crossesLeft--;
            // 9->7 에서 노드 8 감지 → 20cm 더 가서(박스가 노드 8을 확실히 지난 뒤) 14cm 내리기 예약.
            scheduleCarryPreLowerIfArmed(20.0f);
            segStart = captureDriveEnc();
            approachDecel = false;
            intersectionArmed = false;
            intersectionHitCount = 0;
          } else {
            if (!stopAtEnd) break;
            crossFound = true;
            crossMark = captureDriveEnc();
            if (alignSpan <= 0) break;
          }
        }
      }

      if (!crossFound) {
        // 마지막 구간이고 감속 시작점을 지났으면 미리 감속, 그 외엔 순항.
        int target;
        if (crossesLeft > 0 || encoderTraveledSince(segStart) < finalApproachStart) {
          target = cruiseSpeed;
        } else {
          if (!approachDecel) { approachDecel = true; approachMark = captureDriveEnc(); }
          target = decelMarkSpeed(approachMark, decelSpan, cruiseSpeed);
        }
        int speed = smoothRampSpeed(target);
        lastCurSpeed = speed;
        traceLineForward(fl, fc, fr, rl, rc, rr, speed,
            LINE_KP_TRACK_7_9_SOFT, LINE_KP_TRACK_7_9_HARD);
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
    stepTrackLegToLineEnd(DIST_TRACK_8_TO_9_CM, stopAtEnd);
  }
  else if (fromNode == 7 && toNode == 8) {
    stepMainTrackToCross(90.0f, stopAtEnd);
  }
  else if (fromNode == 7 && toNode == 9) {
    stepTrackLegToLineEnd(DIST_TRACK_7_TO_9_CM, stopAtEnd);
  }
  else if (fromNode == 9 && toNode == 7) {
    // 7->9 처럼 연속 주행: 노드 8 을 감속·정지 없이 통과하고 노드 7 에서 정지.
    stepMainTrackMultiCross(270.0f, stopAtEnd, 1, DIST_TRACK_NODE_SPAN_CM,
        SPEED_TRACK_7_9_LINE);
  }
  else if (fromNode == 8 && toNode == 7) {
    // 노드 8을 떠나 20cm 주행 뒤(박스가 노드 8을 지난 뒤) 운반 박스 14cm로 미리 내리기.
    stepMainTrackToCross(270.0f, stopAtEnd, DIST_TRACK_NODE_SPAN_CM, SPEED_TRACK_7_9_LINE, 20.0f);
  }
  else if (fromNode == 9 && toNode == 8) {
    // 9->8 도 7-8/8-7 과 같은 빠른 라인트레이싱으로 통일.
    // 기존 SPEED_9_TO_8 상수를 삭제하고 SPEED_TRACK_7_9_LINE 으로 변경
    stepMainTrackToCross(270.0f, stopAtEnd, DIST_9_TO_8_CM, SPEED_TRACK_7_9_LINE);
  }
  else if (fromNode == 9 && toNode == 10) {
    // 노드 9를 떠나 20cm 지난 시점부터 운반 박스 14cm로 미리 내리기.
    blindDriveAndAlign(HEADING_9_TO_10, 90.0f, stopAtEnd, DIST_TRACK_9_TO_10_CM, 1, true, 20.0f);
  }
  else if (fromNode == 9 && toNode == 11) {
    blindDriveAndAlign(HEADING_9_TO_11, -1.0f, stopAtEnd, DIST_TRACK_9_TO_11_CM, 2, true, 20.0f);
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
  if (cur == 9 && target == 7) return 7;   // 9->7 은 한 다리로 연속(노드 8 통과)
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

  if (enteredZoneForward) {
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

  // 1↔3 배송은 존 안에서 회전한 뒤 곧장 건너가는 무정지(seamless) 지름길 대신,
  // 아래 일반 경로로 처리한다. 일반 경로는 leaveZone 으로 7번까지 '먼저 빠져나온 뒤'
  // 교차로에서 회전하므로, 박스를 들고 3번 존 안에서 제자리 회전(처박힘/충돌)하던
  // 문제가 사라진다.
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
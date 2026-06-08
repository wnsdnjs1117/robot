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

void runZoneEntry(bool reverse, int zone, int openSpeed) {
  ZoneMotionProfile profile = getZoneProfile(zone);
  float extraDistance = reverse ? profile.entryReverseExtra : profile.entryForwardExtra;
  lineTraceLastEdge = 0;
  resetLineTracePid();
  DriveEncMark motionStart = captureDriveEnc();
  long extraSpan = toEncoderCounts(extraDistance);

  bool sawLine = false;
  bool sawCenter = false;
  bool pastLine = false;
  DriveEncMark lineStartMark = {0, 0};
  DriveEncMark pastLineMark = {0, 0};

  // 완벽한 일자(0 1 0)를 한 번 잡으면 그 지점부터 가속을 시작(자세 래칭).
  bool perfectAligned = false;
  long accelStartTraveled = 0;

  // 라인 끝단에서 한쪽 에지만 남아 홱 틀어지는 것 방지: 가운데를 잃은 뒤 0.1cm만 직진.
  DriveEncMark centerLostMark = {0, 0};
  bool centerLostArmed = false;

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readLineSensors(fl, fc, fr, rl, rc, rr);
    bool onLine = reverse ? rearOnLine(rl, rc, rr) : frontOnLine(fl, fc, fr);
    long traveled = encoderTraveledSince(motionStart);

    if (!pastLine) {
      if (onLine) {
        if (!sawLine) {
          sawLine = true;
          lineStartMark = captureDriveEnc();
        }

        bool centerOn = reverse ? (rc != 0) : (fc != 0);
        if (centerOn) {
          sawCenter = true;
          centerLostArmed = false;   // 가운데가 다시 켜지면 끝단 직진 락 해제
        }

        // 완벽한 일자(0 1 0)를 처음 감지하면 래칭하고 그 지점을 가속 기준점으로 삼는다.
        bool isPerfectStraight = reverse ? (!rl && rc && !rr) : (!fl && fc && !fr);
        if (isPerfectStraight && !perfectAligned) {
          perfectAligned = true;
          accelStartTraveled = traveled;
        }

        // 속도: 일자 잡히기 전엔 저속으로 끈질기게 교정, 잡힌 뒤엔 목표 속도로 가속.
        int speed;
        if (!perfectAligned) {
          speed = RAMP_MIN_SPEED;
        } else {
          speed = calcRampUpSpeed(traveled - accelStartTraveled,
              rampAccelSpanCounts(openSpeed), openSpeed);
        }
        speed = smoothRampSpeed(speed);

        int dir = reverse ? -1 : 1;

        // 조향: 일자든 아니든 끝까지 라인트레이싱(일자면 오차가 작아 소프트 조향만 걸림).
        // 단, 가운데를 잃은 직후 0.1cm만 라인 끝단으로 보고 직진해 끝단 틀어짐만 막는다.
        if (sawCenter && !centerOn) {
          if (!centerLostArmed) {
            centerLostArmed = true;
            centerLostMark = captureDriveEnc();
          }
          if (encoderTraveledSince(centerLostMark) < toEncoderCounts(0.1f)) {
            setWheelSpeeds(dir * speed, dir * speed);
          } else if (reverse) {
            traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
          } else {
            traceLineForward(fl, fc, fr, rl, rc, rr, speed);
          }
        } else if (reverse) {
          traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
        } else {
          traceLineForward(fl, fc, fr, rl, rc, rr, speed);
        }

      } else if (sawLine) {
        pastLine = true;
        pastLineMark = captureDriveEnc();
        // 끝까지 일자를 못 잡고 저속으로 왔다면 블라인드 진입 순간부터 가속 시작.
        if (!perfectAligned) accelStartTraveled = traveled;
        if (extraSpan <= 0) break;
      } else {
        // 회전 직후 라인을 밟기 전: 저속 진입.
        int speed = smoothRampSpeed(RAMP_MIN_SPEED);
        int dir = reverse ? -1 : 1;
        setWheelSpeeds(dir * speed, dir * speed);
      }
    } else {
      // 존 내부 블라인드 추가 거리: 가속 ↔ 도착 감속 중 작은 값으로 부드럽게.
      long totalAccelTraveled = encoderTraveledSince(motionStart) - accelStartTraveled;
      int upSpeed = calcRampUpSpeed(totalAccelTraveled, rampAccelSpanCounts(openSpeed), openSpeed);
      int downSpeed = decelMarkSpeed(pastLineMark, extraSpan, openSpeed);
      int speed = smoothRampSpeed((upSpeed < downSpeed) ? upSpeed : downSpeed);
      if (finishEncoderSpan(pastLineMark, extraSpan, speed)) break;
      int dir = reverse ? -1 : 1;
      setWheelSpeeds(dir * speed, dir * speed);
    }
    driveLoopTick();
  }

  stopMotors();
}

void runFinishApproachFrom11() {
  rotateToHeading(HEADING_11_TO_FINISH);
  resetLineTracePid();
  resetRampSpeedLimiter(RAMP_MIN_SPEED);
  const int cruiseSpeed = SPEED_OPEN_TRACK_REV;
  DriveEncMark motionStart = captureDriveEnc();
  long accelSpan = rampAccelSpanCounts(cruiseSpeed);
  long blindSpan = toEncoderCounts(DIST_FINISH_BLIND_CONFIRM_CM);

  bool seenBlind = false;
  bool blindArmed = false;
  DriveEncMark blindMark = {0, 0};

  // 11번 세로선 위(블라인드 전)에서는 후진 라인트레이싱으로 정렬한다.
  // 가운데(rc)를 잃으면 0.1cm만 직진해 끝단 하드조향 틀어짐을 막은 뒤 다시 추종.
  bool sawCenter = false;
  bool centerLostArmed = false;
  DriveEncMark centerLostMark = {0, 0};

  while (true) {
    int rl, rc, rr;
    readRearLineSensors(rl, rc, rr);
    bool onLine = rearOnLine(rl, rc, rr);
    long traveled = encoderTraveledSince(motionStart);
    if (rc != 0) { sawCenter = true; centerLostArmed = false; }

    if (!seenBlind) {
      if (!onLine) {
        if (!blindArmed) {
          blindArmed = true;
          blindMark = captureDriveEnc();
        } else if (encoderTraveledSince(blindMark) >= blindSpan)
          seenBlind = true;
      } else {
        blindArmed = false;
      }
    } else if (onLine) {
      break;
    }

    int speed = smoothRampSpeed(calcRampUpSpeed(traveled, accelSpan, cruiseSpeed));

    if (!seenBlind && onLine) {
      // 11번 세로선 추종(후진). 끝단에서 가운데를 잃으면 0.1cm만 직진(끝단 가드).
      if (sawCenter && rc == 0) {
        if (!centerLostArmed) {
          centerLostArmed = true;
          centerLostMark = captureDriveEnc();
        }
        if (encoderTraveledSince(centerLostMark) < toEncoderCounts(0.1f))
          setWheelSpeeds(-speed, -speed);
        else
          traceLineReverse(rl, rc, rr, 0, 0, 0, speed);
      } else {
        traceLineReverse(rl, rc, rr, 0, 0, 0, speed);
      }
    } else {
      setWheelSpeeds(-speed, -speed);
    }
    driveLoopTick();
  }

  stopMotors();
  // 스타트박스에 닿은 뒤 10cm 더 후진
  driveDistanceCm(DIST_FINISH_AFTER_TOUCH_CM, -SPEED_OPEN_TRACK_REV, true);
  // 지정 각도만큼 꺾기 (현재 접근 heading 기준 상대 회전)
  rotateToHeading(HEADING_11_TO_FINISH + FINISH_TURN_DEG);
  // 최종 후진 주차 이동
  driveDistanceCm(DIST_FINISH_PARK_REV_CM, -SPEED_OPEN_TRACK_REV, true);
  stopMotors();
}

}  // namespace

int countScannedBoxesInZones1to4() {
  int count = 0;
  for (int z = 1; z <= 4; z++)
    if (boxes[z].found) count++;
  return count;
}

void traceUntilIntersection(bool stopAtEnd) { traceUntilIntersection(stopAtEnd, SPEED_LINE_FOLLOW_FWD); }

void traceUntilIntersection(bool stopAtEnd, int cruiseSpeed) {
  traceUntilIntersection(stopAtEnd, cruiseSpeed, {0, 0}, 0.0f);
}

void traceUntilIntersection(bool stopAtEnd, int cruiseSpeed,
    DriveEncMark legStart, float legSpanCm) {
  resetLineTracePid();
  clearIntersectionCross();

  long startEnc = labs(prizm.readEncoderCount(1));
  long accelSpan = rampAccelSpanCounts(cruiseSpeed);
  long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);

  // legSpanCm 이 주어지면 legStart 부터의 누적 거리를 기준으로 교차로 도달
  // 전에 미리 감속한다(예: 9->8 비대칭 구간).
  bool useApproach = (legSpanCm > 0.0f);
  long approachStart = useApproach
      ? trackLegApproachStartCounts(legSpanCm, cruiseSpeed) : 0;
  bool approachDecel = false;
  DriveEncMark approachMark = {0, 0};

  intersectionArmed = true;
  intersectionHitCount = 0;
  bool crossFound = false;
  DriveEncMark crossMark = {0, 0};
  int lastCurSpeed = RAMP_MIN_SPEED;

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long traveled = currentEnc - startEnc;

    int fl2, fc2, fr2;
    readFrontLineSensors(fl2, fc2, fr2);
    int rl = 0, rc = 0, rr = 0;

    if (!crossFound) {
      bool isCross = frontCrossFull(fl2, fc2, fr2);
      if (isCross)
        intersectionHitCount++;
      else
        intersectionHitCount = 0;
      if (!isCross) intersectionArmed = true;

      if (intersectionArmed && intersectionHitCount >= CROSS_CONFIRM) {
        if (!stopAtEnd) break;
        crossFound = true;
        crossMark = captureDriveEnc();
        if (alignSpan <= 0) break;
      } else {
        int speed = calcRampUpSpeed(traveled, accelSpan, cruiseSpeed);
        if (useApproach && encoderTraveledSince(legStart) >= approachStart) {
          if (!approachDecel) {
            approachDecel = true;
            approachMark = captureDriveEnc();
          }
          int down = decelMarkSpeed(approachMark, rampDecelSpanCounts(cruiseSpeed),
              cruiseSpeed);
          if (down < speed) speed = down;
        }
        speed = smoothRampSpeed(speed);
        lastCurSpeed = speed;
        traceLineForward(fl2, fc2, fr2, rl, rc, rr, speed);
        driveLoopTick();
        continue;
      }
    }

    int speed = crossAlignSpeed(crossMark, alignSpan, lastCurSpeed);
    speed = smoothRampSpeed(speed);
    if (finishAlignSpan(crossMark, alignSpan, speed)) break;
    traceLineForward(fl2, fc2, fr2, rl, rc, rr, speed);
    driveLoopTick();
  }
}

void traceUntilIntersection() { traceUntilIntersection(true); }

void enterZoneForward(int zone) { runZoneEntry(false, zone, SPEED_OPEN_ZONE_FWD); }

void enterZoneReverse(int zone) { runZoneEntry(true, zone, SPEED_OPEN_ZONE_REV); }

void crossToOppositeZone(int targetZone, int fromZone, bool enableScan) {
  ZoneMotionProfile targetProfile = getZoneProfile(targetZone);
  lineTraceLastEdge = 0;
  resetLineTracePid();

  if (enableScan && fromZone > 0) beginZoneScan(fromZone);

  DriveEncMark motionStart = captureDriveEnc();
  long extraSpan = toEncoderCounts(targetProfile.entryReverseExtra);

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readLineSensors(fl, fc, fr, rl, rc, rr);
    if (rearOnLine(rl, rc, rr)) {
      break;
    }
    int speed = rampMarkSpeed(motionStart, SPEED_OPEN_ZONE_REV);
    speed = smoothRampSpeed(speed);
    setWheelSpeeds(-speed, -speed);
    driveLoopTick();
  }

  if (enableScan) beginZoneScan(targetZone);

  while (true) {
    int fl, fc, fr, rl, rc, rr;
    readLineSensors(fl, fc, fr, rl, rc, rr);
    bool onLine = rearOnLine(rl, rc, rr);

    if (onLine) {
      int speed = rampMarkSpeed(motionStart, SPEED_OPEN_ZONE_REV);
      speed = smoothRampSpeed(speed);
      traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
    } else {
      DriveEncMark pastLineMark = captureDriveEnc();
      if (extraSpan <= 0) break;
      while (true) {
        int speed = decelMarkSpeed(pastLineMark, extraSpan, SPEED_OPEN_ZONE_REV);
        speed = smoothRampSpeed(speed);
        if (finishEncoderSpan(pastLineMark, extraSpan, speed)) break;
        setWheelSpeeds(-speed, -speed);
        driveLoopTick();
      }
      break;
    }
    driveLoopTick();
  }

  stopMotors();
}

void driveOntoMainTrack() {
  headingDeg = 270.0f;

  // 스타트박스 구간은 가속 없이 START_LINE_SEARCH_SPEED 정속으로 라인을 찾으러 간다.
  // 속도 제한기를 그 값으로 초기화해 첫 틱부터 정확히 그 속도가 나오게 한다(25 튐 방지).
  resetRampSpeedLimiter(START_LINE_SEARCH_SPEED);

  while (true) {
    int fl, fc, fr;
    readFrontLineSensors(fl, fc, fr);
    if (frontOnLine(fl, fc, fr)) break;
    int speed = smoothRampSpeed(START_LINE_SEARCH_SPEED);
    setWheelSpeeds(speed, speed);
    driveLoopTick();
  }

  driveDistanceCm(DIST_START_TO_13_CM, SPEED_OPEN_TRACK_FWD, true);
  
  // 13->9 진입 시 12->9 처럼 stopAtEnd=false로 진입하여 감속 없이 정렬거리 통과
  driveTrackLegBlind(HEADING_13_TO_9, -1.0f, false, DIST_TRACK_13_TO_9_CM, 1, true);
  
  // 정렬 직후 브레이크와 동시에 270도 칼각 회전
  rotateToHeading(270.0f);
  
  // 저속 강제 직진 대신, MapRouter의 고속 주행 로직에 위임
  intersectionNode = 9;
  driveToIntersectionNode(8);
}

void driveToFinishArea() {
  // node==11이면 어느 경우든 추가 주행 없이 접근, 아니면 11번까지 이동 후 접근.
  if (intersectionNode != 11) driveToIntersectionNode(11);
  runFinishApproachFrom11();

  finishFromZone6Exit = false;

  playBeep(BUZZER_FINISH_MS);   // 마무리 부저
  prizm.setGreenLED(HIGH);
}

static int finishZoneSearch(int z) {
  endZoneScan();
  stopMotors();
  return z;
}

int searchQrInZones1to4() {
  beginZoneScan(2);
  rotateToHeading(0.0f);
  enterZoneAt(2);
  waitForZoneScan(2);
  if (countScannedBoxesInZones1to4() >= 2) return finishZoneSearch(2);

  moveBetweenZones(2, 4, zoneMoveOpts(true, true));
  waitForZoneScan(4);
  if (countScannedBoxesInZones1to4() >= 2) return finishZoneSearch(4);

  moveBetweenZones(4, 1, zoneMoveOpts(true, true));
  waitForZoneScan(1);
  if (countScannedBoxesInZones1to4() >= 2) return finishZoneSearch(1);

  moveBetweenZones(1, 3, zoneMoveOpts(true, true));
  waitForZoneScan(3);
  return finishZoneSearch(3);
}

int rescanMissingQrZones1to4() {
  int tries = 0;
  // 1 -> 3 -> 2 -> 4 순서대로 재탐색을 진행합니다.
  const int scanOrder[4] = {1, 3, 2, 4};

  while (countScannedBoxesInZones1to4() < 2 && tries < MAX_RESCAN_TRIES) {
    int lastVisitedZone = 0; // 0이면 현재 로봇이 트랙(교차로)에 있다는 뜻

    for (int i = 0; i < 4 && countScannedBoxesInZones1to4() < 2; i++) {
      int targetZ = scanOrder[i];
      if (boxes[targetZ].found) continue; // 이미 찾은 박스존은 패스

      if (lastVisitedZone == 0) {
        // 루프 첫 시작 혹은 이전 방문 구역이 없는 경우: 정상적으로 트랙에서 진입
        beginZoneScan(targetZ);
        navigateToZone(targetZ);
        waitForZoneScan(targetZ);
      } else {
        // 직전에 탐색을 마치고 현재 특정 Zone에 들어가 있는 경우: 
        // moveBetweenZones를 활용하여(alreadyInFromZone=true) 자연스럽게 다음 Zone으로 이동합니다.
        // 이때 1->3 혹은 2->4처럼 맞은편인 경우, 교차로 탈출 없이 곧바로 다이렉트 후진 진입이 실행됩니다.
        moveBetweenZones(lastVisitedZone, targetZ, zoneMoveOpts(true, true));
        waitForZoneScan(targetZ);
      }
      
      lastVisitedZone = targetZ;

      // 2개를 모두 찾았다면 스캔을 끝내고 탐색 완료
      if (countScannedBoxesInZones1to4() >= 2) {
        return finishZoneSearch(targetZ);
      }
    }
    
    // 네 군데를 다 돌았는데도 실패했다면, 다음 try를 위해 현재 머무는 Zone에서 빠져나옵니다.
    if (lastVisitedZone != 0) {
      leaveZone(lastVisitedZone);
    }
    tries++;
  }
  return 0;
}
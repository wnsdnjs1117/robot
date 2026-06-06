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
   bool pastLine = false;
   DriveEncMark lineStartMark = {0, 0};
   DriveEncMark pastLineMark = {0, 0};
 
   if (reverse) DPRINTF("\n+Rev Z:");
   else          DPRINTF("\n+Fwd Z:");
   DPRINT(zone);
 
   while (true) {
     int fl, fc, fr, rl, rc, rr;
     readLineSensors(fl, fc, fr, rl, rc, rr);
     bool onLine = reverse ? rearOnLine(rl, rc, rr) : frontOnLine(fl, fc, fr);
 
     if (!pastLine) {
       if (onLine) {
         if (!sawLine) {
           DPRINTF(" L1");
           sawLine = true;
           lineStartMark = captureDriveEnc();
         }
         int speed = rampMarkSpeed(motionStart, openSpeed);
         speed = smoothRampSpeed(speed);
         if (reverse) traceLineReverse(rl, rc, rr, fl, fc, fr, speed);
         else         traceLineForward(fl, fc, fr, rl, rc, rr, speed);
       } else if (sawLine) {
         DPRINTF(" L0");
         pastLine = true;
         pastLineMark = captureDriveEnc();
         if (extraSpan <= 0) break;
       } else {
         int speed = rampMarkSpeed(motionStart, openSpeed);
         speed = smoothRampSpeed(speed);
         int dir = reverse ? -1 : 1;
         setWheelSpeeds(dir * speed, dir * speed);
       }
     } else {
       int speed = decelMarkSpeed(pastLineMark, extraSpan, openSpeed);
       speed = smoothRampSpeed(speed);
       if (finishEncoderSpan(pastLineMark, extraSpan, speed)) break;
       int dir = reverse ? -1 : 1;
       setWheelSpeeds(dir * speed, dir * speed);
     }
     driveLoopTick();
   }
 
   stopMotors();
   DPRINTF(" Done (실제 이동: ");
   DPRINT((float)encoderTraveledSince(motionStart) / COUNTS_PER_CM);
   DPRINTLNF(" cm)");
 }

 void runFinishApproachFrom11() {
   rotateToHeading(HEADING_11_TO_FINISH);
   resetLineTracePid();
   resetRampSpeedLimiter(RAMP_MIN_SPEED);
   const int cruiseSpeed = SPEED_OPEN_TRACK_REV;
  DriveEncMark motionStart = captureDriveEnc();
  long accelSpan = rampAccelSpanCounts(cruiseSpeed);
  long blindSpan = toEncoderCounts(DIST_FINISH_BLIND_CONFIRM_CM);

  // 11번 선 → 끊김(블라인드) → 다시 라인 = 스타트박스(후방 센서 접촉).
  bool seenBlind = false;
  bool blindArmed = false;
  DriveEncMark blindMark = {0, 0};

  while (true) {
    int rl, rc, rr;
    readRearLineSensors(rl, rc, rr);
    bool onLine = rearOnLine(rl, rc, rr);
    long traveled = encoderTraveledSince(motionStart);

    if (!seenBlind) {
      if (!onLine) {  // 11번 선을 벗어나 블라인드 진입 — 연속 거리로 확정
        if (!blindArmed) { blindArmed = true; blindMark = captureDriveEnc(); }
        else if (encoderTraveledSince(blindMark) >= blindSpan) seenBlind = true;
      } else {
        blindArmed = false;  // 아직 11번 선 위 — 리셋
      }
    } else if (onLine) {  // 블라인드를 밟은 뒤 다시 만난 라인 = 스타트박스
      break;
    }

    int speed = smoothRampSpeed(calcRampUpSpeed(traveled, accelSpan, cruiseSpeed));
    setWheelSpeeds(-speed, -speed);
    driveLoopTick();
  }

   stopMotors();
   rotateToHeading(HEADING_FINISH_PARK);
   driveDistanceCm(DIST_FINISH_PARK_REV_CM, -SPEED_OPEN_TRACK_REV, true);
   stopMotors();
 }

 } // namespace

int countScannedBoxesInZones1to4() {
  int count = 0;
  for (int z = 1; z <= 4; z++) if (boxes[z].found) count++;
  return count;
}

 void alignOnTrackHeading(int openSpeed, float alignCm) {
   resetLineTracePid();
   lineTraceLastEdge = 0;
   DriveEncMark motionStart = captureDriveEnc();
   long alignSpan = toEncoderCounts(alignCm);
   bool onLine = false;
   DriveEncMark lineMark = {0, 0};
   int lastCurSpeed = RAMP_MIN_SPEED;
 
   while (true) {
     int fl, fc, fr, rl, rc, rr;
     readLineSensors(fl, fc, fr, rl, rc, rr);
 
     if (!onLine) {
       if (frontOnLine(fl, fc, fr)) {
         onLine = true;
         lineMark = captureDriveEnc(); // 출발점이 아니라 라인에 닿은 시점을 정렬 시작점으로 수정!
         if (alignSpan <= 0) break;
       } else {
         int speed = rampMarkSpeed(motionStart, openSpeed);
         speed = smoothRampSpeed(speed);
         lastCurSpeed = speed; // 실제 속도 기록
         setWheelSpeeds(speed, speed);
         driveLoopTick();
         continue;
       }
     }
 
     // 라인에 닿은 후 부드럽게 감속
     int speed = crossAlignSpeed(lineMark, alignSpan, lastCurSpeed);
     speed = smoothRampSpeed(speed);
     if (finishAlignSpan(lineMark, alignSpan, speed)) break;
     traceLineForward(fl, fc, fr, rl, rc, rr, speed);
     driveLoopTick();
   }
 }
 
 void traceUntilIntersection(bool stopAtEnd) {
   traceUntilIntersection(stopAtEnd, SPEED_LINE_FOLLOW_FWD);
 }
 
 void traceUntilIntersection(bool stopAtEnd, int cruiseSpeed) {
   resetLineTracePid();
   clearIntersectionCross();
 
   long startEnc = labs(prizm.readEncoderCount(1));
   long accelSpan = rampAccelSpanCounts(cruiseSpeed);
   long alignSpan = toEncoderCounts(DIST_CROSS_ALIGN_CM);
 
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
      if (isCross) intersectionHitCount++;
      else intersectionHitCount = 0;
      if (!isCross) intersectionArmed = true;

      if (intersectionArmed && intersectionHitCount >= CROSS_CONFIRM) {
        if (!stopAtEnd) break;
        crossFound = true;
        crossMark = captureDriveEnc();
        if (alignSpan <= 0) break;
      } else {
        int speed = calcRampUpSpeed(traveled, accelSpan, cruiseSpeed);
        speed = smoothRampSpeed(speed);
        lastCurSpeed = speed;
        traceLineForward(fl2, fc2, fr2, rl, rc, rr, speed);
        driveLoopTick();
        continue;
      }
    }
 
     // 글로벌 최고속도가 아닌 방금 기록된 실제 속도를 기반으로 정렬 감속
     int speed = crossAlignSpeed(crossMark, alignSpan, lastCurSpeed);
     speed = smoothRampSpeed(speed);
     if (finishAlignSpan(crossMark, alignSpan, speed)) break;
     traceLineForward(fl2, fc2, fr2, rl, rc, rr, speed);
     driveLoopTick();
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
 
   while (true) {
     int fl, fc, fr, rl, rc, rr;
     readLineSensors(fl, fc, fr, rl, rc, rr);
     if (rearOnLine(rl, rc, rr)) { DPRINTF(" FindLine"); break; }
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
       DPRINTF(" L0");
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
  DPRINTLNF(" Cross Done");
 }
 
 void driveOntoMainTrack() {
   headingDeg = 270;
   long startEnc = labs(prizm.readEncoderCount(1));
   long accelSpan = rampAccelSpanCounts(START_LINE_SEARCH_SPEED);
 
   while (true) {
     long traveled = labs(prizm.readEncoderCount(1)) - startEnc;
     int fl, fc, fr;
     readFrontLineSensors(fl, fc, fr);
     if (frontOnLine(fl, fc, fr)) break;
     int speed = calcRampUpSpeed(traveled, accelSpan, START_LINE_SEARCH_SPEED);
     speed = smoothRampSpeed(speed);
     setWheelSpeeds(speed, speed);
     driveLoopTick();
   }
 
   driveDistanceCm(DIST_START_TO_13_CM, SPEED_OPEN_TRACK_FWD, true);
   driveTrackLegBlind(HEADING_13_TO_9, -1, true, DIST_TRACK_13_TO_9_CM, 1, true);
   rotateToHeading(270);
   traceUntilIntersection(true);
   intersectionNode = 8;
 }
 
void driveToFinishArea() {
  if (finishFromZone6Exit && intersectionNode == 11) {
    runFinishApproachFrom11();
  } else {
    if (intersectionNode != 11) {
      driveToIntersectionNode(11);
    }
    runFinishApproachFrom11();
  }

  finishFromZone6Exit = false;

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
 
 int rescanMissingQrZones1to4() {
   int tries = 0;
   while (countScannedBoxesInZones1to4() < 2 && tries < MAX_RESCAN_TRIES) {
     for (int z = 1; z <= 4 && countScannedBoxesInZones1to4() < 2; z++) {
       if (boxes[z].found) continue;
       beginZoneScan(z);
       navigateToZone(z);
       waitForZoneScan(z);
       endZoneScan();
       // 마지막 박스를 인식해 2개를 채웠다면 이 존에서 나가지 않고 머무른 채 반환 —
       // 호출부가 바로 그 자리에서 배송해 불필요한 탈출·재진입을 없앤다.
       if (countScannedBoxesInZones1to4() >= 2) return z;
       leaveZone(z);
     }
    tries++;
  }
  return 0;
}
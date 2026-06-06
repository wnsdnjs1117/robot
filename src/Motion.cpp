/* ============================================================
 * Motion.cpp - 듀얼 PID 조향 및 가감속 거리 주행
 * ============================================================ */
 #include "Motion.h"
 #include "Config.h"
 #include "Lift.h"
 #include "BoxMap.h"
 
 bool enableTeeZoneSteering = false;
 
 static long encoderRemaining(DriveEncMark mark, long targetSpan) {
   return targetSpan - encoderTraveledSince(mark);
 }
 
 static int iround(float v) {
   return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
 }
 
// 루프당 1회 엔코더 캐시 — 거리추적·속도PID가 공유해 I2C 중복읽기 제거(루프 가속)
static long g_encL = 0, g_encR = 0;

// 속도 PID가 측정한 실제 바퀴 속도 (명령속도 0~100 환산, 10ms 정규화) — 급제동 거리 산정용
static int g_measuredSpeed = 0;

void refreshDriveEncoders() {
  g_encL = prizm.readEncoderCount(1);
  g_encR = prizm.readEncoderCount(2);
}

DriveEncMark captureDriveEnc() {
  // 이벤트(라인 인식 등) 기준점은 항상 최신값으로 — 인식 시점 정확도 보존
  g_encL = prizm.readEncoderCount(1);
  g_encR = prizm.readEncoderCount(2);
  return { g_encL, g_encR };
}

long encoderTraveledSince(DriveEncMark mark) {
  long dl = labs(g_encL - mark.left);
  long dr = labs(g_encR - mark.right);
  return (dl + dr) / 2;
}
 
 int rampMarkSpeed(DriveEncMark start, int maxSpeed) {
   return calcRampUpSpeed(encoderTraveledSince(start),
       rampAccelSpanCounts(maxSpeed), maxSpeed);
 }
 
 int decelMarkSpeed(DriveEncMark mark, long totalSpan, int cruiseSpeed) {
   long remaining = totalSpan - encoderTraveledSince(mark);
   if (remaining <= 0) return RAMP_MIN_SPEED;
   long decelSpan = rampDecelSpanCounts(cruiseSpeed);
   if (totalSpan <= decelSpan)
     return calcRampDownSpeed(remaining, totalSpan, cruiseSpeed);
   if (remaining > decelSpan) return cruiseSpeed;
   return calcRampDownSpeed(remaining, decelSpan, cruiseSpeed);
 }
 
 int crossAlignSpeed(DriveEncMark mark, long alignSpan, int cruiseSpeed) {
   if (alignSpan <= 0) return RAMP_MIN_SPEED;
   long decelSpan = rampDecelSpanCounts(cruiseSpeed);
   if (decelSpan > alignSpan) decelSpan = alignSpan;
   return calcRampDownSpeed(encoderRemaining(mark, alignSpan), decelSpan, cruiseSpeed);
 }
 
static void stopForTarget(int /*curSpeed*/) {
  stopMotors();
}

// 급제동 시작 거리 — 현재속도에 비례해 DIST_BRAKE_CATCH_CM(최저속)→DIST_BRAKE_CATCH_MAX_CM(속도100) 선형 증가.
// 명령속도와 '실측 바퀴속도(관성 반영)' 중 큰 값 기준 — 감속램프로 명령속도가 낮아져도
// 실제로 빠르게 굴러가면 더 일찍 급제동을 건다.
static long brakeCatchCounts(int curSpeed) {
  int v = (g_measuredSpeed > curSpeed) ? g_measuredSpeed : curSpeed;
  float t = (float)v / 100.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  float cm = DIST_BRAKE_CATCH_CM + (DIST_BRAKE_CATCH_MAX_CM - DIST_BRAKE_CATCH_CM) * t;
  return (long)toEncoderCounts(cm);
}

bool finishEncoderRemaining(long remainingCounts, int curSpeed) {
  if (remainingCounts <= 0) {
    stopForTarget(curSpeed);
    return true;
  }
  if (remainingCounts <= brakeCatchCounts(curSpeed)) {
    stopForTarget(curSpeed);
    return true;
  }
  return false;
}

bool finishEncoderSpan(DriveEncMark mark, long targetSpan, int curSpeed) {
  return finishEncoderRemaining(encoderRemaining(mark, targetSpan), curSpeed);
}

bool finishAlignSpan(DriveEncMark alignStart, long alignSpanCounts, int curSpeed) {
  if (alignSpanCounts <= 0) return false;
  if (encoderTraveledSince(alignStart) >= alignSpanCounts) {
    stopMotors();
    return true;
  }
  return finishEncoderRemaining(encoderRemaining(alignStart, alignSpanCounts), curSpeed);
}
 
 int calcRampUpSpeed(long traveledCounts, long accelCounts, int maxSpeed) {
   if (accelCounts <= 0 || traveledCounts >= accelCounts) return maxSpeed;
   int speed = RAMP_MIN_SPEED
       + (int)((long)(maxSpeed - RAMP_MIN_SPEED) * traveledCounts / accelCounts);
   return (speed < RAMP_MIN_SPEED) ? RAMP_MIN_SPEED : speed;
 }
 
 int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed) {
   if (decelCounts <= 0 || remainingCounts >= decelCounts) return startSpeed;
   if (remainingCounts <= 0) return RAMP_MIN_SPEED;
   int speed = RAMP_MIN_SPEED
       + (int)((long)(startSpeed - RAMP_MIN_SPEED) * remainingCounts / decelCounts);
   return (speed < RAMP_MIN_SPEED) ? RAMP_MIN_SPEED : speed;
 }
 
 static int rampLimiterPrev = RAMP_MIN_SPEED;
 
 void resetRampSpeedLimiter(int initialSpeed) {
   rampLimiterPrev = initialSpeed;
 }
 
 int smoothRampSpeed(int targetSpeed) {
   if (targetSpeed > rampLimiterPrev + RAMP_MAX_SPEED_STEP)
     targetSpeed = rampLimiterPrev + RAMP_MAX_SPEED_STEP;
   rampLimiterPrev = targetSpeed;
   return targetSpeed;
 }
 
 void delayWithTicks(unsigned long ms) {
   unsigned long start = millis();
   while (millis() - start < ms) {
     liftUpTick(); liftDownTick();
     checkDebugKey();
   }
 }
 
 void playBeep(unsigned long ms) {
   pinMode(PIN_BUZZER, OUTPUT);
   unsigned long end = millis() + ms;
   while (millis() < end) {
     digitalWrite(PIN_BUZZER, HIGH); delayMicroseconds(500);
     digitalWrite(PIN_BUZZER, LOW);  delayMicroseconds(500);
   }
   digitalWrite(PIN_BUZZER, LOW);
 }
 
 void setWheelSpeeds(int left, int right) {
   checkDebugKey();
 
   if (left == 0 && right == 0) { prizm.setMotorSpeeds(0, 0); return; }
 
   static unsigned long lastTime = 0;
   static long lastEncL = 0, lastEncR = 0;
   static int outL = 0, outR = 0, lastReqL = 0, lastReqR = 0;
 
   unsigned long now = millis();
   unsigned long dt = now - lastTime;
 
   if (left != lastReqL || right != lastReqR) { outL = left; outR = right; }
   lastReqL = left; lastReqR = right;
 
  if (dt >= MOTOR_VELOCITY_PID_MS) {
    refreshDriveEncoders();  // 10ms마다 1회만 엔코더 읽고 캐시에 저장 — 거리추적이 재사용
    long encL = g_encL;
    long encR = g_encR;
    long curVelL = labs(encL - lastEncL);
     long curVelR = labs(encR - lastEncR);
     // 실측 속도를 10ms로 정규화해 명령속도(0~100) 스케일로 환산 — 급제동 거리 산정용
     if (dt > 0 && VELOCITY_TARGET_FACTOR > 0.0f) {
       float per10 = ((curVelL + curVelR) * 0.5f) * (float)MOTOR_VELOCITY_PID_MS / (float)dt;
       g_measuredSpeed = (int)(per10 / VELOCITY_TARGET_FACTOR);
     }
     float targetL = abs(left) * VELOCITY_TARGET_FACTOR;
     float targetR = abs(right) * VELOCITY_TARGET_FACTOR;
     int outLmag = abs(left) + (int)((targetL - curVelL) * VELOCITY_KP);
     int outRmag = abs(right) + (int)((targetR - curVelR) * VELOCITY_KP);
     outL = (left >= 0) ? outLmag : -outLmag;
     outR = (right >= 0) ? outRmag : -outRmag;
     outL = constrain(outL, left - VELOCITY_MAX_CORRECTION, left + VELOCITY_MAX_CORRECTION);
     outR = constrain(outR, right - VELOCITY_MAX_CORRECTION, right + VELOCITY_MAX_CORRECTION);
     lastTime = now; lastEncL = encL; lastEncR = encR;
   }
 
   int finalL = outL, finalR = outR;
   float speedMag = abs(finalL);
   if (speedMag < 20.0f) speedMag = 20.0f;
   if (speedMag > 100.0f) speedMag = 100.0f;
   float comp = 1.021f + (0.985f - 1.021f) * ((speedMag - 20.0f) / 80.0f);
   if (finalL > 0) finalL = (int)(finalL * comp + 0.5f);
   else if (finalL < 0) finalL = (int)(finalL * comp - 0.5f);
 
   if (finalL > 0 && finalR > 0) { finalL += MOTOR_OFFSET_L; finalR += MOTOR_OFFSET_R; }
   else if (finalL < 0 && finalR < 0) { finalL -= MOTOR_OFFSET_L; finalR -= MOTOR_OFFSET_R; }
 
   prizm.setMotorSpeeds(-(constrain(finalL, -100, 100) * 7), constrain(finalR, -100, 100) * 7);
 }
 
 void stopMotors() {
   prizm.setMotorPower(1, 125);
   prizm.setMotorPower(2, 125);
   delayWithTicks(80);
   prizm.setMotorSpeeds(0, 0);
 }
 
 static long spinDegToCounts(float deg) {
   return (long)((SPIN_90_COUNTS / 90.0f) * deg + 0.5f);
 }
 
static void spinMotorSpeeds(bool clockwise, int speed) {
  checkDebugKey();
  if (speed <= 0) {
    prizm.setMotorSpeeds(0, 0);
    return;
  }
  speed = constrain(speed, RAMP_MIN_SPEED, 100);
  if (clockwise) prizm.setMotorSpeeds(-speed * 7, -speed * 7);
  else           prizm.setMotorSpeeds(speed * 7, speed * 7);
}

// 라인 로직 없이 지정 카운트만큼 최저속으로 제자리 회전 (오버슈트 복구용)
static void spinPlainCounts(bool clockwise, long counts) {
  if (counts <= 0) return;
  resetRampSpeedLimiter(RAMP_MIN_SPEED);
  long startL = prizm.readEncoderCount(1);
  long startR = prizm.readEncoderCount(2);
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1) - startL)
              + labs(prizm.readEncoderCount(2) - startR)) / 2;
    if (pos >= counts) break;
    spinMotorSpeeds(clockwise, RAMP_MIN_SPEED);
    liftUpTick(); liftDownTick();
  }
  spinMotorSpeeds(clockwise, 0);
  delayWithTicks(40);
  setWheelSpeeds(0, 0);
}
 
 void rotateByDegrees(int degrees, bool clockwise) {
   setWheelSpeeds(0, 0);
   delayWithTicks(40);
   resetRampSpeedLimiter(RAMP_MIN_SPEED);
   int absDeg = degrees >= 0 ? degrees : -degrees;
   float compDeg = (float)absDeg * (1.0f - SPIN_OVERSHOOT_COMP_FRAC);
   long targetCounts = spinDegToCounts(compDeg);
   if (targetCounts <= 0) return;
 
   long startL = prizm.readEncoderCount(1);
   long startR = prizm.readEncoderCount(2);
   
   long idealAccelSpan = spinDegToCounts(rampSpinAccelDeg(SPIN_SPEED));
   long idealDecelSpan = spinDegToCounts(rampSpinDecelDeg(SPIN_SPEED));
   long accelSpan, decelSpan;

   if (targetCounts >= idealAccelSpan + idealDecelSpan) {
     accelSpan = idealAccelSpan;
     decelSpan = idealDecelSpan;
   } else {
     accelSpan = targetCounts * idealAccelSpan / (idealAccelSpan + idealDecelSpan);
     decelSpan = targetCounts - accelSpan; 
   }

   long endDecelSpan = spinDegToCounts(SPIN_END_DECEL_DEG);
   if (endDecelSpan > decelSpan / 2) endDecelSpan = decelSpan / 2;
 
   int peakSpeed = SPIN_SPEED;
   if (idealAccelSpan > 0 && accelSpan < idealAccelSpan) {
     peakSpeed = RAMP_MIN_SPEED + (int)((long)(SPIN_SPEED - RAMP_MIN_SPEED) * accelSpan / idealAccelSpan);
   }
 
   int fl0, fc0, fr0;
   readFrontLineSensors(fl0, fc0, fr0);
   bool skipLineTrim = (fc0 != 0);
   bool oppositeWasOn = clockwise ? (fl0 != 0) : (fr0 != 0);
   bool lineTrimmed = false;
   int lastCurSpeed = RAMP_MIN_SPEED;
 
   while (true) {
    long pos = (labs(prizm.readEncoderCount(1) - startL) + labs(prizm.readEncoderCount(2) - startR)) / 2;
    long remaining = targetCounts - pos;

    if (remaining <= 0) break;

    if (!skipLineTrim && !lineTrimmed && pos >= (long)(targetCounts * SPIN_LINE_TRIM_MIN_FRAC)) {
      int fl, fc, fr;
      readFrontLineSensors(fl, fc, fr);
      (void)fc;
      bool oppositeOn = clockwise ? (fl != 0) : (fr != 0);
      if (oppositeOn && !oppositeWasOn) {
        // 70% 이후 반대쪽 센서에 라인 — 남은 각의 절반만 마저 회전
        long newRemaining = (long)(remaining * SPIN_LINE_TRIM_REMAIN_FRAC);
        targetCounts = pos + newRemaining;
        remaining = newRemaining;
        lineTrimmed = true;
         
         decelSpan = remaining;
         if (decelSpan <= 0) decelSpan = 1;
         accelSpan = 0; 
         peakSpeed = lastCurSpeed; 
         endDecelSpan = min(spinDegToCounts(SPIN_END_DECEL_DEG), decelSpan / 2);
       }
     }
 
     int curSpeed;
     if (pos < accelSpan && !lineTrimmed) {
       curSpeed = calcRampUpSpeed(pos, accelSpan, peakSpeed);
     } else {
       curSpeed = peakSpeed;
       if (remaining <= decelSpan)
         curSpeed = min(curSpeed, calcRampDownSpeed(remaining, decelSpan, peakSpeed));
       if (remaining <= endDecelSpan)
         curSpeed = min(curSpeed, calcRampDownSpeed(remaining, endDecelSpan, peakSpeed));
     }
     
    curSpeed = smoothRampSpeed(curSpeed);
    lastCurSpeed = curSpeed; 

    spinMotorSpeeds(clockwise, curSpeed);
    liftUpTick(); liftDownTick();
  }

  spinMotorSpeeds(clockwise, 0);
  delayWithTicks(40);
  setWheelSpeeds(0, 0);

  // 절반만 마저 돌았는데 반대쪽 센서에서 라인이 사라졌으면(오버슈트) 역방향 10° 복구
  if (lineTrimmed) {
    int fl, fc, fr;
    readFrontLineSensors(fl, fc, fr);
    bool oppositeStillOn = clockwise ? (fl != 0) : (fr != 0);
    if (!oppositeStillOn)
      spinPlainCounts(!clockwise, spinDegToCounts(SPIN_LINE_RECOVER_DEG));
  }
}
 
 void readFrontLineSensors(int& left, int& center, int& right) {
   left   = digitalRead(PIN_LINE_FRONT_LEFT);
   center = digitalRead(PIN_LINE_FRONT_CENTER);
   right  = digitalRead(PIN_LINE_FRONT_RIGHT);
   if (INVERT_LINE_SENSORS) { left = !left; center = !center; right = !right; }
 }
 
 void readRearLineSensors(int& left, int& center, int& right) {
   left   = (analogRead(PIN_LINE_REAR_LEFT)   >= REAR_LINE_THRESHOLD) ? 1 : 0;
   center = (analogRead(PIN_LINE_REAR_CENTER) >= REAR_LINE_THRESHOLD) ? 1 : 0;
   right  = (analogRead(PIN_LINE_REAR_RIGHT)  >= REAR_LINE_THRESHOLD) ? 1 : 0;
 }
 
 void readLineSensors(int& fl, int& fc, int& fr, int& rl, int& rc, int& rr) {
   readFrontLineSensors(fl, fc, fr);
   readRearLineSensors(rl, rc, rr);
 }
 
void driveLoopTick() {
  // 주행 중에는 리프트만 틱. QR(HuskyLens) I2C는 정지 스캔 드웰(waitForZoneScan)에서만
  // 수행한다 — request()가 최대 30ms 블로킹이라 주행 루프 속도를 떨어뜨려 위치 오차를 유발.
  liftUpTick();
  liftDownTick();
}
 
 bool frontOnLine(int left, int center, int right) { return left || center || right; }
 bool rearOnLine(int left, int center, int right) { return left || center || right; }
 
 namespace {
 float lineFwdIntegral = 0;
 float lineFwdLastError = 0;
 float lineRevIntegral = 0;
 float lineRevLastError = 0;

int lineTraceCruiseSpeed(int baseSpeed, int absError) {
  if (absError < 2) return baseSpeed;
  float f = LINE_HARD_STEER_SPEED_FACTOR;
  if (absError >= 3) f *= LINE_HARD_STEER_SPEED_FACTOR; // 라인 이탈 — 더 감속
  int speed = (int)(baseSpeed * f);
  return speed < RAMP_MIN_SPEED ? RAMP_MIN_SPEED : speed;
}
}
 
 void resetLineTracePid() {
   lineFwdIntegral = 0;
   lineFwdLastError = 0;
   lineRevIntegral = 0;
   lineRevLastError = 0;
   resetRampSpeedLimiter(RAMP_MIN_SPEED);
 }
 
 void clearIntersectionCross() {
   int fl, fc, fr;
   readFrontLineSensors(fl, fc, fr);
   if (!(fl == 1 && fc == 1 && fr == 1)) return;
 
   long clearEnc = labs(prizm.readEncoderCount(1));
   while (true) {
     readFrontLineSensors(fl, fc, fr);
     if (!(fl == 1 && fc == 1 && fr == 1)) break;
     traceLineForward(fl, fc, fr, 0, 0, 0, RAMP_MIN_SPEED);
     liftUpTick(); liftDownTick();
   }
   long creepSpan = toEncoderCounts(3.0f);
   while (labs(prizm.readEncoderCount(1) - clearEnc) < creepSpan) {
     setWheelSpeeds(RAMP_MIN_SPEED, RAMP_MIN_SPEED);
     liftUpTick(); liftDownTick();
   }
 }
 
 void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int baseSpeed,
     float kpSoft, float kpHard) {
   int posFront = 0, posRear = 0;

   if      (fl && !fc && !fr) { posFront = -2; lineTraceLastEdge = 1; }
   else if (fl && fc && !fr)  { posFront = -1; lineTraceLastEdge = 1; }
   else if (!fl && fc && !fr) { posFront =  0; lineTraceLastEdge = 0; }
   else if (!fl && fc && fr)  { posFront =  1; lineTraceLastEdge = 2; }
   else if (!fl && !fc && fr) { posFront =  2; lineTraceLastEdge = 2; }
   else if (fl && fc && fr)   { posFront =  0; lineTraceLastEdge = 0; }
   else { posFront = (lineTraceLastEdge == 1) ? -3 : ((lineTraceLastEdge == 2) ? 3 : 0); }

   if      (rl && !rc && !rr) posRear = -2;
   else if (rl && rc && !rr)  posRear = -1;
   else if (!rl && rc && !rr) posRear =  0;
   else if (!rl && rc && rr)  posRear =  1;
   else if (!rl && !rc && rr) posRear =  2;
   else posRear = 0;

   float error = (float)posFront;
   lineFwdIntegral += error;
   float derivative = error - lineFwdLastError;
   int absError = abs((int)error);
   bool hardSteer = absError >= 2;
   float kp = hardSteer ? kpHard : kpSoft;
   int cruise = lineTraceCruiseSpeed(baseSpeed, absError);
   if (hardSteer) resetRampSpeedLimiter(cruise); // 감속 유지 — 다음 프레임 급복귀 방지
   float steer = error * kp + lineFwdIntegral * LINE_KI + derivative * LINE_KD;

   int leftSpeed  = cruise + (int)iround(steer);
   int rightSpeed = cruise - (int)iround(steer);

   if ((fl || fc || fr) && (rl || rc || rr) && !(fl && fc && fr) && !(rl && rc && rr)) {
     float align = (posFront - posRear) * LINE_ALIGN_GAIN;
     leftSpeed  += (int)iround(align);
     rightSpeed -= (int)iround(align);
   }

   if (fl == 1 && fc == 0 && fr == 0 && rl == 1 && rc == 0 && rr == 0) {
     leftSpeed -= EDGE_SYNC_GAIN; rightSpeed += EDGE_SYNC_GAIN;
   } else if (fl == 0 && fc == 0 && fr == 1 && rl == 0 && rc == 0 && rr == 1) {
     leftSpeed += EDGE_SYNC_GAIN; rightSpeed -= EDGE_SYNC_GAIN;
   }

   lineFwdLastError = error;
   setWheelSpeeds(leftSpeed, rightSpeed);
 }

 void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int baseSpeed) {
   traceLineForward(fl, fc, fr, rl, rc, rr, baseSpeed, LINE_KP_FWD_SOFT, LINE_KP_FWD_HARD);
 }
 
 void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr) {
   traceLineForward(fl, fc, fr, rl, rc, rr, SPEED_LINE_FOLLOW_FWD);
 }
 
 void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr, int baseSpeed) {
   int posRear = 0, posFront = 0;
 
   if      (rl && !rc && !rr) { posRear = -2; lineTraceLastEdge = 1; }
   else if (rl && rc && !rr)  { posRear = -1; lineTraceLastEdge = 1; }
   else if (!rl && rc && !rr) { posRear =  0; lineTraceLastEdge = 0; }
   else if (!rl && rc && rr)  { posRear =  1; lineTraceLastEdge = 2; }
   else if (!rl && !rc && rr) { posRear =  2; lineTraceLastEdge = 2; }
   else if (rl && rc && rr)   { posRear =  0; lineTraceLastEdge = 0; }
   else { posRear = (lineTraceLastEdge == 1) ? -3 : ((lineTraceLastEdge == 2) ? 3 : 0); }
 
   if      (fl && !fc && !fr) posFront = -2;
   else if (fl && fc && !fr)  posFront = -1;
   else if (!fl && fc && !fr) posFront =  0;
   else if (!fl && fc && fr)  posFront =  1;
   else if (!fl && !fc && fr) posFront =  2;
   else posFront = 0;
 
   float error = (float)posRear;
   lineRevIntegral += error;
   float derivative = error - lineRevLastError;
   int absError = abs((int)error);
   bool hardSteer = absError >= 2;
   float kp = hardSteer ? LINE_KP_REV_HARD : LINE_KP_REV_SOFT;
   int cruise = lineTraceCruiseSpeed(baseSpeed, absError);
   if (hardSteer) resetRampSpeedLimiter(cruise); // 감속 유지 — 다음 프레임 급복귀 방지
   float steer = error * kp + lineRevIntegral * LINE_KI + derivative * LINE_KD;

   int leftSpeed  = -cruise - (int)iround(steer);
   int rightSpeed = -cruise + (int)iround(steer);
 
   if ((rl || rc || rr) && (fl || fc || fr) && !(rl && rc && rr) && !(fl && fc && fr)) {
     float align = (posRear - posFront) * LINE_ALIGN_GAIN;
     leftSpeed  -= (int)iround(align);
     rightSpeed += (int)iround(align);
   }
 
   if (rl == 1 && rc == 0 && rr == 0 && fl == 1 && fc == 0 && fr == 0) {
     leftSpeed += EDGE_SYNC_GAIN; rightSpeed -= EDGE_SYNC_GAIN;
   } else if (rl == 0 && rc == 0 && rr == 1 && fl == 0 && fc == 0 && fr == 1) {
     leftSpeed -= EDGE_SYNC_GAIN; rightSpeed += EDGE_SYNC_GAIN;
   }
 
   lineRevLastError = error;
   setWheelSpeeds(leftSpeed, rightSpeed);
 }
 
 void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr) {
   traceLineReverse(rl, rc, rr, fl, fc, fr, SPEED_LINE_FOLLOW_REV);
 }
 
 void correctTrackLegOvershoot(DriveEncMark legStart, float plannedSpanCm) {
   long planned = toEncoderCounts(plannedSpanCm);
   long traveled = encoderTraveledSince(legStart);
   float errorCm = (float)(traveled - planned) / COUNTS_PER_CM;
 
   if (errorCm > DIST_TRACK_OVERSHOOT_MIN_CM) {
     float corr = errorCm;
     if (corr > DIST_TRACK_OVERSHOOT_MAX_CM) corr = DIST_TRACK_OVERSHOOT_MAX_CM;
     driveDistanceCm(corr, -SPEED_LINE_FOLLOW_REV, true);
     playBeep(BUZZER_OVERSHOOT_CORR_MS);
   } else if (errorCm < -DIST_TRACK_OVERSHOOT_MIN_CM) {
     float corr = -errorCm;
     if (corr > DIST_TRACK_OVERSHOOT_MAX_CM) corr = DIST_TRACK_OVERSHOOT_MAX_CM;
     driveDistanceCm(corr, SPEED_LINE_FOLLOW_FWD, true);
     playBeep(BUZZER_OVERSHOOT_CORR_MS);
   }
 }
 
 void driveDistanceCm(float cm, int speed, bool stopAtEnd) {
   float absCm = cm >= 0.0f ? cm : -cm;
   if (absCm <= 0.0f) { if (stopAtEnd) stopMotors(); return; }
 
   resetRampSpeedLimiter(RAMP_MIN_SPEED);
   int maxSpeed = abs(speed);
   int direction = (speed >= 0) ? 1 : -1;
   if (cm < 0) direction = -direction;
 
   DriveEncMark startEnc = captureDriveEnc();
   long targetCounts = toEncoderCounts(absCm);
   
   long idealAccelSpan = rampAccelSpanCounts(maxSpeed);
   long idealDecelSpan = stopAtEnd ? rampDecelSpanCounts(maxSpeed) : 0;
   long accelSpan, decelSpan;

   if (targetCounts >= idealAccelSpan + idealDecelSpan) {
     accelSpan = idealAccelSpan;
     decelSpan = idealDecelSpan;
   } else {
     if (idealAccelSpan + idealDecelSpan > 0) {
       accelSpan = targetCounts * idealAccelSpan / (idealAccelSpan + idealDecelSpan);
       decelSpan = targetCounts - accelSpan;
     } else {
       accelSpan = targetCounts / 2;
       decelSpan = targetCounts / 2;
     }
   }
 
   int peakSpeed = maxSpeed;
   if (idealAccelSpan > 0 && accelSpan < idealAccelSpan) {
     peakSpeed = RAMP_MIN_SPEED + (int)((long)(maxSpeed - RAMP_MIN_SPEED) * accelSpan / idealAccelSpan);
   }
 
   while (true) {
     long traveled = encoderTraveledSince(startEnc);
     long remaining = targetCounts - traveled;
 
     int curSpeed;
     if (traveled < accelSpan) {
       curSpeed = calcRampUpSpeed(traveled, accelSpan, peakSpeed); 
     } else if (stopAtEnd && remaining <= decelSpan) {
       curSpeed = calcRampDownSpeed(remaining, decelSpan, peakSpeed); 
     } else {
       curSpeed = peakSpeed; 
     }
 
     curSpeed = smoothRampSpeed(curSpeed);
 
     if (stopAtEnd) {
       if (finishEncoderRemaining(remaining, curSpeed)) break;
     } else if (remaining <= 0) {
       break;
     }
 
     setWheelSpeeds(direction * curSpeed, direction * curSpeed);
     driveLoopTick();
   }
 }
 
 void driveOverLinesAndAlign(int lineCount, float alignCm, int speed, bool stopAtEnd) {
   resetRampSpeedLimiter(RAMP_MIN_SPEED);
   DriveEncMark motionStart = captureDriveEnc();
   long startEnc = labs(prizm.readEncoderCount(1));
   long nextIgnoreEnc = startEnc + toEncoderCounts(DIST_IGNORE_NODE_CM);
   int linesPassed = 0;
   int phase = 0; 
 
   int maxSpeed = abs(speed);
   long accelSpan = rampAccelSpanCounts(maxSpeed);
   long alignSpan = toEncoderCounts(alignCm);
   DriveEncMark alignMark = {0, 0};
   int speedAtLine = maxSpeed;
   int lastCurSpeed = RAMP_MIN_SPEED;
   float plannedLegSpanCm = 0.0f;
 
   while (true) {
     long currentEnc = labs(prizm.readEncoderCount(1));
     long traveled = currentEnc - startEnc;
 
     if (phase == 0) {
       if (currentEnc >= nextIgnoreEnc) phase = 1;
       int curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
       curSpeed = smoothRampSpeed(curSpeed);
       lastCurSpeed = curSpeed; // 실제 속도 기록
       setWheelSpeeds(curSpeed, curSpeed);
     }
     else if (phase == 1) {
       int fl, fc, fr;
       readFrontLineSensors(fl, fc, fr);
       if (frontOnLine(fl, fc, fr)) {
         linesPassed++;
         if (linesPassed >= lineCount) {
           plannedLegSpanCm = (float)encoderTraveledSince(motionStart) / COUNTS_PER_CM;
           if (alignSpan <= 0) {
             if (stopAtEnd) stopMotors();
             break;
           }
           phase = 2;
           alignMark = captureDriveEnc();
           plannedLegSpanCm += alignCm;
           speedAtLine = lastCurSpeed; // 방금 전까지의 실제 최고 속도로 세팅!
         } else {
           phase = 0;
           nextIgnoreEnc = currentEnc + toEncoderCounts(DIST_IGNORE_NODE_CM);
         }
       }
       if (phase == 1) {
         int curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
         curSpeed = smoothRampSpeed(curSpeed);
         lastCurSpeed = curSpeed; // 실제 속도 기록
         setWheelSpeeds(curSpeed, curSpeed);
       }
     }
 
    if (phase == 2) {
      long remaining = encoderRemaining(alignMark, alignSpan);
      long decelSpan = rampDecelSpanCounts(speedAtLine);
      if (decelSpan > alignSpan) decelSpan = alignSpan;

      int curSpeed = calcRampDownSpeed(remaining, decelSpan, speedAtLine);
      curSpeed = smoothRampSpeed(curSpeed); // 스무딩 적용

      if (stopAtEnd && finishAlignSpan(alignMark, alignSpan, curSpeed)) break;
      if (!stopAtEnd && encoderTraveledSince(alignMark) >= alignSpan) break;
      if (!stopAtEnd && remaining <= 0) break;
      setWheelSpeeds(curSpeed, curSpeed);
    }
 
     driveLoopTick();
   }
 
  if (stopAtEnd && plannedLegSpanCm > 0.0f)
    correctTrackLegOvershoot(motionStart, plannedLegSpanCm);
}
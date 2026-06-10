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
 
static long g_encL = 0, g_encR = 0;
static int g_measuredSpeed = 0;

void refreshDriveEncoders() {
  g_encL = prizm.readEncoderCount(1);
  g_encR = prizm.readEncoderCount(2);
}

DriveEncMark captureDriveEnc() {
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
   // 목표 속도가 기본 출발 속도(RAMP_MIN) 이하면 가속 계산을 생략하고 그 속도 그대로 적용.
   // (RAMP_MIN 미만을 목표로 하면 기존 수식이 RAMP_MIN으로 강제 클램프되어 25→10 튐 발생)
   if (maxSpeed <= RAMP_MIN_SPEED) return maxSpeed;
   if (accelCounts <= 0 || traveledCounts >= accelCounts) return maxSpeed;
   int speed = RAMP_MIN_SPEED
       + (int)((long)(maxSpeed - RAMP_MIN_SPEED) * traveledCounts / accelCounts);
   return (speed < RAMP_MIN_SPEED) ? RAMP_MIN_SPEED : speed;
 }

 int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed) {
   // 감속 시작 속도가 기본 정지 속도(RAMP_MIN) 이하면 감속 계산을 생략하고 그대로 적용.
   if (startSpeed <= RAMP_MIN_SPEED) return startSpeed;
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
     driveLoopTick();
   }
 }
 
 static unsigned long g_beepEndMs        = 0;
 static unsigned long g_beepLastToggleUs = 0;
 static bool          g_beepPinHigh      = false;

 void updateBeep();

 void playBeep(unsigned long ms) {
   startBeep(ms);
   while (g_beepEndMs != 0) updateBeep();
 }

 void startBeep(unsigned long ms) {
   pinMode(PIN_BUZZER, OUTPUT);
   g_beepEndMs        = millis() + ms;
   if (g_beepEndMs == 0) g_beepEndMs = 1;
   g_beepLastToggleUs = micros();
   g_beepPinHigh      = false;
   digitalWrite(PIN_BUZZER, LOW);
 }

 void updateBeep() {
   if (g_beepEndMs == 0) return;
   if (millis() >= g_beepEndMs) {
     g_beepEndMs   = 0;
     g_beepPinHigh = false;
     digitalWrite(PIN_BUZZER, LOW);
     return;
   }
   unsigned long nowUs = micros();
   if (nowUs - g_beepLastToggleUs >= BUZZER_TONE_HALF_US) {
     g_beepLastToggleUs = nowUs;
     g_beepPinHigh      = !g_beepPinHigh;
     digitalWrite(PIN_BUZZER, g_beepPinHigh ? HIGH : LOW);
   }
 }
 
 void setWheelSpeeds(int left, int right) {
   if (left == 0 && right == 0) { prizm.setMotorSpeeds(0, 0); return; }
 
   static unsigned long lastTime = 0;
   static long lastEncL = 0, lastEncR = 0;
   static int outL = 0, outR = 0, lastReqL = 0, lastReqR = 0;
 
   unsigned long now = millis();
   unsigned long dt = now - lastTime;
 
   if (left != lastReqL || right != lastReqR) { outL = left; outR = right; }
   lastReqL = left; lastReqR = right;
 
  if (dt >= MOTOR_VELOCITY_PID_MS) {
    refreshDriveEncoders();
    long encL = g_encL;
    long encR = g_encR;
    long curVelL = labs(encL - lastEncL);
     long curVelR = labs(encR - lastEncR);
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
   delayWithTicks(10);
   prizm.setMotorSpeeds(0, 0);
 }
 
 static long spinDegToCounts(float deg) {
   return (long)((SPIN_90_COUNTS / 90.0f) * deg + 0.5f);
 }
 
static void spinMotorSpeeds(bool clockwise, int speed) {
  if (speed <= 0) {
    prizm.setMotorSpeeds(0, 0);
    return;
  }
  speed = constrain(speed, RAMP_MIN_SPEED, 100);
  if (clockwise) prizm.setMotorSpeeds(-speed * 7, -speed * 7);
  else           prizm.setMotorSpeeds(speed * 7, speed * 7);
}

static void spinPlainCounts(bool clockwise, long counts) {
  if (counts <= 0) return;
  long startL = prizm.readEncoderCount(1);
  long startR = prizm.readEncoderCount(2);
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1) - startL)
              + labs(prizm.readEncoderCount(2) - startR)) / 2;
    if (pos >= counts) break;
    spinMotorSpeeds(clockwise, RAMP_MIN_SPEED); // 복구용 저속 회전
    driveLoopTick();
  }
  spinMotorSpeeds(clockwise, 0);
  delayWithTicks(10);
  setWheelSpeeds(0, 0);
}

void rotateByDegrees(float degrees, bool clockwise) {
  setWheelSpeeds(0, 0);
  delayWithTicks(40);  // 턴 직전: 직진 관성을 죽인 뒤 회전 기준 엔코더 캡처(짧으면 회전 조기 종료)

  float absDeg = degrees >= 0.0f ? degrees : -degrees;
  float compDeg = absDeg * (1.0f - SPIN_OVERSHOOT_COMP_FRAC);
  long targetCounts = spinDegToCounts(compDeg);
  if (targetCounts <= 0) return;

  long startL = prizm.readEncoderCount(1);
  long startR = prizm.readEncoderCount(2);

  bool lineTrimArmed = false;
  bool lineTrimmed = false;
  bool oppositeSeen = false;

  while (true) {
    long pos = (labs(prizm.readEncoderCount(1) - startL) + labs(prizm.readEncoderCount(2) - startR)) / 2;
    long remaining = targetCounts - pos;

    if (remaining <= 0) break;

    int fl, fc, fr;
    readFrontLineSensors(fl, fc, fr);
    bool oppositeOn = clockwise ? (fl != 0) : (fr != 0);

    if (pos >= (long)(targetCounts * SPIN_OPPOSITE_CHECK_FRAC) && oppositeOn) {
      oppositeSeen = true;
    }

    if (!lineTrimmed && pos >= (long)(targetCounts * SPIN_LINE_TRIM_MIN_FRAC)) {
      if (!oppositeOn) {
        lineTrimArmed = true;
      } else if (lineTrimArmed) {
        long newRemaining = (long)(remaining * SPIN_LINE_TRIM_REMAIN_FRAC);
        targetCounts = pos + newRemaining;
        remaining = newRemaining;
        lineTrimmed = true;
      }
    }

    // [수정] 스무스 회전 완전 제거. 일정한 최대 속도(SPIN_SPEED)로 회전
    spinMotorSpeeds(clockwise, SPIN_SPEED);
    driveLoopTick();
  }

  spinMotorSpeeds(clockwise, 0);
  delayWithTicks(10);
  setWheelSpeeds(0, 0);

  // 회전 종료 후 10ms 동안 정지 상태로 두어 관성 오버스핀이 잦아든 뒤 라인 상태를 확인한다.
  delayWithTicks(10);

  // 트리밍이 작동했거나, 회전 도중 선을 보았을 경우(불이 켜졌었다면)
  if (lineTrimmed || oppositeSeen) {
    int fl, fc, fr;
    readFrontLineSensors(fl, fc, fr);
    
    // 현재 모든 전방 센서(fl, fc, fr)가 꺼져 있다면, 선을 완전히 지나친 오버스핀이므로 즉시 복구
    if (fl == 0 && fc == 0 && fr == 0) {
      spinPlainCounts(!clockwise, spinDegToCounts(SPIN_LINE_RECOVER_DEG));
    }
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
 
// 운반 박스 미리 내리기 예약: 장애물 통과 지점부터 일정 거리(주행) 후 14cm로 부분 하강.
// 거리 측정은 주행 엔코더(g_encL/g_encR) 기준이라 driveLoopTick 이 도는 모든 주행에서 진행된다.
static bool g_carryPreLowerPending = false;
static DriveEncMark g_carryPreLowerMark = {0, 0};
static long g_carryPreLowerCounts = 0;

void scheduleCarryPreLower(float afterCm) {
  g_carryPreLowerPending = true;
  g_carryPreLowerMark = captureDriveEnc();
  g_carryPreLowerCounts = toEncoderCounts(afterCm);
}

void cancelCarryPreLower() { g_carryPreLowerPending = false; }

static void carryPreLowerTick() {
  if (!g_carryPreLowerPending) return;
  if (encoderTraveledSince(g_carryPreLowerMark) >= g_carryPreLowerCounts) {
    g_carryPreLowerPending = false;
    liftDownToStart(LIFT_CARRY_LOW_CM);
  }
}

void driveLoopTick() {
  liftUpTick();
  liftDownTick();
  carryPreLowerTick();
  updateBeep();
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
  if (absError >= 3) f *= LINE_HARD_STEER_SPEED_FACTOR;
  int speed = (int)(baseSpeed * f);
  return speed < RAMP_MIN_SPEED ? RAMP_MIN_SPEED : speed;
}

void traceLineCore(int pl, int pc, int pr, int sl, int sc, int sr, int baseSpeed,
    float kpSoft, float kpHard, float& integ, float& lastErr, int dir) {
  int posP = 0, posS = 0;
  if      (pl && !pc && !pr) { posP = -2; lineTraceLastEdge = 1; }
  else if (pl && pc && !pr)  { posP = -1; lineTraceLastEdge = 1; }
  else if (!pl && pc && !pr) { posP =  0; lineTraceLastEdge = 0; }
  else if (!pl && pc && pr)  { posP =  1; lineTraceLastEdge = 2; }
  else if (!pl && !pc && pr) { posP =  2; lineTraceLastEdge = 2; }
  else if (pl && pc && pr)   { posP =  0; lineTraceLastEdge = 0; }
  else { posP = (lineTraceLastEdge == 1) ? -3 : ((lineTraceLastEdge == 2) ? 3 : 0); }

  if      (sl && !sc && !sr) posS = -2;
  else if (sl && sc && !sr)  posS = -1;
  else if (!sl && sc && !sr) posS =  0;
  else if (!sl && sc && sr)  posS =  1;
  else if (!sl && !sc && sr) posS =  2;
  else posS = 0;

  float error = (float)posP;
  integ += error;
  float derivative = error - lastErr;
  int absError = abs((int)error);
  bool hardSteer = absError >= 2;
  float kp = hardSteer ? kpHard : kpSoft;
  int cruise = lineTraceCruiseSpeed(baseSpeed, absError);
  if (hardSteer) resetRampSpeedLimiter(cruise);
  float steer = error * kp + integ * LINE_KI + derivative * LINE_KD;

  int leftSpeed  = cruise + iround(steer);
  int rightSpeed = cruise - iround(steer);

  if ((pl || pc || pr) && (sl || sc || sr) && !(pl && pc && pr) && !(sl && sc && sr)) {
    float align = (posP - posS) * LINE_ALIGN_GAIN;
    leftSpeed  += iround(align);
    rightSpeed -= iround(align);
  }

  if (pl == 1 && pc == 0 && pr == 0 && sl == 1 && sc == 0 && sr == 0) {
    leftSpeed -= EDGE_SYNC_GAIN; rightSpeed += EDGE_SYNC_GAIN;
  } else if (pl == 0 && pc == 0 && pr == 1 && sl == 0 && sc == 0 && sr == 1) {
    leftSpeed += EDGE_SYNC_GAIN; rightSpeed -= EDGE_SYNC_GAIN;
  }

  lastErr = error;
  setWheelSpeeds(dir * leftSpeed, dir * rightSpeed);
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
     driveLoopTick();
   }
   long creepSpan = toEncoderCounts(3.0f);
   while (labs(prizm.readEncoderCount(1) - clearEnc) < creepSpan) {
     setWheelSpeeds(RAMP_MIN_SPEED, RAMP_MIN_SPEED);
     driveLoopTick();
   }
 }
 
 void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int baseSpeed,
     float kpSoft, float kpHard) {
   traceLineCore(fl, fc, fr, rl, rc, rr, baseSpeed, kpSoft, kpHard,
       lineFwdIntegral, lineFwdLastError, +1);
 }

 void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int baseSpeed) {
   traceLineForward(fl, fc, fr, rl, rc, rr, baseSpeed, LINE_KP_FWD_SOFT, LINE_KP_FWD_HARD);
 }
 
 void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr) {
   traceLineForward(fl, fc, fr, rl, rc, rr, SPEED_LINE_FOLLOW_FWD);
 }
 
 void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr, int baseSpeed) {
   traceLineCore(rl, rc, rr, fl, fc, fr, baseSpeed, LINE_KP_REV_SOFT, LINE_KP_REV_HARD,
       lineRevIntegral, lineRevLastError, -1);
 }
 
 void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr) {
   traceLineReverse(rl, rc, rr, fl, fc, fr, SPEED_LINE_FOLLOW_REV);
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

   while (true) {
     long currentEnc = labs(prizm.readEncoderCount(1));
     long traveled = currentEnc - startEnc;
 
     if (phase == 0) {
       if (currentEnc >= nextIgnoreEnc) phase = 1;
       int curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
       curSpeed = smoothRampSpeed(curSpeed);
       lastCurSpeed = curSpeed;
       setWheelSpeeds(curSpeed, curSpeed);
     }
     else if (phase == 1) {
       int fl, fc, fr;
       readFrontLineSensors(fl, fc, fr);
       if (frontOnLine(fl, fc, fr)) {
         linesPassed++;
         if (linesPassed >= lineCount) {
           if (alignSpan <= 0) {
             if (stopAtEnd) stopMotors();
             break;
           }
           phase = 2;
           alignMark = captureDriveEnc();
           speedAtLine = lastCurSpeed;
         } else {
           phase = 0;
           nextIgnoreEnc = currentEnc + toEncoderCounts(DIST_IGNORE_NODE_CM);
         }
       }
       if (phase == 1) {
         int curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
         curSpeed = smoothRampSpeed(curSpeed);
         lastCurSpeed = curSpeed;
         setWheelSpeeds(curSpeed, curSpeed);
       }
     }
 
    if (phase == 2) {
      long remaining = encoderRemaining(alignMark, alignSpan);
      long decelSpan = rampDecelSpanCounts(speedAtLine);
      if (decelSpan > alignSpan) decelSpan = alignSpan;

      int curSpeed = calcRampDownSpeed(remaining, decelSpan, speedAtLine);
      curSpeed = smoothRampSpeed(curSpeed);

      if (stopAtEnd && finishAlignSpan(alignMark, alignSpan, curSpeed)) break;
      if (!stopAtEnd && encoderTraveledSince(alignMark) >= alignSpan) break;
      if (!stopAtEnd && remaining <= 0) break;
      setWheelSpeeds(curSpeed, curSpeed);
    }
 
     driveLoopTick();
   }
}
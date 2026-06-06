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

DriveEncMark captureDriveEnc() {
  return { prizm.readEncoderCount(1), prizm.readEncoderCount(2) };
}

long encoderTraveledSince(DriveEncMark mark) {
  long dl = labs(prizm.readEncoderCount(1) - mark.left);
  long dr = labs(prizm.readEncoderCount(2) - mark.right);
  return (dl + dr) / 2;
}

int rampMarkSpeed(DriveEncMark start, int maxSpeed) {
  return calcRampUpSpeed(encoderTraveledSince(start),
      rampCmCountsAtSpeed(RAMP_ACCEL_CM, maxSpeed), maxSpeed);
}

int decelMarkSpeed(DriveEncMark mark, long totalSpan, int cruiseSpeed) {
  long remaining = totalSpan - encoderTraveledSince(mark);
  if (remaining <= 0) return RAMP_MIN_SPEED;
  long decelSpan = rampCmCountsAtSpeed(RAMP_DECEL_CM, cruiseSpeed);
  if (totalSpan <= decelSpan)
    return calcRampDownSpeed(remaining, totalSpan, cruiseSpeed);
  if (decelSpan > totalSpan / 3) decelSpan = totalSpan / 3;
  if (remaining > decelSpan) return cruiseSpeed;
  return calcRampDownSpeed(remaining, decelSpan, cruiseSpeed);
}

int crossAlignSpeed(DriveEncMark mark, long alignSpan) {
  if (alignSpan <= 0) return RAMP_MIN_SPEED;
  return calcRampDownSpeed(encoderRemaining(mark, alignSpan), alignSpan, SPEED_LINE_FOLLOW_FWD);
}

bool finishEncoderRemaining(long remainingCounts, int curSpeed) {
  if (remainingCounts <= 0) {
    stopMotors();
    return true;
  }
  if (remainingCounts <= (long)toEncoderCounts(DIST_BRAKE_CATCH_CM)) {
    if (curSpeed > RAMP_MIN_SPEED + 2) stopMotors();
    else prizm.setMotorSpeeds(0, 0);
    return true;
  }
  return false;
}

bool finishEncoderSpan(DriveEncMark mark, long targetSpan, int curSpeed) {
  return finishEncoderRemaining(encoderRemaining(mark, targetSpan), curSpeed);
}

int calcRampUpSpeed(long traveledCounts, long accelCounts, int maxSpeed) {
  if (accelCounts <= 0) return maxSpeed;
  int speed = (traveledCounts < accelCounts)
      ? RAMP_MIN_SPEED + (int)((long)(maxSpeed - RAMP_MIN_SPEED) * traveledCounts / accelCounts)
      : maxSpeed;
  return (speed < RAMP_MIN_SPEED) ? RAMP_MIN_SPEED : speed;
}

int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed) {
  if (decelCounts <= 0) return (startSpeed < RAMP_MIN_SPEED) ? RAMP_MIN_SPEED : startSpeed;
  int speed = RAMP_MIN_SPEED
      + (int)((long)(startSpeed - RAMP_MIN_SPEED) * remainingCounts / decelCounts);
  return (speed < RAMP_MIN_SPEED) ? RAMP_MIN_SPEED : speed;
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

  if (dt >= 10) {
    long encL = prizm.readEncoderCount(1);
    long encR = prizm.readEncoderCount(2);
    long curVelL = labs(encL - lastEncL);
    long curVelR = labs(encR - lastEncR);
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

// setWheelSpeeds(L,R) → prizm(-L*7, R*7) — TestMode t 명령과 동일 부호
static void spinMotorSpeeds(bool clockwise, int speed) {
  if (speed <= 0) {
    prizm.setMotorSpeeds(0, 0);
    return;
  }
  speed = constrain(speed, RAMP_MIN_SPEED, 100);
  int left  = clockwise ?  speed : -speed;
  int right = clockwise ? -speed :  speed;
  prizm.setMotorSpeeds(-left * 7, right * 7);
}

static void spinSoftStop(bool clockwise, int fromSpeed) {
  int s = fromSpeed;
  while (s > RAMP_MIN_SPEED) {
    s -= 4;
    if (s < RAMP_MIN_SPEED) s = RAMP_MIN_SPEED;
    spinMotorSpeeds(clockwise, s);
    delayWithTicks(20);
  }
  if (s > 0) {
    spinMotorSpeeds(clockwise, RAMP_MIN_SPEED);
    delayWithTicks(30);
  }
  prizm.setMotorSpeeds(0, 0);
  delayWithTicks(40);
  setWheelSpeeds(0, 0);
}

void rotateByDegrees(int degrees, bool clockwise) {
  prizm.setMotorSpeeds(0, 0);
  delayWithTicks(40);
  int absDeg = degrees >= 0 ? degrees : -degrees;
  long targetCounts = spinDegToCounts((float)absDeg);
  if (targetCounts <= 0) return;

  long startL = prizm.readEncoderCount(1);
  long startR = prizm.readEncoderCount(2);
  long accelSpan = min(spinDegToCounts(rampDegAtSpeed(RAMP_SPIN_ACCEL_DEG, SPIN_SPEED)), targetCounts / 2);
  long decelSpan = min(spinDegToCounts(rampDegAtSpeed(RAMP_SPIN_DECEL_DEG, SPIN_SPEED)), targetCounts / 2);
  int fl0, fc0, fr0;
  readFrontLineSensors(fl0, fc0, fr0);
  bool skipLineTrim = (fc0 != 0);
  bool oppositeWasOn = clockwise ? (fl0 != 0) : (fr0 != 0);
  bool lineTrimmed = false;
  int curSpeed = RAMP_MIN_SPEED;

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
        targetCounts = pos + remaining / 2;
        remaining = targetCounts - pos;
        lineTrimmed = true;
        if (remaining <= 0) break;
      }
    }

    long effDecelSpan = min(decelSpan, targetCounts - pos);
    if (effDecelSpan <= 0) effDecelSpan = 1;
    if (pos < accelSpan)
      curSpeed = calcRampUpSpeed(pos, accelSpan, SPIN_SPEED);
    else if (remaining <= effDecelSpan)
      curSpeed = calcRampDownSpeed(remaining, effDecelSpan, SPIN_SPEED);
    else
      curSpeed = SPIN_SPEED;

    spinMotorSpeeds(clockwise, curSpeed);
    liftUpTick(); liftDownTick();
  }
  spinSoftStop(clockwise, curSpeed);
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

bool frontOnLine(int left, int center, int right) { return left || center || right; }
bool rearOnLine(int left, int center, int right) { return left || center || right; }

namespace {
float lineFwdIntegral = 0;
float lineFwdLastError = 0;
float lineRevIntegral = 0;
float lineRevLastError = 0;
} // namespace

void resetLineTracePid() {
  lineFwdIntegral = 0;
  lineFwdLastError = 0;
  lineRevIntegral = 0;
  lineRevLastError = 0;
}

void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int baseSpeed) {
  int leftSpeed  = baseSpeed;
  int rightSpeed = baseSpeed;
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
  float kp = (abs((int)error) >= 2) ? LINE_KP_FWD_HARD : LINE_KP_FWD_SOFT;
  float steer = error * kp + lineFwdIntegral * LINE_KI + derivative * LINE_KD;

  leftSpeed  += (int)iround(steer);
  rightSpeed -= (int)iround(steer);

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

void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr) {
  traceLineForward(fl, fc, fr, rl, rc, rr, SPEED_LINE_FOLLOW_FWD);
}

void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr, int baseSpeed) {
  int leftSpeed  = -baseSpeed;
  int rightSpeed = -baseSpeed;
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
  float kp = (abs((int)error) >= 2) ? LINE_KP_REV_HARD : LINE_KP_REV_SOFT;
  float steer = error * kp + lineRevIntegral * LINE_KI + derivative * LINE_KD;

  leftSpeed  -= (int)iround(steer);
  rightSpeed += (int)iround(steer);

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

void driveDistanceCm(float cm, int speed, bool stopAtEnd) {
  float absCm = cm >= 0.0f ? cm : -cm;
  if (absCm <= 0.0f) { if (stopAtEnd) stopMotors(); return; }

  int maxSpeed = abs(speed);
  int direction = (speed >= 0) ? 1 : -1;
  if (cm < 0) direction = -direction;

  DriveEncMark startEnc = captureDriveEnc();
  long targetCounts = toEncoderCounts(absCm);
  long accelSpan  = min((long)rampCmCountsAtSpeed(RAMP_ACCEL_CM, maxSpeed), targetCounts / 2);
  long decelSpan  = stopAtEnd ? min((long)rampCmCountsAtSpeed(RAMP_DECEL_CM, maxSpeed), targetCounts / 2) : 0;

  while (true) {
    long traveled = encoderTraveledSince(startEnc);
    long remaining = targetCounts - traveled;

    int curSpeed;
    if (traveled < accelSpan) {
      curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
    } else if (stopAtEnd && remaining < decelSpan) {
      curSpeed = calcRampDownSpeed(remaining, decelSpan, maxSpeed);
    } else {
      curSpeed = maxSpeed;
    }

    if (stopAtEnd) {
      if (finishEncoderRemaining(remaining, curSpeed)) break;
    } else if (remaining <= 0) {
      break;
    }

    setWheelSpeeds(direction * curSpeed, direction * curSpeed);
    liftUpTick(); liftDownTick(); pollZoneScan();
  }
}

void driveOverLinesAndAlign(int lineCount, float alignCm, int speed, bool stopAtEnd) {
  long startEnc = labs(prizm.readEncoderCount(1));
  long nextIgnoreEnc = startEnc + toEncoderCounts(DIST_IGNORE_NODE_CM);
  int linesPassed = 0;
  int phase = 0; // 0=무시구간 1=라인탐색 2=정렬

  int maxSpeed = abs(speed);
  long accelSpan = rampCmCountsAtSpeed(RAMP_ACCEL_CM, maxSpeed);
  long alignSpan = toEncoderCounts(alignCm);
  DriveEncMark alignMark = {0, 0};
  int speedAtLine = maxSpeed;

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long traveled = currentEnc - startEnc;

    if (phase == 0) {
      if (currentEnc >= nextIgnoreEnc) phase = 1;
      int curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
      setWheelSpeeds(curSpeed, curSpeed);
    }
    else if (phase == 1) {
      int fl, fc, fr;
      readFrontLineSensors(fl, fc, fr);
      if (frontOnLine(fl, fc, fr)) {
        linesPassed++;
        if (linesPassed >= lineCount) {
          phase = 2;
          alignMark = captureDriveEnc();
          speedAtLine = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
          if (alignSpan <= 0) {
            if (stopAtEnd) stopMotors();
            break;
          }
        } else {
          phase = 0;
          nextIgnoreEnc = currentEnc + toEncoderCounts(DIST_IGNORE_NODE_CM);
        }
      }
      if (phase == 1) {
        int curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
        setWheelSpeeds(curSpeed, curSpeed);
      }
    }

    if (phase == 2) {
      long remaining = encoderRemaining(alignMark, alignSpan);
      int curSpeed = calcRampDownSpeed(remaining, alignSpan, speedAtLine);
      if (stopAtEnd) {
        if (finishEncoderRemaining(remaining, curSpeed)) break;
      } else if (remaining <= 0) {
        break;
      }
      int fl, fc, fr, rl, rc, rr;
      readFrontLineSensors(fl, fc, fr);
      readRearLineSensors(rl, rc, rr);
      traceLineForward(fl, fc, fr, rl, rc, rr, curSpeed);
    }

    liftUpTick(); liftDownTick(); pollZoneScan();
  }
}

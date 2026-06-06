/* ============================================================
 * Motion.cpp - 듀얼 PID 조향 및 가감속 거리 주행
 * ============================================================ */
#include "Motion.h"
#include "Config.h"
#include "Lift.h"
#include "BoxMap.h"
#include <math.h>

bool enableTeeZoneSteering = false;

DriveEncMark captureDriveEnc() {
  return { prizm.readEncoderCount(1), prizm.readEncoderCount(2) };
}

long encoderTraveledSince(DriveEncMark mark) {
  long dl = labs(prizm.readEncoderCount(1) - mark.left);
  long dr = labs(prizm.readEncoderCount(2) - mark.right);
  return (dl + dr) / 2;
}

long rampAccelSpanCounts() { return toEncoderCounts(RAMP_ACCEL_CM); }

int blindRampSpeed(DriveEncMark start, int maxSpeed) {
  return calcRampUpSpeed(encoderTraveledSince(start), rampAccelSpanCounts(), maxSpeed);
}

int blindDecelSpeed(DriveEncMark mark, long totalSpan, int cruiseSpeed) {
  long remaining = totalSpan - encoderTraveledSince(mark);
  if (remaining <= 0) return 0;
  long decelSpan = (long)toEncoderCounts(RAMP_DECEL_CM);
  if (decelSpan > totalSpan / 2) decelSpan = totalSpan / 2;
  if (remaining > decelSpan) return cruiseSpeed;
  return (int)((long)cruiseSpeed * remaining / decelSpan);
}

int calcRampUpSpeed(long traveledCounts, long accelCounts, int maxSpeed) {
  if (accelCounts <= 0) return maxSpeed;
  int speed = (traveledCounts < accelCounts)
      ? RAMP_MIN_SPEED + (int)((long)(maxSpeed - RAMP_MIN_SPEED) * traveledCounts / accelCounts)
      : maxSpeed;
  return (speed < 10) ? 10 : speed;
}

int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed) {
  if (decelCounts <= 0) return (startSpeed < 10) ? 10 : startSpeed;
  int speed = 10 + (int)((long)(startSpeed - 10) * remainingCounts / decelCounts);
  return (speed < 10) ? 10 : speed;
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

void rotateByDegrees(int degrees, bool clockwise) {
  stopMotors();
  delayWithTicks(40);
  float compDeg = fabs((float)degrees) - 1.0f;
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0f) * compDeg);
  if (targetCounts <= 0) return;

  long startL = prizm.readEncoderCount(1);
  long startR = prizm.readEncoderCount(2);
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1) - startL) + labs(prizm.readEncoderCount(2) - startR)) / 2;
    if (targetCounts - pos <= 0) break;
    if (clockwise) setWheelSpeeds(SPIN_SPEED, -SPIN_SPEED);
    else           setWheelSpeeds(-SPIN_SPEED, SPIN_SPEED);
    liftUpTick(); liftDownTick();
  }
  stopMotors();
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

  leftSpeed  += (int)round(steer);
  rightSpeed -= (int)round(steer);

  if ((fl || fc || fr) && (rl || rc || rr) && !(fl && fc && fr) && !(rl && rc && rr)) {
    float align = (posFront - posRear) * LINE_ALIGN_GAIN;
    leftSpeed  += (int)round(align);
    rightSpeed -= (int)round(align);
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

  leftSpeed  -= (int)round(steer);
  rightSpeed += (int)round(steer);

  if ((rl || rc || rr) && (fl || fc || fr) && !(rl && rc && rr) && !(fl && fc && fr)) {
    float align = (posRear - posFront) * LINE_ALIGN_GAIN;
    leftSpeed  -= (int)round(align);
    rightSpeed += (int)round(align);
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
  float absCm = fabs(cm);
  if (absCm <= 0.0f) { if (stopAtEnd) stopMotors(); return; }

  int maxSpeed = abs(speed);
  int direction = (speed >= 0) ? 1 : -1;
  if (cm < 0) direction = -direction;

  DriveEncMark startEnc = captureDriveEnc();
  long targetCounts = toEncoderCounts(absCm);
  long accelSpan  = min((long)toEncoderCounts(RAMP_ACCEL_CM), targetCounts / 2);
  long decelSpan  = stopAtEnd ? min((long)toEncoderCounts(RAMP_DECEL_CM), targetCounts / 2) : 0;

  while (true) {
    long traveled = encoderTraveledSince(startEnc);
    long remaining = targetCounts - traveled;
    if (remaining <= 0) break;

    int curSpeed;
    if (traveled < accelSpan) {
      curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
    } else if (stopAtEnd && remaining < decelSpan) {
      curSpeed = calcRampDownSpeed(remaining, decelSpan, maxSpeed);
    } else {
      curSpeed = maxSpeed;
    }

    setWheelSpeeds(direction * curSpeed, direction * curSpeed);
    liftUpTick(); liftDownTick(); pollZoneScan();
  }
  if (stopAtEnd) stopMotors();
}

void driveOverLinesAndAlign(int lineCount, float alignCm, int speed, bool stopAtEnd) {
  long startEnc = labs(prizm.readEncoderCount(1));
  long nextIgnoreEnc = startEnc + toEncoderCounts(DIST_IGNORE_NODE_CM);
  int linesPassed = 0;
  int phase = 0; // 0=무시구간 1=라인탐색 2=정렬

  int maxSpeed = abs(speed);
  long accelSpan = toEncoderCounts(RAMP_ACCEL_CM);
  long alignSpan = toEncoderCounts(alignCm);
  long alignTargetEnc = 0;
  int speedAtLastLine = maxSpeed;

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long traveled = currentEnc - startEnc;
    int curSpeed;

    if (phase == 0) {
      if (currentEnc >= nextIgnoreEnc) phase = 1;
      curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
      setWheelSpeeds(curSpeed, curSpeed);
    }
    else if (phase == 1) {
      int fl, fc, fr;
      readFrontLineSensors(fl, fc, fr);
      if (frontOnLine(fl, fc, fr)) {
        linesPassed++;
        if (linesPassed >= lineCount) {
          phase = 2;
          alignTargetEnc = currentEnc + alignSpan;
          speedAtLastLine = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
          if (alignSpan <= 0 && !stopAtEnd) break;
        } else {
          phase = 0;
          nextIgnoreEnc = currentEnc + toEncoderCounts(DIST_IGNORE_NODE_CM);
        }
      }
      curSpeed = calcRampUpSpeed(traveled, accelSpan, maxSpeed);
      setWheelSpeeds(curSpeed, curSpeed);
    }
    else {
      long remaining = alignTargetEnc - currentEnc;
      if (remaining <= 0) break;
      curSpeed = stopAtEnd
          ? calcRampDownSpeed(remaining, alignSpan, speedAtLastLine)
          : speedAtLastLine;
      setWheelSpeeds(curSpeed, curSpeed);
    }
    liftUpTick(); liftDownTick(); pollZoneScan();
  }
  if (stopAtEnd) stopMotors();
}

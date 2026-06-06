/* ============================================================
 * Motion.h - 모터·센서·라인 추종·거리 주행 API
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

extern bool enableTeeZoneSteering; // T자 구역(1·3) 탈출 시 엣지 동기화 조향

void delayWithTicks(unsigned long ms);
void playBeep(unsigned long ms);
void setWheelSpeeds(int left, int right);
void stopMotors();
void rotateByDegrees(int degrees, bool clockwise);

void readFrontLineSensors(int& left, int& center, int& right);
void readRearLineSensors(int& left, int& center, int& right);
void readLineSensors(int& fl, int& fc, int& fr, int& rl, int& rc, int& rr);
bool frontOnLine(int left, int center, int right);
bool rearOnLine(int left, int center, int right);
inline bool frontCrossFull(int left, int center, int right) {
  return left == 1 && center == 1 && right == 1;
}

void driveLoopTick(); // lift tick + (QR 스캔 중일 때만) pollZoneScan

void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr);
void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int speed);
void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr);
void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr, int speed);
void resetLineTracePid();
void clearIntersectionCross(); // 111 교차 패턴 저속 이탈 + 3cm 크립

void driveDistanceCm(float cm, int speed, bool stopAtEnd = true);
void driveOverLinesAndAlign(int lineCount, float alignCm, int speed, bool stopAtEnd = true);

struct DriveEncMark { long left; long right; };
void correctTrackLegOvershoot(DriveEncMark legStart, float plannedSpanCm);
// planned 대비 초과→후진 / 부족→전진, 각 최대 DIST_TRACK_OVERSHOOT_MAX_CM

// 선형 가속 RAMP_MIN→max, 구간=rampAccelSpanCounts(cruise)
int calcRampUpSpeed(long traveledCounts, long accelCounts, int maxSpeed);
// 선형 감속 start→RAMP_MIN, 구간=rampDecelSpanCounts(cruise)
int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed);

void resetRampSpeedLimiter(int initialSpeed);
int smoothRampSpeed(int targetSpeed); // 상승만 RAMP_MAX_SPEED_STEP 제한

DriveEncMark captureDriveEnc();
long encoderTraveledSince(DriveEncMark mark);
// 정지 목표 도달 시 stopMotors(125) — finishEncoderRemaining / finishEncoderSpan
bool finishEncoderRemaining(long remainingCounts, int curSpeed);
bool finishEncoderSpan(DriveEncMark mark, long targetSpan, int curSpeed);
// 바퀴축 정렬 — 목표 거리 초과 시 즉시 stopMotors(125)
bool finishAlignSpan(DriveEncMark alignStart, long alignSpanCounts);
int rampMarkSpeed(DriveEncMark start, int maxSpeed);
int decelMarkSpeed(DriveEncMark mark, long totalSpan, int cruiseSpeed);
int crossAlignSpeed(DriveEncMark mark, long alignSpan, int cruiseSpeed);

#endif

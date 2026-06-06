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
bool frontOnLine(int left, int center, int right);
bool rearOnLine(int left, int center, int right);

void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr);
void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int speed);
void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr);
void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr, int speed);
void resetLineTracePid();

void driveDistanceCm(float cm, int speed, bool stopAtEnd = true);
void driveOverLinesAndAlign(int lineCount, float alignCm, int speed, bool stopAtEnd = true);

// 선형 가속 RAMP_MIN→max, 구간=rampAccelSpanCounts(cruise)
int calcRampUpSpeed(long traveledCounts, long accelCounts, int maxSpeed);
// 선형 감속 start→RAMP_MIN, 구간=rampDecelSpanCounts(cruise)
int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed);

struct DriveEncMark { long left; long right; };
DriveEncMark captureDriveEnc();
long encoderTraveledSince(DriveEncMark mark);
bool finishEncoderRemaining(long remainingCounts, int curSpeed);
bool finishEncoderSpan(DriveEncMark mark, long targetSpan, int curSpeed);
int rampMarkSpeed(DriveEncMark start, int maxSpeed);
int decelMarkSpeed(DriveEncMark mark, long totalSpan, int cruiseSpeed);
int crossAlignSpeed(DriveEncMark mark, long alignSpan, int cruiseSpeed);

#endif

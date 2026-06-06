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

int calcRampUpSpeed(long traveledCounts, long accelCounts, int maxSpeed);
int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed);

struct DriveEncMark { long left; long right; };
DriveEncMark captureDriveEnc();
long encoderTraveledSince(DriveEncMark mark);
long rampAccelSpanCounts();
int blindRampSpeed(DriveEncMark start, int maxSpeed);
int blindDecelSpeed(DriveEncMark mark, long totalSpan, int cruiseSpeed);

#endif

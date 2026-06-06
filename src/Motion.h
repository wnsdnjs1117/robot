/* ============================================================
 * Motion.h - 모터·센서·라인 추종·거리 주행 API
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

extern bool enableTeeZoneSteering;

void delayWithTicks(unsigned long ms);
void playBeep(unsigned long ms);
void startBeep(unsigned long ms);
void setWheelSpeeds(int left, int right);
void stopMotors();
void rotateByDegrees(float degrees, bool clockwise); // 변경됨: float

void readFrontLineSensors(int& left, int& center, int& right);
void readRearLineSensors(int& left, int& center, int& right);
void readLineSensors(int& fl, int& fc, int& fr, int& rl, int& rc, int& rr);
bool frontOnLine(int left, int center, int right);
bool rearOnLine(int left, int center, int right);
inline bool frontCrossFull(int left, int center, int right) {
  return left == 1 && center == 1 && right == 1;
}

void driveLoopTick(); 

void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr);
void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int speed);
void traceLineForward(int fl, int fc, int fr, int rl, int rc, int rr, int speed,
    float kpSoft, float kpHard);
void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr);
void traceLineReverse(int rl, int rc, int rr, int fl, int fc, int fr, int speed);
void resetLineTracePid();
void clearIntersectionCross();

void driveDistanceCm(float cm, int speed, bool stopAtEnd = true);
void driveOverLinesAndAlign(int lineCount, float alignCm, int speed, bool stopAtEnd = true);

struct DriveEncMark { long left; long right; };
void correctTrackLegOvershoot(DriveEncMark legStart, float plannedSpanCm);

int calcRampUpSpeed(long traveledCounts, long accelCounts, int maxSpeed);
int calcRampDownSpeed(long remainingCounts, long decelCounts, int startSpeed);

void resetRampSpeedLimiter(int initialSpeed);
int smoothRampSpeed(int targetSpeed); 

DriveEncMark captureDriveEnc();
long encoderTraveledSince(DriveEncMark mark);
bool finishEncoderRemaining(long remainingCounts, int curSpeed);
bool finishEncoderSpan(DriveEncMark mark, long targetSpan, int curSpeed);
bool finishAlignSpan(DriveEncMark alignStart, long alignSpanCounts, int curSpeed);
int rampMarkSpeed(DriveEncMark start, int maxSpeed);
int decelMarkSpeed(DriveEncMark mark, long totalSpan, int cruiseSpeed);
int crossAlignSpeed(DriveEncMark mark, long alignSpan, int cruiseSpeed);

#endif
/* ============================================================
 * Motion.h - 하드웨어 구동 및 제어 인터페이스 (가감속 지원 추가)
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

extern bool enableEdgeSteering;

void safeDelay(unsigned long ms);
void beep(unsigned long ms);
void drive(int l, int r);
void stopAll();
void turnAngle(int degrees, bool isRight);

void readSensors(int& L, int& C, int& R);
void readRearSensors(int& RL, int& RC, int& RR);
bool anyLine(int L, int C, int R);
bool anyRearLine(int RL, int RC, int RR);

// 전진 시 전후방 동기화 라인트레이싱 (기본 속도 / 지정 속도)
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR);
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR, int baseSpeed);

// 후진 시 전후방 동기화 라인트레이싱 (기본 속도 / 지정 속도)
void reverseLineFollowStep(int RL, int RC, int RR, int FL, int FC, int FR);
void reverseLineFollowStep(int RL, int RC, int RR, int FL, int FC, int FR, int baseSpeed);

// ============================================================
// ★ 고정 거리 이동 (칼각: 일정 속도 → 즉시 정지, 이동/남은 거리 cm 출력)
// ============================================================
void driveDistance(float cm, int speed);              // 일정 속도 직진/후진(음수=후진) 후 즉시 정지
void driveStraightSmooth(float cm, int maxSpd);       // (호환 래퍼) driveDistance 호출
void driveExtraDecel(float cm, int startSpd);         // (호환 래퍼) driveDistance 호출

#endif
/* ============================================================
 * Motion.h - 하드웨어 구동 및 제어 인터페이스 (가감속 지원 추가)
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

extern bool enableEdgeSteering;

void safeDelay(unsigned long ms);
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
// ★ S-커브 가감속 이동 함수들 (부드러운 출발 및 정지)
// ============================================================
void driveStraightSmooth(float cm, int maxSpd);       // 센서 없이 직진 (가감속)
void lineFollowSmooth(float cm, int maxSpd);          // 라인 따라 전진 (가감속)
void reverseLineFollowSmooth(float cm, int maxSpd);   // 라인 따라 후진 (가감속)
void driveExtraDecel(float cm, int startSpd);         // ★ 블라인드 탐지 후 바퀴 축까지 부드럽게 감속 정지

#endif
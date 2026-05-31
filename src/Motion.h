/* ============================================================
 * Motion.h - 하드웨어 구동 및 제어 인터페이스
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

// 전진 시 전후방 동기화 라인트레이싱
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR);

// 후진 시 전후방 동기화 라인트레이싱 (파라미터 6개로 확장됨)
void reverseLineFollowStep(int RL, int RC, int RR, int FL, int FC, int FR);

#endif
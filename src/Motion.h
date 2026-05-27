/* ============================================================
 * Motion.h - 저수준 모터 및 센서 제어 함수 선언
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

// ── [1] 모터 기본 제어 ────────────────────────────────────────
void drive(int l, int r);
void stopAll();
void reverseStraight(int counts);

// ── [2] 회전 제어 ────────────────────────────────────────────
void turnAngle(int degrees, bool isRight);  // 사다리꼴 프로파일 + 관성 보정

// ── [3] 센서 읽기 ────────────────────────────────────────────
void readSensors(int& L, int& C, int& R);
void readRearSensors(int& RL, int& RC, int& RR);
bool anyLine(int L, int C, int R);
bool anyRearLine(int RL, int RC, int RR);

// ── [4] 라인 트레이싱 ────────────────────────────────────────
void lineFollowStep(int L, int C, int R);
void lineFollowStepReverse(int L, int C, int R);
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR);

// ── [5] 교차로 감지 ──────────────────────────────────────────
bool detectCrossing(int L, int C, int R);

#endif

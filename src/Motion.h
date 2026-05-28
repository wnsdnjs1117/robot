/* ============================================================
 * Motion.h - 저수준 모터 및 센서 제어 함수 선언
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

void drive(int l, int r);
void stopAll();
void softStop();

void turnAngle(int degrees, bool isRight);  // 사다리꼴 프로파일 + 관성 보정
bool turnToLine(bool isRight, int maxDegrees);  // 회전 방향의 새 라인에 정렬 정지

void readSensors(int& L, int& C, int& R);
void readRearSensors(int& RL, int& RC, int& RR);
bool anyLine(int L, int C, int R);
bool anyRearLine(int RL, int RC, int RR);

void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR);

bool detectCrossing(int L, int C, int R);

#endif

/* ============================================================
 * Motion.h - 저수준 모터 및 센서 제어 함수 선언
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

void drive(int l, int r);
void stopAll();
void turnAngle(int degrees, bool isRight);
void reverseStraight(int counts);
void readSensors(int& L, int& C, int& R);
bool anyLine(int L, int C, int R);
void lineFollowStep(int L, int C, int R);
void lineFollowStepReverse(int L, int C, int R);
bool detectCrossing(int L, int C, int R);

// 후방 센서 (아날로그 A1/A2/A3)
void readRearSensors(int& RL, int& RC, int& RR);
bool anyRearLine(int RL, int RC, int RR);

// 전/후방 이중 센서 라인트레이싱 (전방 주도 + 후방 각도 교정)
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR);

// 블라인드 구간 출발 전 후방 센서로 라인 수직 정렬
void alignHeadingOnLine();

#endif
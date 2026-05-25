/* ============================================================
 * Motion.h - 저수준 모터 및 센서 제어 함수 선언
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

void drive(int l, int r);
void stopAll();
void turnAngle(int degrees, bool isRight);  // 동적 각도 회전 함수
void reverseStraight(int counts);
void readSensors(int& L, int& C, int& R);
bool anyLine(int L, int C, int R);
void lineFollowStep(int L, int C, int R);
void lineFollowStepReverse(int L, int C, int R);
bool detectCrossing(int L, int C, int R);

#endif
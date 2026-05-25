/* ============================================================
 * Motion.h - 저수준 제어 함수 선언
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

void drive(int l, int r);
void stopAll();
void spinLeft90();
void spinRight90();
void reverseStraight(int counts);
void readSensors(int& L, int& C, int& R);
bool anyLine(int L, int C, int R);
void lineFollowStep(int L, int C, int R);
void lineFollowStepReverse(int L, int C, int R);
bool detectCrossing(int L, int C, int R);

#endif

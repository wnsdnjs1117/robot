/* ============================================================
 * Lift.h - 듀얼 리프트 높이 기반 제어 헤더
 * ============================================================ */
#ifndef LIFT_H
#define LIFT_H

#include <Arduino.h>
#include <PRIZM.h>

void liftUp();
void liftUpTick();
void liftUpWait();
void liftDown();
void liftDownStart();
void liftDownTick();
void liftDownWait();
void liftDownUntilClear();

extern EXPANSION exc;
extern const int EXP_ID;
extern const int LIFT_L;
extern const int LIFT_R;
extern float heightL;
extern float heightR;

#endif

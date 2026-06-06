/* ============================================================
 * Lift.h - 듀얼 리프트 높이 기반 제어 헤더
 * ============================================================ */
#ifndef LIFT_H
#define LIFT_H

#include <Arduino.h>
#include <PRIZM.h>

void liftUpStart(float targetCm);
bool liftUpClearReached();
void liftUpWaitClear();
void liftUpTick();
void liftDownStart();
void liftDownToStart(float targetCm);  // 바닥까지가 아닌 중간 높이까지만 하강(이동 중 미리 내리기)
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

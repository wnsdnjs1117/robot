/* ============================================================
 * Lift.h - 연장 보드 듀얼 리프트 저항 감지 제어 헤더
 * ============================================================ */
#ifndef LIFT_H
#define LIFT_H

#include <Arduino.h>
#include <PRIZM.h>   // EXPANSION 타입 선언을 위해 필요

// 미션 연동 리프트 함수
void liftUp();
void liftDown();
void liftDownStart();
void liftDownTick();
void liftDownWait();

// EXPANSION 컨트롤러 외부 접근용
extern EXPANSION exc;
extern const int EXP_ID;
extern const int LIFT_L;
extern const int LIFT_R;

#endif
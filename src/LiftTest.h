/* ============================================================
 * LiftTest.h - 연장 보드 듀얼 리프트 저항 감지 제어 헤더
 * ============================================================ */
#ifndef LIFT_TEST_H
#define LIFT_TEST_H

// 리프트 제어 메인 테스트 함수
void runLiftStallTest();

// ★ 미션 연동 리프트 함수
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
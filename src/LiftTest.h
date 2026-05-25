/* ============================================================
 * LiftTest.h - 연장 보드 듀얼 리프트 저항 감지 제어 헤더
 * ============================================================ */
#ifndef LIFT_TEST_H
#define LIFT_TEST_H

// 리프트 제어 메인 테스트 함수
void runLiftStallTest();

// ★ 미션 연동 리프트 함수
// liftUp()        : 리프트를 24cm까지 올림. 15cm 이상이 되어야 함수가 반환됨 (블로킹)
// liftDown()      : 리프트를 바닥(0cm)까지 내림. 10cm 이하가 되어야 함수가 반환됨 (블로킹)
// liftDownStart() : 하강 시작만 하고 즉시 반환 (논블로킹 — 주행과 동시 사용)
// liftDownTick()  : 하강 중 매 루프마다 호출하여 모터 제어 (논블로킹)
// liftDownWait()  : 하강이 완전히 끝날 때까지 블로킹 대기
void liftUp();
void liftDown();
void liftDownStart();
void liftDownTick();
void liftDownWait();

#endif
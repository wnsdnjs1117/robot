/* ============================================================
 * LiftTest.h - 연장 보드 듀얼 리프트 저항 감지 제어 헤더
 * ============================================================ */
#ifndef LIFT_TEST_H
#define LIFT_TEST_H

#include <Arduino.h>
#include <PRIZM.h>

// 리프트 제어 메인 테스트 함수
void runLiftStallTest();

// ── 블로킹 API (주행 전 대기) ────────────────────────────────
// liftUp()   : 양쪽 15cm 이상 오르면 반환 — 나머지는 비동기로 24cm까지 완료
// liftDown() : 양쪽 10cm 이하로 내려오면 반환 — 나머지는 비동기로 0cm까지 완료
void liftUp();
void liftDown();

// ── 논블로킹 상승 API ────────────────────────────────────────
void liftUpStart();   // 상승 시작 (즉시 반환)
void liftUpTick();    // 매 루프 틱: 모터 제어 1사이클 수행
void liftUpWait();    // 24cm 도달까지 블로킹 대기

// ── 논블로킹 하강 API ────────────────────────────────────────
void liftDownStart();  // 하강 시작 (즉시 반환)
void liftDownTick();   // 매 루프 틱: 모터 제어 1사이클 수행
void liftDownWait();   // 0cm 착지까지 블로킹 대기

// ── 통합 틱 (이동 루프에서 호출) ─────────────────────────────
// 상승/하강 중인 경우 해당 틱을 1회 실행
// followToCrossing(), exitZone() 등 이동 루프 내에서 사용
void liftActiveTick();

// EXPANSION 컨트롤러 외부 접근용
extern EXPANSION exc;
extern const int EXP_ID;
extern const int LIFT_L;
extern const int LIFT_R;

#endif

/* ============================================================
 * Navigation.h - 고수준 주행 시나리오 제어 헤더
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

// ── [1] 라인 추종 → 교차로 ───────────────────────────────────
void followToCrossing();
void forwardToCrossing();

// ── [2] 블라인드 구간 출발 정렬 ──────────────────────────────
void alignHeadingOnLine();  // 후방 센서로 라인 수직 정렬 (최대 5도 보정)

// ── [3] 존 진입/탈출 ─────────────────────────────────────────
void enterZone();
void reverseEnterZone();
void reverseAcrossToOppositeZone();

// ── [4] 특수 경로 ────────────────────────────────────────────
void goToMainLine();
void returnToFinish();
int  qrSearchStage();  // 탐색 완료 구역 번호(1~4) 반환

#endif

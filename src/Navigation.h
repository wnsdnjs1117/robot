/* ============================================================
 * Navigation.h - 고수준 주행 시나리오 제어 헤더
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

void followToCrossing();
void forwardToCrossing();
void reverseAcrossToOppositeZone();
void enterZone();
void goToMainLine();

// ★ void에서 int로 변경! (구역 번호를 반환하기 위함)
int qrSearchStage();

#endif
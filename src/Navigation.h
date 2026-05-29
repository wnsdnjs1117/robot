/* ============================================================
 * Navigation.h - 고수준 주행 시나리오 제어 헤더
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

void followToCrossing();
void followToCrossing(bool stopAtEnd);  // ★ 누락되었던 오버로드 선언 추가

void enterZone();
void reverseEnterZone();
void reverseAcrossToOppositeZone();

void goToMainLine();
void returnToFinish();
int qrSearchStage();

#endif
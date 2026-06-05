/* ============================================================
 * Navigation.h - 고수준 주행 시나리오 제어 헤더
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

void followToCrossing();
void followToCrossing(bool stopAtEnd);  // ★ 누락되었던 오버로드 선언 추가

void enterZone(int zone);
void reverseEnterZone(int zone);
void reverseAcrossToOppositeZone(int zone);

void goToMainLine();
void returnToFinish();
int qrSearchStage();
void rescanZones1to4();   // 1~4구역 박스 2개를 모두 인식할 때까지 미발견 존 재스캔

#endif
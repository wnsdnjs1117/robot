/* ============================================================
 * Navigation.h - 고수준 주행 시나리오 제어 헤더
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

void followToCrossing();
void followToCrossing(bool stopAtEnd);

void enterZone(int zone);
void reverseEnterZone(int zone);
// ★ 불필요해진 reverseAcrossToOppositeZone 삭제로 메모리 절약

void goToMainLine();
void returnToFinish();
int qrSearchStage();
void rescanZones1to4();

#endif
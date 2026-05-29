/* ============================================================
 * Navigation.h - 고수준 주행 시나리오 제어 헤더
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

void followToCrossing();

void alignHeadingOnLine();  // 후방 센서로 라인 수직 정렬 (최대 5도 보정)

void enterZone();
void reverseEnterZone();
void reverseAcrossToOppositeZone();

void goToMainLine();
void returnToFinish();
int  qrSearchStage();  // 탐색 완료 구역 번호(1~4) 반환

#endif

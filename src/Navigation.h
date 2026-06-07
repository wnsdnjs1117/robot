/* ============================================================
 * Navigation.h - 고수준 주행 시나리오
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Motion.h"  // DriveEncMark

void traceUntilIntersection();
void traceUntilIntersection(bool stopAtEnd);
void traceUntilIntersection(bool stopAtEnd, int cruiseSpeed);
// legSpanCm 이 주어지면 legStart 부터의 주행 거리를 기준으로 교차로 도달 전
// 미리 감속한다(비대칭 구간 9->8 대응). legSpanCm <= 0 이면 감속 없음.
void traceUntilIntersection(bool stopAtEnd, int cruiseSpeed,
    DriveEncMark legStart, float legSpanCm);

void enterZoneForward(int zone);
void enterZoneReverse(int zone);
void crossToOppositeZone(int targetZone, int fromZone, bool enableScan = true);

void driveOntoMainTrack();
void driveToFinishArea();
int  searchQrInZones1to4();
int  rescanMissingQrZones1to4();  // 마지막으로 박스를 인식한 1~4존(머무는 중)을 반환, 없으면 0
int  countScannedBoxesInZones1to4();

#endif

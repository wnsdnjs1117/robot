/* ============================================================
 * Navigation.h - 고수준 주행 시나리오
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

void traceUntilIntersection();
void traceUntilIntersection(bool stopAtEnd);
void traceUntilIntersection(bool stopAtEnd, int cruiseSpeed);

void enterZoneForward(int zone);
void enterZoneReverse(int zone);
void crossToOppositeZone(int targetZone, int fromZone, bool enableScan = true);

void alignOnTrackHeading(int openSpeed, float alignCm);
void driveOntoMainTrack();
void driveToFinishArea();
int  searchQrInZones1to4();
int  rescanMissingQrZones1to4();  // 마지막으로 박스를 인식한 1~4존(머무는 중)을 반환, 없으면 0
int  countScannedBoxesInZones1to4();

#endif

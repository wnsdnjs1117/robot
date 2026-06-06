/* ============================================================
 * Navigation.h - 고수준 주행 시나리오
 * ============================================================ */
#ifndef NAVIGATION_H
#define NAVIGATION_H

void traceUntilIntersection();
void traceUntilIntersection(bool stopAtEnd);

void enterZoneForward(int zone);
void enterZoneReverse(int zone);
void crossToOppositeZone(int targetZone, int fromZone, bool enableScan = true);

void alignOnTrackHeading(int openSpeed, float alignCm);
void driveOntoMainTrack();
void driveToFinishArea();
int  searchQrInZones1to4();
void rescanMissingQrZones1to4();

#endif

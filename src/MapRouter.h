/* ============================================================
 * MapRouter.h - 교차로 노드 기반 경로 이동
 * ============================================================ */
#ifndef MAP_ROUTER_H
#define MAP_ROUTER_H

#include <Arduino.h>

extern float headingDeg;          // 현재 방위각 (0=북, 90=동, 180=남, 270=서)
extern int   intersectionNode;    // 현재 교차로 노드 (7~11)
extern bool  enteredZoneForward;  // 마지막 구역 진입이 전진이었는지
extern bool  finishFromZone6Exit; // 6번 탈출 직후 11번 — 피니시 전용 경로

void rotateToHeading(float targetDeg); // 변경됨: float
void driveTrackLegBlind(float targetHeading, float alignHeading, bool stopAtEnd,
    float legSpanCm, int lineCount = 1, bool anyFrontLine = false); // 변경됨: float
void driveToIntersectionNode(int targetNode);

struct ZoneMoveOptions {
  bool scanQr;
  bool alreadyInFromZone;
};

inline ZoneMoveOptions zoneMoveOpts(bool scanQr = false, bool alreadyInFromZone = false) {
  ZoneMoveOptions o;
  o.scanQr = scanQr;
  o.alreadyInFromZone = alreadyInFromZone;
  return o;
}

void enterZoneAt(int zone);
void moveBetweenZones(int fromZone, int toZone, ZoneMoveOptions opts);
void leaveZone(int zone);
void navigateToZone(int zone);

#endif
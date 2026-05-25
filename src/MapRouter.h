/* ============================================================
 * MapRouter.h - 다이나믹 방위 제어 기반 격자 맵 라우터
 * ============================================================ */
#ifndef MAP_ROUTER_H
#define MAP_ROUTER_H

#include <Arduino.h>

void executeBlindRun();

extern int  robotHeading;
extern int  currentNode;        // 현재 로봇이 서 있는 교차로 노드 (7,8,9,10,11)
extern bool lastEntryWasForward; // 마지막 구역 진입이 전진이었는가 (exitZone 탈출 방식 결정)

void moveAbsoluteDirection(int targetDir);
void goToNodeFromHub8(int node);       // 기존 유지 (호환성)
void returnToHub8FromNode(int node, bool cameOutForward);  // 기존 유지

// ★ 허브 불경유 직접 라우팅
void exitZone(int zone);               // 구역 내부 → 해당 교차로 노드로 탈출
void goToZoneDirect(int zone);         // 현재 노드에서 목적 구역으로 최단 이동 후 enterZone()
void moveToNode(int toNode);           // 현재 노드 → 목표 노드 최단 이동

#endif
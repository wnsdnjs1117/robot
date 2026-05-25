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
void goToNodeFromHub8(int node);
void returnToHub8FromNode(int node, bool cameOutForward);

void turnToHeading(int target);        // 현재 헤딩 → 목표 헤딩 최소 회전
void exitZone(int zone);
void goToZoneDirect(int zone);
void moveToNode(int toNode);

#endif
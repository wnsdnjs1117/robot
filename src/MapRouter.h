/* ============================================================
 * MapRouter.h - 다이나믹 방위 제어 기반 격자 맵 라우터
 * ============================================================ */
#ifndef MAP_ROUTER_H
#define MAP_ROUTER_H

#include <Arduino.h>

void executeBlindRun();

// ★ 외부 파일(MissionFlow 등)에서 이 변수를 사용할 수 있도록 extern 선언!
extern int robotHeading;

void moveAbsoluteDirection(int targetDir);
void goToNodeFromHub8(int node);
void returnToHub8FromNode(int node, bool cameOutForward);

#endif
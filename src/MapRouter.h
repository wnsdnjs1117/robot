/* ============================================================
 * MapRouter.h - 경기장 절대 좌표 기반 노드 간 자율주행 라우터
 * ============================================================ */
#ifndef MAP_ROUTER_H
#define MAP_ROUTER_H

#include <Arduino.h>

// 공통 유틸리티: 라인이 끊어진 구간(Blind 구간)을 돌파하는 함수
void executeBlindRun();

// [탐색 구역 간 이동]
void moveNode1ToNode4(bool cameOutForward);
void moveNode2ToNode3(bool cameOutForward);
void moveNode3ToNode4(bool cameOutForward);

// [입고(5) / 출고(6) 구역 연계 이동]
void moveNode5ToNode3(bool cameToNode5Forward);
void moveNode2ToNode6(bool cameToNode2Forward);
void moveNode6ToNode3(bool cameToNode6Forward);
void moveNode5ToNode1(bool cameToNode5Forward);

#endif
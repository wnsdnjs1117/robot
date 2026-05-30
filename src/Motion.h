/* ============================================================
 * Motion.h - 하드웨어 구동 및 라인 트레이싱 제어 헤더
 * ============================================================ */
#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include <PRIZM.h>

extern PRIZM prizm;
extern int lastSensorState;

// 모터 기본 제어 및 Non-blocking 유틸리티
void drive(int l, int r);
void stopAll();
void turnAngle(int degrees, bool isRight);
void safeDelay(unsigned long ms); // ★ 시스템 전체 Non-blocking 딜레이 함수

// 센서 입력
void readSensors(int& L, int& C, int& R);
void readRearSensors(int& RL, int& RC, int& RR);
bool anyLine(int L, int C, int R);
bool anyRearLine(int RL, int RC, int RR);

// 라인 트레이싱 및 교차로 판정
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR);
void reverseLineFollowStep(int RL, int RC, int RR);
bool detectCrossing(int L, int C, int R);

// T자 사거리 등에서 1 1 0 / 0 1 1 조향을 허용할지 결정하는 스위치
extern bool enableEdgeSteering; 

#endif
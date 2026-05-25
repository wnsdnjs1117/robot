/* ============================================================
 * Config.h - 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// 1. 센서 핀 설정
const int SENSOR_LEFT = 2;
const int SENSOR_CENTER = 3;
const int SENSOR_RIGHT = 4;
const bool INVERT_SENSORS = false;

// 2. 주행 속도 설정
const int STRAIGHT_SPEED = 24;  // 라인 없는 구간 직진 속도
const int SPEED = 24;           // 일반 라인트레이싱 속도
const int BACK_SPEED = 21;      // 구역 퇴출 수직 후진 속도
const int SPIN_SPEED = 27;      // 제자리 스핀 턴 회전 속도

// 3. 엔코더 제어 거리 설정
const int START_ESCAPE_COUNTS = 1500;  // 스타트 선 밟은 후 추가 이탈 거리
const int SPIN_90_COUNTS = 1200;       // 90도 회전 시 필요한 엔코더 카운트
const int CROSS_ALIGN_COUNTS = 250;    // 교차로 감지 후 축 정렬 과전진 거리
const int ZONE_ENTER_COUNTS = 2200;    // 교차로 ➔ 구역 안쪽 진입 거리
const int ZONE_DEPTH_COUNTS = 1500;    // 구역 내부 수직 관통 깊이

// 4. 주행 환경 설정
const bool WEST_IS_LEFT = true;  // 서쪽 방향이 왼쪽인지 여부
const int CROSS_CONFIRM = 2;     // 교차로 인식 노이즈 필터링 카운트

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
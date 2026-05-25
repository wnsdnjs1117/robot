/* ============================================================
 * Config.h - 공용 설정
 *   파라미터(const) + 전역 객체/변수 선언(extern)
 *   ※ 모든 .cpp 파일이 이 헤더를 include 합니다.
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ============================================================
//  ★ 사용자 조정 파라미터 (경기장 환경에 맞춰 수정) ★
// ============================================================

// 1. 센서 핀 설정 (기본값: 흰바닥 0, 선 1)
const int SENSOR_LEFT = 2;
const int SENSOR_CENTER = 3;
const int SENSOR_RIGHT = 4;
const bool INVERT_SENSORS = false;

// 2. 주행 속도 설정 (DPS 단위 환산용 기본 스케일)
const int STRAIGHT_SPEED = 24;  // 라인 없는 구간 직진 속도
const int SPEED = 24;           // 일반 라인트레이싱 속도
const int BACK_SPEED = 21;      // 구역 퇴출 후진 속도
const int SPIN_SPEED = 27;      // 제자리 회전 속도

// 3. 엔코더 제어 거리 설정 (카운트 단위)
const int START_ESCAPE_COUNTS = 1500;  // 스타트선 통과 후 추가 직진 거리
// 리프트를 테스트 @@ 로컬!
const int SPIN_90_COUNTS = 1200;     // 90도 회전 엔코더 값
const int CROSS_ALIGN_COUNTS = 250;  // 교차로 감지 후 과전진(정렬) 거리
const int ZONE_ENTER_COUNTS = 2200;  // 교차로 -> 구역 안쪽 진입 거리
const int ZONE_DEPTH_COUNTS = 1500;  // 후진으로 교차로 관통하는 거리

// 4. 주행 환경 설정
const bool WEST_IS_LEFT = true;  // 첫 회전 방향 (좌회전=true)
const int CROSS_CONFIRM = 2;     // 교차로 노이즈 제거용 연속 감지 횟수

// ============================================================
//  전역 객체/변수 (실제 정의는 main.cpp 에 있음)
// ============================================================
extern PRIZM prizm;
extern int lastSensorState;  // 직전 센서 위치 (1:왼쪽, 2:오른쪽)
extern bool crossingArmed;
extern int crossingStable;

// (박스 위치 관련 자료구조는 BoxMap.h 로 분리됨)

#endif

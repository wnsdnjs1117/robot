/* ============================================================
 * Config.h - 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// 1. 센서 핀 설정
constexpr int SENSOR_LEFT = 2;
constexpr int SENSOR_CENTER = 3;
constexpr int SENSOR_RIGHT = 4;
constexpr bool INVERT_SENSORS = false;
constexpr int BUZZER_PIN = 5;  // 부저 핀 (Arduino tone() 사용)

// 2. 주행 속도 설정
constexpr int STRAIGHT_SPEED = 24;  // 라인 없는 구간 직진 속도
constexpr int SPEED = 24;           // 일반 라인트레이싱 속도
constexpr int BACK_SPEED = 21;      // 구역 퇴출 수직 후진 속도
constexpr int SPIN_SPEED = 27;      // 제자리 스핀 턴 회전 속도

// 3. 엔코더 제어 거리 설정
constexpr int START_ESCAPE_COUNTS = 1500;  // 스타트 선 밟은 후 추가 이탈 거리
constexpr int FINISH_ENTRY_COUNTS = 1500;  // FINISH 구역 진입 거리
constexpr int SPIN_90_COUNTS = 1200;       // 90도 회전 시 필요한 엔코더 카운트
constexpr int CROSS_ALIGN_COUNTS = 250;    // 교차로 감지 후 축 정렬 과전진 거리
constexpr int ZONE_ENTER_COUNTS = 2200;    // 교차로 ➔ 구역 안쪽 전진 진입 거리
constexpr int ZONE_DEPTH_COUNTS = 1500;    // 구역 내부 수직 관통 깊이 (탐색용)
constexpr int ZONE_EXIT_REV_COUNTS =
    2000;  // 구역 내부 → 교차로 후진 거리
           // (전진 진입 후 탈출 시. 현장 튜닝 필요)

// 4. 주행 환경 설정
constexpr bool WEST_IS_LEFT = true;  // 서쪽 방향이 왼쪽인지 여부
constexpr int CROSS_CONFIRM   = 2;  // +자 교차로 인식 노이즈 필터링 카운트
constexpr int T_CROSS_CONFIRM = 5;  // T자 교차로 인식 임계값 (25ms, 오감지 방지용)

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;
extern int crossingStableT;

#endif
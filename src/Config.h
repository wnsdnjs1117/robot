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

// 후방 센서 (아날로그 핀, analogRead >= REAR_SENSOR_THRESHOLD → 감지)
constexpr int SENSOR_REAR_LEFT   = A1;
constexpr int SENSOR_REAR_CENTER = A2;
constexpr int SENSOR_REAR_RIGHT  = A3;
constexpr int REAR_SENSOR_THRESHOLD = 200;

// 2. 주행 속도 설정
constexpr int STRAIGHT_SPEED = 24;  // 라인 없는 구간 직진 속도
constexpr int SPEED = 24;           // 일반 라인트레이싱 속도
constexpr int BACK_SPEED = 21;      // 구역 퇴출 수직 후진 속도
constexpr int SPIN_SPEED = 27;      // 제자리 스핀 턴 회전 속도
constexpr int BLIND_SPEED = 18;     // 블라인드 구간(라인 없음) 전용 저속

// 3. 엔코더 제어 거리 설정
constexpr int START_ESCAPE_COUNTS  = 1500;  // 스타트 선 밟은 후 추가 이탈 거리
constexpr int FINISH_ENTRY_COUNTS  = 1500;  // FINISH 구역 진입 거리
constexpr int SPIN_90_COUNTS       = 1200;  // 90도 회전 시 필요한 엔코더 카운트
constexpr int CROSS_ALIGN_COUNTS   = 250;   // 교차로 감지 후 축 정렬 과전진 거리
constexpr int ZONE_ENTER_COUNTS    = 2400;  // 노드7 전진 탈출 전용 (exitZone)
constexpr int ZONE_ENTER_EXTRA     = 400;   // 전진 진입: 라인 끊긴 후 추가 직진
constexpr int ZONE_DEPTH_EXTRA     = 400;   // 후진 진입: 라인 끊긴 후 추가 후진
constexpr int ZONE_EXIT_REV_COUNTS = 2000;  // 구역 내부 → 교차로 후진 거리

// 4. 주행 환경 설정
constexpr bool WEST_IS_LEFT = true;       // 서쪽 방향이 왼쪽인지 여부
constexpr int CROSS_CONFIRM = 2;          // 교차로 인식 노이즈 필터링 카운트
constexpr int ANGULAR_GAIN = 3;           // 전/후방 이중 센서 각도 교정 배율
constexpr int REAR_TO_AXLE_COUNTS = 150;  // 후방 센서 → 차축 거리

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
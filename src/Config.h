/* ============================================================
 * Config.h - 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [1] 센서 핀 ──────────────────────────────────────────────
constexpr int SENSOR_LEFT = 2;
constexpr int SENSOR_CENTER = 3;
constexpr int SENSOR_RIGHT = 4;
constexpr bool INVERT_SENSORS = false;
constexpr int BUZZER_PIN = 5;  // 부저 핀 (Arduino tone() 사용)

constexpr int SENSOR_REAR_LEFT = A1;  // 후방 아날로그 (바퀴축과 동일 위치)
constexpr int SENSOR_REAR_CENTER = A2;
constexpr int SENSOR_REAR_RIGHT = A3;
constexpr int REAR_SENSOR_THRESHOLD = 200;  // analogRead >= 200 → 라인 감지

// ── [2] 모터 속도 ────────────────────────────────────────────
constexpr int STRAIGHT_SPEED = 24;  // 라인 없는 구간 직진 속도
constexpr int SPEED          = 24;  // 일반 라인트레이싱 속도
constexpr int BACK_SPEED     = 21;  // 구역 후진 기본 속도
constexpr int SPIN_SPEED     = 27;  // 제자리 스핀 턴 회전 속도
constexpr int BLIND_SPEED    = 18;  // 블라인드 구간(라인 없음) 전용 속도

// ── [3] 엔코더 거리 (물리 측정값) ────────────────────────────
constexpr int SPIN_90_COUNTS = 1200;       // 90도 회전 엔코더 카운트
constexpr int CROSS_ALIGN_COUNTS = 250;    // 교차로 감지 후 축 정렬 과전진 거리
constexpr int START_ESCAPE_COUNTS = 1800;  // 스타트 선 밟은 후 추가 이탈 거리
constexpr int FINISH_ENTRY_COUNTS = 1500;  // FINISH 구역 진입 거리
constexpr int REAR_TO_AXLE_COUNTS = 0;  // 후방 센서 → 차축 거리 (0: 동일 위치)

// ── [4] 존 진입/탈출 거리 ────────────────────────────────────
constexpr int ZONE_ENTER_COUNTS    = 2400;  // 노드7 전진 탈출 전용 (exitZone)
constexpr int ZONE_ENTER_EXTRA     = 1000;  // 전진 진입: 라인 끊긴 후 추가 직진
constexpr int ZONE_DEPTH_EXTRA     = 1000;  // 후진 진입: 라인 끊긴 후 추가 후진
constexpr int ZONE_EXIT_REV_COUNTS = 2000;  // 구역 내부 → 교차로 후진 거리
constexpr int ZONE_FOLLOW_MAX      = 3500;  // 후진 라인 추종 최대 거리 (무한루프 방지)

// ── [5] 제어 파라미터 (튜닝값) ──────────────────────────────
constexpr bool WEST_IS_LEFT      = true;  // 서쪽 방향이 왼쪽인지 여부
constexpr int  CROSS_CONFIRM     = 2;     // 십자 교차로 인식 노이즈 필터링 카운트
constexpr int  T_CROSS_CONFIRM   = 3;     // T자 교차로 인식 노이즈 필터링 카운트
constexpr int  ANGULAR_GAIN      = 3;     // 전/후방 이중 센서 각도 교정 배율
constexpr int  ALIGN_MAX_COUNTS  = 67;    // alignHeadingOnLine 최대 회전량 (≈5도)
constexpr int  SPIN_BRAKE_LEAD   = 15;    // turnAngle 관성 보정 선행 제동 카운트
constexpr int  BACK_STEER_DIFF   = 8;     // 후진 조향 좌우 차동 크기 (단측 감지 시 적용)

// 전역 객체 선언
extern PRIZM prizm;
extern int  lastSensorState;
extern bool crossingArmed;
extern int  crossingStable;
extern int  crossingStableT;

#endif

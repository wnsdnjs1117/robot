/* ============================================================
 * Config.h - 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [1] 센서 핀 ──────────────────────────────────────────────
constexpr int  SENSOR_LEFT           = 2;
constexpr int  SENSOR_CENTER         = 3;
constexpr int  SENSOR_RIGHT          = 4;
constexpr bool INVERT_SENSORS        = false;
constexpr int  BUZZER_PIN            = 5;    // 부저 핀 (Arduino tone() 사용)

constexpr int  SENSOR_REAR_LEFT      = A1;   // 후방 아날로그 (차축 후방 24cm, 1000 count)
constexpr int  SENSOR_REAR_CENTER    = A2;
constexpr int  SENSOR_REAR_RIGHT     = A3;
constexpr int  REAR_SENSOR_THRESHOLD = 200;  // analogRead >= 200 → 라인 감지

// ── [2] 모터 속도 ────────────────────────────────────────────
constexpr int STRAIGHT_SPEED = 24;  // 라인 없는 구간 직진 속도
constexpr int SPEED          = 24;  // 일반 라인트레이싱 속도
constexpr int BACK_SPEED     = 21;  // 구역 후진 속도
constexpr int SPIN_SPEED     = 27;  // 제자리 스핀 턴 회전 속도
constexpr int BLIND_SPEED    = 18;  // 블라인드 구간(라인 없음) 전용 저속

// ── [3] 엔코더 거리 (물리 측정값) ────────────────────────────
constexpr int SPIN_90_COUNTS      = 1200;  // 90도 회전 엔코더 카운트
constexpr int CROSS_ALIGN_COUNTS  = 250;   // 교차로 감지 후 축 정렬 과전진 거리
constexpr int START_ESCAPE_COUNTS = 1500;  // 스타트 선 밟은 후 추가 이탈 거리
constexpr int FINISH_ENTRY_COUNTS = 1500;  // FINISH 구역 진입 거리
constexpr int REAR_TO_AXLE_COUNTS = 1000;   // 후방 센서 → 차축 거리 (24cm ≈ 1000 count)

// ── [4] 존 진입/탈출 거리 ────────────────────────────────────
constexpr int ZONE_ENTER_COUNTS    = 2400;  // 노드7 전진 탈출 전용 (exitZone)
constexpr int ZONE_ENTER_EXTRA     = 1000;  // 전진 진입: 라인 끊긴 후 추가 직진
constexpr int ZONE_DEPTH_EXTRA     = 1000;  // 후진 진입: 라인 끊긴 후 추가 후진
constexpr int ZONE_EXIT_REV_COUNTS = 2000;  // 구역 내부 → 교차로 후진 거리
constexpr int ZONE_FOLLOW_MAX      = 3500;  // 후진 라인 추종 최대 거리 (무한루프 방지)

// ── [5] 제어 파라미터 (튜닝값) ──────────────────────────────
constexpr bool WEST_IS_LEFT     = true;  // 서쪽 방향이 왼쪽인지 여부
constexpr int  CROSS_CONFIRM    = 2;     // 교차로 인식 노이즈 필터링 카운트
constexpr int  ANGULAR_GAIN     = 3;     // 전/후방 이중 센서 각도 교정 배율
constexpr int  ALIGN_MAX_COUNTS = 67;    // alignHeadingOnLine 최대 회전량 (≈5도)
constexpr int  SPIN_BRAKE_LEAD  = 15;    // turnAngle 관성 보정 선행 제동 카운트

// 라인 정렬 회전 (turnToLine): 회전 방향에 그어진 라인을 만나면 정렬 정지
constexpr int  TURN_LINE_ARM_DEG = 30;   // 이 각도 이상 회전 + 시작 라인 이탈 후부터 감지
constexpr int  TURN_LINE_MAX_DEG = 100;  // 라인 못 찾을 때 무한 회전 방지 한계각

// ── [6] 방향 상수 (robotHeading) ───────────────────────────────
// turnToHeading() 인자 및 robotHeading 값에 사용 (int 산술 호환)
constexpr int HDG_N = 0;  // 북 – 구역(1~6) 입구 방향
constexpr int HDG_E = 1;  // 동 – 노드 번호 증가 / FINISH 방향
constexpr int HDG_S = 2;  // 남 – 스타트 / 남쪽 구역(3·4) 방향
constexpr int HDG_W = 3;  // 서 – 노드 번호 감소 방향

// ── [7] 기타 거리 상수 ──────────────────────────────────────────
constexpr int NODE7_EXIT_COUNTS = 700;  // 노드7 T자 교차로 탈출 엔코더 거리

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif

/* ============================================================
 * Config.h - 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// 경기장 레이아웃 (위 = 북)
//
//   [1구역]  [2구역]             [5입고]   [6출고]
//      │        │                   │        │
//     [7]──────[8]──────[9]  ···  [10]  ··· [11] ··[12]
//      │        │                                    ·
//   [3구역]  [4구역]                                  ·
//                                              ┌─────┴──────┐
//                                              │  출발/도착  │
//                                              │   START    │
//                                              └────────────┘
//
//   ─── : 검은 라인   ··· : 블라인드 구간   구역-노드: 1,3→[7] / 2,4→[8] / 5→[10] / 6→[11]

// ── [0] 단위 변환 ────────────────────────────────────────────
// 22cm = 1000 counts (물리 측정값)
constexpr float COUNTS_PER_CM = 1000.0f / 23.0f;
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); }

// ── [1] 센서 핀 ──────────────────────────────────────────────
constexpr int SENSOR_LEFT = 2;
constexpr int SENSOR_CENTER = 3;
constexpr int SENSOR_RIGHT = 4;
constexpr bool INVERT_SENSORS = false;
constexpr int BUZZER_PIN = 5;  // 부저 핀 (Arduino tone() 사용)

constexpr int SENSOR_REAR_LEFT = A1;  // 후방 아날로그
constexpr int SENSOR_REAR_CENTER = A2;
constexpr int SENSOR_REAR_RIGHT = A3;
constexpr int REAR_SENSOR_THRESHOLD = 200;  // analogRead >= 200 → 라인 감지

// ── [2] 모터 속도 ────────────────────────────────────────────
constexpr int STRAIGHT_SPEED = 25;  // 라인 없는 구간 직진 속도
constexpr int SPEED = 35;           // 일반 라인트레이싱 속도
constexpr int BACK_SPEED = 25;      // 구역 후진 속도
constexpr int SPIN_SPEED = 40;      // 제자리 스핀 턴 회전 속도
constexpr int BLIND_SPEED = 35;     // 블라인드 구간(라인 없음) 전용 저속

// ── [3] 엔코더 거리 ──────────────────────────────────────────
constexpr int SPIN_90_COUNTS = 1210;            // 90도 회전 엔코더 카운트 (회전용, cm 무관)
constexpr int CROSS_ALIGN_COUNTS = CM(4.5f);    // 교차로 감지 후 축 정렬 과전진 (6cm)
constexpr int START_ESCAPE_COUNTS = CM(28.0f);  // 스타트 선 밟은 후 추가 이탈 거리 (31cm)
constexpr int FINISH_ENTRY_COUNTS = CM(36.0f);  // FINISH 구역 진입 거리 (36cm)
constexpr int REAR_TO_AXLE_COUNTS = CM(24.0f);  // 후방 센서 → 차축 거리 (24cm)

// ── [4] 존 진입 거리 (실측값 입력) ──────────────────────────
// ZONE_ENTER_EXTRA  : 전진 진입 — 전방 센서가 라인을 잃은 순간부터 목표 정지점까지
// ZONE_DEPTH_EXTRA  : 후진 진입 — 후방 센서가 라인을 잃은 순간부터 목표 정지점까지
// ZONE_FOLLOW_MAX   : 후진 중 안전 한계 (구역 깊이보다 넉넉하게)
// NODE8_EXIT_QUAL   : 2·4존 후진 탈출 시 교차로 감지 허용 최소 거리 (ZONE_ENTER_EXTRA와 맞출 것)
constexpr int ZONE_ENTER_EXTRA  = CM(33.0f);  // ★ 실측 필요
constexpr int ZONE_DEPTH_EXTRA  = CM(25.0f);  // ★ 실측 필요
constexpr int ZONE_FOLLOW_MAX   = CM(40.0f);  // 안전 한계 (실측값보다 크게)
constexpr int NODE8_EXIT_QUAL   = CM(28.0f);  // 교차로 감지 최소 이동 거리 (≈ZONE_ENTER_EXTRA - 전후 센서 간격)

// ── [5] 제어 파라미터 (튜닝값) ──────────────────────────────
constexpr float LIFT_UP_CLEAR_CM   = 15.0f;  // 상승 중 주행 허가 높이 (cm)
constexpr float LIFT_DOWN_CLEAR_CM = 10.0f;  // 하강 중 주행 허가 높이 (cm)
constexpr int DRIVE_BIAS = 2;         // 좌 모터 가속 편향 보정: +값 → 좌↓ 우↑ (직진 우편향 시 양수)
constexpr bool WEST_IS_LEFT = true;   // 서쪽 방향이 왼쪽인지 여부
constexpr int CROSS_CONFIRM = 2;      // 교차로 인식 노이즈 필터링 카운트
constexpr int ANGULAR_GAIN = 3;       // 전/후방 이중 센서 각도 교정 배율
constexpr int ALIGN_MAX_COUNTS = 67;  // alignHeadingOnLine 최대 회전량 (≈5도)
constexpr int SPIN_BRAKE_LEAD = 15;   // turnAngle 관성 보정 선행 제동 카운트

// 라인 정렬 회전 (turnToLine)
constexpr int TURN_LINE_ARM_DEG = 40;  // 이 각도 이상 회전 + 시작 라인 이탈 후부터 감지
constexpr int TURN_LINE_MAX_DEG = 91;  // 라인 못 찾을 때 무한 회전 방지 한계각

// ── [6] 방향 상수 (robotHeading) ───────────────────────────────
constexpr int HDG_N = 0;  // 북 – 구역(1~6) 입구 방향
constexpr int HDG_E = 1;  // 동 – 노드 번호 증가 / FINISH 방향
constexpr int HDG_S = 2;  // 남 – 스타트 / 남쪽 구역(3·4) 방향
constexpr int HDG_W = 3;  // 서 – 노드 번호 감소 방향

// ── [7] 기타 거리 상수 ──────────────────────────────────────────
constexpr int NODE7_EXIT_COUNTS = CM(33.0f);      // 존1·3 후진진입→전진탈출: 정지점→노드7 (★실측)
constexpr int NODE7_REV_EXIT_COUNTS = CM(60.0f);  // 존1·3 전진진입→후진탈출: 정지점→노드7 (★실측)
constexpr int ZONE5_EXIT_COUNTS = CM(56.0f);      // 존5  전진진입→후진탈출: 정지점→노드10 (★실측)
constexpr int ZONE6_EXIT_COUNTS = CM(56.0f);      // 존6  전진진입→후진탈출: 정지점→노드11 (★실측)
constexpr int NODE_11_12_COUNTS = CM(70.0f);      // 노드11→12 블라인드 거리 (실측 후 조정)
constexpr int BLIND_NODE_MAX = CM(70.0f);         // 블라인드 구간 폴백 (10↔11 = 70cm + 여유)

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif

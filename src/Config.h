/* ============================================================
 * Config.h - 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [DEBUG] 0 = 경기용 무음, 1 = 시리얼 디버그 출력 ─────────────
#define ROBOT_DEBUG 1

#if ROBOT_DEBUG
#define DPRINT(x) Serial.print(x)
#define DPRINTLN(x) Serial.println(x)
#define DPRINTF(x) Serial.print(F(x))
#define DPRINTLNF(x) Serial.println(F(x))
#else
#define DPRINT(x)
#define DPRINTLN(x)
#define DPRINTF(x)
#define DPRINTLNF(x)
#endif

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
constexpr float COUNTS_PER_CM = 1000.0f / 23.5f;
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

// ── [1.5] 센서-바퀴축 간격 / 차체 기하 ──────────────────────────
constexpr float FRONT_SENSOR_OFFSET = 7.5f;   // 전방 센서 → 바퀴축 (cm)
constexpr float REAR_SENSOR_OFFSET  = 25.0f;  // 후방 센서 → 바퀴축 (cm)
constexpr float AXLE_TO_LIFT_CM     = 11.0f;  // 바퀴축 → 리프트 (cm, 후방 방향)

// ── [2] 모터 속도 ────────────────────────────────────────────
constexpr int STRAIGHT_SPEED = 35;  // 라인 없는 구간 직진 속도
constexpr int SPEED = 50;           // 일반 라인트레이싱 속도
constexpr int BACK_SPEED = 30;      // 구역 후진 속도
constexpr int SPIN_SPEED = 50;      // 제자리 스핀 턴 회전 속도
constexpr int BLIND_SPEED = 50;     // 블라인드 구간(라인 없음) 전용 저속

// ── [3] 엔코더 거리 ──────────────────────────────────────────
constexpr int SPIN_90_COUNTS = 1210;  // 90도 회전 엔코더 카운트 (회전용, cm 무관)

// 아래 세 줄은 센서 오프셋에서 자동 계산 — 직접 편집하지 마세요
constexpr int CROSS_ALIGN_COUNTS  = CM(FRONT_SENSOR_OFFSET);  // 교차로 감지 후 축 정렬 과전진
constexpr int REAR_TO_AXLE_COUNTS = CM(REAR_SENSOR_OFFSET);   // 후방 센서 → 차축 거리

constexpr float START_ESCAPE_AXLE_CM = 22.5f;  // ★ 스타트 이탈 후 바퀴축 이동 거리 (cm)
constexpr int START_ESCAPE_COUNTS = CM(FRONT_SENSOR_OFFSET + START_ESCAPE_AXLE_CM);

constexpr int FINISH_ENTRY_COUNTS = CM(36.0f);  // FINISH 구역 진입 거리 (36cm)

// ── [4] 존 진입 거리 ─────────────────────────────────────────
// ZONE_LIFT_DEPTH : 노드에서 리프트가 멈춰야 할 깊이 (cm, 리프트 기준)
//   전진/후진 방향 차이를 공식이 자동 흡수 — 이 값 하나만 수정하면 됨.
//
//   트리거: 구역 유도선(ZONE_LINE_LENGTH) 끝에서 센서가 선 소실 → resetEncoders → EXTRA 맹주행
//   전진: 트리거 시 축=LINE-FRONT=22.5cm → ZONE_ENTER_EXTRA = CM(LIFT_DEPTH - LINE + FRONT + LIFT)
//   후진: 트리거 시 축=LINE-REAR= 5.0cm → ZONE_DEPTH_EXTRA = CM(LIFT_DEPTH - LINE + REAR  - LIFT)
constexpr float ZONE_LINE_LENGTH = 30.0f;   // 구역 유도선 길이 (cm)
constexpr float ZONE_LIFT_DEPTH  = 50.0f;   // ★ 실측값 (노드 → 리프트 정지점, cm)

constexpr int ZONE_ENTER_EXTRA = CM(ZONE_LIFT_DEPTH - ZONE_LINE_LENGTH + FRONT_SENSOR_OFFSET + AXLE_TO_LIFT_CM);
// = CM(50 - 30 + 7.5 + 11) = CM(38.5)
constexpr int ZONE_DEPTH_EXTRA = CM(ZONE_LIFT_DEPTH - ZONE_LINE_LENGTH + REAR_SENSOR_OFFSET  - AXLE_TO_LIFT_CM);
// = CM(50 - 30 + 25 - 11) = CM(34.0)
constexpr int ZONE_FOLLOW_MAX  = CM(40.0f);  // 유도선 추적 안전 한계 (최대 이동 ~5cm << 40cm)
constexpr int NODE8_EXIT_QUAL  = CM(AXLE_TO_LIFT_CM + ZONE_LIFT_DEPTH);  // 후진 탈출 교차로 감지 최소 이동량
// = CM(11 + 50) = CM(61.0)

// ── [5] 제어 파라미터 (튜닝값) ──────────────────────────────
constexpr float LIFT_UP_CLEAR_CM = 8.0f;    // 상승 중 주행 허가 높이 (cm)
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;  // 하강 중 주행 허가 높이 (cm)
constexpr int DRIVE_BIAS = 0;               // 좌 모터 가속 편향 보정: +값 → 좌↓ 우↑ (직진 우편향 시 양수)
constexpr bool WEST_IS_LEFT = true;         // 서쪽 방향이 왼쪽인지 여부
constexpr int CROSS_CONFIRM = 2;            // 교차로 인식 노이즈 필터링 카운트
constexpr int ANGULAR_GAIN = 3;             // 전/후방 이중 센서 각도 교정 배율
constexpr int ALIGN_MAX_COUNTS = 67;        // alignHeadingOnLine 최대 회전량 (≈5도)
constexpr int SPIN_BRAKE_LEAD = 15;         // turnAngle 관성 보정 선행 제동 카운트

// 라인 정렬 회전 (turnToLine)
constexpr int TURN_LINE_ARM_DEG = 30;  // 이 각도 이상 회전 + 시작 라인 이탈 후부터 감지
constexpr int TURN_LINE_MAX_DEG = 92;  // 라인 못 찾을 때 무한 회전 방지 한계각

// ── [6] 블라인드 구간 엔코더 차동 보정 파라미터 ────────────────
constexpr int BLIND_CORR_DEADZONE = 3;  // 무시할 최소 엔코더 차이
constexpr int BLIND_CORR_GAIN     = 8;  // 보정 나눗수
constexpr int BLIND_CORR_CAP      = 6;  // 최대 보정량

// ── [7] 방향 상수 (robotHeading) ───────────────────────────────
constexpr int HDG_N = 0;  // 북 – 구역(1~6) 입구 방향
constexpr int HDG_E = 1;  // 동 – 노드 번호 증가 / FINISH 방향
constexpr int HDG_S = 2;  // 남 – 스타트 / 남쪽 구역(3·4) 방향
constexpr int HDG_W = 3;  // 서 – 노드 번호 감소 방향

// ── [8] 기타 거리 상수 ──────────────────────────────────────────
constexpr int NODE7_EXIT_COUNTS = CM(33.0f);      // 존1·3 후진진입→전진탈출: 정지점→노드7 (★실측)
constexpr int NODE7_REV_EXIT_COUNTS = CM(58.0f);  // 존1·3 전진진입→후진탈출: 정지점→노드7 (★실측)
constexpr int ZONE5_EXIT_COUNTS = CM(56.0f);      // 존5  전진진입→후진탈출: 정지점→노드10 (★실측)
constexpr int ZONE6_EXIT_COUNTS = CM(56.0f);      // 존6  전진진입→후진탈출: 정지점→노드11 (★실측)
constexpr int NODE_11_12_COUNTS = CM(70.0f);      // 노드11→12 블라인드 거리 (실측 후 조정)
constexpr int BLIND_NODE_MAX = CM(70.5f);         // 블라인드 구간 폴백 (10↔11 = 70cm + 여유)

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif

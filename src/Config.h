/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ============================================================
// [1] 펌웨어 모드 선택
// ============================================================
// 0 = 대회용 자율 주행 펌웨어 (기본값)
// 1 = 카메라·센서 하드웨어 테스트 전용 (자율 주행 코드 제외)
#define RUN_TEST_MODE 0

// 0 = 실제 HuskyLens로 QR ID를 읽어 박스 목적지 결정
// 1 = 가상 랜덤 배치(setupRandomLayout)로 동작 — 카메라·박스 없이 전체 미션 테스트
#define QR_SIMULATION 0

// ============================================================
// [2] HuskyLens / QR 스캔 파라미터
// ============================================================
constexpr int MAX_RESCAN_TRIES = 5;            // 박스 재스캔 시 최대 시도 횟수 (무한루프 방지)
constexpr unsigned long SCAN_DWELL_MS = 3000;  // 존 정지 후 QR 대기 시간 (ms)
constexpr unsigned long SCAN_POLL_MS  = 30;    // scanTick I2C 폴링 간격 (ms) — 낮을수록 인식률↑, 스티어링 부하↑

// ============================================================
// [3] 디버그 시리얼 출력
// ============================================================
// 1 = Serial.print로 상태 출력 활성화 / 0 = 완전 제거 (플래시 절약)
#define ROBOT_DEBUG 1

#if ROBOT_DEBUG
#define DPRINT(x)    Serial.print(x)
#define DPRINTLN(x)  Serial.println(x)
#define DPRINTF(x)   Serial.print(F(x))    // 문자열을 플래시(PROGMEM)에 두어 SRAM 절약
#define DPRINTLNF(x) Serial.println(F(x))
#else
#define DPRINT(x)
#define DPRINTLN(x)
#define DPRINTF(x)
#define DPRINTLNF(x)
#endif

// Motion.cpp 등에서 호출하는 디버그 키 감지 함수 (테스트 모드 아닐 때 빈 함수로 무효화)
inline void checkDebugKey() {}

// ============================================================
// [4] 엔코더 / 거리 변환
// ============================================================
// 80cm 이동 시 3570 카운트 — 실측값 기준
constexpr float COUNTS_PER_CM = 3570.0f / 80.0f;
// cm를 엔코더 카운트로 변환하는 헬퍼 (반올림 포함)
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); }

// ============================================================
// [5] 핀 번호 및 하드웨어
// ============================================================
constexpr int SENSOR_LEFT   = 2;  // 전방 라인센서 왼쪽  (디지털 입력)
constexpr int SENSOR_CENTER = 3;  // 전방 라인센서 중앙  (디지털 입력)
constexpr int SENSOR_RIGHT  = 4;  // 전방 라인센서 오른쪽 (디지털 입력)
constexpr bool INVERT_SENSORS = false;  // 센서 극성 반전 여부 (흰 바탕 + 검은 선 = false)
constexpr int BUZZER_PIN = 5;     // 압전 부저 핀

constexpr int SENSOR_REAR_LEFT   = A1;  // 후방 라인센서 왼쪽  (아날로그 입력)
constexpr int SENSOR_REAR_CENTER = A2;  // 후방 라인센서 중앙  (아날로그 입력)
constexpr int SENSOR_REAR_RIGHT  = A3;  // 후방 라인센서 오른쪽 (아날로그 입력)
constexpr int REAR_SENSOR_THRESHOLD = 200;  // 후방 센서 ON 판정 아날로그 임계값 (0~1023)

// ============================================================
// [6] 로봇 물리 치수 (바퀴축 기준 실측)
// ============================================================
constexpr float ROBOT_LENGTH_CM              = 35.0f;  // 로봇 전체 길이 (cm)
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM =  6.0f;  // 바퀴축 → 전방 센서 거리 (cm)
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM  =  4.0f;  // 바퀴축 → 후방 센서 거리 (cm)
constexpr float DIST_AXIS_TO_LIFT_CM         = 10.5f;  // 바퀴축 → 리프트 중심 거리 (cm)
constexpr float LINE_THICKNESS_CM            =  1.0f;  // 선 두께 보정값 (실제 2cm, 제동 여유 -1cm)

// ============================================================
// [7] 존 진입 / 탈출 거리 설정
// ============================================================

// 교차로 감지 후 바퀴축이 교차로 중심에 오도록 추가 이동할 거리
constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM;

// 교차로 직후 연속 교차로를 오인식하지 않도록 무시할 짧은 이동 거리
constexpr float DIST_IGNORE_NODE_CM = 5.0f;

// ── [7-1] 존별 진입/탈출 개별 설정 테이블 ──────────────────────────────────────
struct ZoneCfg {
  // 진입: 존 입구 선을 통과한 뒤 추가로 이동할 거리 (박스 위치까지)
  float entryFwdExtra = 35.0f;  // 전진 진입 시 추가 이동 거리 (cm)
  float entryRevExtra = 12.0f;  // 후진 진입 시 추가 이동 거리 (cm)

  // 탈출: 후방(또는 전방) 센서가 메인 라인을 감지한 뒤 라인을 타고 이동할 거리
  float exitFwdExtra  =  0.0f;  // 전진 탈출 시 라인 추종 거리 (cm)
  float exitRevExtra  =  0.0f;  // 후진 탈출 시 라인 추종 거리 (cm)
};

inline ZoneCfg zoneCfg(int z) {
  ZoneCfg cfg;  // 기본값: entryFwdExtra=35, entryRevExtra=12, exit 0/0

  if (z == 1) {
    // 1번 존은 7번 노드(T자형 교차로) — 교차 감지 불가, 선 감지 후 거리로 탈출
    cfg.exitFwdExtra = 35.0f;  // 전진 탈출: 선 감지 후 35cm 추종
    cfg.exitRevExtra = 33.0f;  // 후진 탈출: 선 감지 후 33cm 추종
  }
  else if (z == 3) {
    // 3번 존도 7번 노드 — 1번과 동일 이유로 거리 기반 탈출
    cfg.exitFwdExtra = 37.0f;  // 전진 탈출: 선 감지 후 37cm 추종
    cfg.exitRevExtra = 35.0f;  // 후진 탈출: 선 감지 후 35cm 추종
  }
  else if (z == 5 || z == 6) {
    // 5/6번 존: 항상 전진 진입 → 항상 후진 탈출만 존재
    // 메인 라인(가로선) 감지 후 28cm 라인 추종 후 정지
    cfg.exitRevExtra = 28.0f;
  }
  // 2, 4번 존은 십자(┼) 교차로 — 교차 감지로 탈출하므로 exit extra 불필요 (기본값 0)

  return cfg;
}

// ============================================================
// [8] 속도 설정
// ============================================================
// ── driveDistance 전용 가속/감속 프로파일 (라인 트레이싱과 무관) ──
constexpr int   RAMP_MIN_SPEED = 20;    // 가속 시작·감속 끝 최저 속도 (이하면 모터 실속)
constexpr float RAMP_ACCEL_CM  =  8.0f; // 가속 구간 거리 (cm)
constexpr float RAMP_DECEL_CM  = 12.0f; // 감속 구간 거리 (cm) — 제동 여유를 위해 가속보다 길게

constexpr int SPEED               = 35;  // 라인 트레이싱 전진 기본 속도
constexpr int BACK_SPEED          = 30;  // 라인 트레이싱 후진 기본 속도
constexpr int STRAIGHT_SPEED      = 40;  // 노드 간 맹목 직진 속도
constexpr int BLIND_SPEED         = 40;  // 라인 없는 구간 맹목 전진 속도

constexpr int ZONE_ENTRY_BLIND_SPEED      = 40;  // 존 진입 전진 속도
constexpr int ZONE_ENTRY_BLIND_BACK_SPEED = 40;  // 존 진입 후진 속도
constexpr int ZONE_EXIT_BLIND_SPEED       = 40;  // 존 탈출 전진 속도
constexpr int ZONE_EXIT_BLIND_BACK_SPEED  = 40;  // 존 탈출 후진 속도

// ============================================================
// [9] 라인 감지 안정화
// ============================================================
constexpr int EXIT_LINE_CONFIRM     = 2;   // 탈출 라인 연속 감지 횟수 (노이즈 조기 트리거 방지)
constexpr int START_LINE_SEARCH_SPEED = 25; // 스타트→메인라인 최초 탐색 저속 (얇은 선 놓침 방지)

// ============================================================
// [10] 회전 설정
// ============================================================
constexpr int SPIN_SPEED     = 40;    // 제자리 회전 속도
constexpr int SPIN_90_COUNTS = 1185;  // 90도 회전에 필요한 엔코더 카운트 (실측)

// ============================================================
// [11] PID 조향 게인
// ============================================================
// ── 전진 라인 트레이싱 ──
constexpr float LINE_KP_FWD_SOFT = 1.2f;  // 전진 P게인 (오차 ±1 이하, 완만한 커브)
constexpr float LINE_KP_FWD_HARD = 4.5f;  // 전진 P게인 (오차 ±2 이상, 급커브)
// ── 후진 라인 트레이싱 ──
constexpr float LINE_KP_REV_SOFT = 1.0f;  // 후진 P게인 (오차 ±1 이하)
constexpr float LINE_KP_REV_HARD = 2.2f;  // 후진 P게인 (오차 ±2 이상)

constexpr float LINE_KI         = 0.0f;  // I게인 (적분, 현재 미사용)
constexpr float LINE_KD         = 9.0f;  // D게인 (미분, 진동 억제)
constexpr float LINE_ALIGN_GAIN = 1.5f;  // 전후방 센서 정렬 보정 게인 (평행 유지)

// ── 속도 피드백 (엔코더 기반 속도 보정) ──
constexpr float VELOCITY_KP             = 0.2f;  // 속도 오차 P게인
constexpr float VELOCITY_TARGET_FACTOR  = 0.5f;  // 목표 속도 → 엔코더 카운트 환산 비율
constexpr int   VELOCITY_MAX_CORRECTION = 10;    // 속도 보정 최대 편차 (±10)
constexpr int   EDGE_SYNC_GAIN          =  5;    // 전후방 엣지 불일치 시 추가 보정 강도

// ── 기타 ──
constexpr int MOTOR_OFFSET_L = 0;   // 좌측 모터 직진 오프셋 (직진 미세 조정, 0=보정 없음)
constexpr int MOTOR_OFFSET_R = 0;   // 우측 모터 직진 오프셋
constexpr int CROSS_CONFIRM  = 1;   // 교차로(1.1.1) 연속 감지 횟수 (1=즉시 인정)

// ============================================================
// [12] 노드 이동 방위각 및 거리
// ============================================================
// 방위각 규약: 0=북(N), 90=동(E), 180=남(S), 270=서(W)
// turnToHeading(deg, true) = 우회전(시계방향), heading +1

constexpr int   HEADING_10_TO_12    = 210;
constexpr float DIST_10_TO_12_CM    = 50.0f;
constexpr int   HEADING_11_TO_12    = 250;
constexpr float DIST_11_TO_12_CM    = 110.0f;
constexpr int   HEADING_12_TO_9_2   = 310;

constexpr float DIST_START_TO_13_CM = 90.0f;   // 스타트 → 노드 13 직진 거리
constexpr int   HEADING_13_TO_9     = 305;
constexpr int   HEADING_9_TO_13     = 150;
constexpr float DIST_9_TO_13_CM     = 40.0f;
constexpr int   HEADING_10_TO_13    = 180;
constexpr float DIST_10_TO_13_CM    = 55.0f;
constexpr int   HEADING_13_TO_START = 90;
constexpr float DIST_FINISH_ENTRY_CM = 40.0f;  // 피니시 진입 최종 전진 거리

constexpr int HEADING_9_TO_10  =  83;
constexpr int HEADING_9_TO_11  =  87;
constexpr int HEADING_10_TO_11 =  90;
constexpr int HEADING_11_TO_10 = 270;

// ============================================================
// [13] 리프트 파라미터
// ============================================================
constexpr float LIFT_COUNTS_PER_CM  = 200.0f;  // 리프트 엔코더 카운트/cm
constexpr float LIFT_MAX_HEIGHT_CM  =  24.0f;  // 리프트 최대 허용 높이 (cm)
constexpr float LIFT_NEAR_FLOOR_CM  =   5.0f;  // 바닥 근접 판정 높이 — 이하에서 저속 전환 (cm)
constexpr float LIFT_UP_CLEAR_CM    =   5.0f;  // 픽업 후 이동을 위해 올려야 할 최소 높이 (cm)
constexpr float LIFT_DOWN_CLEAR_CM  =   0.0f;  // 박스 내려놓기 목표 높이 (cm)

constexpr int LIFT_UP_POWER   = 100;  // 상승 최대 출력 (0~100)
constexpr int LIFT_DOWN_POWER = 100;  // 하강 최대 출력 (0~100)

constexpr float LIFT_UP_SLOW_ZONE_CM   = 20.0f;  // 이 높이 이상 → 저속 상승으로 전환
constexpr int   LIFT_UP_SLOW_POWER_L   = 40;     // 저속 상승 좌측 출력
constexpr int   LIFT_UP_SLOW_POWER_R   = 40;     // 저속 상승 우측 출력
constexpr float LIFT_DOWN_SLOW_ZONE_CM =  8.0f;  // 이 높이 이하 → 저속 하강으로 전환
constexpr int   LIFT_DOWN_SLOW_POWER_L = 20;     // 저속 하강 좌측 출력
constexpr int   LIFT_DOWN_SLOW_POWER_R = 20;     // 저속 하강 우측 출력
constexpr float LIFT_SYNC_GAIN              = 5.0f;   // 좌우 리프트 동기화 보정 게인
constexpr unsigned long LIFT_TICK_INTERVAL_MS = 10;   // 리프트 제어 주기 (ms)
constexpr unsigned long LIFT_FLOOR_TIME_MS    = 2000; // 바닥 도달 후 안착 대기 시간 (ms)

// ============================================================
// [14] 외부 변수 참조
// ============================================================
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif

/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ============================================================
// ★ [펌웨어 모드 선택] ★
// 0: 대회용 자율 주행 펌웨어 컴파일 (테스트 코드가 0 byte가 됨)
// 1: 하드 카메라/센서 테스트 전용 펌웨어 컴파일 (자율 주행 코드 제외)
// ============================================================
#define RUN_TEST_MODE 0

// ============================================================
// ★ [QR 시뮬레이션] ★
// 0: 실제 HuskyLens로 QR ID를 읽어 박스 목적지 결정
// 1: 가상 박스맵(setupRandomLayout)으로 동작 — 카메라/박스 없이 전체 미션 테스트
// ============================================================
#define QR_SIMULATION 0

// ── HuskyLens / 스캔 파라미터 ──
constexpr int MAX_RESCAN_TRIES = 5;            // 재스캔 무한루프 안전 상한
constexpr unsigned long SCAN_DWELL_MS = 3000;  // 존 내 정지 스캔(대기) 시간
constexpr unsigned long SCAN_POLL_MS = 30;     // scanTick I2C 폴링 간격(↓: 탈출 중 인식률↑, 스티어링 보호 균형)

// ── [디버깅 모드] ──────────────────────────────────────────
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

// ★ [컴파일 에러 방지용] Motion.cpp 등에서 호출하는 기존 함수를 무효화(빈 함수로 처리)
inline void checkDebugKey() {}

// ★ [엔코더 실측값 반영] 80cm 이동 시 3450 카운트
constexpr float COUNTS_PER_CM = 3570.0f / 80.0f;
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); }

// ── [1] 핀 번호 및 하드웨어 센서 ─────────────────────────────────────
constexpr int SENSOR_LEFT = 2;
constexpr int SENSOR_CENTER = 3;
constexpr int SENSOR_RIGHT = 4;
constexpr bool INVERT_SENSORS = false;
constexpr int BUZZER_PIN = 5;

constexpr int SENSOR_REAR_LEFT = A1;
constexpr int SENSOR_REAR_CENTER = A2;
constexpr int SENSOR_REAR_RIGHT = A3;
constexpr int REAR_SENSOR_THRESHOLD = 200;

// ── [2] 로봇 하드웨어 물리적 치수 (★ 바퀴축 기준 실측) ───────────────────
constexpr float ROBOT_LENGTH_CM = 35.0f;
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;
constexpr float DIST_AXIS_TO_LIFT_CM = 10.5f;
constexpr float LINE_THICKNESS_CM = 1.0f; // 실제로는 2cm이지만 제동거리 감안해서 -1cm 보정

// ── [3] 존(Zone) 진입/탈출 수치 ─────────────────────────────────────
constexpr float DIST_ZONE_DEPTH_CM = 20.0f;
constexpr float LINE_LEN_ZONE_12_CM = 28.0f;
constexpr float LINE_LEN_ZONE_3456_CM = 30.0f;

// ★ 반대편 존으로 후진 횡단 시 최소 이동 거리(가드)
constexpr float ZONE_CROSS_MIN_CM = 20.0f;

// ── [3-1] ★ 존(1~6) 진입/탈출 개별 설정 테이블 ──
struct ZoneCfg {
  // 여기서 기본값을 아예 정해버립니다. (함수 안에서 0을 쓸 필요가 없어짐)
  float entryFwdExtra = 35.0f; 
  float entryRevExtra = 12.0f; 
  float exitFwdExtra = 0.0f;   
  float exitRevExtra = 0.0f;   
};

inline ZoneCfg zoneCfg(int z) {
  ZoneCfg cfg; // 위에서 정한 35, 12, 0, 0이 자동으로 들어갑니다.

  // ★ 안 쓰는 구역(0)은 아예 적지도 않습니다. 
  // 오직 특별한 숫자가 필요한 1번과 3번 구역만 딱 적어줍니다.
  
  if (z == 1) {
    cfg.exitFwdExtra = 35.0f;
    cfg.exitRevExtra = 33.0f;
  } 
  else if (z == 3) {
    cfg.exitFwdExtra = 37.0f;  
    cfg.exitRevExtra = 35.0f;  
  }

  return cfg;
}

// ── [4] 속도 및 조향 제어 ─────────────────────────────────────────
constexpr int SPEED = 50;
constexpr int BACK_SPEED = 50;
constexpr int STRAIGHT_SPEED = 50;
constexpr int BLIND_SPEED = 60;

constexpr int ZONE_ENTRY_BLIND_SPEED = 60;
constexpr int ZONE_ENTRY_BLIND_BACK_SPEED = 60;
constexpr int ZONE_EXIT_BLIND_SPEED = 60;
constexpr int ZONE_EXIT_BLIND_BACK_SPEED = 60;

// ── 라인 감지 안정화 ──
constexpr int EXIT_LINE_CONFIRM = 2;        // 탈출 시 목표 라인 감지 확정 횟수(노이즈 조기트리거 방지)
constexpr int START_LINE_SEARCH_SPEED = 25; // 스타트→메인라인 첫 탐색 저속(얇은 선 놓침 방지)

constexpr int SPIN_SPEED = 40;
constexpr int SPIN_90_COUNTS = 1185;

// ★ 센서 반응과 조향이 더 빠릿빠릿해지도록 P, D 게인 수치 약간 더 상향
constexpr float LINE_KP_FWD_SOFT = 1.2f;  // 1.0 -> 1.2
constexpr float LINE_KP_FWD_HARD = 4.5f;  // 4.0 -> 4.5
constexpr float LINE_KP_REV_SOFT = 1.0f;  // 0.8 -> 1.0
constexpr float LINE_KP_REV_HARD = 2.2f;  // 1.8 -> 2.2
constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 9.0f;           // 8.0 -> 9.0
constexpr float LINE_ALIGN_GAIN = 1.5f;

constexpr float VELOCITY_KP = 0.2f;
constexpr float VELOCITY_TARGET_FACTOR = 0.5f;
constexpr int VELOCITY_MAX_CORRECTION = 10;
constexpr int EDGE_SYNC_GAIN = 5;

constexpr int MOTOR_OFFSET_L = 0;
constexpr int MOTOR_OFFSET_R = 0;
constexpr int CROSS_CONFIRM = 1;
constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM;

// ── [5] 이동 노드 방위각 및 거리 ─────────────────────────────────────
constexpr int HEADING_10_TO_12 = 210;
constexpr float DIST_10_TO_12_CM = 50.0f;
constexpr int HEADING_11_TO_12 = 250;
constexpr float DIST_11_TO_12_CM = 110.0f;
constexpr int HEADING_12_TO_9_2 = 310;

constexpr float DIST_START_TO_13_CM = 90.0f;
constexpr int HEADING_13_TO_9 = 310.0;
constexpr int HEADING_9_TO_13 = 150;
constexpr float DIST_9_TO_13_CM = 40.0f;
constexpr int HEADING_10_TO_13 = 180;
constexpr float DIST_10_TO_13_CM = 55.0f;
constexpr int HEADING_13_TO_START = 90;
constexpr float DIST_FINISH_ENTRY_CM = 40.0f;

constexpr int HEADING_9_TO_10 = 85;
constexpr int HEADING_9_TO_11 = 88;
constexpr float DIST_IGNORE_NODE_CM = 5.0f;
constexpr int HEADING_10_TO_11 = 90;
constexpr int HEADING_11_TO_10 = 270;

// ── [6] 리프트 파라미터 ───────────────────────────────────────────────
constexpr float LIFT_COUNTS_PER_CM = 200.0f;
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;
constexpr float LIFT_NEAR_FLOOR_CM = 5.0f;
constexpr float LIFT_UP_CLEAR_CM = 5.0f;
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;

constexpr int LIFT_UP_POWER = 100;
constexpr int LIFT_DOWN_POWER = 100;

constexpr float LIFT_UP_SLOW_ZONE_CM = 20.0f;
constexpr int LIFT_UP_SLOW_POWER_L = 40;
constexpr int LIFT_UP_SLOW_POWER_R = 40;
constexpr float LIFT_DOWN_SLOW_ZONE_CM = 8.0f;
constexpr int LIFT_DOWN_SLOW_POWER_L = 20;
constexpr int LIFT_DOWN_SLOW_POWER_R = 20;
constexpr float LIFT_SYNC_GAIN = 5.0f;
constexpr unsigned long LIFT_TICK_INTERVAL_MS = 10;
constexpr unsigned long LIFT_FLOOR_TIME_MS = 2000;

// ── [7] 외부 변수 참조 ────────────────────────────────────────────────
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
/* ============================================================
 * Config.h — 로봇 튜닝 파라미터 (Arduino UNO + TETRIX PRIZM)
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ============================================================
// [1] 펌웨어 모드 · 디버그
// ============================================================

#define RUN_TEST_MODE 0
#define QR_SIMULATION 0
#define ROBOT_DEBUG 0

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

inline void checkDebugKey() {}

// ============================================================
// [2] QR 스캔 · 재탐색
// ============================================================

constexpr int MAX_RESCAN_TRIES = 99;
constexpr unsigned long SCAN_DWELL_MS = 1500;
constexpr unsigned long SCAN_POLL_MS = 30;
constexpr unsigned long BUZZER_QR_FOUND_MS = 1000;
constexpr unsigned int BUZZER_TONE_HZ = 1000;
constexpr unsigned long BUZZER_TONE_HALF_US = 500000UL / BUZZER_TONE_HZ;

// ============================================================
// [3] 핀 · 센서 하드웨어
// ============================================================

constexpr int PIN_LINE_FRONT_LEFT = 2;
constexpr int PIN_LINE_FRONT_CENTER = 3;
constexpr int PIN_LINE_FRONT_RIGHT = 4;
constexpr bool INVERT_LINE_SENSORS = false;

constexpr int PIN_BUZZER = 5;

constexpr int PIN_LINE_REAR_LEFT = A1;
constexpr int PIN_LINE_REAR_CENTER = A2;
constexpr int PIN_LINE_REAR_RIGHT = A3;
constexpr int REAR_LINE_THRESHOLD = 200;

// ============================================================
// [4] 물리 치수 · 엔코더 (바퀴축 기준, cm)
// ============================================================

constexpr float ROBOT_LENGTH_CM = 35.0f;
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;
constexpr float DIST_AXIS_TO_LIFT_CM = 10.5f;
constexpr float LINE_THICKNESS_CM = 2.0f;
constexpr float COUNTS_PER_CM = 3570.0f / 80.0f;

inline int toEncoderCounts(float cm) {
  return (int)(cm * COUNTS_PER_CM + 0.5f);
}

// ============================================================
// [5] 교차로 감지 · 목표 정지 (트랙 공통)
// ============================================================

constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM;
constexpr float DIST_BRAKE_CATCH_CM = 0.5f;
constexpr float DIST_BRAKE_CATCH_MAX_CM = 5.0f;
constexpr float DIST_IGNORE_NODE_CM = 5.0f;
constexpr int CROSS_CONFIRM = 1;
constexpr int EXIT_LINE_CONFIRM = 1;

// ============================================================
// [6] 박스 존(1~6) 진입 · 탈출 프로파일
// ============================================================

struct ZoneMotionProfile {
  float entryForwardExtra = 34.5f;
  float entryReverseExtra = 11.0f;
  float exitForwardExtra = 0.0f;
  float exitReverseExtra = 0.0f;
};

inline ZoneMotionProfile getZoneProfile(int zone) {
  ZoneMotionProfile p;
  if (zone == 1) {
    p.exitForwardExtra = 35.0f;
    p.exitReverseExtra = 33.0f;
  } else if (zone == 3) {
    p.exitForwardExtra = 37.0f;
    p.exitReverseExtra = 35.0f;
  } else if (zone == 5 || zone == 6) {
    p.exitReverseExtra = 27.5f;
  }
  return p;
}

inline long zoneCrossApproachDecelSpan(int zone, bool reverse) {
  if (zone == 2) {
    float cm = reverse ? getZoneProfile(1).exitReverseExtra
                       : getZoneProfile(1).exitForwardExtra;
    return toEncoderCounts(cm);
  }
  if (zone == 4) {
    float cm = reverse ? getZoneProfile(3).exitReverseExtra
                       : getZoneProfile(3).exitForwardExtra;
    return toEncoderCounts(cm);
  }
  return 0;
}

// ============================================================
// [7] 직진 가·감속 램프
// ============================================================

constexpr int RAMP_MIN_SPEED = 25;
constexpr int RAMP_REF_SPEED = 40;
constexpr float RAMP_ACCEL_CM = 10.0f;
constexpr float RAMP_DECEL_CM = 17.0f;
constexpr int RAMP_MAX_SPEED_STEP = 10;

inline float rampCruiseFactor(int speed) {
  int s = speed < 0 ? -speed : speed;
  if (s <= 0) return 0.0f;
  return (float)s / (float)RAMP_REF_SPEED;
}

inline int rampAccelSpanCounts(int cruiseSpeed) {
  return toEncoderCounts(RAMP_ACCEL_CM * rampCruiseFactor(cruiseSpeed));
}

inline int rampDecelSpanCounts(int cruiseSpeed) {
  return toEncoderCounts(RAMP_DECEL_CM * rampCruiseFactor(cruiseSpeed));
}

// ============================================================
// [8] 주행 cruise 속도 (모터 출력 0~100)
// ============================================================

constexpr int SPEED_LINE_FOLLOW_FWD = 45;
constexpr int SPEED_LINE_FOLLOW_REV = 35;
constexpr int SPEED_OPEN_ZONE_FWD = 60;
constexpr int SPEED_OPEN_ZONE_REV = 60;
constexpr int SPEED_OPEN_TRACK_FWD = 80;
constexpr int SPEED_OPEN_TRACK_REV = 60;
constexpr int SPEED_9_TO_8 = 40;
constexpr int START_LINE_SEARCH_SPEED = 30;

// ============================================================
// [9] 제자리 회전 (spin)
// ============================================================

constexpr int SPIN_SPEED = 30;
constexpr int SPIN_90_COUNTS = 1170;
constexpr float RAMP_SPIN_ACCEL_DEG = 20.0f;
constexpr float RAMP_SPIN_DECEL_DEG = 40.0f;
constexpr float SPIN_END_DECEL_DEG = 10.0f;
constexpr float SPIN_OVERSHOOT_COMP_FRAC = 0.005f;

constexpr float SPIN_LINE_TRIM_MIN_FRAC = 0.60f;
constexpr float SPIN_LINE_TRIM_REMAIN_FRAC = 0.5f;
constexpr float SPIN_LINE_RECOVER_DEG = 20.0f;

inline float rampSpinAccelDeg(int spinSpeed) {
  return RAMP_SPIN_ACCEL_DEG * rampCruiseFactor(spinSpeed);
}

inline float rampSpinDecelDeg(int spinSpeed) {
  return RAMP_SPIN_DECEL_DEG * rampCruiseFactor(spinSpeed);
}

// ============================================================
// [10] 메인 트랙 가로축 노드 7 — 8 — 9
// ============================================================

constexpr int SPEED_TRACK_7_9_LINE = 70;
constexpr float LINE_KP_TRACK_7_9_SOFT = 1.4f;
constexpr float LINE_KP_TRACK_7_9_HARD = 2.2f;

constexpr float DIST_TRACK_NODE_SPAN_CM = 70.0f;
// 9->8 은 8->9(70cm)와 비대칭. 실측상 9->8 구간은 약 35cm 이므로
// 미리 감속할 수 있도록 별도 거리를 둔다.
constexpr float DIST_9_TO_8_CM = 35.0f;
constexpr float DIST_TRACK_7_TO_9_CM = DIST_TRACK_NODE_SPAN_CM * 2.0f;
constexpr float DIST_TRACK_NODE8_PASS_HALF_CM = 8.0f;
constexpr float DIST_NODE_DETECT_CRAWL_CM = 5.0f;

constexpr float DIST_TRACK_OVERSHOOT_MIN_CM = 0.2f;
constexpr float DIST_TRACK_OVERSHOOT_MAX_CM = 0.5f;
constexpr unsigned long BUZZER_OVERSHOOT_CORR_MS = 200;

inline long trackLegApproachStartCounts(float legSpanCm, int cruiseSpeed) {
  return toEncoderCounts(legSpanCm - RAMP_DECEL_CM * rampCruiseFactor(cruiseSpeed));
}

inline long trackNodeApproachStartCounts(int cruiseSpeed) {
  return trackLegApproachStartCounts(DIST_TRACK_NODE_SPAN_CM, cruiseSpeed);
}

// ============================================================
// [11] 맵 경로 — 스타트 · 13번 · 9번
// ============================================================

constexpr float DIST_START_TO_13_CM = 90.0f;
constexpr float HEADING_13_TO_9 = 305.0f;
constexpr float DIST_TRACK_13_TO_9_CM = 70.0f;

// ============================================================
// [12] 맵 경로 — 9 · 10 · 11 · 12 (남쪽 루프)
// ============================================================

constexpr float HEADING_9_TO_10 = 88.0f;
constexpr float HEADING_9_TO_11 = 89.0f;
constexpr float HEADING_10_TO_11 = 88.0f;
constexpr float HEADING_11_TO_10 = 272.0f;

constexpr float HEADING_10_TO_12 = 240.0f;
constexpr float DIST_10_TO_12_CM = 55.0f;

constexpr float HEADING_11_TO_12 = 255.0f;
constexpr float DIST_11_TO_12_CM = 115.0f;

constexpr float HEADING_12_TO_9_2 = 310.0f;

constexpr float DIST_TRACK_9_TO_10_CM = 60.0f;
constexpr float DIST_TRACK_10_TO_11_CM = 70.0f;
constexpr float DIST_TRACK_9_TO_11_CM = 130.0f;
constexpr float DIST_TRACK_12_TO_9_CM = 20.0f;

// ============================================================
// [13] 맵 경로 — 피니시 (11번 경유 → 스타트박스)
// ============================================================

constexpr float HEADING_11_TO_FINISH = 350.0f;
constexpr float HEADING_FINISH_PARK = 0.0f;
constexpr float DIST_FINISH_BLIND_CONFIRM_CM = 5.0f;
constexpr float DIST_FINISH_PARK_REV_CM = 15.0f;

// ============================================================
// [14] PID 라인 추종 · 직진 보정
// ============================================================

constexpr float LINE_KP_FWD_SOFT = 1.2f;
constexpr float LINE_KP_FWD_HARD = 1.8f;
constexpr float LINE_KP_REV_SOFT = 1.0f;
constexpr float LINE_KP_REV_HARD = 1.3f;
constexpr float LINE_HARD_STEER_SPEED_FACTOR = 0.5f;

constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 9.0f;
constexpr float LINE_ALIGN_GAIN = 1.5f;
constexpr float VELOCITY_KP = 0.2f;
constexpr unsigned long MOTOR_VELOCITY_PID_MS = 10;
constexpr float VELOCITY_TARGET_FACTOR = 0.5f;
constexpr int VELOCITY_MAX_CORRECTION = 10;
constexpr int EDGE_SYNC_GAIN = 5;

constexpr int MOTOR_OFFSET_L = 0;
constexpr int MOTOR_OFFSET_R = 0;

// ============================================================
// [15] 리프트 (Expansion DC #1·#2)
// ============================================================

constexpr float LIFT_COUNTS_PER_CM = 200.0f;
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;
constexpr float LIFT_CARRY_HIGH_CM = LIFT_MAX_HEIGHT_CM;
constexpr float LIFT_CARRY_LOW_CM = 15.0f;

constexpr float LIFT_NEAR_FLOOR_CM = 5.0f;
constexpr float LIFT_UP_CLEAR_CM = 5.0f;
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;
constexpr int LIFT_UP_POWER = 100;
constexpr int LIFT_DOWN_POWER = 100;

constexpr float LIFT_UP_SLOW_ZONE_CM = 20.0f;
constexpr int LIFT_UP_SLOW_POWER_L = 40;
constexpr int LIFT_UP_SLOW_POWER_R = 40;

constexpr float LIFT_DOWN_SLOW_ZONE_CM = 10.0f;
constexpr int LIFT_DOWN_SLOW_POWER_L = 15;
constexpr int LIFT_DOWN_SLOW_POWER_R = 15;

constexpr float LIFT_SYNC_GAIN = 5.0f;
constexpr unsigned long LIFT_TICK_INTERVAL_MS = 15;
constexpr unsigned long LIFT_FLOOR_TIME_MS = 2000;

// ============================================================
// [16] 전역 객체 · 상태 (정의: main.cpp, MapRouter.cpp 등)
// ============================================================

extern PRIZM prizm;
extern int lineTraceLastEdge;
extern bool intersectionArmed;
extern int intersectionHitCount;

// ============================================================
// [17] 레거시 별칭 (예전 이름·스크립트 호환, 변경 불필요)
// ============================================================

constexpr int SENSOR_LEFT = PIN_LINE_FRONT_LEFT;
constexpr int SENSOR_CENTER = PIN_LINE_FRONT_CENTER;
constexpr int SENSOR_RIGHT = PIN_LINE_FRONT_RIGHT;
constexpr int BUZZER_PIN = PIN_BUZZER;
constexpr int SENSOR_REAR_LEFT = PIN_LINE_REAR_LEFT;
constexpr int SENSOR_REAR_CENTER = PIN_LINE_REAR_CENTER;
constexpr int SENSOR_REAR_RIGHT = PIN_LINE_REAR_RIGHT;
constexpr int REAR_SENSOR_THRESHOLD = REAR_LINE_THRESHOLD;

inline int CM(float cm) { return toEncoderCounts(cm); }

using ZoneCfg = ZoneMotionProfile;
inline ZoneCfg zoneCfg(int z) { return getZoneProfile(z); }

constexpr int SPEED_TRACE_FWD = SPEED_LINE_FOLLOW_FWD;
constexpr int SPEED_TRACE_REV = SPEED_LINE_FOLLOW_REV;
constexpr int SPEED_BLIND_FWD = SPEED_OPEN_TRACK_FWD;
constexpr int SPEED_BLIND_REV = SPEED_OPEN_TRACK_REV;

#endif
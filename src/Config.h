/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ============================================================
// [1] 펌웨어 모드 선택
// ============================================================
// RUN_TEST_MODE: 0 = 대회용 자율주행 / 1 = 센서·리프트·이동 테스트 메뉴
#define RUN_TEST_MODE 0
// QR_SIMULATION: 0 = 실제 HuskyLens QR / 1 = 랜덤 가상 배치(카메라 불필요)
#define QR_SIMULATION 0

// ============================================================
// [2] HuskyLens / QR 스캔
// ============================================================
constexpr int MAX_RESCAN_TRIES = 5; // 1~4구역 재스캔 최대 반복 (무한루프 방지)
constexpr unsigned long SCAN_DWELL_MS = 3000; // 존 정지 후 QR 대기 시간 (ms)
constexpr unsigned long SCAN_POLL_MS = 30; // pollZoneScan() I2C 폴링 간격 (ms)

// ============================================================
// [3] 디버그 시리얼 출력
// ============================================================
#define ROBOT_DEBUG 0 // 0=플래시 절약 / 1=시리얼 디버그

#if ROBOT_DEBUG
#define DPRINT(x) Serial.print(x)
#define DPRINTLN(x) Serial.println(x)
#define DPRINTF(x) Serial.print(F(x)) // 문자열을 PROGMEM(플래시)에 저장
#define DPRINTLNF(x) Serial.println(F(x))
#else
#define DPRINT(x)
#define DPRINTLN(x)
#define DPRINTF(x)
#define DPRINTLNF(x)
#endif

inline void checkDebugKey() {}

// ============================================================
// [4] 엔코더 / 거리 변환
// ============================================================
constexpr float COUNTS_PER_CM =
    3570.0f / 80.0f; // 80cm 주행 시 3570 카운트 (실측)
constexpr int toEncoderCounts(float cm) {
  return (int)(cm * COUNTS_PER_CM + 0.5f);
}

// ============================================================
// [5] 핀 번호 및 하드웨어
// ============================================================
constexpr int PIN_LINE_FRONT_LEFT = 2;   // 전방 라인센서 좌 (디지털)
constexpr int PIN_LINE_FRONT_CENTER = 3; // 전방 라인센서 중앙
constexpr int PIN_LINE_FRONT_RIGHT = 4;  // 전방 라인센서 우
constexpr bool INVERT_LINE_SENSORS =
    false; // true = 센서 극성 반전 (흰 바탕·검은 선)
constexpr int PIN_BUZZER = 5;

constexpr int PIN_LINE_REAR_LEFT = A1; // 후방 라인센서 좌 (아날로그)
constexpr int PIN_LINE_REAR_CENTER = A2;
constexpr int PIN_LINE_REAR_RIGHT = A3;
constexpr int REAR_LINE_THRESHOLD = 200; // 후방 센서 ON 판정 (0~1023)

// ============================================================
// [6] 로봇 물리 치수 (바퀴축 기준, cm)
// ============================================================
constexpr float ROBOT_LENGTH_CM = 35.0f;             // 로봇 전체 길이
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f; // 바퀴축 → 전방 센서
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;  // 바퀴축 → 후방 센서
constexpr float DIST_AXIS_TO_LIFT_CM = 10.5f;        // 바퀴축 → 리프트 중심
constexpr float LINE_THICKNESS_CM =
    1.0f; // 선 두께 보정 (실제 2cm − 제동 여유 1cm)

// ============================================================
// [7] 존 진입 / 탈출 거리
// ============================================================
constexpr float DIST_CROSS_ALIGN_CM =
    DIST_AXIS_TO_FRONT_SENSOR_CM; // 교차로 바퀴축 정렬 (6cm)
constexpr float DIST_BRAKE_CATCH_CM =
    0.2f; // 목표 직전 125 강제제동 (관성·저속 잔행 흡수)
constexpr float DIST_IGNORE_NODE_CM = 5.0f; // 교차로 직후 오인식 무시 거리

// 구역별 진입·탈출 추가 이동 거리 (cm)
struct ZoneMotionProfile {
  float entryForwardExtra = 35.0f; // 전진 진입: 입구 선 통과 후 추가 전진
  float entryReverseExtra = 12.0f; // 후진 진입: 입구 선 통과 후 추가 후진
  float exitForwardExtra = 0.0f;   // 전진 탈출: 탈출선 감지 후 라인 추종 거리
  float exitReverseExtra = 0.0f;   // 후진 탈출: 탈출선 감지 후 라인 추종 거리
};

inline ZoneMotionProfile getZoneProfile(int zone) {
  ZoneMotionProfile p;
  // 1·3구역: T자 교차로 — 교차 감지 불가, 거리 기반 탈출
  if (zone == 1) {
    p.exitForwardExtra = 34.0f;
    p.exitReverseExtra = 32.0f;
  } else if (zone == 3) {
    p.exitForwardExtra = 36.0f;
    p.exitReverseExtra = 34.0f;
  }
  // 2·4: 십자(┼) 센서로 회전 위치 결정 — exit extra 없음 (crossToOppositeZone은 별도)
  // 5·6구역: 항상 전진 진입 → 후진 탈출만
  else if (zone == 5 || zone == 6) {
    p.exitReverseExtra = 26.0f;
  }
  return p;
}

// 2·4번 존 → 8번 노드: 1·3 탈출 거리는 교차로 접근 감속 구간 참고용만
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
// [8] 주행 속도
// ============================================================
// ── 가·감속 — cruise 속도 40 기준 cm(·deg) 입력, cruise≠40이면 거리만 ×(speed/40) ──
constexpr int RAMP_MIN_SPEED = 15;       // 가속 시작·감속 끝 최저 속도
constexpr int RAMP_REF_SPEED = 40;       // 아래 거리 입력 기준 cruise
constexpr float RAMP_ACCEL_CM = 12.0f;  // 가속 거리 (cm) @ cruise 40
constexpr float RAMP_DECEL_CM = 18.0f;  // 감속 거리 (cm) @ cruise 40

// ── 속도 ──
constexpr int SPEED_LINE_FOLLOW_FWD = 50; // 라인 추종 전진
constexpr int SPEED_LINE_FOLLOW_REV = 50; // 라인 추종 후진
constexpr int SPEED_OPEN_ZONE_FWD = 50;   // 맹목 전진 — 박스 존 안
constexpr int SPEED_OPEN_ZONE_REV = 50;   // 맹목 후진 — 박스 존 안
constexpr int SPEED_OPEN_TRACK_FWD = 50;  // 맹목 전진 — 메인 트랙
constexpr int SPEED_OPEN_TRACK_REV = 50;  // 맹목 후진 — 메인 트랙

// ============================================================
// [9] 라인 / 교차로 감지 안정화
// ============================================================
constexpr int EXIT_LINE_CONFIRM = 2; // 탈출 라인 연속 감지 횟수 (노이즈 방지)
constexpr int START_LINE_SEARCH_SPEED = 25; // 스타트→메인라인 최초 탐색 속도

// ============================================================
// [10] 제자리 회전
// ============================================================
constexpr int SPIN_SPEED = 40;               // 회전 모터 출력
constexpr int SPIN_90_COUNTS = 1170;         // 90° 회전 엔코더 카운트 (실측)
constexpr float RAMP_SPIN_ACCEL_DEG = 12.0f; // 회전 가속 거리 (deg) @ SPIN_SPEED 40
constexpr float RAMP_SPIN_DECEL_DEG = 12.0f; // 회전 감속 거리 (deg) @ SPIN_SPEED 40
constexpr float SPIN_LINE_TRIM_MIN_FRAC =
    0.70f; // 오버슈팅 방지: 목표 각도의 70% 이후부터 라인 감지

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
inline float rampSpinAccelDeg(int spinSpeed) {
  return RAMP_SPIN_ACCEL_DEG * rampCruiseFactor(spinSpeed);
}
inline float rampSpinDecelDeg(int spinSpeed) {
  return RAMP_SPIN_DECEL_DEG * rampCruiseFactor(spinSpeed);
}

// ============================================================
// [11] PID 라인 추종 게인
// ============================================================
constexpr float LINE_KP_FWD_SOFT = 1.2f; // 전진 P (오차 ±1)
constexpr float LINE_KP_FWD_HARD = 4.5f; // 전진 P (오차 ±2 이상)
constexpr float LINE_KP_REV_SOFT = 1.0f; // 후진 P (오차 ±1)
constexpr float LINE_KP_REV_HARD = 2.2f; // 후진 P (오차 ±2 이상)

constexpr float LINE_KI = 0.0f;         // 적분 (미사용)
constexpr float LINE_KD = 9.0f;         // 미분 (진동 억제)
constexpr float LINE_ALIGN_GAIN = 1.5f; // 전·후방 센서 정렬 보정

constexpr float VELOCITY_KP = 0.2f; // 엔코더 속도 피드백 P
constexpr float VELOCITY_TARGET_FACTOR = 0.5f;
constexpr int VELOCITY_MAX_CORRECTION = 10;
constexpr int EDGE_SYNC_GAIN = 5; // T자 구역 좌우 엣지 동기화

constexpr int MOTOR_OFFSET_L = 0; // 좌측 모터 직진 미세 보정
constexpr int MOTOR_OFFSET_R = 0;
constexpr int CROSS_CONFIRM = 1; // 교차로(1·1·1) 연속 감지 횟수

// ============================================================
// [12] 맵 노드 간 방위각·거리
// 방위: 0=북, 90=동, 180=남, 270=서 / rotateToHeading(deg) = 최소 각도 회전
// ============================================================
constexpr int HEADING_10_TO_12 = 210;
constexpr float DIST_10_TO_12_CM = 50.0f;
constexpr int HEADING_11_TO_12 = 250;
constexpr float DIST_11_TO_12_CM = 110.0f;
constexpr int HEADING_12_TO_9_2 = 310;

constexpr float DIST_START_TO_13_CM = 90.0f; // 스타트 → 13번 노드
constexpr int HEADING_13_TO_9 = 305;
constexpr int HEADING_9_TO_13 = 150;
constexpr float DIST_9_TO_13_CM = 40.0f;
constexpr int HEADING_10_TO_13 = 180;
constexpr float DIST_10_TO_13_CM = 55.0f;
constexpr int HEADING_13_TO_START = 90;
constexpr float DIST_FINISH_ENTRY_CM = 40.0f; // 피니시 라인 진입 거리

constexpr int HEADING_9_TO_10 = 86;
constexpr int HEADING_9_TO_11 = 88;
constexpr int HEADING_10_TO_11 = 88;
constexpr int HEADING_11_TO_10 = 270;

// ============================================================
// [13] 리프트
// ============================================================
constexpr float LIFT_COUNTS_PER_CM = 200.0f;
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f; // 상승 한계
constexpr float LIFT_NEAR_FLOOR_CM = 5.0f;  // 바닥 근접 감지
constexpr float LIFT_UP_CLEAR_CM = 5.0f;    // 주행 허가 상승 높이
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;  // 주행 허가 하강 높이

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

// ============================================================
// [14] 전역 상태 (main.cpp에서 정의)
// ============================================================
extern PRIZM prizm;
extern int lineTraceLastEdge;
extern bool intersectionArmed;
extern int intersectionHitCount;

// ── 구 이름 별칭 (외부 문서·스크립트 호환) ──
constexpr int SENSOR_LEFT = PIN_LINE_FRONT_LEFT;
constexpr int SENSOR_CENTER = PIN_LINE_FRONT_CENTER;
constexpr int SENSOR_RIGHT = PIN_LINE_FRONT_RIGHT;
constexpr int BUZZER_PIN = PIN_BUZZER;
constexpr int SENSOR_REAR_LEFT = PIN_LINE_REAR_LEFT;
constexpr int SENSOR_REAR_CENTER = PIN_LINE_REAR_CENTER;
constexpr int SENSOR_REAR_RIGHT = PIN_LINE_REAR_RIGHT;
constexpr int REAR_SENSOR_THRESHOLD = REAR_LINE_THRESHOLD;
constexpr int CM(float cm) { return toEncoderCounts(cm); }
using ZoneCfg = ZoneMotionProfile;
inline ZoneCfg zoneCfg(int z) { return getZoneProfile(z); }
constexpr int SPEED_TRACE_FWD = SPEED_LINE_FOLLOW_FWD;
constexpr int SPEED_TRACE_REV = SPEED_LINE_FOLLOW_REV;
constexpr int SPEED_BLIND_FWD = SPEED_OPEN_TRACK_FWD;
constexpr int SPEED_BLIND_REV = SPEED_OPEN_TRACK_REV;

#endif

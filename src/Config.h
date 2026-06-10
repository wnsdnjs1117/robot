/* ============================================================
 * Config.h — 로봇 튜닝 파라미터 (Arduino UNO + TETRIX PRIZM)
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ============================================================
// [1] 펌웨어 모드
// ============================================================
#define RUN_TEST_MODE 0  // 1이면 테스트 모드 실행
#define QR_SIMULATION 0  // 1이면 QR 스캔 시뮬레이션 활성화

// ============================================================
// [2] 핀 · 센서 하드웨어
// ============================================================
constexpr int PIN_LINE_FRONT_LEFT = 2;       // 전방 좌측 라인 센서 핀
constexpr int PIN_LINE_FRONT_CENTER = 3;     // 전방 중앙 라인 센서 핀
constexpr int PIN_LINE_FRONT_RIGHT = 4;      // 전방 우측 라인 센서 핀
constexpr bool INVERT_LINE_SENSORS = false;  // 라인 센서 값 반전 여부 (흰선/검은선)

constexpr int PIN_BUZZER = 5;  // 부저 핀

constexpr int PIN_LINE_REAR_LEFT = A1;    // 후방 좌측 아날로그 라인 센서 핀
constexpr int PIN_LINE_REAR_CENTER = A2;  // 후방 중앙 아날로그 라인 센서 핀
constexpr int PIN_LINE_REAR_RIGHT = A3;   // 후방 우측 아날로그 라인 센서 핀
constexpr int REAR_LINE_THRESHOLD = 200;  // 후방 아날로그 센서 흑/백 판단 임계값

////////////////////// cm단위 count계산
constexpr float COUNTS_PER_CM = 3700.0f / 80.0f;  // 1cm 이동에 해당하는 엔코더 카운트
constexpr int SPIN_90_COUNTS = 1170;              // 90도 회전에 해당하는 기준 엔코더 카운트
//////////////
// ============================================================
// [3] 물리 치수 · 엔코더 (바퀴축 기준, cm)
// ============================================================
constexpr float ROBOT_LENGTH_CM = 35.0f;              // 로봇 전체 세로 길이
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;  // 바퀴 중심축 ~ 전방 센서까지의 거리
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;   // 바퀴 중심축 ~ 후방 센서까지의 거리
constexpr float DIST_AXIS_TO_LIFT_CM = 10.5f;         // 바퀴 중심축 ~ 리프트까지의 거리
constexpr float LINE_THICKNESS_CM = 2.0f;             // 바닥 라인의 두께

inline int toEncoderCounts(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); }

// ============================================================
// [4] 부저 · 사운드
// ============================================================
constexpr unsigned int BUZZER_TONE_HZ = 1000;                             // 부저 톤 주파수 (Hz)
constexpr unsigned long BUZZER_TONE_HALF_US = 500000UL / BUZZER_TONE_HZ;  // 부저 반주기 (us)
constexpr unsigned long BUZZER_QR_FOUND_MS = 500;                         // QR 인식 성공 시 부저 울림 시간 (ms)
constexpr unsigned long BUZZER_OVERSHOOT_CORR_MS = 200;                   // 오버슛 복구 시 부저 울림 시간
constexpr unsigned long BUZZER_FINISH_MS = 2000;                          // 미션 종료 성공 시 부저 유지 시간

// ============================================================
// [5] QR 스캔 · 재탐색
// ============================================================
constexpr int MAX_RESCAN_TRIES = 99;           // QR 스캔 실패 시 최대 재시도 횟수
constexpr unsigned long SCAN_DWELL_MS = 1500;  // QR 스캔 대기 시간 (ms)
constexpr unsigned long SCAN_POLL_MS = 10;     // QR 스캔 폴링 주기 (ms)

// ============================================================
// [6] 직진 가·감속 램프 (부드러운 출발/정지)
// ============================================================
constexpr int RAMP_MIN_SPEED = 35;      // 가감속 최소(출발/도착) 속도
constexpr int RAMP_REF_SPEED = 40;      // 가감속 비율 계산을 위한 기준 속도
constexpr float RAMP_ACCEL_CM = 13.0f;  // 목표 속도 도달에 필요한 가속 구간 거리
constexpr float RAMP_DECEL_CM = 15.0f;  // 정지를 위한 감속 구간 거리
constexpr int RAMP_MAX_SPEED_STEP = 5;  // 모터 1틱당 허용되는 최대 속도 변화량

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
// [7] 주행 cruise 속도 (모터 출력 0~100)
// ============================================================
constexpr int SPEED_LINE_FOLLOW_FWD = 40;    // 전진 라인 트레이싱 기본 속도
constexpr int SPEED_LINE_FOLLOW_REV = 35;    // 후진 라인 트레이싱 기본 속도
constexpr int SPEED_OPEN_ZONE_FWD = 65;      // 박스존 내 전진 개방(블라인드) 속도
constexpr int SPEED_OPEN_ZONE_REV = 45;      // 박스존 내 후진 개방(블라인드) 속도
constexpr int SPEED_OPEN_TRACK_FWD = 85;     // 일반 트랙 전진 개방(블라인드) 속도
constexpr int SPEED_OPEN_TRACK_REV = 60;     // 일반 트랙 후진 개방(블라인드) 속도
constexpr int SPEED_TRACK_7_9_LINE = 70;     // 7, 8, 9번 메인 트랙 고속 주행 속도
constexpr int START_LINE_SEARCH_SPEED = 15;  // 스타트 직후 최초 라인 탐색 진입 속도

// ============================================================
// [8] PID 라인 추종 · 조향 게인
// ============================================================
constexpr float LINE_KP_FWD_SOFT = 1.2f;              // 전진 라인 P게인 (오차가 작을 때)
constexpr float LINE_KP_FWD_HARD = 1.8f;              // 전진 라인 P게인 (오차가 클 때)
constexpr float LINE_KP_REV_SOFT = 1.0f;              // 후진 라인 P게인 (오차가 작을 때)
constexpr float LINE_KP_REV_HARD = 1.4f;              // 후진 라인 P게인 (오차가 클 때)
constexpr float LINE_KP_TRACK_7_9_SOFT = 1.5f;        // 메인 트랙(7-8-9) 라인 부드러운 조향 게인
constexpr float LINE_KP_TRACK_7_9_HARD = 2.2f;        // 메인 트랙(7-8-9) 라인 강한 조향 게인
constexpr float LINE_HARD_STEER_SPEED_FACTOR = 0.6f;  // 강한 조향 시 속도 감속 비율

constexpr float LINE_KI = 0.0f;                      // 라인 트레이싱 I게인
constexpr float LINE_KD = 9.0f;                      // 라인 트레이싱 D게인
constexpr float LINE_ALIGN_GAIN = 1.5f;              // 전/후방 센서 편차 기반 차체 정렬 게인
constexpr float VELOCITY_KP = 0.2f;                  // 양륜 직진 동기화 P게인
constexpr unsigned long MOTOR_VELOCITY_PID_MS = 10;  // 모터 속도 제어 루프 주기 (ms)
constexpr float VELOCITY_TARGET_FACTOR = 0.5f;       // 모터 파워 대비 엔코더 속도 목표 변환비
constexpr int VELOCITY_MAX_CORRECTION = 10;          // 속도 제어 최대 보정 출력 한계
constexpr int EDGE_SYNC_GAIN = 5;                    // 라인 엣지(가장자리) 통과 시 동기화 게인

constexpr int MOTOR_OFFSET_L = 0;  // 왼쪽 모터 하드웨어 출력 보정 오프셋
constexpr int MOTOR_OFFSET_R = 0;  // 오른쪽 모터 하드웨어 출력 보정 오프셋

// ============================================================
// [9] 교차로 감지 · 목표 정지 (트랙 공통)
// ============================================================
constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM;  // 교차로 감지 후 정렬을 위한 추가 직진 거리
constexpr float DIST_BRAKE_CATCH_CM = 0.3f;                          // 제동 시 미끄러짐 기본 보정치
constexpr float DIST_BRAKE_CATCH_MAX_CM = 4.0f;                      // 제동 시 미끄러짐 최대 보정치
constexpr float DIST_IGNORE_NODE_CM = 5.0f;                          // 교차로 통과 직후 센서 감지를 무시할 거리
constexpr int CROSS_CONFIRM = 1;                                     // 교차로 연속 감지 인정 횟수
constexpr int EXIT_LINE_CONFIRM = 1;                                 // 탈출 라인 연속 감지 인정 횟수

// ============================================================
// [10] 박스 존(1~6) 진입 · 탈출 프로파일
// ============================================================
struct ZoneMotionProfile {
  float entryForwardExtra = 33.7f;  // 라인 감지 후 박스 존 진입을 위한 추가 주행 거리 (전진)
  float entryReverseExtra = 11.5f;  // 라인 감지 후 박스 존 진입을 위한 추가 주행 거리 (전진/후진)
  float exitForwardExtra = 0.0f;    // 전진 탈출 시 라인 감지 후 추가 주행 거리
  float exitReverseExtra = 0.0f;    // 후진 탈출 시 라인 감지 후 추가 주행 거리
};

inline ZoneMotionProfile getZoneProfile(int zone) {
  ZoneMotionProfile p;
  if (zone == 1) {
    p.exitForwardExtra = 34.0f;
    p.exitReverseExtra = 31.0f;  // [실측 반영]
  } else if (zone == 3) {
    p.exitForwardExtra = 36.0f;
    p.exitReverseExtra = 33.0f;  // [실측 반영]
  } else if (zone == 5 || zone == 6) {
    p.exitReverseExtra = 27.5f;
  }
  return p;
}

inline long zoneCrossApproachDecelSpan(int zone, bool reverse) {
  if (zone == 2) {
    float cm = reverse ? getZoneProfile(1).exitReverseExtra : getZoneProfile(1).exitForwardExtra;
    return toEncoderCounts(cm);
  }
  if (zone == 4) {
    float cm = reverse ? getZoneProfile(3).exitReverseExtra : getZoneProfile(3).exitForwardExtra;
    return toEncoderCounts(cm);
  }
  return 0;
}

// ============================================================
// [11] 제자리 회전 (Spin Turn)
// ============================================================
constexpr int SPIN_SPEED = 35;                      // [실측 반영] 제자리 회전 속도 (고정)
constexpr float SPIN_OVERSHOOT_COMP_FRAC = 0.005f;  // 회전 관성으로 인한 오버슛 사전 보정 비율

constexpr float SPIN_LINE_TRIM_REMAIN_FRAC = 0.6f;  // 라인 감지 후 남은 회전 타겟 감소 비율
constexpr float SPIN_OPPOSITE_CHECK_FRAC = 0.50f;   // 회전 중 반대쪽 센서 감시 시작 지점 (50%)
constexpr float SPIN_LINE_TRIM_MIN_FRAC = 0.80f;    // [실측 반영] 회전 중 라인 정밀 맞춤을 시작할 최소 회전량
constexpr float SPIN_LINE_RECOVER_DEG = 20.0f;      // 라인을 놓친(오버스핀) 경우 되돌아오는 복구 각도 (도)

// ============================================================
// [12] 메인 트랙 가로축 노드 7 — 8 — 9
// ============================================================
constexpr float DIST_TRACK_NODE_SPAN_CM = 68.0f;  // 7-8 및 8-9 노드 간 기본 간격
constexpr float DIST_TRACK_8_TO_9_CM = 55.0f;     // 8->9 진입 시 감속을 시작할 거리
constexpr float DIST_9_TO_8_CM = 50.0f;           // 9->8 진입 시 감속을 시작할 비대칭 거리
constexpr float DIST_TRACK_7_TO_9_CM =
    DIST_TRACK_NODE_SPAN_CM +
    DIST_TRACK_8_TO_9_CM;  // 7->9 연속 거리(7-8 + 8-9 비대칭 반영). 이 값으로 7->9 감속 시점이 정해짐
constexpr float DIST_TRACK_NODE8_PASS_HALF_CM = 8.0f;  // 8번 통과 시 십자선 감지를 무시할 반경
constexpr float DIST_NODE_DETECT_CRAWL_CM = 5.0f;      // 십자선 감지 직전 기어가기 거리

constexpr float DIST_TRACK_OVERSHOOT_MIN_CM = 0.2f;  // 오버슛 최소 무시 거리 (레거시)
constexpr float DIST_TRACK_OVERSHOOT_MAX_CM = 0.5f;  // 오버슛 최대 무시 거리 (레거시)

inline long trackLegApproachStartCounts(float legSpanCm, int cruiseSpeed) {
  return toEncoderCounts(legSpanCm - RAMP_DECEL_CM * rampCruiseFactor(cruiseSpeed));
}

inline long trackNodeApproachStartCounts(int cruiseSpeed) {
  return trackLegApproachStartCounts(DIST_TRACK_NODE_SPAN_CM, cruiseSpeed);
}

// ============================================================
// [13] 맵 경로 — 스타트 · 13번 · 9번
// ============================================================
constexpr float DIST_START_TO_13_CM = 80.0f;    // [실측 반영] 스타트 박스에서 13번 노드까지 거리
constexpr float HEADING_13_TO_9 = 305.0f;       // 13에서 9로 향하는 대각선 각도
constexpr float DIST_TRACK_13_TO_9_CM = 70.0f;  // 13에서 9까지의 대각선 주행 거리

// ============================================================
// [14] 맵 경로 — 9 · 10 · 11 · 12 (남쪽 루프)
// ============================================================
constexpr float HEADING_9_TO_10 = 88.0f;    // 9 -> 10 이동 목표 각도
constexpr float HEADING_9_TO_11 = 89.0f;    // 9 -> 11 이동 목표 각도
constexpr float HEADING_10_TO_11 = 87.0f;   // 10 -> 11 이동 목표 각도
constexpr float HEADING_11_TO_10 = 273.0f;  // 11 -> 10 이동 목표 각도

constexpr float HEADING_10_TO_12 = 245.0f;  // 10 -> 12 이동 목표 각도
constexpr float DIST_10_TO_12_CM = 55.0f;   // 10 -> 12 직진 주행 거리

constexpr float HEADING_11_TO_12 = 255.0f;  // 11 -> 12 이동 목표 각도
constexpr float DIST_11_TO_12_CM = 110.0f;  // 11 -> 12 직진 주행 거리

constexpr float HEADING_12_TO_9_2 = 307.0f;  // 12 -> 9 배송 복귀 목표 각도

constexpr float DIST_TRACK_9_TO_10_CM = 60.0f;   // 9 -> 10 트랙 직진 거리
constexpr float DIST_TRACK_10_TO_11_CM = 70.0f;  // 10 -> 11 트랙 직진 거리
constexpr float DIST_TRACK_9_TO_11_CM = 130.0f;  // 9 -> 11 연속 트랙 거리
constexpr float DIST_TRACK_12_TO_9_CM = 35.0f;   // 12 -> 9 배송 복귀 직진 거리

// ============================================================
// [15] 맵 경로 — 피니시 (11번 경유 → 스타트박스)
// ============================================================
constexpr float HEADING_11_TO_FINISH = 347.0f;         // 11번에서 피니시(스타트박스)로 향하는 각도
constexpr float DIST_FINISH_BLIND_CONFIRM_CM = 10.0f;  // 이만큼 연속 블라인드를 밟은 뒤에야 다음 선을 스타트박스로 인식 (11선↔스타트박스 갭 19cm보다 작아야 함)
constexpr float DIST_FINISH_AFTER_TOUCH_CM = 5.0f;     // 스타트박스 후방 터치 후 추가 후진 거리
constexpr float FINISH_TURN_DEG = 7.0f;                // 벽면 정렬을 위한 마무리 꺾임 각도 (+시계)
constexpr float DIST_FINISH_PARK_REV_CM = 10.0f;       // 꺾은 후 최종 주차 후진 거리

// ============================================================
// [16] 리프트 (Expansion DC #1·#2)
// ============================================================
constexpr float LIFT_COUNTS_PER_CM = 195.0f;              // 리프트 1cm 당 엔코더 카운트
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;               // 리프트 최대 상승 가능 높이
constexpr float LIFT_CARRY_HIGH_CM = LIFT_MAX_HEIGHT_CM;  // 장애물 존 통과용 높은 높이
constexpr float LIFT_CARRY_LOW_CM = 19.5f;                // 1<->3·5<->6 운반 높이(장애물 없음)

constexpr float LIFT_NEAR_FLOOR_CM = 5.0f;  // 바닥 근접으로 판단하는 높이
constexpr float LIFT_UP_CLEAR_CM = 4.0f;    // [실측 반영] 박스 들기 완료 판단 높이
constexpr float LIFT_DOWN_CLEAR_CM = 1.0f;  // 박스 내려놓기 완료 판단 높이
constexpr int LIFT_UP_POWER = 100;          // 상승 기본 파워
constexpr int LIFT_DOWN_POWER = 100;        // 하강 기본 파워

constexpr float LIFT_UP_SLOW_ZONE_CM = 18.0f;  // 상승 중 속도를 줄이기 시작할 높이
constexpr int LIFT_UP_SLOW_POWER_L = 50;       // 상승 슬로우 존 왼쪽 파워
constexpr int LIFT_UP_SLOW_POWER_R = 50;       // 상승 슬로우 존 오른쪽 파워

constexpr float LIFT_DOWN_SLOW_ZONE_CM = 10.0f;  // 하강 중 속도를 줄이기 시작할 높이
constexpr int LIFT_DOWN_SLOW_POWER_L = 20;       // 하강 슬로우 존 왼쪽 파워
constexpr int LIFT_DOWN_SLOW_POWER_R = 20;       // 하강 슬로우 존 오른쪽 파워

constexpr float LIFT_SYNC_GAIN = 5.0f;               // 좌우 리프트 높이 동기화 게인
constexpr unsigned long LIFT_TICK_INTERVAL_MS = 20;  // 리프트 제어 루프 주기 (ms)
constexpr unsigned long LIFT_FLOOR_TIME_MS = 2000;   // 바닥 도달 후 영점 초기화 보장 시간 (ms)

// ============================================================
// [17] 전역 객체 · 상태 (정의: main.cpp, MapRouter.cpp 등)
// ============================================================
extern PRIZM prizm;               // TETRIX PRIZM 메인 컨트롤러 객체+
extern int lineTraceLastEdge;     // 라인 트레이싱 마지막 이탈 방향
extern bool intersectionArmed;    // 교차로 십자선 감지 활성화 플래그
extern int intersectionHitCount;  // 교차로 센서 연속 감지 누적 횟수

// ============================================================
// [18] 레거시 별칭 (예전 스크립트 호환성 유지용)
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

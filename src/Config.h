/* ============================================================
 * Config.h — 로봇 튜닝 파라미터 (Arduino UNO + TETRIX PRIZM)
 *
 * [ 읽는 법 ]
 *  - 모든 상수는 "값 한 줄 + 바로 아래 설명 한 줄" 형식.
 *  - 거리=cm, 각도=°(도), 속도=모터출력 0~100.
 *
 * [ 가·감속 규칙 ]
 *  - 직진: RAMP_ACCEL_CM / RAMP_DECEL_CM  (cruise 40 기준 거리)
 *  - 회전: RAMP_SPIN_ACCEL_DEG / RAMP_SPIN_DECEL_DEG (각도)
 *  - cruise가 40이 아니면 구간 길이만 × (speed / 40) 으로 스케일.
 *
 * [ 방위(heading) ]  0=북, 90=동, 180=남, 270=서 — rotateToHeading()은 최소각 회전.
 * [ 정지 ]          목표 직전 stopMotors() → 모터 125 급제동.
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ============================================================
// [1] 펌웨어 모드 · 디버그
// ============================================================

#define RUN_TEST_MODE 0
// 0 = 대회 자율주행  /  1 = 센서·리프트·이동 테스트 메뉴 (TestMode.cpp)

#define QR_SIMULATION 0
// 0 = HuskyLens 실제 QR  /  1 = 랜덤 가상 배치 (카메라 없이 개발)

#define ROBOT_DEBUG 0
// 0 = Serial 디버그 OFF (플래시 절약, Start 1회로 경기 시작)
// 1 = Serial 디버그 ON  (Start 2회: PrizmBegin + 경기 시작)

#if ROBOT_DEBUG
#define DPRINT(x) Serial.print(x)
#define DPRINTLN(x) Serial.println(x)
#define DPRINTF(x) Serial.print(F(x)) // PROGMEM 문자열
#define DPRINTLNF(x) Serial.println(F(x))
#else
#define DPRINT(x)
#define DPRINTLN(x)
#define DPRINTF(x)
#define DPRINTLNF(x)
#endif

inline void checkDebugKey() {
} // ROBOT_DEBUG=1 일 때만 Motion 루프에서 키 확인 (확장용)

// ============================================================
// [2] QR 스캔 · 재탐색
// ============================================================

constexpr int MAX_RESCAN_TRIES = 99;
// 1~4구역 QR 미발견 시 leave→재진입 최대 반복 (무한루프 방지)

constexpr unsigned long SCAN_DWELL_MS = 1500;
// 존 진입 후 QR 읽기 대기 시간 (ms)

constexpr unsigned long SCAN_POLL_MS = 30;
// pollZoneScan() HuskyLens I2C 폴링 주기 (ms)

constexpr unsigned long BUZZER_QR_FOUND_MS = 1000;
// QR 박스 인식 성공 알림 (ms)

// ============================================================
// [3] 핀 · 센서 하드웨어
// ============================================================

constexpr int PIN_LINE_FRONT_LEFT = 2;
constexpr int PIN_LINE_FRONT_CENTER = 3;
constexpr int PIN_LINE_FRONT_RIGHT = 4;
// 전방 라인 센서 (디지털, PRIZM D2~D4)

constexpr bool INVERT_LINE_SENSORS = false;
// true = 센서 출력 극성 반전 (흰 바탕·검은 선 환경)

constexpr int PIN_BUZZER = 5;
// 피니시·알림 부저

constexpr int PIN_LINE_REAR_LEFT = A1;
constexpr int PIN_LINE_REAR_CENTER = A2;
constexpr int PIN_LINE_REAR_RIGHT = A3;
// 후방 라인 센서 (아날로그 A1~A3)

constexpr int REAR_LINE_THRESHOLD = 200;
// 후방 아날로그 ON 판정 (0~1023, 값 이상이면 선 위)

// ============================================================
// [4] 물리 치수 · 엔코더 (바퀴축 기준, cm)
// ============================================================

constexpr float ROBOT_LENGTH_CM = 35.0f;
// 로봇 전후 전체 길이

constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;
// 바퀴축 → 전방 라인센서 (교차로 정렬 거리와 동일)

constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;
// 바퀴축 → 후방 라인센서 (2·4번 존 역방향 탈출 정렬)

constexpr float DIST_AXIS_TO_LIFT_CM = 10.5f;
// 바퀴축 → 리프트 중심 (참고 치수)

constexpr float LINE_THICKNESS_CM = 1.0f;
// 선 두께 보정 (실측 2cm − 제동 여유 1cm)

constexpr float COUNTS_PER_CM = 3570.0f / 80.0f;
// 주행 엔코더 환산: 80cm 주행 시 3570 카운트 (실측)

inline int toEncoderCounts(float cm) {
  return (int)(cm * COUNTS_PER_CM + 0.5f);
}

// ============================================================
// [5] 교차로 감지 · 목표 정지 (트랙 공통)
// ============================================================

constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM;
// 교차로(1·1·1) 감지 후 바퀴축을 교차 중심에 맞추는 전진 거리 (= 6cm)

constexpr float DIST_BRAKE_CATCH_CM = 0.3f;
// 급제동 시작 거리(저속 기준). 목표가 이만큼 남으면 stopMotors() 급제동.

constexpr float DIST_BRAKE_CATCH_MAX_CM = 2.0f;
// 급제동 시작 거리(속도 100 기준) = 고속에서의 실제 정지거리.
// 현재속도(명령·실측 중 큰 값)에 비례해 DIST_BRAKE_CATCH_CM → 이 값으로 선형 증가.
// 코너 정렬(전진 6cm)에서 고속 진입 시 인식 직후부터 제동 → 정렬 구간 내 정지(오버슈트 방지).

constexpr float DIST_IGNORE_NODE_CM = 5.0f;
// 교차로 통과 직후 십자(111) 오인식 무시 / 저속 크롤 구간 (cm)

constexpr int CROSS_CONFIRM = 1;
// 전방 1·1·1 교차 패턴 연속 확인 횟수 (1=즉시)

constexpr int EXIT_LINE_CONFIRM = 1;
// 존 탈출 라인 연속 감지 횟수 (1=즉시)

// ============================================================
// [6] 박스 존(1~6) 진입 · 탈출 프로파일
// ============================================================

struct ZoneMotionProfile {
  float entryForwardExtra = 34.5f; // 전진 진입: 입구 선 통과 후 추가 전진 (cm)
  float entryReverseExtra = 11.0f; // 후진 진입: 입구 선 통과 후 추가 후진 (cm)
  float exitForwardExtra = 0.0f;   // 전진 탈출: 탈출선 이후 추가 거리 (cm)
  float exitReverseExtra = 0.0f;   // 후진 탈출: 탈출선 이후 추가 거리 (cm)
};

inline ZoneMotionProfile getZoneProfile(int zone) {
  ZoneMotionProfile p;
  if (zone == 1) {
    // 1·3번: T자 존 — 교차 감지 불가, 거리 기반 탈출
    p.exitForwardExtra = 35.0f;
    p.exitReverseExtra = 33.0f;
  } else if (zone == 3) {
    p.exitForwardExtra = 37.0f;
    p.exitReverseExtra = 35.0f;
  } else if (zone == 5 || zone == 6) {
    // 5·6번: 전진 진입 → 후진 탈출만
    p.exitReverseExtra = 27.5f;
  }
  // 2·4번: 십자(┼) 센서로 탈출 위치 결정 (추가 거리 없음, 축 정렬 4cm만)
  return p;
}

inline long zoneCrossApproachDecelSpan(int zone, bool reverse) {
  // 2·4번 존: 1·3번 탈출 거리를 교차 접근 감속 span 참고값으로 사용
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
//     주의: 여기는 '직진(cm)' 전용. 회전 감속은 [9] RAMP_SPIN_DECEL_DEG(°).
// ============================================================

constexpr int RAMP_MIN_SPEED = 25;
// 램프 시작·끝 최저 모터 출력 (가속 하한 / 감속 하한)

constexpr int RAMP_REF_SPEED = 40;
// 거리/각도 입력의 기준 cruise (≠40이면 구간 길이만 × speed/40)

constexpr float RAMP_ACCEL_CM = 10.0f;
// 직진 가속 거리 (cm) @ cruise 40

constexpr float RAMP_DECEL_CM = 15.0f;
// 직진 감속 거리 (cm) @ cruise 40

constexpr int RAMP_MAX_SPEED_STEP = 6;
// 한 제어 주기당 최대 속도 상승 — 급가속 방지 (감속은 제한 없음)

inline float rampCruiseFactor(int speed) {
  // cruise s 일 때 거리 스케일 = s / 40
  int s = speed < 0 ? -speed : speed;
  if (s <= 0)
    return 0.0f;
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

constexpr int SPEED_LINE_FOLLOW_FWD = 40;
// 라인 추종 전진 (트랙·교차로)

constexpr int SPEED_LINE_FOLLOW_REV = 30;
// 라인 추종 후진

constexpr int SPEED_OPEN_ZONE_FWD = 60;
// 맹목 전진 — 박스 존 내부

constexpr int SPEED_OPEN_ZONE_REV = 50;
// 맹목 후진 — 박스 존 내부

constexpr int SPEED_OPEN_TRACK_FWD = 70;
// 맹목 전진 — 메인 트랙·블라인드 정렬

constexpr int SPEED_OPEN_TRACK_REV = 60;
// 맹목 후진 — 메인 트랙

constexpr int SPEED_9_TO_8 = 35;
// 9→8 구간 전용 (13 경유 등 9번 정확 정렬 불가 → 저속)

constexpr int START_LINE_SEARCH_SPEED = 30;
// 스타트 존 → 메인 라인 최초 탐색 속도

// ============================================================
// [9] 제자리 회전 (spin)
//     회전 감속 2단계: RAMP_SPIN_DECEL_DEG(메인) → SPIN_END_DECEL_DEG(마지막 마무리)
// ============================================================

constexpr int SPIN_SPEED = 40;
// 회전 cruise 모터 출력

constexpr int SPIN_90_COUNTS = 1170;
// 90° 회전 평균 엔코더 카운트 (좌·우 평균, 실측)

constexpr float RAMP_SPIN_ACCEL_DEG = 20.0f;
// 회전 가속 구간 각도 (°) — 시작 후 이 각도 동안 RAMP_MIN→SPIN_SPEED

constexpr float RAMP_SPIN_DECEL_DEG = 40.0f;
// 회전 감속(메인) 구간 각도 (°) — 목표 직전 이 각도 동안 SPIN_SPEED→RAMP_MIN

constexpr float SPIN_END_DECEL_DEG = 10.0f;
// 회전 감속(마무리) 구간 각도 (°) — 마지막 이 각도에서 메인 위에 더 가파른 감속 추가

constexpr float SPIN_OVERSHOOT_COMP_FRAC = 0.005f;
// 회전 시작 시 관성 오버슈트 보정 — 목표 각의 0.5%(0.005)만큼 목표를 미리 단축

// --- 회전 중 라인 감지 기반 오버슈트 억제 ---
constexpr float SPIN_LINE_TRIM_MIN_FRAC = 0.50f;
// 목표 각의 이 비율(70%) 이상 회전한 뒤부터 반대쪽 라인 감지를 본다

constexpr float SPIN_LINE_TRIM_REMAIN_FRAC = 0.5f;
// 위 시점에 반대쪽 센서가 라인을 잡으면: 남은 각도의 이 비율만 마저 회전 (0.5=절반)

constexpr float SPIN_LINE_RECOVER_DEG = 20.0f;
// 절반 회전 완료 후 그 반대쪽 센서에서 라인이 사라졌으면 역방향으로 이만큼 복구 (°)

inline float rampSpinAccelDeg(int spinSpeed) {
  return RAMP_SPIN_ACCEL_DEG * rampCruiseFactor(spinSpeed);
}

inline float rampSpinDecelDeg(int spinSpeed) {
  return RAMP_SPIN_DECEL_DEG * rampCruiseFactor(spinSpeed);
}

// ============================================================
// [10] 메인 트랙 가로축 노드 7 — 8 — 9 (동서 직선, 실측 ~70cm)
// ============================================================

constexpr int SPEED_TRACK_7_9_LINE = 55;
// 가로(동서) 라인 추종 속도: 7→8, 8→9, 8→7, 7→9 (9→8은 SPEED_9_TO_8)

constexpr float LINE_KP_TRACK_7_9_SOFT = 1.2f;
constexpr float LINE_KP_TRACK_7_9_HARD = 3.0f;
// 가로 라인 추종 P 게인 (오차 ±1 / ±2 이상)

constexpr float DIST_TRACK_NODE_SPAN_CM = 70.0f;
// 8↔7, 8↔9 노드 간 직선 거리 (접근 감속 시작 계산용)

constexpr float DIST_TRACK_7_TO_9_CM = DIST_TRACK_NODE_SPAN_CM * 2.0f;
// 7→8→9 연속 직선 (8번 통과·정지 없음)

constexpr float DIST_TRACK_NODE8_PASS_HALF_CM = 8.0f;
// 7→9 장구간 — 8번 교차 통과 구간 (±cm, cruise 유지)

constexpr float DIST_NODE_DETECT_CRAWL_CM = 5.0f;
// 노드 라인 도달 이 거리 전에 최저속 도달 → 라인을 천천히 통과(고속 스킵 방지)

constexpr float DIST_TRACK_OVERSHOOT_MIN_CM = 0.2f;
// 노드 간 위치 보정 — 이보다 작으면 무시 (엔코더 노이즈)

constexpr float DIST_TRACK_OVERSHOOT_MAX_CM = 0.5f;
// 노드 간 전진·후진 보정 — 초과/부족 각각 최대 이 거리까지만

constexpr unsigned long BUZZER_OVERSHOOT_CORR_MS = 200;
// 노드 간 위치 보정 실행 시 알림 (ms)

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
// 스타트 → 13번 노드 맹목 거리

constexpr int HEADING_13_TO_9 = 305;
// 13번 → 9번 방향 (°)

constexpr float DIST_TRACK_13_TO_9_CM = 70.0f;
// 13→9 접근 거리 (cm) — 접근 감속 계산용, 정지는 라인 센서

// ============================================================
// [12] 맵 경로 — 9 · 10 · 11 · 12 (남쪽 루프)
//      9↔10·9→11·10↔11·12→9·13→9 구간은 전방 L/C/R 중 하나만 ON이어도 라인 감지
// ============================================================

constexpr int HEADING_9_TO_10 = 88;
constexpr int HEADING_9_TO_11 = 89;
constexpr int HEADING_10_TO_11 = 88;
constexpr int HEADING_11_TO_10 = 272;
// 9·10·11 노드 간 진행 방향 (°)

constexpr int HEADING_10_TO_12 = 240;
constexpr float DIST_10_TO_12_CM = 50.0f;

constexpr int HEADING_11_TO_12 = 250;
constexpr float DIST_11_TO_12_CM = 110.0f;

constexpr int HEADING_12_TO_9_2 = 310;
// 12번 근처 → 9번 교차로 접근 방향 (°)

constexpr float DIST_TRACK_9_TO_10_CM = 60.0f;
// 9↔10 노드 간 직선 (cm) — 접근 감속 계산용, 정지는 라인 센서

constexpr float DIST_TRACK_10_TO_11_CM = 70.0f;
// 10↔11 노드 간 직선 (cm) — 접근 감속 계산용, 정지는 라인 센서

constexpr float DIST_TRACK_9_TO_11_CM = 130.0f;
// 9→11 직행 (cm) — 접근 감속 계산용, 정지는 라인 센서 (10·11 교차 2회)

constexpr float DIST_TRACK_12_TO_9_CM = 45.0f;
// 12→9 접근 (cm) — 접근 감속 계산용, 정지는 라인 센서

// ============================================================
// [13] 맵 경로 — 피니시 (11번 경유 → 스타트박스)
// ============================================================

constexpr int HEADING_11_TO_FINISH = 350;
// 11번 → 스타트 피니시 진입 시 바라볼 방향 (북에서 약간 서쪽, °)

constexpr int HEADING_FINISH_PARK = 0;
// 스타트박스 접촉 후 최종 주차 방향 (북)

constexpr float DIST_FINISH_BLIND_CONFIRM_CM = 5.0f;
// 11번 선이 끊긴 '블라인드(라인 없음)' 구간을 이 거리만큼 연속 통과해야 블라인드로 확정
// (센서 깜빡임 디바운스). 이후 다시 만나는 라인 = 스타트박스

constexpr float DIST_FINISH_PARK_REV_CM = 15.0f;
// 북(0°) 정렬 후 최종 후진 (cm)

// ============================================================
// [14] PID 라인 추종 · 직진 보정
// ============================================================

constexpr float LINE_KP_FWD_SOFT = 1.2f;
// 전진 P 게인 — 센서 오차 ±1

constexpr float LINE_KP_FWD_HARD = 4.0f;
// 전진 P 게인 — 센서 오차 ±2 이상

constexpr float LINE_KP_REV_SOFT = 1.0f;
// 후진 P 게인 — 오차 ±1

constexpr float LINE_KP_REV_HARD = 2.2f;
// 후진 P 게인 — 오차 ±2 이상

constexpr float LINE_HARD_STEER_SPEED_FACTOR = 0.5f;
// hard 조향(오차 ±2↑) 시 cruise × 이 비율 (살짝 감속)

constexpr float LINE_KI = 0.0f;
// 적분 게인 (미사용)

constexpr float LINE_KD = 9.0f;
// 미분 게인 (진동·오버슈트 억제)

constexpr float LINE_ALIGN_GAIN = 1.5f;
// 전·후방 센서 동시 ON 시 좌우 정렬 보정

constexpr float VELOCITY_KP = 0.2f;
// 좌·우 엔코더 속도 P (직진 속도 일치)

constexpr unsigned long MOTOR_VELOCITY_PID_MS = 10;
// setWheelSpeeds() 엔코더 속도 PID 갱신 주기 (ms)

constexpr float VELOCITY_TARGET_FACTOR = 0.5f;
// 목표 틱/주기 = 모터출력 × 이 계수

constexpr int VELOCITY_MAX_CORRECTION = 10;
// 속도 보정 출력 클램프

constexpr int EDGE_SYNC_GAIN = 5;
// T자 존(1·3) 탈출 시 좌·우 엣지 동기화 조향

constexpr int MOTOR_OFFSET_L = 0;
// 좌측 모터 직진 미세 보정 (+/-)

constexpr int MOTOR_OFFSET_R = 0;
// 우측 모터 직진 미세 보정 (+/-)

// ============================================================
// [15] 리프트 (Expansion DC #1·#2)
// ============================================================

constexpr float LIFT_COUNTS_PER_CM = 200.0f;
// 리프트 엔코더 카운트 / cm

constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;
// 상승 한계 높이 (cm)

constexpr float LIFT_NEAR_FLOOR_CM = 5.0f;
// 바닥 근접 판정 높이 (cm)

constexpr float LIFT_UP_CLEAR_CM = 5.0f;
// 이 높이 이상이면 주행 허가 (상승)

constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;
// 이 높이 이하면 주행 허가 (하강)

constexpr int LIFT_UP_POWER = 100;
// 상승 모터 출력 (일반 구간)

constexpr int LIFT_DOWN_POWER = 100;
// 하강 모터 출력 (일반 구간)

constexpr float LIFT_UP_SLOW_ZONE_CM = 20.0f;
// 상승 막바지 감속 구간 (cm, 24cm 한계 근처)

constexpr int LIFT_UP_SLOW_POWER_L = 40;
constexpr int LIFT_UP_SLOW_POWER_R = 40;
// 상승 막바지 좌·우 출력

constexpr float LIFT_DOWN_SLOW_ZONE_CM = 10.0f;
// 하강 막바지 감속 구간 (cm)

constexpr int LIFT_DOWN_SLOW_POWER_L = 20;
constexpr int LIFT_DOWN_SLOW_POWER_R = 20;
// 하강 막바지 좌·우 출력

constexpr float LIFT_SYNC_GAIN = 5.0f;
// 좌·우 높이 편차 동기화 게인

constexpr unsigned long LIFT_TICK_INTERVAL_MS = 15;
// liftUpTick / liftDownTick 주기 (ms)

constexpr unsigned long LIFT_FLOOR_TIME_MS = 2000;
// 바닥 근접 후 완전 하강 대기 (ms)

// ============================================================
// [16] 전역 객체 · 상태 (정의: main.cpp, MapRouter.cpp 등)
// ============================================================

extern PRIZM prizm;
extern int lineTraceLastEdge;    // 마지막 라인 엣지 (0=없음, 1=좌, 2=우)
extern bool intersectionArmed;   // 교차로 재무장 가능
extern int intersectionHitCount; // 111 연속 카운트

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

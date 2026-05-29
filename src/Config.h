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

// ── [0] 단위 변환 ────────────────────────────────────────────
constexpr float COUNTS_PER_CM = 1000.0f / 23.0f;  // 1cm당 엔코더 카운트 (23cm 이동 시 1000카운트 기준)
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); }  // cm 단위를 카운트로 변환하는 함수

// ── [1] 센서 핀 설정 ─────────────────────────────────────────
constexpr int SENSOR_LEFT = 2;          // 전방 좌측 라인 센서 핀
constexpr int SENSOR_CENTER = 3;        // 전방 중앙 라인 센서 핀
constexpr int SENSOR_RIGHT = 4;         // 전방 우측 라인 센서 핀
constexpr bool INVERT_SENSORS = false;  // 센서 값 반전 여부 (하얀 선 맵일 경우 true)
constexpr int BUZZER_PIN = 5;           // 알림음 출력을 위한 부저 핀

constexpr int SENSOR_REAR_LEFT = A1;        // 후방 좌측 아날로그 라인 센서 핀
constexpr int SENSOR_REAR_CENTER = A2;      // 후방 중앙 아날로그 라인 센서 핀
constexpr int SENSOR_REAR_RIGHT = A3;       // 후방 우측 아날로그 라인 센서 핀
constexpr int REAR_SENSOR_THRESHOLD = 200;  // 후방 아날로그 센서가 검은선을 인식하는 기준값

// ── [2] 모터 속도 설정 ───────────────────────────────────────
constexpr int STRAIGHT_SPEED = 30;  // 라인이 없는 빈 공간에서 직진할 때의 기본 속도
constexpr int SPEED = 30;           // 검은 선을 따라가는 라인트레이싱 기본 속도
constexpr int BACK_SPEED = 30;      // 구역 탈출 등 후진할 때의 모터 속도
constexpr int SPIN_SPEED = 30;      // 제자리에서 회전(스핀 턴)할 때의 모터 속도
constexpr int BLIND_SPEED = 30;     // 대각선 등 라인을 찾기 전 맹주행할 때의 저속 모터 속도

// ── [3] 엔코더 거리 설정 (순수 이동 거리) ────────────────────────
constexpr int SPIN_90_COUNTS = 1200;  // 제자리에서 90도 회전하는 데 필요한 바퀴 회전 카운트

constexpr float DIST_CROSS_ALIGN_CM =
    6.0f;  // 교차로(검은선)를 감지한 후 로봇 바퀴축을 교차로 중앙에 맞추기 위해 더 직진하는 거리
constexpr int DIST_CROSS_ALIGN_COUNTS = CM(DIST_CROSS_ALIGN_CM);

constexpr float DIST_REAR_CROSS_ALIGN_CM =
    26.0f;  // 후진 중 교차로를 감지했을 때 바퀴축을 중앙에 맞추기 위해 더 후진하는 거리
constexpr int DIST_REAR_CROSS_ALIGN_COUNTS = CM(DIST_REAR_CROSS_ALIGN_CM);

constexpr float DIST_FINISH_ENTRY_CM = 36.0f;  // 종료(FINISH) 구역의 선을 밟은 후 안쪽으로 깊숙이 들어가는 거리
constexpr int DIST_FINISH_ENTRY_COUNTS = CM(DIST_FINISH_ENTRY_CM);

// ── [4] 제어 파라미터 (튜닝값) ────────────────────────────────
constexpr float LIFT_UP_CLEAR_CM = 10.0f;   // 리프트가 상승할 때 주행을 허가하는 안전 높이(cm)
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;  // 리프트가 하강할 때 주행을 허가하는 안전 높이(cm)
constexpr int DRIVE_BIAS = 0;               // 직진 시 좌우 모터 편차 보정 (양수면 좌측 모터 감속)
constexpr bool WEST_IS_LEFT = true;         // 맵 상에서 서쪽이 로봇 기준 왼쪽인지 여부
constexpr int CROSS_CONFIRM = 2;            // 교차로 노이즈 필터링 (연속 감지 횟수)
constexpr int ANGULAR_GAIN = 3;             // 전/후방 센서를 동시 사용할 때 자세를 교정하는 배율
constexpr int ALIGN_MAX_COUNTS = 67;        // 라인 정렬 시 무한 회전을 방지하기 위한 최대 허용 회전량
constexpr int SPIN_BRAKE_LEAD = 15;         // 목표 각도 도달 전 미리 제동을 거는 카운트 (관성 보정용)

constexpr int TURN_LINE_ARM_DEG = 45;  // 이 각도 이상 회전한 후부터 라인 센서를 다시 감지하기 시작
constexpr int TURN_LINE_MAX_DEG = 91;  // 라인을 못 찾을 때 회전을 멈추는 최대 한계 각도

constexpr int BACK_STEER_STRONG = 7;  // 후진 시 한쪽 센서만 라인을 밟았을 때 강하게 꺾는 조향량
constexpr int BACK_STEER_WEAK = 3;    // 후진 시 두 개 이상 센서가 라인을 밟았을 때 약하게 꺾는 조향량

// ── [5] 특수 구간 주행 설정 (거리 및 절대 각도) ────────────────────────
// ★ 절대 각도 기준: 0(북), 90(동), 180(남), 270(서)

// [5-1] 출발 경로 설정 (START -> 12 -> 9-2)
constexpr float DIST_START_TO_12_CM = 45.0f;  // START 박스에서 12번 빈 공간까지 직진하는 거리
constexpr int DIST_START_TO_12_COUNTS = CM(DIST_START_TO_12_CM);
constexpr int HEADING_12_TO_9 = 315;  // 12번 노드에서 9-2번 노드를 향해 꺾는 절대 각도 (북서쪽)

// [5-2] 복귀 경로 설정 (9-3 -> 12 -> START)
constexpr float DIST_9_TO_9_3_CM = 15.0f;  // 9-2 교차점에서 9-3 지점까지 전진하는 거리
constexpr int DIST_9_TO_9_3_COUNTS = CM(DIST_9_TO_9_3_CM);
constexpr int HEADING_9_3_TO_12 = 225;      // 9-3 지점에서 12번 노드를 향해 꺾는 절대 각도 (남서쪽)
constexpr float DIST_9_3_TO_12_CM = 45.0f;  // 9-3 지점에서 12번 노드까지 라인 없이 맹주행하는 거리
constexpr int DIST_9_3_TO_12_COUNTS = CM(DIST_9_3_TO_12_CM);
constexpr int HEADING_12_TO_START = 90;  // 12번 노드에서 START 박스를 향해 꺾는 절대 각도 (동쪽)

// [5-3] 노드 간 대각선 진입 각도
constexpr int HEADING_9_TO_10 = 45;    // 9-3번 지점에서 10-2번을 향하는 절대 각도 (북동쪽)
constexpr int HEADING_10_TO_9 = 225;   // 10-2번 교차로에서 9-2번을 향하는 절대 각도 (남서쪽)
constexpr int HEADING_10_TO_11 = 90;   // 10-2번에서 11-2번으로 수평 이동하는 절대 각도 (동쪽)
constexpr int HEADING_11_TO_10 = 270;  // 11-2번에서 10-2번으로 수평 이동하는 절대 각도 (서쪽)

// [5-5] 구역(존) 진입 및 탈출 거리 (센서 무시, 순수 지정 거리 이동)
constexpr float DIST_ZONE_ENTER_FWD_CM = 45.0f;  // 전진으로 구역(존)에 들어갈 때 이동할 총 거리
constexpr int DIST_ZONE_ENTER_FWD_COUNTS = CM(DIST_ZONE_ENTER_FWD_CM);

constexpr float DIST_ZONE_ENTER_REV_CM = 45.0f;  // 후진으로 구역(존)에 들어갈 때 이동할 총 거리
constexpr int DIST_ZONE_ENTER_REV_COUNTS = CM(DIST_ZONE_ENTER_REV_CM);

constexpr float DIST_ZONE_EXIT_FWD_CM = 45.0f;  // 구역(존)에서 전진으로 빠져나올 때 이동할 총 거리
constexpr int DIST_ZONE_EXIT_FWD_COUNTS = CM(DIST_ZONE_EXIT_FWD_CM);

constexpr float DIST_ZONE_EXIT_REV_CM = 45.0f;  // 구역(존)에서 후진으로 빠져나올 때 이동할 총 거리
constexpr int DIST_ZONE_EXIT_REV_COUNTS = CM(DIST_ZONE_EXIT_REV_CM);
// ─────────────────────────────────────────────────────────────

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
/* ============================================================
 * Config.h - 공용 설정 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [DEBUG] 디버그 모드 설정 ─────────────────────────────────────
#define ROBOT_DEBUG 1 // 0 = 경기용 무음 모드(빠름), 1 = 시리얼 모니터에 상태 출력

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

// ── [0] 단위 변환 공식 ──────────────────────────────────────────
constexpr float COUNTS_PER_CM = 1000.0f / 23.0f; // 23cm 전진 시 약 1000카운트 발생 (실측 기준 비율)
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); } // cm 단위를 엔코더 카운트로 자동 변환

// ── [1] 핀(Pin) 번호 및 하드웨어 설정 ───────────────────────────
constexpr int SENSOR_LEFT = 2;          // 전방 좌측 디지털 라인 센서 핀
constexpr int SENSOR_CENTER = 3;        // 전방 중앙 디지털 라인 센서 핀
constexpr int SENSOR_RIGHT = 4;         // 전방 우측 디지털 라인 센서 핀
constexpr bool INVERT_SENSORS = false;  // 센서 감지 반전 (검은 바탕에 흰 선일 경우 true로 변경)
constexpr int BUZZER_PIN = 5;           // 경기 종료 알림 등을 울릴 부저 핀

constexpr int SENSOR_REAR_LEFT = A1;    // 후방 좌측 아날로그 라인 센서 핀
constexpr int SENSOR_REAR_CENTER = A2;  // 후방 중앙 아날로그 라인 센서 핀
constexpr int SENSOR_REAR_RIGHT = A3;   // 후방 우측 아날로그 라인 센서 핀
constexpr int REAR_SENSOR_THRESHOLD = 200; // 후방 아날로그 값이 이 수치 이상이면 검은 선으로 인식

// ── [2] 모터 기본 속도 설정 (파워: 0 ~ 100) ──────────────────────
// ★ 각 주행 상황에 맞게 개별적으로 속도를 튜닝할 수 있도록 분리해 두었습니다.
constexpr int STRAIGHT_SPEED = 30; // 맹주행(라인 없이 엔코더로만 직진)할 때의 속도
constexpr int SPEED = 30;          // 검은 선을 따라가는 기본 라인트레이싱 속도
constexpr int BACK_SPEED = 30;     // 존(Zone)에서 빠져나올 때 사용하는 후진 속도
constexpr int SPIN_SPEED = 30;     // 제자리에서 90도 회전(스핀 턴)할 때의 기본 회전 속도
constexpr int BLIND_SPEED = 30;    // 대각선 주행 중 라인을 찾을 때까지 천천히 전진하는 탐색 속도

// ── [3] 공통 이동 거리 설정 (엔코더 카운트 기반) ─────────────────
constexpr int SPIN_90_COUNTS = 1200; // 제자리에서 정확히 90도를 돌기 위해 필요한 바퀴 카운트

constexpr float DIST_CROSS_ALIGN_CM = 6.0f; // 교차로 감지 후 로봇 회전축을 교차점 중앙에 맞추기 위해 더 전진하는 거리
constexpr int DIST_CROSS_ALIGN_COUNTS = CM(DIST_CROSS_ALIGN_CM);

constexpr float DIST_REAR_CROSS_ALIGN_CM = 26.0f; // 후진 중 교차로를 감지했을 때 중앙을 맞추기 위해 더 후진하는 거리
constexpr int DIST_REAR_CROSS_ALIGN_COUNTS = CM(DIST_REAR_CROSS_ALIGN_CM);

constexpr float DIST_FINISH_ENTRY_CM = 36.0f; // 모든 임무를 마치고 START 칸 안으로 깊숙이 들어가는 거리
constexpr int DIST_FINISH_ENTRY_COUNTS = CM(DIST_FINISH_ENTRY_CM);

// ── [4] 주행 및 센서 제어 파라미터 (미세 튜닝용) ─────────────────
// ★ 주행 루프 주기(delay 5ms) 안내
// 하드웨어 주행 코드(Motion, MapRouter 등) 내부의 delay(5)는 초당 150번(150Hz)의 센서 감지를 수행하여 
// 아주 예민하고 부드러운 라인트레이싱을 가능하게 하는 핵심 딜레이입니다.

constexpr float LIFT_UP_CLEAR_CM = 10.0f;   // 리프트가 이 높이 이상 올라가면 주행을 시작해도 됨 (주행/상승 동시 작동용)
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;  // 리프트가 이 높이 이하로 내려가면 주행을 시작해도 됨
constexpr int DRIVE_BIAS = 0;               // 좌우 모터 편차 교정값 (양수면 직진 시 좌측 모터의 힘을 줄임)
constexpr bool WEST_IS_LEFT = true;         // 맵의 방위 기준점 설정
constexpr int CROSS_CONFIRM = 2;            // 교차로 노이즈 필터링 (연속으로 이 횟수만큼 감지되어야 진짜 교차로로 인정)
constexpr int ANGULAR_GAIN = 3;             // 라인 트레이싱 시 차체 틀어짐을 보정하는 민감도 배율 (수치가 클수록 홱홱 꺾음)
constexpr int ALIGN_MAX_COUNTS = 67;        // 라인 정렬 시 무한 루프(계속 헛도는 현상)를 방지하는 최대 허용치
constexpr int SPIN_BRAKE_LEAD = 15;         // 목표 각도 도달 직전에 미리 브레이크를 거는 카운트 (관성 밀림 보정용)

constexpr int BACK_STEER_STRONG = 7;        // 후진 라인트레이싱 중 크게 벗어났을 때 꺾는 강한 조향량
constexpr int BACK_STEER_WEAK   = 3;        // 후진 라인트레이싱 중 조금 벗어났을 때 꺾는 약한 조향량

// ── [5] 특수 구간(대각선 및 빈 공간) 주행 설정 ──────────────────
// ★ 절대 각도 기준: 0(북쪽), 90(동쪽), 180(남쪽), 270(서쪽)

constexpr float DIST_START_TO_12_CM = 45.0f;                 // START 박스에서 12번 위치(빈 공간)까지 직진하는 거리
constexpr int   DIST_START_TO_12_COUNTS = CM(DIST_START_TO_12_CM);
constexpr int   HEADING_12_TO_9 = 315;                       // 12번 위치 도착 후 9-2번 노드를 향해 꺾는 절대 각도 (북서쪽)

constexpr float DIST_9_TO_9_3_CM = 15.0f;                    // 9-2 교차점에서 9-3 지점까지 전진하는 거리
constexpr int   DIST_9_TO_9_3_COUNTS = CM(DIST_9_TO_9_3_CM);
constexpr int   HEADING_9_3_TO_12 = 135;                     // 9-3 지점에서 12번 노드를 향해 꺾는 절대 각도 (남동쪽)
constexpr float DIST_9_3_TO_12_CM = 45.0f;                   // 9-3 지점에서 12번 노드까지 맹주행 돌파하는 거리
constexpr int   DIST_9_3_TO_12_COUNTS = CM(DIST_9_3_TO_12_CM);
constexpr int   HEADING_12_TO_START = 90;                    // 12번 노드에서 START 박스를 향해 꺾는 절대 각도 (동쪽)

constexpr int   HEADING_9_TO_10 = 45;                        // 9-3번에서 10-2번으로 진입할 때의 절대 각도 (북동쪽)
constexpr int   HEADING_10_TO_9 = 225;                       // 10-2번에서 9-2번으로 진입할 때의 절대 각도 (남서쪽)
constexpr int   HEADING_10_TO_11 = 90;                       // 10-2번에서 11-2번으로 수평 이동할 때의 각도 (동쪽)
constexpr int   HEADING_11_TO_10 = 270;                      // 11-2번에서 10-2번으로 수평 이동할 때의 각도 (서쪽)

// [구역(존 1~6) 진입 및 탈출 이동 거리] 센서 유무를 무시하고 무조건 이 거리만큼만 들어갔다 나옵니다.
constexpr float DIST_ZONE_ENTER_FWD_CM = 45.0f;  // 전진으로 존에 들어갈 때 목표 이동 거리
constexpr int   DIST_ZONE_ENTER_FWD_COUNTS = CM(DIST_ZONE_ENTER_FWD_CM);
constexpr float DIST_ZONE_ENTER_REV_CM = 45.0f;  // 후진으로 존에 들어갈 때 목표 이동 거리
constexpr int   DIST_ZONE_ENTER_REV_COUNTS = CM(DIST_ZONE_ENTER_REV_CM);
constexpr float DIST_ZONE_EXIT_FWD_CM = 45.0f;   // 전진으로 존에서 빠져나올 때 목표 이동 거리
constexpr int   DIST_ZONE_EXIT_FWD_COUNTS = CM(DIST_ZONE_EXIT_FWD_CM);
constexpr float DIST_ZONE_EXIT_REV_CM = 45.0f;   // 후진으로 존에서 빠져나올 때 목표 이동 거리
constexpr int   DIST_ZONE_EXIT_REV_COUNTS = CM(DIST_ZONE_EXIT_REV_CM);

// ── [6] 리프트 제어 파라미터 (초고속 반응형 튜닝) ─────────────────────────────
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;     // 리프트가 최대로 올라갈 수 있는 제한 높이 (천장 도달치)
constexpr float LIFT_RIGHT_OFFSET_CM = 0.6f;    // 우측 리프트가 미세하게 낮을 경우 이를 보정하기 위한 높이 오차
constexpr float LIFT_COUNTS_PER_CM = 200.0f;    // 리프트 모터가 1cm 이동할 때 발생하는 엔코더 카운트 수

// 100ms 당 목표 속도 및 파워 제어값 (내부적으로 10ms 단위로 분할하여 부드럽게 PID 제어됨)
constexpr int LIFT_TARGET_SPEED = 220;          // 리프트 상승/하강 기본 목표 속도
constexpr int LIFT_MAX_POWER = 50;              // 리프트 모터 최대 파워 제한 (너무 세면 부품 파손 위험)
constexpr int LIFT_DOWN_STALL_THRESHOLD = 100;  // 하강 중 속도가 이 수치 이하로 떨어지면 바닥에 닿은 것(스톨)으로 간주
constexpr int LIFT_EMERGENCY_SPEED = 60;        // 상승 중 속도가 이 수치 이하로 떨어지면 걸린 것으로 판단

// ★ 리프트 타이밍 제어 (ms 단위)
constexpr unsigned long LIFT_TICK_INTERVAL_MS = 10;   // 제어 주기 (10ms 초고속 반응)
constexpr unsigned long LIFT_GRACE_PERIOD_MS = 200;   // 출발 직후 모터 시동이 걸릴 때까지 바닥 검사를 면제하는 유예 시간 (0.2초)
constexpr unsigned long LIFT_HARD_JAM_PHASE_MS = 600; // 하드잼(완전 밀착) 즉시 정지 모드가 활성화되는 시간 구간 (0.2초 ~ 0.6초)

// ★ 노이즈 필터링 횟수 제어 (1틱 = 10ms 기준)
constexpr int LIFT_HARD_JAM_THRESHOLD = 5;      // 움직임이 거의 없다고 판단하는 수치
constexpr int LIFT_HARD_JAM_CONFIRM_COUNT = 20; // 위 수치가 20틱(200ms) 연속 감지 시 즉시 정지 확정
constexpr int LIFT_UP_EMERGENCY_COUNT = 50;     // 상승 중 비상 상태가 50틱(500ms) 연속 감지 시 정지 확정
constexpr int LIFT_DOWN_EMERGENCY_COUNT = 30;   // 하강 중 착지 상태가 30틱(300ms) 연속 감지 시 착지 확정 (흔들림 노이즈 무시)
// ─────────────────────────────────────────────────────────────

extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
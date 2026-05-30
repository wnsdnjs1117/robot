/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * 사용자는 이 파일의 수치들만 조절하여 로봇의 모든 움직임을 제어합니다.
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [DEBUG] 디버그(출력) 모드 설정 ─────────────────────────────────────
// 0을 입력하면 시리얼 통신을 꺼서 로봇 반응 속도를 극대화(경기용)합니다.
// 1을 입력하면 시리얼 모니터에 상태를 출력합니다(테스트용).
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

// ── [0] 단위 변환 공식 (물리적 실측 비율) ──────────────────────────────────
// 로봇을 직접 굴려보고, 23cm를 갔을 때 엔코더에 찍힌 카운트 수치를 1000.0f 자리에 적어주세요.
constexpr float COUNTS_PER_CM = 1000.0f / 23.0f; 

// (건드리지 마세요) cm 단위로 입력된 상수를 엔코더 카운트 수치로 자동 변환하는 함수입니다.
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); } 

// ── [1] 핀(Pin) 번호 및 하드웨어 센서 설정 ───────────────────────────
// 전방 디지털 라인 센서를 꽂은 아날로그/디지털 핀 번호를 적어주세요.
constexpr int SENSOR_LEFT = 2;          
constexpr int SENSOR_CENTER = 3;        
constexpr int SENSOR_RIGHT = 4;         

// 검은 바닥에 흰 선을 쓸 경우 true로, 흰 바닥에 검은 선이면 false로 두세요.
constexpr bool INVERT_SENSORS = false;  

// 경기 종료 알림을 울릴 부저(스피커) 핀 번호를 적어주세요.
constexpr int BUZZER_PIN = 5;           

// 후방 아날로그 라인 센서를 꽂은 핀 번호를 적어주세요.
constexpr int SENSOR_REAR_LEFT = A1;    
constexpr int SENSOR_REAR_CENTER = A2;  
constexpr int SENSOR_REAR_RIGHT = A3;   

// 아날로그 센서가 빛을 흡수(검은 선)했을 때 읽히는 임계값(0~1023)을 적어주세요.
constexpr int REAR_SENSOR_THRESHOLD = 200; 

// ── [2] 모터 구동 속도 설정 (파워 범위: 0 ~ 100) ──────────────────────
// 주행 상황별 모터 파워(속도)를 0부터 100 사이의 숫자로 적어주세요.
constexpr int STRAIGHT_SPEED = 30; // 선 없이 허공을 엔코더 거리로만 직진할 때의 속도
constexpr int SPEED = 30;          // 검은 선을 따라가는 평상시 전진 라인트레이싱 속도
constexpr int BACK_SPEED = 30;     // 존(Zone)에서 빠져나올 때 쓰는 후진 라인트레이싱 속도
constexpr int SPIN_SPEED = 30;     // 제자리에서 90도 스핀 턴을 할 때의 바퀴 회전 속도
constexpr int BLIND_SPEED = 30;    // 교차로 통과 후 바닥에서 다음 선을 찾을 때까지 천천히 나아가는 탐색 속도

// ── [3] 공통 이동 거리 설정 (순수 CM 단위) ─────────────────
// 제자리에서 로봇 차체가 정확히 90도를 돌기 위해 바퀴가 굴러가야 하는 엔코더 카운트 수치를 적어주세요.
constexpr int SPIN_90_COUNTS = 1200; 

// 아래는 실제 자로 잰 거리(cm)를 적어주세요. (소수점 f 유지)
// 십자가 교차로 인식 후, 바퀴 중심축을 교차점 정중앙에 맞추기 위해 '더 전진'해야 하는 거리(cm)
constexpr float DIST_CROSS_ALIGN_CM = 6.0f; 

// 후진 중 교차로 인식 후, 바퀴 중심축을 교차점 정중앙에 맞추기 위해 '더 후진'해야 하는 거리(cm)
constexpr float DIST_REAR_CROSS_ALIGN_CM = 26.0f; 

// 모든 임무 종료 후 START 박스 안으로 안전하게 쏙 들어가기 위해 전진하는 거리(cm)
constexpr float DIST_FINISH_ENTRY_CM = 36.0f; 

// ── [4] 주행 및 센서 제어 파라미터 (미세 튜닝용) ─────────────────
constexpr float LIFT_UP_CLEAR_CM = 10.0f;   // 리프트가 이 높이(cm)를 넘으면 주행 시작 가능 (동시 작동용)
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;  // 리프트가 이 높이(cm) 이하로 내려가야 주행 시작 가능
constexpr int DRIVE_BIAS = 0;               // 직진 시 왼쪽으로 휘면 양수를, 오른쪽으로 휘면 음수를 넣어 직진성을 맞추세요.
constexpr bool WEST_IS_LEFT = true;         // 지도 상에서 서쪽이 왼쪽 방향이면 true
constexpr int CROSS_CONFIRM = 2;            // 교차로 센서 감지 횟수 (이 숫자만큼 연속 감지되어야 멈춤. 예: 2)
constexpr int ANGULAR_GAIN = 3;             // 라인 트레이싱 복원력 (수치를 키우면 선을 벗어날 때 홱홱 꺾습니다)
constexpr int ALIGN_MAX_COUNTS = 67;        // 턴 헛돌기 방지용 허용 카운트 (건드리지 않음)
constexpr int SPIN_BRAKE_LEAD = 15;         // 관성 밀림 방지용 브레이크 시점 (수치를 키우면 목표보다 일찍 브레이크를 밟습니다)

constexpr int BACK_STEER_STRONG = 7;        // 후진 트레이싱 중 선을 크게 벗어났을 때의 꺾임 강도
constexpr int BACK_STEER_WEAK   = 3;        // 후진 트레이싱 중 선을 미세하게 벗어났을 때의 꺾임 강도

// ── [5] 특수 구간(대각선 및 빈 공간) 주행 설정 ──────────────────
// 자로 잰 이동 거리(cm)와 꺾어야 할 방위각(도)을 적어주세요. 
// (방위각 기준: 0=북, 90=동, 180=남, 270=서)
constexpr float DIST_START_TO_12_CM = 45.0f; // START 박스 앞 교차로에서 12번 허공 노드까지의 전진 거리(cm)
constexpr int   HEADING_12_TO_9 = 315;       // 12번 위치 도착 후 9-2번 노드를 향해 틀어야 하는 북서쪽 각도

constexpr int   HEADING_9_3_TO_12 = 135;     // 9-3 노드에서 12번을 향해 틀어야 하는 남동쪽 각도
constexpr float DIST_9_3_TO_12_CM = 45.0f;   // 9-3에서 12번까지 허공을 가로지르는 직진 거리(cm)
constexpr int   HEADING_12_TO_START = 90;    // 12번에서 START를 향해 꺾어야 하는 동쪽 각도

constexpr int   HEADING_9_TO_10 = 45;        // 9-3번에서 10-2번 진입 시 꺾을 각도 (북동쪽)
constexpr int   HEADING_10_TO_9 = 225;       // 10-2번에서 9-2번 복귀 시 꺾을 각도 (남서쪽)
constexpr int   HEADING_10_TO_11 = 90;       // 10-2번에서 11-2번 수평 이동 시 꺾을 각도 (동쪽)
constexpr int   HEADING_11_TO_10 = 270;      // 11-2번에서 10-2번 수평 복귀 시 꺾을 각도 (서쪽)

// ★ [존 1~4 구역] 메인 복도(7, 8번 노드)와 연결된 구역 진입/탈출 거리 
// 로봇이 구역으로 들어갈 때와 빠져나올 때 목표로 하는 거리(cm)를 적어주세요.
constexpr float DIST_ZONE_ENTER_FWD_CM = 45.0f;  // 전진으로 존에 진입할 목표 거리(cm)
constexpr float DIST_ZONE_ENTER_REV_CM = 45.0f;  // 후진으로 존에 진입할 목표 거리(cm)
constexpr float DIST_ZONE_EXIT_FWD_CM = 45.0f;   // 존에서 전진으로 빠져나올 목표 거리(cm)
constexpr float DIST_ZONE_EXIT_REV_CM = 45.0f;   // 존에서 후진으로 빠져나올 목표 거리(cm)

// ★ [존 5~6 전용] 대각선 노드(10-2, 11-2) 전용 짧은 탈출 거리
// 대각선 교차점과 존이 너무 가깝기 때문에, 메인 복도보다 짧게 빠져나오도록 거리를 따로 적어주세요.
constexpr float DIST_ZONE56_EXIT_FWD_CM = 20.0f; // 존 5, 6에서 전진으로 탈출할 짧은 거리(cm)
constexpr float DIST_ZONE56_EXIT_REV_CM = 20.0f; // 존 5, 6에서 후진으로 탈출할 짧은 거리(cm)

// ── [6] 듀얼 리프트 모터 제어 파라미터 ─────────────────────────────
constexpr float LIFT_COUNTS_PER_CM = 200.0f;    // 리프트가 1cm 오르락내리락할 때 찍히는 엔코더 카운트 수치 적기
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;     // 리프트가 최대로 올라갈 수 있는 제한 고도 (천장 높이 cm)
constexpr float LIFT_RIGHT_OFFSET_CM = 0.6f;    // 우측 리프트가 조립 오차로 덜 올라간다면 그 오차만큼 높이(cm)를 더해줌

constexpr int LIFT_TARGET_SPEED = 200;          // 상승/하강 제어 시 모터 목표 속도
constexpr int LIFT_MAX_POWER = 50;              // 모터가 타지 않도록 걸어두는 최대 파워 제한 (0~100)
constexpr int LIFT_DOWN_STALL_THRESHOLD = 140;  // 하강 시 실시간 속도가 이 수치 이하로 뚝 떨어지면 바닥에 닿은 것으로 확정
constexpr int LIFT_EMERGENCY_SPEED = 60;        // 상승 시 속도가 이 수치 이하로 떨어지면 어딘가에 끼인(고장) 것으로 판단

// (아래는 모터 보호용 비상정지 타이머입니다. 가급적 건드리지 마세요.)
constexpr unsigned long LIFT_TICK_INTERVAL_MS = 10;   
constexpr unsigned long LIFT_GRACE_PERIOD_MS = 200;   
constexpr unsigned long LIFT_HARD_JAM_PHASE_MS = 600; 
constexpr int LIFT_HARD_JAM_THRESHOLD = 5;      
constexpr int LIFT_HARD_JAM_CONFIRM_COUNT = 20; 
constexpr int LIFT_UP_EMERGENCY_COUNT = 50;     
constexpr int LIFT_DOWN_EMERGENCY_COUNT = 30;   
// ─────────────────────────────────────────────────────────────

extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
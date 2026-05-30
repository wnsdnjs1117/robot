/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * 사용자는 이 파일의 수치들만 조절하여 로봇의 모든 움직임을 제어합니다.
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [0] 디버그 모드 및 단위 변환 공식 ──────────────────────────────────────

// 1을 넣으면 시리얼 모니터에 상태를 출력하고, 0을 넣으면 통신을 꺼서 로봇 반응 속도를 극대화(대회용)합니다.
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

// 로봇 바퀴가 23cm를 굴러갔을 때 엔코더에 찍힌 카운트 수치(예: 1000)를 적어주세요. (단위 변환용 기준값)
constexpr float COUNTS_PER_CM = 1000.0f / 23.0f; 

// (건드리지 마세요) 우리가 cm 단위로 적은 숫자를 로봇이 이해하는 엔코더 카운트로 자동 변환해 주는 마법의 함수입니다.
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); } 


// ── [1] 핀(Pin) 번호 및 하드웨어 센서 설정 ───────────────────────────

// 전방 디지털 라인 센서를 꽂은 PRIZM 컨트롤러의 디지털 핀 번호
constexpr int SENSOR_LEFT = 2;          
constexpr int SENSOR_CENTER = 3;        
constexpr int SENSOR_RIGHT = 4;         

// 센서 반전 설정 (검은 바닥에 흰 선을 쓰면 true, 흰 바닥에 검은 선을 쓰면 false)
constexpr bool INVERT_SENSORS = false;  

// 대회 종료 시 삑! 소리를 낼 부저(스피커) 핀 번호
constexpr int BUZZER_PIN = 5;           

// 후방 아날로그 라인 센서를 꽂은 아날로그 핀 번호
constexpr int SENSOR_REAR_LEFT = A1;    
constexpr int SENSOR_REAR_CENTER = A2;  
constexpr int SENSOR_REAR_RIGHT = A3;   

// 후방 아날로그 센서가 빛을 흡수(검은 선 위)했다고 판단할 기준 수치 (0~1023 중 보통 200~500 사이)
constexpr int REAR_SENSOR_THRESHOLD = 200; 


// ── [2] 로봇 하드웨어 물리적 치수 (기하학적 보정용) ────────────────────

// (매우 중요) 바퀴 회전축(엔코더 중심)을 기준으로 각 부품이 떨어져 있는 거리(cm)를 자로 재서 적어주세요.
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;  // 바퀴축 ~ 전방 라인 센서까지의 거리
constexpr float DIST_AXIS_TO_LIFT_CM = 11.0f;         // 바퀴축 ~ 짐을 싣는 리프트 중앙까지의 거리 (뒤쪽)
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 24.0f;  // 바퀴축 ~ 후방 라인 센서까지의 거리 (꼬리 길이)


// ── [3] 모터 구동 속도 설정 (파워 범위: 0 ~ 100) ────────────────────────────────

constexpr int STRAIGHT_SPEED = 30; // 선이 없는 허공을 쌩으로 직진할 때의 기본 속도
constexpr int SPEED = 30;          // 검은 선을 따라가는 평상시 전진 트레이싱 속도
constexpr int BACK_SPEED = 20;     // 존에서 빠져나올 때 쓰는 후진 트레이싱 속도 (꼬리가 길어 피시테일 방지를 위해 20 권장)
constexpr int SPIN_SPEED = 30;     // 제자리에서 90도로 휙! 돌 때의 바퀴 회전 속도
constexpr int BLIND_SPEED = 30;    // 교차로 통과 직후, 다음 선이 나타날 때까지 조심스럽게 나아가는 탐색 속도


// ── [4] 조향(Steering) 및 자세 교정 파라미터 (미세 튜닝용) ─────────────

// 양쪽 바퀴 모터의 힘이 달라 로봇이 삐딱하게 갈 때 씁니다. (왼쪽으로 휘면 양수, 오른쪽으로 휘면 음수 입력)
constexpr int DRIVE_BIAS = 0;               

// 제자리 회전 시, 관성 때문에 목표 각도를 지나쳐버리는(오버슈팅) 것을 막기 위해 미리 브레이크를 밟는 카운트 시점
constexpr int SPIN_BRAKE_LEAD = 15;         

// [전진 정렬 강도] 선을 따라갈 때 선에서 벗어나면 다시 중심으로 꺾고 들어오는 조향 힘 (수치가 클수록 홱홱 꺾임)
constexpr int ANGULAR_GAIN = 3;             

// [전진-후방 융합 교정 강도] 직진 중인데 로봇 꼬리가 삐뚤어졌을 때, 후방 센서가 이를 감지하여 차체를 선과 일직선(평행)으로 밀어 넣는 힘
constexpr int REAR_ALIGN_GAIN = 3;          

// [후진 정렬 강도] 꼬리가 긴 로봇이 후진할 때 S자로 흔들리는(피시테일) 현상 방지용 조향 힘
constexpr int BACK_STEER_STRONG = 7;        // 센서가 선을 완전히 한쪽으로 크게 벗어났을 때 복귀하는 힘
constexpr int BACK_STEER_WEAK   = 3;        // 센서가 중심과 바깥쪽에 걸쳐서 미세하게 벗어났을 때 복귀하는 힘


// ── [5] 거리 및 방위각 시나리오 설정 (cm 및 degree 단위) ──────────────────────────────────

// 지도 상에서 로봇 기준 서쪽(West) 방향이 왼쪽인지 여부
constexpr bool WEST_IS_LEFT = true;         

// 제자리에서 로봇 차체가 정확히 90도를 돌기 위해 바퀴가 굴러가야 하는 엔코더 카운트 수치
constexpr int SPIN_90_COUNTS = 1200;        

// 교차로 센서 감지 횟수 (노이즈 방지용. 이 숫자만큼 연속으로 교차로가 감지되어야 진짜 교차로로 인정함)
constexpr int CROSS_CONFIRM = 2;            

// 가로 교차선 인식 후 즉시 멈추지 않고, '바퀴의 회전 중심축'을 교차점 정중앙에 정확히 올리기 위해 더 이동하는 거리
constexpr float DIST_CROSS_ALIGN_CM = 6.0f;       // 전진 진입 시 더 전진하는 거리
constexpr float DIST_REAR_CROSS_ALIGN_CM = 26.0f; // 후진 진입 시 더 후진하는 거리

// ★ [존(Zone) 진입 깊이 설정] 
// 센서가 선 끊김을 감지한 시점부터 기하학적 덧셈/뺄셈을 거쳐 '리프트의 중앙'이 존 안쪽으로 몇 cm 더 들어갈지 결정
constexpr float DIST_ZONE_DEPTH_CM = 15.0f;      

// ★ [존(Zone) 탈출 총 거리 설정]
constexpr float DIST_ZONE_EXIT_FWD_CM = 45.0f;   // 1~4구역에서 전진으로 빠져나올 때 목표 거리
constexpr float DIST_ZONE_EXIT_REV_CM = 45.0f;   // 1~4구역에서 후진으로 빠져나올 때 목표 거리
constexpr float DIST_ZONE56_EXIT_FWD_CM = 20.0f; // 5, 6구역에서 전진으로 탈출할 때 거리 (교차로가 가까우므로 짧게 설정)
constexpr float DIST_ZONE56_EXIT_REV_CM = 20.0f; // 5, 6구역에서 후진으로 탈출할 때 거리 (짧게 설정)

// 특수 구간 노드 이동 방위(Degree: 0=북, 90=동, 180=남, 270=서) 및 허공 직진 거리
constexpr float DIST_START_TO_12_CM = 45.0f; // START -> 12번 노드(빈 공간) 전진 거리
constexpr int   HEADING_12_TO_9 = 315;       // 12번 -> 9-2번 방향(북서쪽) 방위각
constexpr int   HEADING_9_3_TO_12 = 135;     // 9-3번 -> 12번 방향(남동쪽) 방위각
constexpr float DIST_9_3_TO_12_CM = 45.0f;   // 9-3번 -> 12번 복귀 허공 직진 거리
constexpr int   HEADING_12_TO_START = 90;    // 12번 -> START 방향(동쪽) 방위각
constexpr float DIST_FINISH_ENTRY_CM = 36.0f; // 모든 임무 종료 후 START 박스로 안전하게 골인하기 위한 진입 거리

constexpr int   HEADING_9_TO_10 = 45;        // 9-3번 -> 10-2번 (북동쪽)
constexpr int   HEADING_10_TO_9 = 225;       // 10-2번 -> 9-2번 복귀 (남서쪽)
constexpr int   HEADING_10_TO_11 = 90;       // 10-2번 -> 11-2번 (동쪽)
constexpr int   HEADING_11_TO_10 = 270;      // 11-2번 -> 10-2번 복귀 (서쪽)


// ── [6] 듀얼 리프트 모터 제어 파라미터 (물건 승강기용) ─────────────────────────────

constexpr float LIFT_UP_CLEAR_CM = 10.0f;       // 리프트가 이 높이를 넘으면 바닥에 끌리지 않는다고 판단 (동시 주행 가능)
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;      // 리프트가 완전히 내려갔다고 판단하는 높이
constexpr float LIFT_COUNTS_PER_CM = 200.0f;    // 리프트 1cm 승강 시 필요한 엔코더 카운트
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;     // 리프트가 올라갈 수 있는 물리적 최고 한계 높이 (천장 충돌 방지)
constexpr float LIFT_RIGHT_OFFSET_CM = 0.6f;    // 듀얼 리프트 조립 오차로 인해 우측이 덜 올라갈 경우 더해주는 보정 높이

constexpr int LIFT_TARGET_SPEED = 200;          // 상승/하강 제어 시 모터 목표 속도
constexpr int LIFT_MAX_POWER = 50;              // 모터 과부하 및 타는 것을 방지하기 위한 파워 제한 (0~100)
constexpr int LIFT_DOWN_STALL_THRESHOLD = 140;  // 하강 중 실시간 속도가 이 수치 이하로 뚝 떨어지면 바닥에 닿았다고 확정
constexpr int LIFT_EMERGENCY_SPEED = 60;        // 상승 중 실시간 속도가 이 수치 이하로 떨어지면 어딘가 끼였다고 판단 (비상정지)

// (아래는 리프트 모터 보호용 비상정지 타이머 및 조건 설정입니다. 안전을 위해 가급적 건드리지 마세요.)
constexpr unsigned long LIFT_TICK_INTERVAL_MS = 10;   // 리프트 상태 체크 주기 (10ms)
constexpr unsigned long LIFT_GRACE_PERIOD_MS = 200;   // 출발 직후 가속 구간에서는 속도가 낮아도 비상정지를 유예하는 시간
constexpr unsigned long LIFT_HARD_JAM_PHASE_MS = 600; // 하드 잼(완벽한 끼임) 판정을 시작할 유예 시간
constexpr int LIFT_HARD_JAM_THRESHOLD = 5;            // 모터 속도가 이 이하로 떨어지면 하드 잼 의심
constexpr int LIFT_HARD_JAM_CONFIRM_COUNT = 20;       // 하드 잼 의심이 이 횟수 연속 발생 시 강제 종료
constexpr int LIFT_UP_EMERGENCY_COUNT = 50;           // 일반 끼임 의심 연속 발생 한계치 (상승)
constexpr int LIFT_DOWN_EMERGENCY_COUNT = 30;         // 일반 끼임 의심 연속 발생 한계치 (하강)
// ─────────────────────────────────────────────────────────────

// 외부 변수 선언 (다른 C++ 파일들과 공유하기 위함)
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
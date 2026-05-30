/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [0] 디버그 모드 및 단위 변환 공식 ──────────────────────────────────────
#define ROBOT_DEBUG    1
#define LIFT_TEST_MODE 0  // 1로 바꾸면 리프트 단독 테스트 모드 (s/u/d 키 제어)

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

constexpr float COUNTS_PER_CM = 1000.0f / 23.0f; 
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); } 

// ── [1] 핀(Pin) 번호 및 하드웨어 센서 설정 ───────────────────────────
constexpr int SENSOR_LEFT = 2;          
constexpr int SENSOR_CENTER = 3;        
constexpr int SENSOR_RIGHT = 4;         
constexpr bool INVERT_SENSORS = false;  
constexpr int BUZZER_PIN = 5;           

constexpr int SENSOR_REAR_LEFT = A1;    
constexpr int SENSOR_REAR_CENTER = A2;  
constexpr int SENSOR_REAR_RIGHT = A3;   
constexpr int REAR_SENSOR_THRESHOLD = 200; 

// ── [2] 로봇 하드웨어 물리적 치수 (기하학적 보정용) ────────────────────
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;  
constexpr float DIST_AXIS_TO_LIFT_CM = 11.0f;         
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 24.0f;  

// ── [3] 모터 구동 속도 설정 (0 ~ 100) ────────────────────────────────
constexpr int STRAIGHT_SPEED = 30; 
constexpr int SPEED = 30;          
constexpr int BACK_SPEED = 30;     
constexpr int SPIN_SPEED = 30;     
constexpr int BLIND_SPEED = 30;    

// ── [4] 조향(Steering) 및 자세 교정 파라미터 (미세 튜닝용) ─────────────
constexpr int DRIVE_BIAS = 0;               
constexpr int SPIN_BRAKE_LEAD = 15;         
constexpr int ANGULAR_GAIN = 3;             
constexpr int REAR_ALIGN_GAIN = 3;          
constexpr int BACK_STEER_STRONG = 7;        
constexpr int BACK_STEER_WEAK   = 3;        

// ★ 양바퀴 속도 동기화 (직진 보정) 파라미터
constexpr float DRIVE_SYNC_KP = 0.05f;             // 보정 강도 (0.02 ~ 0.1 권장, 흔들리면 낮출 것)
constexpr int   DRIVE_SYNC_MAX_CORRECTION = 10;    // 한 번에 들어가는 최대 보정 속도 (급격한 꺾임 방지)

// ── [5] 거리 및 방위각 시나리오 설정 ──────────────────────────────────
constexpr bool WEST_IS_LEFT = true;         
constexpr int SPIN_90_COUNTS = 1200;        
constexpr int CROSS_CONFIRM = 2;            

constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM; 
constexpr float DIST_REAR_CROSS_ALIGN_CM = DIST_AXIS_TO_REAR_SENSOR_CM; 

constexpr float DIST_ZONE_DEPTH_CM = 15.0f;      
constexpr float LINE_LEN_ZONE_12_CM = 28.0f;     
constexpr float LINE_LEN_ZONE_34_CM = 30.0f;   
constexpr float DIST_ZONE56_EXIT_CM = 40.0f; 

constexpr float DIST_START_TO_12_CM = 1000.0f; 
constexpr int   HEADING_12_TO_9 = 290;       
constexpr int   HEADING_9_3_TO_12 = 110;     
constexpr float DIST_9_3_TO_12_CM = 60.0f;   
constexpr int   HEADING_12_TO_START = 90;    

constexpr int   HEADING_11_TO_START = 170;   
constexpr float DIST_FINISH_ENTRY_CM = 20.0f; 

constexpr int   HEADING_9_TO_10 = 88;        
constexpr int   HEADING_10_TO_9 = 268;       
constexpr int   HEADING_10_TO_11 = 90;       
constexpr int   HEADING_11_TO_10 = 270;      

// ── [6] 듀얼 리프트 모터 제어 파라미터 ─────────────────────────────
constexpr float LIFT_COUNTS_PER_CM        = 200.0f;
constexpr float LIFT_MAX_HEIGHT_CM        = 24.0f;   // 상승 정지 높이 (cm)
constexpr float LIFT_NEAR_FLOOR_CM        = 5.0f;    // 타이머 하강 전환 높이 (cm)
constexpr float LIFT_UP_CLEAR_CM          = 5.0f;    // liftUp() 블로킹 해제 높이
constexpr float LIFT_DOWN_CLEAR_CM        = 0.0f;    // liftDownUntilClear() 해제 높이

constexpr int   LIFT_UP_POWER             = 100;     // 기본 상승 모터 파워
constexpr int   LIFT_DOWN_POWER           = 100;     // 기본 하강 모터 파워

// == 감속 구간 진입 시 좌/우 독립 속도 파라미터 ==
constexpr float LIFT_UP_SLOW_ZONE_CM      = 20.0f;   // 상승 시 감속 시작 기준 높이 (cm)
constexpr int   LIFT_UP_SLOW_POWER_L      = 20;      // 상승 감속 속도 파워 (왼쪽)
constexpr int   LIFT_UP_SLOW_POWER_R      = 25;      // 상승 감속 속도 파워 (오른쪽)

constexpr float LIFT_DOWN_SLOW_ZONE_CM    = 10.0f;   // 하강 시 감속 시작 기준 높이 (cm)
constexpr int   LIFT_DOWN_SLOW_POWER_L    = 15;      // 하강 감속 속도 파워 (왼쪽)
constexpr int   LIFT_DOWN_SLOW_POWER_R    = 20;      // 하강 감속 속도 파워 (오른쪽)

constexpr float LIFT_SYNC_GAIN            = 5.0f;    // 좌우 높이 편차 동기화 게인 (테스트 후 가감 조절)
constexpr unsigned long LIFT_TICK_INTERVAL_MS  = 10;
constexpr unsigned long LIFT_FLOOR_TIME_MS     = 2000; // 5cm 이하 진입 후 추가 하강 시간 (ms)
// ─────────────────────────────────────────────────────────────

extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
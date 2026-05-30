/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [0] 디버그 모드 및 단위 변환 공식 ──────────────────────────────────────
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
constexpr int BACK_SPEED = 20;     
constexpr int SPIN_SPEED = 30;     
constexpr int BLIND_SPEED = 30;    

// ── [4] 조향(Steering) 및 자세 교정 파라미터 (미세 튜닝용) ─────────────
constexpr int DRIVE_BIAS = 0;               
constexpr int SPIN_BRAKE_LEAD = 15;         
constexpr int ANGULAR_GAIN = 3;             
constexpr int REAR_ALIGN_GAIN = 3;          
constexpr int BACK_STEER_STRONG = 7;        
constexpr int BACK_STEER_WEAK   = 3;        

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

constexpr float DIST_START_TO_12_CM = 45.0f; 
constexpr int   HEADING_12_TO_9 = 290;       
constexpr int   HEADING_9_3_TO_12 = 110;     
constexpr float DIST_9_3_TO_12_CM = 60.0f;   
constexpr int   HEADING_12_TO_START = 90;    

constexpr int   HEADING_11_TO_START = 170;   
constexpr float DIST_FINISH_ENTRY_CM = 20.0f; 

constexpr int   HEADING_9_TO_10 = 40;        
constexpr int   HEADING_10_TO_9 = 265;       
constexpr int   HEADING_10_TO_11 = 90;       
constexpr int   HEADING_11_TO_10 = 270;      

// ── [6] 듀얼 리프트 모터 제어 파라미터 ─────────────────────────────
constexpr float LIFT_COUNTS_PER_CM = 200.0f;

// ★ [수정 완료] 주행 허용 기준 높이 변경
constexpr float LIFT_UP_CLEAR_CM = 5.0f;     // 상승 시 리프트가 5cm만 올라가도 주행 시작 허용
constexpr float LIFT_DOWN_CLEAR_CM = 1.0f;   // 하강 시 리프트가 1cm 이하로 내려오면 주행 시작 허용

constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;   
constexpr float LIFT_RIGHT_OFFSET_CM = 0.6f;    

constexpr int LIFT_TARGET_SPEED = 200;          
constexpr int LIFT_DOWN_STALL_THRESHOLD = 140;  
constexpr int LIFT_MAX_POWER = 50;              
constexpr int LIFT_EMERGENCY_SPEED = 60;        

// ★ [신규 추가] 하강 시 정상 속도(Cruise) 판정 변수 분리
constexpr float LIFT_CRUISE_SPEED_RATIO = 0.90f;  // 목표 속도의 몇 %에 도달해야 정상 하강으로 볼 것인가 (현재 90%)
constexpr int LIFT_CRUISE_CONFIRM_COUNT = 20;     // 정상 속도를 몇 번(10ms 당 1번) 유지해야 바닥 탐지를 시작할 것인가 (20 = 0.2초)

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
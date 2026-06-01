/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

#define ROBOT_DEBUG    1
#define LIFT_TEST_MODE 0
#define SENSOR_TEST_MODE 0  

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

constexpr float COUNTS_PER_CM = 3350.0f / 80.0f;
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); } 

// ── [1] 핀 번호 및 하드웨어 센서 ────────────────────────────────────
constexpr int SENSOR_LEFT = 2;          
constexpr int SENSOR_CENTER = 3;        
constexpr int SENSOR_RIGHT = 4;         
constexpr bool INVERT_SENSORS = false;  
constexpr int BUZZER_PIN = 5;           

constexpr int SENSOR_REAR_LEFT = A1;    
constexpr int SENSOR_REAR_CENTER = A2;  
constexpr int SENSOR_REAR_RIGHT = A3;   
constexpr int REAR_SENSOR_THRESHOLD = 128; 

// ── [2] 로봇 하드웨어 물리적 치수 ────────────────────────────────────
constexpr float ROBOT_LENGTH_CM = 35.0f;              
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;  
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;   
constexpr float DIST_AXIS_TO_LIFT_CM = 11.5f;         

// ── [3] 존(Zone) 진입/탈출 수치 (★ 사용자 세부 계산치 100% 반영) ────────────
constexpr float DIST_ZONE_DEPTH_CM = 20.0f;           
constexpr float LINE_LEN_ZONE_12_CM = 28.0f;          
constexpr float LINE_LEN_ZONE_3456_CM = 30.0f;        

constexpr float ENTRY_FWD_EXTRA_CM = 37.5f; 
constexpr float ENTRY_REV_EXTRA_CM = 12.5f; 

// 1번 구역 및 3번 구역 탈출 추가 주행 거리 (라인 두께 1cm 포함)
constexpr float EXIT_REV_EXTRA_1_CM = 33.0f;     // 1번 후진 탈출 (28 + 4 + 1)
constexpr float EXIT_FWD_EXTRA_1_CM = 35.0f;     // 1번 전진 탈출 (28 + 6 + 1)
constexpr float EXIT_REV_EXTRA_3_CM = 35.0f;     // 3번 후진 탈출 (30 + 4 + 1)
constexpr float EXIT_FWD_EXTRA_3_CM = 37.0f;     // 3번 전진 탈출 (30 + 6 + 1)

// 5, 6번 구역 맹목적 탈출 시 파라미터 
constexpr float EXIT_REV_SPECIAL_56_CM = 28.0f;  

// 2번, 4번 교차로(8번 노드) 정렬 거리 (라인 두께 1cm 포함)
constexpr float ALIGN_AXIS_FRONT_CM = 7.0f;      // 전진 시 (6 + 1)
constexpr float ALIGN_AXIS_REAR_CM = 5.0f;       // 후진 시 (4 + 1)

// ── [4] 조향 및 모터 속도 파라미터 ──────────────────────────────────

// ★ 케이스 1: 라인을 따라갈 때 (센서 기반 라인 트레이싱)
constexpr int SPEED = 40;          // 전진 라인트레이싱 기본 속도
constexpr int BACK_SPEED = 30;     // 후진 라인트레이싱 기본 속도

// ★ 케이스 2: 라인을 따라가지 않을 때 (인코더 직진 및 맹목적 주행)
constexpr int STRAIGHT_SPEED = 60; // 라인 없이 지정된 거리(cm)만큼 직진할 때의 속도
constexpr int BLIND_SPEED = 60;    // 라인을 만날 때까지 센서 무시하고 전/후진할 때의 속도

// 회전 속도 (라인과 무관)
constexpr int SPIN_SPEED = 30;     // 제자리 턴 속도

constexpr int MOTOR_OFFSET_L = 0;           
constexpr int MOTOR_OFFSET_R = 0;           
constexpr float RIGHT_MOTOR_MULTIPLIER = 0.0f; 

constexpr float VELOCITY_KP = 0.2f;               
constexpr float VELOCITY_TARGET_FACTOR = 0.5f;    
constexpr int   VELOCITY_MAX_CORRECTION = 10; 
constexpr int   SPIN_BRAKE_LEAD = 15; 

constexpr int   REAR_ALIGN_GAIN = 5; 
constexpr bool  WEST_IS_LEFT = true;         
constexpr int   SPIN_90_COUNTS = 1200;        
constexpr int   CROSS_CONFIRM = 1;            
constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM; 

// ── [5] 새 이동 규칙 맞춤 (12, 13번 경유) 특수 노드 파라미터 ─────────
constexpr int   HEADING_10_TO_12 = 180;   
constexpr float DIST_10_TO_12_CM = 40.0f; 
constexpr int   HEADING_11_TO_12 = 250;   
constexpr float DIST_11_TO_12_CM = 85.0f; 
constexpr int   HEADING_12_TO_9_2 = 285;  

constexpr float DIST_START_TO_13_CM = 90.0f; 
constexpr int   HEADING_13_TO_9 = 300.0;       

constexpr int   HEADING_9_TO_13 = 150;     
constexpr float DIST_9_TO_13_CM = 40.0f;   
constexpr int   HEADING_10_TO_13 = 180;    
constexpr float DIST_10_TO_13_CM = 40.0f;  
constexpr int   HEADING_13_TO_START = 90;  

constexpr float DIST_FINISH_ENTRY_CM = 40.0f; 

constexpr int   HEADING_9_TO_10 = 84;        
constexpr int   HEADING_9_TO_11 = 88;         // ★ 9->11 다이렉트 주행 시 바라볼 각도
constexpr float DIST_IGNORE_10_CM = 20.0f;    // ★ 9->11 이동 시 10번 선을 밟은 후 무시하고 밀고 나갈 거리
constexpr int   HEADING_10_TO_11 = 90;       
constexpr int   HEADING_11_TO_10 = 270;      

// ── [6] 리프트 파라미터 ───────────────────────────────────────────────
constexpr float LIFT_COUNTS_PER_CM        = 200.0f;
constexpr float LIFT_MAX_HEIGHT_CM        = 24.0f;   
constexpr float LIFT_NEAR_FLOOR_CM        = 5.0f;    
constexpr float LIFT_UP_CLEAR_CM          = 5.0f;    
constexpr float LIFT_DOWN_CLEAR_CM        = 0.0f;    
constexpr int   LIFT_UP_POWER             = 100;     
constexpr int   LIFT_DOWN_POWER           = 100;     
constexpr float LIFT_UP_SLOW_ZONE_CM      = 20.0f;   
constexpr int   LIFT_UP_SLOW_POWER_L      = 40;      
constexpr int   LIFT_UP_SLOW_POWER_R      = 40;      
constexpr float LIFT_DOWN_SLOW_ZONE_CM    = 8.0f;    
constexpr int   LIFT_DOWN_SLOW_POWER_L    = 25;      
constexpr int   LIFT_DOWN_SLOW_POWER_R    = 25;      
constexpr float LIFT_SYNC_GAIN            = 3.0f;    
constexpr unsigned long LIFT_TICK_INTERVAL_MS  = 10;
constexpr unsigned long LIFT_FLOOR_TIME_MS     = 2000; 

extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
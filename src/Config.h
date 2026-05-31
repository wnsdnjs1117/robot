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

// ★ 사용자가 실측한 엔코더 카운트 비율 적용
constexpr float COUNTS_PER_CM = 1000.0f / 20.0f; // cm값만 변경할 것. 예상보다 짧게 이동한다면, cm값을 줄여보세요.
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
constexpr float ROBOT_LENGTH_CM = 35.0f;              // 전체 세로 길이
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;  // 바퀴축 ~ 전방센서
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;   // 바퀴축 ~ 후방센서
constexpr float DIST_AXIS_TO_LIFT_CM = 11.5f;         // 바퀴축 ~ 리프트 (로봇 중앙)

// ── [3] 존(Zone) 진입/탈출 수치 ─────────────────────────────────────
constexpr float DIST_ZONE_DEPTH_CM = 20.0f;           
constexpr float LINE_LEN_ZONE_12_CM = 28.0f;          
constexpr float LINE_LEN_ZONE_3456_CM = 30.0f;        

constexpr float ENTRY_FWD_EXTRA_CM = 37.5f; // 전진 진입 보정 (20 + 17.5)
constexpr float ENTRY_REV_EXTRA_CM = 27.5f; // 후진 진입 보정 (20 + 7.5)

constexpr float EXIT_REV_EXTRA_12_CM = 33.0f;     // 후진 1/2 탈출 (28 + 1 + 4)
constexpr float EXIT_REV_EXTRA_3456_CM = 35.0f;   // 후진 3/4/5/6 탈출 (30 + 1 + 4)
constexpr float EXIT_FWD_EXTRA_12_CM = 35.0f;     // 전진 1/2 탈출 (28 + 1 + 6)
constexpr float EXIT_FWD_EXTRA_3456_CM = 37.0f;   // 전진 3/4/5/6 탈출 (30 + 1 + 6)
constexpr float EXIT_REV_SPECIAL_56_CM = 30.0f;   // 특수 5/6 탈출 (30 + 1 + 4 - 5)

// ★ 선 두께 2cm 반영: 끝단 감지 후 정중앙(1cm)까지 더 가도록 +1cm 적용 (유지됨)
constexpr float ALIGN_AXIS_FRONT_CM = 7.0f; // 전방 1.1.1 감지 후 이동: 원래 6 + 1(선 절반)
constexpr float ALIGN_AXIS_REAR_CM = 5.0f;  // 후방 1.1.1 감지 후 이동: 원래 4 + 1(선 절반)

// ── [4] 조향 및 모터 속도 파라미터 (원본 복구) ──────────────────────────
constexpr int STRAIGHT_SPEED = 30; 
constexpr int SPEED = 30;          
constexpr int BACK_SPEED = 30;     
constexpr int SPIN_SPEED = 30;     
constexpr int BLIND_SPEED = 30;    

constexpr int MOTOR_OFFSET_L = 0;           
constexpr int MOTOR_OFFSET_R = 0;           

constexpr float VELOCITY_KP = 0.2f;               
constexpr float VELOCITY_TARGET_FACTOR = 0.5f;    
constexpr int   VELOCITY_MAX_CORRECTION = 10; 
constexpr int   SPIN_BRAKE_LEAD = 15; // 회전 정지 오프셋

constexpr int   REAR_ALIGN_GAIN = 5; // 일직선 보정 강도
constexpr bool  WEST_IS_LEFT = true;         
constexpr int   SPIN_90_COUNTS = 1200;        
constexpr int   CROSS_CONFIRM = 2;            

// ★ 일반 교차로 정착 시 축 정렬 (전방 센서 기준 6cm + 선 절반 1cm = 7.0cm 적용 유지)
constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM + 1.0f; 

// ── [5] 13번 노드 및 특수 방위각 (사용자 실측 데이터 적용됨) ───────────────
constexpr float DIST_10_TO_13_CM = 40.0f;    
constexpr int   HEADING_13_TO_9_2 = 225;     

constexpr float DIST_START_TO_12_CM = 80.0f; 
constexpr int   HEADING_12_TO_9 = 290;       
constexpr int   HEADING_9_3_TO_12 = 110;     
constexpr float DIST_9_3_TO_12_CM = 60.0f;   
constexpr int   HEADING_12_TO_START = 90;    
constexpr int   HEADING_11_TO_START = 170;   
constexpr float DIST_FINISH_ENTRY_CM = 20.0f; 

constexpr int   HEADING_9_TO_10 = 88;        
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
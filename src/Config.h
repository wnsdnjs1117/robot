/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * 사용자는 이 파일의 수치들만 조절하여 로봇의 모든 움직임을 제어합니다.
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
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;  // 앞쪽 라인센서까지 거리
constexpr float DIST_AXIS_TO_LIFT_CM = 11.0f;         // 뒤쪽 리프트(짐 싣는 곳)까지 거리
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 24.0f;  // 뒤쪽 라인센서까지 거리 (꼬리 길이)

// ── [3] 모터 구동 속도 설정 (0 ~ 100) ────────────────────────────────
constexpr int STRAIGHT_SPEED = 30; // 허공 직진 속도
constexpr int SPEED = 30;          // 전진 라인트레이싱 속도
constexpr int BACK_SPEED = 20;     // 후진 라인트레이싱 속도 (안정성 위해 20 권장)
constexpr int SPIN_SPEED = 30;     // 제자리 회전 속도
constexpr int BLIND_SPEED = 30;    // 탐색 직진 속도

// ── [4] 조향(Steering) 및 자세 교정 파라미터 (미세 튜닝용) ─────────────
constexpr int DRIVE_BIAS = 0;               // 직진 편향 오차 교정값
constexpr int SPIN_BRAKE_LEAD = 15;         // 관성 밀림 방지용 회전 브레이크 시점

constexpr int ANGULAR_GAIN = 3;             // 전진 정렬 강도
constexpr int REAR_ALIGN_GAIN = 3;          // 전진-후방 융합 교정 강도 (일직선 맞춤)

constexpr int BACK_STEER_STRONG = 7;        // 후진 중 크게 벗어났을 때 복귀 힘
constexpr int BACK_STEER_WEAK   = 3;        // 후진 중 미세하게 벗어났을 때 복귀 힘

// ── [5] 거리 및 방위각 시나리오 설정 ──────────────────────────────────
constexpr bool WEST_IS_LEFT = true;         
constexpr int SPIN_90_COUNTS = 1200;        
constexpr int CROSS_CONFIRM = 2;            
constexpr float DIST_CROSS_ALIGN_CM = 6.0f; 
constexpr float DIST_REAR_CROSS_ALIGN_CM = 26.0f; 

constexpr float DIST_ZONE_DEPTH_CM = 15.0f;      // 존 진입 깊이
constexpr float DIST_ZONE_EXIT_FWD_CM = 45.0f;   // 1~4구역 전진 탈출 거리
constexpr float DIST_ZONE_EXIT_REV_CM = 45.0f;   // 1~4구역 후진 탈출 거리
constexpr float DIST_ZONE56_EXIT_FWD_CM = 20.0f; // 5, 6구역 짧은 전진 탈출
constexpr float DIST_ZONE56_EXIT_REV_CM = 20.0f; // 5, 6구역 짧은 후진 탈출

constexpr float DIST_START_TO_12_CM = 45.0f; 
constexpr int   HEADING_12_TO_9 = 315;       
constexpr int   HEADING_9_3_TO_12 = 135;     
constexpr float DIST_9_3_TO_12_CM = 45.0f;   
constexpr int   HEADING_12_TO_START = 90;    

constexpr int   HEADING_11_TO_START = 135;   // ★ [신규] 11번 상단에서 START 향해 트는 각도
constexpr float DIST_FINISH_ENTRY_CM = 36.0f; // START 박스 진입 주차 거리

constexpr int   HEADING_9_TO_10 = 45;        
constexpr int   HEADING_10_TO_9 = 225;       
constexpr int   HEADING_10_TO_11 = 90;       
constexpr int   HEADING_11_TO_10 = 270;      

// ── [6] 듀얼 리프트 모터 제어 파라미터 ─────────────────────────────
constexpr float LIFT_UP_CLEAR_CM = 10.0f;   
constexpr float LIFT_DOWN_CLEAR_CM = 0.0f;  
constexpr float LIFT_COUNTS_PER_CM = 200.0f;    
constexpr float LIFT_MAX_HEIGHT_CM = 24.0f;     
constexpr float LIFT_RIGHT_OFFSET_CM = 0.6f;    
constexpr int LIFT_TARGET_SPEED = 200;          
constexpr int LIFT_MAX_POWER = 50;              
constexpr int LIFT_DOWN_STALL_THRESHOLD = 140;  
constexpr int LIFT_EMERGENCY_SPEED = 60;        
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
/* ============================================================
 * Config.h - 로봇 공용 설정 파라미터 및 전역 변수 선언부
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// ── [디버깅 및 테스트 모드] ──────────────────────────────────────────
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

// ★ [엔코더 실측값 반영] 80cm 이동 시 3450 카운트
constexpr float COUNTS_PER_CM = 3420.0f / 80.0f; // 나눈 결과값이 적어지면 로봇도 조금 갑니다.
constexpr int CM(float cm) { return (int)(cm * COUNTS_PER_CM + 0.5f); } 

// ── [1] 핀 번호 및 하드웨어 센서 ─────────────────────────────────────
constexpr int SENSOR_LEFT = 2;          
constexpr int SENSOR_CENTER = 3;        
constexpr int SENSOR_RIGHT = 4;         
constexpr bool INVERT_SENSORS = false;  
constexpr int BUZZER_PIN = 5;           

constexpr int SENSOR_REAR_LEFT = A1;    
constexpr int SENSOR_REAR_CENTER = A2;  
constexpr int SENSOR_REAR_RIGHT = A3;   
constexpr int REAR_SENSOR_THRESHOLD = 200; 

// ── [2] 로봇 하드웨어 물리적 치수 ────────────────────────────────────
constexpr float ROBOT_LENGTH_CM = 35.0f;              // 로봇의 전체 길이
constexpr float DIST_AXIS_TO_FRONT_SENSOR_CM = 6.0f;  // 바퀴 축 ~ 전방 센서 거리
constexpr float DIST_AXIS_TO_REAR_SENSOR_CM = 4.0f;   // 바퀴 축 ~ 후방 센서 거리
constexpr float DIST_AXIS_TO_LIFT_CM = 11.5f;         // 바퀴 축 ~ 리프트 중심 거리

// ── [3] 존(Zone) 진입/탈출 수치 (★ 사용자 세부 계산치 반영) ──────────────
// 진입/탈출 기본 치수
constexpr float DIST_ZONE_DEPTH_CM = 20.0f;           
constexpr float LINE_LEN_ZONE_12_CM = 28.0f;          
constexpr float LINE_LEN_ZONE_3456_CM = 30.0f;        

constexpr float ENTRY_FWD_EXTRA_CM = 37.5f;           // 전진 진입 시 추가 거리
constexpr float ENTRY_REV_EXTRA_CM = 12.5f;           // 후진 진입 시 추가 거리

// 1번, 3번 구역 탈출 추가 주행 거리 (라인 두께 1cm 포함)
constexpr float EXIT_REV_EXTRA_1_CM = 33.0f;          // 1번 후진 탈출 (28+4+1)
constexpr float EXIT_FWD_EXTRA_1_CM = 35.0f;          // 1번 전진 탈출 (28+6+1)
constexpr float EXIT_REV_EXTRA_3_CM = 35.0f;          // 3번 후진 탈출 (30+4+1)
constexpr float EXIT_FWD_EXTRA_3_CM = 37.0f;          // 3번 전진 탈출 (30+6+1)

// 5, 6번 구역 맹목적 탈출 시 파라미터 
constexpr float EXIT_REV_SPECIAL_56_CM = 29.0f;  

// 2번, 4번 교차로(8번 노드) 정렬 거리 (라인 두께 1cm 제외된 실측값)
constexpr float ALIGN_AXIS_FRONT_CM = 6.0f;           // 전진 교차로 정렬 거리
constexpr float ALIGN_AXIS_REAR_CM = 4.0f;            // 후진 교차로 정렬 거리

// ── [4] 속도 및 조향 제어 ─────────────────────────────────────────

// [4-1] 기본 라인트레이싱 속도
constexpr int SPEED = 35;                // 전진 라인트레이싱 속도
constexpr int BACK_SPEED = 30;           // 후진 라인트레이싱 속도

// [4-2] 맹목적 주행 속도 (선이 없는 허공 구간)
constexpr int STRAIGHT_SPEED = 40;       // 인코더 지정 거리 직진 속도
constexpr int BLIND_SPEED = 40;          // 선을 만날 때까지 달리는 속도

// [4-3] 존(Zone) 진출입 특수 맹목 구간 속도
constexpr int ZONE_ENTRY_BLIND_SPEED = 40;       // 진입 시 전진 속도
constexpr int ZONE_ENTRY_BLIND_BACK_SPEED = 40;  // 진입 시 후진 속도
constexpr int ZONE_EXIT_BLIND_SPEED = 40;        // 탈출 시 전진 속도 (선을 찾을때까지)
constexpr int ZONE_EXIT_BLIND_BACK_SPEED = 40;   // 탈출 시 후진 속도 (선을 찾을때까지)

// [4-4] 제자리 회전(스핀) 속도 및 각도 제어
constexpr int SPIN_SPEED = 40;           // 제자리 턴 기본 속도
constexpr int SPIN_90_COUNTS = 1100;     // ★ 90도 회전 시 기준 엔코더 값 (모든 각도의 기준)
constexpr int SPIN_BRAKE_LEAD = 30;      // 회전 목표 도달 전 미리 브레이크 잡는 수치

// [4-5] 조향(PID/동기화) 게인값
constexpr float VELOCITY_KP = 0.2f;               
constexpr float VELOCITY_TARGET_FACTOR = 0.5f;    
constexpr int   VELOCITY_MAX_CORRECTION = 10; 
constexpr int   REAR_ALIGN_GAIN = 5;         // 전/후방 센서 오차 시 조향 강도
constexpr int   EDGE_SYNC_GAIN = 5;         // 가장자리(100/001) 아슬아슬할 때 안으로 밀어넣는 힘

// [4-6] 모터/기타 설정
constexpr int MOTOR_OFFSET_L = 0;           
constexpr int MOTOR_OFFSET_R = 0;           
constexpr float RIGHT_MOTOR_MULTIPLIER = 0.0f; 
constexpr bool  WEST_IS_LEFT = true;         
constexpr int   CROSS_CONFIRM = 1;            
constexpr float DIST_CROSS_ALIGN_CM = DIST_AXIS_TO_FRONT_SENSOR_CM; 

// ── [5] 이동 노드 방위각 및 거리 (특수 맵 규격) ───────────────────────

// [5-1] 10번, 11번, 12번 노드 간 이동
constexpr int   HEADING_10_TO_12 = 210;   
constexpr float DIST_10_TO_12_CM = 50.0f; 
constexpr int   HEADING_11_TO_12 = 250;      // 실측 턴 각도 반영
constexpr float DIST_11_TO_12_CM = 110.0f;    // 실측 거리 반영
constexpr int   HEADING_12_TO_9_2 = 310;  

// [5-2] START <-> 9번, 13번 노드 간 이동
constexpr float DIST_START_TO_13_CM = 90.0f; 
constexpr int   HEADING_13_TO_9 = 305.0;     // 실측 각도 반영
constexpr int   HEADING_9_TO_13 = 150;     
constexpr float DIST_9_TO_13_CM = 40.0f;   
constexpr int   HEADING_10_TO_13 = 180;    
constexpr float DIST_10_TO_13_CM = 55.0f;  
constexpr int   HEADING_13_TO_START = 90;  
constexpr float DIST_FINISH_ENTRY_CM = 40.0f; 

// [5-3] 9, 10, 11번 등 상호 노드 이동 방위 및 공통 무시 규칙
constexpr int   HEADING_9_TO_10 = 83;        
constexpr int   HEADING_9_TO_11 = 87;         
constexpr float DIST_IGNORE_NODE_CM = 5.0f;  // ★ 공통: 노드 이동 시 출발 직후 선(교차로)을 무시하고 강제로 밀고 나갈 최소 거리 (10cm)
constexpr int   HEADING_10_TO_11 = 90;       
constexpr int   HEADING_11_TO_10 = 270;      

// ── [6] 리프트 파라미터 ───────────────────────────────────────────────
constexpr float LIFT_COUNTS_PER_CM        = 200.0f;
constexpr float LIFT_MAX_HEIGHT_CM        = 24.0f;   
constexpr float LIFT_NEAR_FLOOR_CM        = 5.0f;    
constexpr float LIFT_UP_CLEAR_CM          = 5.0f;    
constexpr float LIFT_DOWN_CLEAR_CM        = 0.0f;    

// 리프트 모터 속도 (실측 반영)
constexpr int   LIFT_UP_POWER             = 50;      // 기본 상승 파워
constexpr int   LIFT_DOWN_POWER           = 100;      // 기본 하강 파워

constexpr float LIFT_UP_SLOW_ZONE_CM      = 20.0f;   
constexpr int   LIFT_UP_SLOW_POWER_L      = 40;      
constexpr int   LIFT_UP_SLOW_POWER_R      = 40;      
constexpr float LIFT_DOWN_SLOW_ZONE_CM    = 8.0f;    
constexpr int   LIFT_DOWN_SLOW_POWER_L    = 25;      
constexpr int   LIFT_DOWN_SLOW_POWER_R    = 25;      
constexpr float LIFT_SYNC_GAIN            = 3.0f;    
constexpr unsigned long LIFT_TICK_INTERVAL_MS  = 10;
constexpr unsigned long LIFT_FLOOR_TIME_MS     = 2000; 

// ── [7] 외부 변수 참조 ────────────────────────────────────────────────
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
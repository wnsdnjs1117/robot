/* ============================================================
 * Config.h - 공용 파라미터 및 전역 변수 선언
 * ============================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <PRIZM.h>

// 1. 센서 핀 설정
const int SENSOR_LEFT = 2;
const int SENSOR_CENTER = 3;
const int SENSOR_RIGHT = 4;
const bool INVERT_SENSORS = false;

// 2. 주행 속도 설정
const int STRAIGHT_SPEED = 24;
const int SPEED = 24;
const int BACK_SPEED = 21;
const int SPIN_SPEED = 27;

// 3. 엔코더 제어 거리 설정
const int START_ESCAPE_COUNTS = 1500;
const int SPIN_90_COUNTS = 1200;
const int CROSS_ALIGN_COUNTS = 250;
const int ZONE_ENTER_COUNTS = 2200;
const int ZONE_DEPTH_COUNTS = 1500;

// 4. 주행 환경 설정
const bool WEST_IS_LEFT = true;
const int CROSS_CONFIRM = 2;

// 전역 객체 선언
extern PRIZM prizm;
extern int lastSensorState;
extern bool crossingArmed;
extern int crossingStable;

#endif
/* ============================================================
 * main.cpp - 로봇 자율주행 / 테스트 모드 통합 진입점
 * 제어기: Arduino UNO + TETRIX PRIZM
 * ============================================================ */
#include <Arduino.h>
#include <PRIZM.h>
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MissionFlow.h"
#include "Motion.h"
#include "Navigation.h"
#include "HuskyQR.h"

#if RUN_TEST_MODE == 1
#include "TestMode.h"
#endif

PRIZM prizm;
int lastSensorState = 0;
bool crossingArmed = true;
int crossingStable = 0;

// ────────────────────────────────────────────────────────────
// [1] 테스트 모드용 펌웨어 빌드 (Config.h 에서 1로 설정 시)
// ────────────────────────────────────────────────────────────
#if RUN_TEST_MODE == 1
void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();

  extern const int EXP_ID;
  exc.controllerEnable(EXP_ID);
  safeDelay(10);
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_REAR_LEFT, INPUT);
  pinMode(SENSOR_REAR_CENTER, INPUT);
  pinMode(SENSOR_REAR_RIGHT, INPUT);
  prizm.resetEncoders();

  Serial.println(F("==== 하드웨어 테스트 모드 펌웨어 시작 ===="));
  
  // 테스트 메뉴 무한 루프 진입
  runTestMenu(); 
}

void loop() {
  // 테스트 모드에선 루프 함수 사용 안 함
}

// ────────────────────────────────────────────────────────────
// [2] 실제 대회용 자율 주행 펌웨어 빌드 (Config.h 에서 0으로 설정 시)
// ────────────────────────────────────────────────────────────
#else
void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();

  extern const int EXP_ID;
  exc.controllerEnable(EXP_ID);
  safeDelay(10);
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_REAR_LEFT, INPUT);
  pinMode(SENSOR_REAR_CENTER, INPUT);
  pinMode(SENSOR_REAR_RIGHT, INPUT);
  prizm.resetEncoders();

#if QR_SIMULATION
  // 가상 맵 데이터 생성 (카메라/박스 없이 전체 미션 테스트)
  setupRandomLayout();
#else
  // 실제 QR: 박스맵은 0에서 시작해 스캔으로만 채워진다
  for (int i = 0; i < 7; i++) { boxes[i].present = false; boxes[i].found = false; boxes[i].destination = 0; }
  HuskyQR::begin();   // Wire는 PrizmBegin()이 이미 시작함
#endif

  Serial.println(F("시스템 준비 완료. 초록색 Start 버튼을 누르면 자율 주행을 시작합니다."));

  // 스타트 버튼 대기
  prizm.setGreenLED(HIGH);
  while (prizm.readStartButton() == 0) { delay(10); }
  prizm.setGreenLED(LOW);
  safeDelay(200);
}

void loop() {
  // [1단계] 탐색
  executeStage1_Search();

  safeDelay(2000);

  // [2단계] 배송
  executeStage2_Delivery();

  // 종료 및 복귀
  returnToFinish();
  while (true) { delay(100); } 
}
#endif
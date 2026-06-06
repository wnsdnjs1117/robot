/* ============================================================
 * main.cpp - 로봇 자율주행 / 테스트 모드 진입점
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
int  lineTraceLastEdge = 0;
bool intersectionArmed = true;
int  intersectionHitCount = 0;

static void initHardware() {
  Serial.begin(9600);
  prizm.PrizmBegin();

  exc.controllerEnable(EXP_ID);
  delayWithTicks(10);
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  pinMode(PIN_LINE_FRONT_LEFT, INPUT);
  pinMode(PIN_LINE_FRONT_CENTER, INPUT);
  pinMode(PIN_LINE_FRONT_RIGHT, INPUT);
  prizm.resetEncoders();
}

#if RUN_TEST_MODE == 1

void setup() {
  initHardware();
  Serial.println(F("==== 하드웨어 테스트 모드 펌웨어 시작 ===="));
  runTestMenu();
}

void loop() {}

#else

void setup() {
  initHardware();

#if QR_SIMULATION
  setupRandomLayout();
#else
  for (int i = 0; i < 7; i++) {
    boxes[i].present = false;
    boxes[i].found = false;
    boxes[i].destination = 0;
  }
  HuskyQR::begin();
#endif

#if ROBOT_DEBUG
  // PrizmBegin에서 이미 1회 Start — 디버그 시 2번째 누름으로 경기 시작
  Serial.println(F("시스템 준비 완료. 초록색 Start 버튼을 누르면 자율 주행을 시작합니다."));
  prizm.setGreenLED(HIGH);
  while (prizm.readStartButton() == 0) delay(10);
  prizm.setGreenLED(LOW);
  delayWithTicks(200);
#endif
  // ROBOT_DEBUG 0: PrizmBegin의 Start 직후 loop()에서 경기 시작
}

void loop() {
  runSearchPhase();
  runDeliveryPhase();
  driveToFinishArea();
  while (true) delay(100);
}

#endif

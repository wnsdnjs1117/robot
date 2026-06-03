/* ============================================================
 * main.cpp - 로봇 자율주행 마스터 시스템 진입점
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

#include "LiftTest.h" // ★ LiftTest.h 삭제하지 않고 여기서 정상 호출
#include "SensorTest.h" // ★ SensorTest.h 삭제하지 않고 여기서 정상 호출
#include "MoveTest.h" // ★ 시리얼 이동/회전 테스트 모드

// 하드웨어 제어 인스턴스 및 상태 변수 정의
PRIZM prizm;
int lastSensorState = 0;
bool crossingArmed = true;
int crossingStable = 0;

#if !LIFT_TEST_MODE && !SENSOR_TEST_MODE && !MOVE_TEST_MODE // 일반 자율주행 모드 (테스트 모드 전부 비활성화)
void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();

  // EXPANSION 리프트 컨트롤러 초기화
  extern const int EXP_ID;
  exc.controllerEnable(EXP_ID);
  delay(10);
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_REAR_LEFT, INPUT);
  pinMode(SENSOR_REAR_CENTER, INPUT);
  pinMode(SENSOR_REAR_RIGHT, INPUT);
  prizm.resetEncoders();

  // 가상 맵 데이터 셔플 생성
  setupRandomLayout();

  // 시스템 준비 완료. 스타트 버튼 대기
  prizm.setGreenLED(HIGH);
  while (prizm.readStartButton() == 0) {
    delay(10);
  }
  prizm.setGreenLED(LOW);
  delay(200);
}

void loop() {
  // [1단계] 탐색: 박스 2개를 발견하면 즉시 해당 구역에서 주행 셧다운!
  executeStage1_Search();

  // 기구학적 안정을 위한 2초 대기
  delay(2000);

  // [2단계] 배송: 멈춘 위치(존)에서부터 다이나믹하게 최단 거리 배송 수행!
  executeStage2_Delivery();

  // 모든 임무 완료 시 FINISH 구역 복귀 → 시스템 락다운
  returnToFinish();
  while (true);
}
#endif // !LIFT_TEST_MODE
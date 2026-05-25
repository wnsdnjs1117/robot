/* ============================================================
 * [1단계] QR 탐색 단계 - 메인
 * 제어기: Arduino UNO + TETRIX PRIZM
 *
 * [파일 구성]
 *   Config.h          : 파라미터 + 전역변수 선언
 *   Motion.h/.cpp     : 저수준 - 모터, 센서, 라인트레이싱
 *   Navigation.h/.cpp : 고수준 - 주행 시나리오
 *   BoxMap.h/.cpp     : 박스 위치 관리 + QR 스캔 시뮬레이션
 *   LiftTest.h/.cpp   : 리프트 프로토타입 (독립 테스트용)
 *   main.cpp          : 전역변수 정의 + setup/loop
 * ============================================================ */
#include "BoxMap.h"
#include "Config.h"
#include "LiftTest.h"
#include "Motion.h"
#include "Navigation.h"

// ============================================================
//  전역 객체/변수 "정의" (Config.h 의 extern 실체)
// ============================================================
PRIZM prizm;
int lastSensorState = 0;
bool crossingArmed = true;
int crossingStable = 0;

// ============================================================
void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();

  // [리프트 단독 테스트] - 무한 루프라 주행을 막으므로 평소엔 주석 처리.
  //   리프트만 시험할 때 아래 줄의 주석을 풀어서 사용하세요.
  // runLiftStallTest();

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  prizm.resetEncoders();

  // 이번 판 박스 배치를 랜덤 생성 + 정답지 시리얼 출력
  setupRandomLayout();

  prizm.setGreenLED(HIGH);
  while (prizm.readStartButton() == 0) {
    delay(10);
  }
  prizm.setGreenLED(LOW);
  delay(200);
}

void loop() {
  goToMainLine();
  qrSearchStage();   // 내부에서 박스 4개 확정 시 정지 + 결과 출력

  prizm.setGreenLED(HIGH);
  while (true);      // 1단계 종료
}

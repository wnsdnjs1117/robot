/* ============================================================
 * MapRouter.cpp - 동서남북 방위 및 더블 점프 로직 구현부
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "LiftTest.h"
#include "Motion.h"
#include "Navigation.h"

// ============================================================
// [공통 유틸리티] 라인 끊긴 구간 생 직진(Blind Run) 점프 함수
// ============================================================
void executeBlindRun() {
  prizm.resetEncoders();

  // 1. 선 탈출
  while (abs(prizm.readEncoderCount(1)) < 400) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    // updateLiftInMotion(); // <--- 이 부분을 지우거나 주석 처리
    delay(5);
  }

  // 2. 다음 선 찾을 때까지 직진
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;

    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    // updateLiftInMotion(); // <--- 여기도 지우거나 주석 처리
    delay(5);
  }
  stopAll();
}

// ============================================================
// [경로 1] 2번(우측상단) ➔ 6번(출고) : 동쪽 더블 점프
// ============================================================
void moveNode2ToNode6(bool cameToNode2Forward) {
  // 1. 2번 ➔ 8번 (남쪽 이동)
  if (cameToNode2Forward) {
    reverseAcrossToOppositeZone();  // 북쪽 보고 있으니 후진
    spinRight90();                  // 북쪽 -> 우회전 -> 동쪽 보기
  } else {
    followToCrossing();  // 남쪽 보고 있으니 전진
    spinLeft90();        // 남쪽 -> 좌회전 -> 동쪽 보기
  }

  // 2. 8번 ➔ 9번 (라인 이동)
  followToCrossing();

  // 3. 9번 ➔ 10번 ➔ 11번 (동쪽 더블 점프)
  Serial.println(F(">> [BLIND] 9->10 1차 점프"));
  executeBlindRun();  // 10번 도착

  Serial.println(F(">> [BLIND] 10->11 2차 점프"));
  executeBlindRun();  // 11번 도착

  // 4. 11번에서 북쪽(6번)으로 회전 및 진입
  spinLeft90();
  followToCrossing();
}

// ============================================================
// [경로 2] 6번(출고) ➔ 3번(좌측하단) : 서쪽 더블 점프 후 남하
// ============================================================
void moveNode6ToNode3(bool cameToNode6Forward) {
  // 1. 6번 ➔ 11번 (남쪽 이동)
  if (cameToNode6Forward) {
    reverseAcrossToOppositeZone();
    spinLeft90();  // 서쪽 보기
  } else {
    followToCrossing();
    spinRight90();  // 서쪽 보기
  }

  // 2. 11번 ➔ 10번 ➔ 9번 (서쪽 더블 점프)
  Serial.println(F(">> [BLIND] 11->10 1차 점프"));
  executeBlindRun();

  Serial.println(F(">> [BLIND] 10->9 2차 점프"));
  executeBlindRun();

  // 3. 9번 ➔ 8번 ➔ 7번 (서쪽 라인 연속 주행)
  followToCrossing();  // 8번 도착
  followToCrossing();  // 7번 도착

  // 4. 7번에서 남쪽(3번)으로 회전 및 진입
  spinLeft90();
  followToCrossing();
}

// ============================================================
// [경로 3] 5번(입고) ➔ 1번(좌측상단) : 서쪽 싱글 점프 후 북상
// ============================================================
void moveNode5ToNode1(bool cameToNode5Forward) {
  // 1. 5번 ➔ 10번 (남쪽 이동)
  if (cameToNode5Forward) {
    reverseAcrossToOppositeZone();
    spinLeft90();  // 서쪽 보기
  } else {
    followToCrossing();
    spinRight90();  // 서쪽 보기
  }

  // 2. 10번 ➔ 9번 (서쪽 싱글 점프)
  Serial.println(F(">> [BLIND] 10->9 싱글 점프"));
  executeBlindRun();

  // 3. 9번 ➔ 8번 ➔ 7번 (서쪽 라인 연속 주행)
  followToCrossing();  // 8번 도착
  followToCrossing();  // 7번 도착

  // 4. 7번에서 북쪽(1번)으로 회전 및 진입
  spinRight90();
  followToCrossing();
}

// ============================================================
// [경로 4] 5번(입고) ➔ 3번(좌측하단) : 서쪽 싱글 점프 후 남하
// ============================================================
void moveNode5ToNode3(bool cameToNode5Forward) {
  // 1. 5번 ➔ 10번 (남쪽 이동 및 서쪽 정렬)
  if (cameToNode5Forward) {
    reverseAcrossToOppositeZone();
    spinLeft90();
  } else {
    followToCrossing();
    spinRight90();
  }

  // 2. 10번 ➔ 9번 (서쪽 싱글 점프)
  executeBlindRun();

  // 3. 9번 ➔ 8번 ➔ 7번 (서쪽 연속 주행)
  followToCrossing();
  followToCrossing();

  // 4. 7번에서 남쪽(3번)으로 회전 및 진입
  spinLeft90();
  followToCrossing();
}
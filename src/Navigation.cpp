/* ============================================================
 * Navigation.cpp - 즉시 종료 및 구역 번호 반환형 탐색 알고리즘
 * ============================================================ */
#include "Navigation.h"

#include "BoxMap.h"
#include "Config.h"
#include "Motion.h"

// 교차로 감지 시까지 라인트레이싱 후 정렬 정지
void followToCrossing() {
  crossingArmed = true;
  crossingStable = 0;
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (detectCrossing(L, C, R)) {
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
        drive(SPEED, SPEED);
        delay(5);
      }
      stopAll();
      return;
    }
    lineFollowStep(L, C, R);
    delay(5);
  }
}

void forwardToCrossing() { followToCrossing(); }

// 구역 내부에서 수직 후진 기동으로 교차로를 지나 반대 구역까지 관통 주행
void reverseAcrossToOppositeZone() {
  crossingArmed = true;
  crossingStable = 0;
  lastSensorState = 0;

  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (detectCrossing(L, C, R)) break;
    lineFollowStepReverse(L, C, R);
    delay(5);
  }
  stopAll();
  delay(100);

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < ZONE_DEPTH_COUNTS) {
    int L, C, R;
    readSensors(L, C, R);
    lineFollowStepReverse(L, C, R);
    delay(5);
  }
  stopAll();
}

// 구역 내부 진입용 엔코더 직진 런
void enterZone() {
  prizm.resetEncoders();
  lastSensorState = 0;

  while (abs(prizm.readEncoderCount(1)) < ZONE_ENTER_COUNTS) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) {
      lineFollowStep(L, C, R);
    } else {
      drive(SPEED, SPEED);
    }
    delay(5);
  }
  stopAll();
}

// 스타트 박스 이탈 및 2행 메인라인 합류
void goToMainLine() {
  Serial.println(F(">>> [START-RUN] 스타트 박스 탈출"));
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < START_ESCAPE_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(200);

  Serial.println(F(">>> [START-RUN] 서쪽(좌측) 방향 전환"));
  if (WEST_IS_LEFT)
    turnAngle(90, false);
  else
    turnAngle(90, true);

  Serial.println(F(">>> [START-RUN] 메인라인 진입"));
  int passedLines = 0;
  bool lineArmed = true;
  int lineStable = 0;

  while (passedLines < 2) {
    int L, C, R;
    readSensors(L, C, R);
    bool onLine = anyLine(L, C, R);
    if (onLine)
      lineStable++;
    else
      lineStable = 0;
    if (onLine && lineArmed && lineStable >= CROSS_CONFIRM) {
      passedLines++;
      lineArmed = false;
    }
    if (!onLine) lineArmed = true;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (!anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
}

// ★ [리팩토링 핵심] 2개 발견 즉시 현재 구역 ID(1~4)를 반환하는 탐색 엔진
int qrSearchStage() {
  int randomFound = 0;

  Serial.println(F("\n--- [2구역 탐색] ---"));
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 2;
  }  // 2구역 안에서 즉시 종료

  Serial.println(F("\n--- [4구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 4;
  }  // 4구역 안에서 즉시 종료

  Serial.println(F("\n--- [1구역 탐색] ---"));
  forwardToCrossing();
  turnAngle(90, false);
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 1;
  }  // 1구역 안에서 즉시 종료

  Serial.println(F("\n--- [3구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  scanZone(3);
  stopAll();
  printSearchResult();
  return 3;  // 3구역 안에서 최종 종료
}
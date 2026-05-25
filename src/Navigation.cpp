#include "Navigation.h"

#include "BoxMap.h"
#include "Config.h"
#include "Motion.h"

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

void enterZone() {
  prizm.resetEncoders();
  lastSensorState = 0;

  while (abs(prizm.readEncoderCount(1)) < ZONE_ENTER_COUNTS) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R))
      lineFollowStep(L, C, R);
    else
      drive(SPEED, SPEED);
    delay(5);
  }
  stopAll();
}

void goToMainLine() {
  Serial.println(F(">>> [스텝 1] 스타트 박스 탈출"));
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

  Serial.println(F(">>> [스텝 2] 서쪽 회전"));
  if (WEST_IS_LEFT)
    turnAngle(90, false);
  else
    turnAngle(90, true);

  Serial.println(F(">>> [스텝 3] 라인 통과 및 메인라인 진입"));
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

void qrSearchStage() {
  int randomFound = 0;

  Serial.println(F("\n--- [2구역 탐색] ---"));
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return;
  }

  Serial.println(F("\n--- [4구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return;
  }

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
    return;
  }

  Serial.println(F("\n--- [3구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  scanZone(3);
  stopAll();
  printSearchResult();
}
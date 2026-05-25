/* ============================================================
 * Navigation.cpp - 후진 관련 모든 정렬/조향 로직 완전 제거 버전
 * ============================================================ */
#include "Navigation.h"

#include "BoxMap.h"
#include "Config.h"
#include "Motion.h"

// 전진 중 교차로 감지 후 정렬 및 정지
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

// ★ [완벽 수정]: 후진 시 센서 조건 및 물리 정렬(Align)을 완전히 제외
// 이제 이 함수는 처음부터 끝까지 오직 완전한 일직선 후진만 수행합니다.
void reverseAcrossToOppositeZone() {
  crossingArmed = true;
  crossingStable = 0;
  lastSensorState = 0;

  // Step A: 교차로(1,1,1)를 만날 때까지 아무것도 따지지 않고 오직 일직선 후진
  // lineFollowStepReverse가 이제 무조건 drive(-BACK_SPEED, -BACK_SPEED)만
  // 하므로 센서가 사선으로 닿든 말든 묵묵히 일자로만 밀고 갑니다.
  while (true) {
    int L, C, R;
    readSensors(L, C, R);

    if (detectCrossing(L, C, R))
      break;  // 오직 교차로(1,1,1) 도착 여부만 확인하여 탈출

    lineFollowStepReverse(L, C, R);
    delay(5);
  }
  stopAll();  // 교차로 도착 시 브레이크
  delay(100);

  // Step B: 교차점 통과 후 반대편 구역 깊이만큼 진입 (역시 무조건 일직선 후진)
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < ZONE_DEPTH_COUNTS) {
    int L, C, R;
    readSensors(L, C, R);

    // 센서가 선을 보든 안 보든 무조건 일직선 후진 명령만 들어갑니다.
    lineFollowStepReverse(L, C, R);
    delay(5);
  }
  stopAll();
}

// 박스 구역 진입 (전진 - 기존 유지)
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

// 탐색 종료 처리: 정지 + 결과 출력
static void finishSearch() {
  stopAll();
  printSearchResult();
}

// 스타트 박스 탈출 주행
void goToMainLine() {
  Serial.println(">>> [스텝 1] 스타트 박스 탈출");
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

  Serial.println(">>> [스텝 2] 서쪽(좌측) 90도 회전");
  if (WEST_IS_LEFT)
    spinLeft90();
  else
    spinRight90();

  Serial.println(">>> [스텝 3] 출고선/입고선 통과 카운트");
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

  Serial.println(">>> [스텝 4] 메인라인 진입");
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
}

// 1단계 탐색 시나리오
void qrSearchStage() {
  int randomFound = 0;

  // --- 2구역 ---
  Serial.println(F("\n--- [2구역 탐색] ---"));
  followToCrossing();
  spinRight90();
  enterZone();
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    finishSearch();
    return;
  }

  // --- 4구역 (후진) ---
  Serial.println(F("\n--- [4구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    finishSearch();
    return;
  }

  // --- 1구역 (전진 + 좌회전 + 우회전) ---
  Serial.println(F("\n--- [1구역 탐색] ---"));
  forwardToCrossing();
  spinLeft90();
  followToCrossing();
  spinRight90();
  enterZone();
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) {
    finishSearch();
    return;
  }

  // --- 3구역 (후진) ---
  Serial.println(F("\n--- [3구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  scanZone(3);
  finishSearch();
}
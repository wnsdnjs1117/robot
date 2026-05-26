/* ============================================================
 * Navigation.cpp - 즉시 종료 및 구역 번호 반환형 탐색 알고리즘
 * ============================================================ */
#include "Navigation.h"

#include "BoxMap.h"
#include "Config.h"
#include "LiftTest.h"
#include "MapRouter.h"
#include "Motion.h"

// 교차로 감지 시까지 라인트레이싱 후 정렬 정지
void followToCrossing() {
  // 이미 교차로(L=C=R=1) 위에 있으면 벗어날 때까지 직진 후 탐색 시작
  // (exitZone 후진 탈출 직후 교차로 위에 멈춰있는 경우 대응)
  {
    int L, C, R;
    readSensors(L, C, R);
    if (L == 1 && C == 1 && R == 1) {
      // 교차로를 완전히 벗어날 때까지 직진
      while (true) {
        readSensors(L, C, R);
        if (!(L == 1 && C == 1 && R == 1)) break;
        drive(SPEED, SPEED);
        delay(5);
      }
      // 벗어난 후 한 박자 더 전진 (센서가 완전히 교차로 밖으로)
      for (int i = 0; i < 10; i++) {
        drive(SPEED, SPEED);
        delay(5);
      }
      stopAll();
      delay(50);
    }
  }

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
// 구역 내부에서 교차로를 통과해 반대 구역까지 후진 관통 (1단계 탐색 전용)
// 후진 중 센서 사용 불가 → 전체 엔코더 기반
// ZONE_EXIT_REV_COUNTS: 구역→교차로, ZONE_DEPTH_COUNTS: 교차로→반대 구역
void reverseAcrossToOppositeZone() {
  lastSensorState = 0;

  // 1단계: 구역 내부 → 교차로 통과 거리 후진
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < ZONE_EXIT_REV_COUNTS) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  // 2단계: 교차로 통과 후 반대 구역 안쪽까지 추가 후진
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < ZONE_DEPTH_COUNTS) {
    drive(-BACK_SPEED, -BACK_SPEED);
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
    liftDownTick();
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < START_ESCAPE_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
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
    liftDownTick();
    delay(5);
  }
  // 라인을 완전히 벗어날 때까지 전진
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (!anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }
  // 메인라인 감지까지 전진
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }
  // 라인 위에서 CROSS_ALIGN_COUNTS만큼 더 전진해 교차로 중심에 정렬
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();
    delay(5);
  }
  stopAll();
}

// ★ FINISH 구역 복귀 및 부저 울림
// currentNode 기반으로 어디서든 최단 경로로 FINISH 진입
void returnToFinish() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [FINISH] FINISH 구역 복귀 기동"));

  // FINISH(START)는 9번 노드에서 동쪽+남쪽으로 접근
  // 9번 → 동쪽 직진 → 라인 2개 → 남쪽으로 꺾어 진입
  // 9번 노드로 이동 후 동향으로 FINISH 방향 직진
  moveToNode(9);

  // 동향으로 정렬 후 FINISH 방향으로 전진
  turnToHeading(1);

  // 메인라인을 동쪽으로 따라가며 라인 2개 통과 (FINISH 앞까지)
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
  stopAll();
  delay(200);

  // 남쪽으로 회전 → FINISH 진입
  turnAngle(90, true);  // 동쪽 → 남쪽
  robotHeading = 2;

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < FINISH_ENTRY_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  Serial.println(F(">> [FINISH] FINISH 구역 진입 완료!"));

  // [5] 부저 1~2초 울리기 (대회 규정: 1초~2초)
  //     PRIZM에 전용 buzzer API 없으므로 Arduino tone() 사용
  //     부저가 별도 핀에 연결된 경우 BUZZER_PIN 값 조정 필요
  tone(BUZZER_PIN, 1000);  // 1000Hz
  delay(1500);             // 1.5초 (규정 범위 내)
  noTone(BUZZER_PIN);

  // [6] 부저 후 로봇 완전 정지 (규정: 부저 후 움직이면 종료 불인정)
  prizm.setGreenLED(HIGH);
  Serial.println(F(">> [FINISH] 부저 완료. 경기 종료."));
  Serial.println(F("========================================\n"));
}

// ★ [리팩토링 핵심] 2개 발견 즉시 현재 구역 ID(1~4)를 반환하는 탐색 엔진
int qrSearchStage() {
  int randomFound = 0;

  Serial.println(F("\n--- [2구역 탐색] ---"));
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  lastEntryWasForward = true;  // 전진 진입
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 2;
  }

  Serial.println(F("\n--- [4구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;  // 후진 진입
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 4;
  }

  Serial.println(F("\n--- [1구역 탐색] ---"));
  forwardToCrossing();
  turnAngle(90, false);
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  lastEntryWasForward = true;  // 전진 진입
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 1;
  }

  Serial.println(F("\n--- [3구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;  // 후진 진입
  scanZone(3);
  stopAll();
  printSearchResult();
  return 3;
}
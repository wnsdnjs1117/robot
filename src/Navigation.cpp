/* ============================================================
 * Navigation.cpp - 즉시 종료 및 구역 번호 반환형 탐색 알고리즘
 * ============================================================ */
#include "Navigation.h"

#include "BoxMap.h"
#include "Config.h"
#include "LiftTest.h"
#include "MapRouter.h"
#include "Motion.h"

// ── [1] 라인 추종 → 교차로 ───────────────────────────────────

// 교차로 감지 시까지 라인트레이싱 후 정렬 정지 (전방 센서만 사용)
void followToCrossing() {
  // 이미 교차로(L=C=R=1) 위에 있으면 벗어날 때까지 직진 후 탐색 시작
  // (exitZone 후진 탈출 직후 교차로 위에 멈춰있는 경우 대응)
  {
    int L, C, R;
    readSensors(L, C, R);
    if (L == 1 && C == 1 && R == 1) {
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
    lineFollowStep(L, C, R);  // 전진 = 전방 센서만 사용
    delay(5);
  }
}

void forwardToCrossing() { followToCrossing(); }

// ── [2] 블라인드 구간 출발 정렬 ──────────────────────────────
//
// 후방 센서를 이용해 로봇을 라인에 수직 정렬한다.
//   Phase 1: 후방 센서가 라인에 닿을 때까지 조금씩 전진
//   Phase 2: 6가지 패턴 분기로 미세 회전 정렬 (최대 ALIGN_MAX_COUNTS = 5도)
//
// 패턴 처리:
//   000 → 라인 완전 이탈, 강제 중단
//   010 → 중앙만 감지, 충분히 정렬된 것으로 간주
//   111 → 교차로 위, 정렬 불필요
//   1xx → 좌측 감지 → CCW 회전 (RR 끌어당김)
//   xx1 → 우측 감지 → CW 회전 (RL 끌어당김)
//   11x → 좌+중 감지 → 미세 CW
//   x11 → 중+우 감지 → 미세 CCW

void alignHeadingOnLine() {
  // Phase 1: 후방 센서가 라인에 닿을 때까지 전진
  prizm.resetEncoders();
  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    if (anyRearLine(RL, RC, RR)) break;
    if (abs(prizm.readEncoderCount(1)) > REAR_TO_AXLE_COUNTS + 100) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(50);

  // Phase 2: 패턴별 미세 회전 정렬 (5도 제한)
  prizm.resetEncoders();
  for (int t = 0; t < 60; t++) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);

    if (abs(prizm.readEncoderCount(1)) >= ALIGN_MAX_COUNTS) break;  // 5도 초과 방지

    if (RL && RR)           break;  // 양측 감지 → 완전 정렬
    if (!RL && RC && !RR)   break;  // 중앙만 감지 → 충분히 정렬
    if (RL && RC && RR)     break;  // 교차로 위 → 정렬 불필요
    if (!RL && !RC && !RR)  break;  // 라인 이탈 → 강제 중단

    if      (RL && !RC && !RR) drive(-4,  4);   // 좌측만 → CCW
    else if (!RL && !RC && RR) drive( 4, -4);   // 우측만 → CW
    else if (RL &&  RC && !RR) drive( 4, -4);   // 좌+중  → 미세 CW
    else if (!RL &&  RC && RR) drive(-4,  4);   // 중+우  → 미세 CCW

    delay(10);
  }
  stopAll();
  delay(50);
}

// ── [3] 존 진입/탈출 ─────────────────────────────────────────

// 전진으로 존 진입: 전방 라인 추종 → 끊기면 ZONE_ENTER_EXTRA 추가 전진 (전방 센서만)
void enterZone() {
  lastSensorState = 0;

  // Phase 1: 전방 라인이 끊길 때까지 라인트레이싱 (전방 센서만)
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (!anyLine(L, C, R)) break;
    lineFollowStep(L, C, R);
    delay(5);
  }

  // Phase 2: 라인 끊긴 지점부터 ZONE_ENTER_EXTRA 추가 전진 (직진)
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < ZONE_ENTER_EXTRA) {
    drive(SPEED, SPEED);
    delay(5);
  }
  stopAll();
}

// 후진으로 존 진입: 후방 센서로 라인 추종 → 끊기면 ZONE_DEPTH_EXTRA 추가 후진 (후방 센서만)
void reverseEnterZone() {
  lastSensorState = 0;
  bool lineWasFound = false;  // 라인을 한 번이라도 감지했는지 추적

  // Phase 2-A: 후방 센서로 라인 추종하며 후진 (최대 ZONE_FOLLOW_MAX까지)
  // lineWasFound=true 후 라인이 끊기면 Phase 2-B로 이행
  prizm.resetEncoders();
  while (true) {
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);

    bool rearHasLine = anyRearLine(RL, RC, RR);

    if (rearHasLine) lineWasFound = true;

    // 라인이 한번 발견된 후 끊기면 Phase 2-B로
    if (lineWasFound && !rearHasLine) break;
    // 안전 탈출 (무한루프 방지)
    if (abs(prizm.readEncoderCount(1)) >= ZONE_FOLLOW_MAX) break;

    // 후진 조향: 후방 센서만 사용
    int lsp = -BACK_SPEED, rsp = -BACK_SPEED;
    if (rearHasLine) {
      if      (RL && !RC && !RR) { lsp = -(BACK_SPEED - 10); rsp = -(BACK_SPEED + 10); }
      else if (!RL && !RC && RR) { lsp = -(BACK_SPEED + 10); rsp = -(BACK_SPEED - 10); }
    }
    drive(lsp, rsp);
    delay(5);
  }

  // Phase 2-B: 라인 끊긴 지점부터 ZONE_DEPTH_EXTRA 추가 후진 (직진)
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < ZONE_DEPTH_EXTRA) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(5);
  }
  stopAll();
}

// 구역 내부에서 수직 후진 기동으로 교차로를 지나 반대 구역까지 관통 주행
void reverseAcrossToOppositeZone() {
  lastSensorState = 0;

  // 1단계: 구역 내부 → 교차로 통과까지 엔코더 기반 후진
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < ZONE_EXIT_REV_COUNTS) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  // 2단계: 반대 구역 라인 추종 후진 → 끊기면 ZONE_DEPTH_EXTRA 추가
  reverseEnterZone();
}

// ── [4] 특수 경로 ────────────────────────────────────────────

// 스타트 박스 이탈 및 메인라인 합류
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
  bool lineArmed  = true;
  int  lineStable = 0;

  while (passedLines < 2) {
    int L, C, R;
    readSensors(L, C, R);
    bool onLine = anyLine(L, C, R);
    if (onLine)  lineStable++;
    else         lineStable = 0;
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

// FINISH 구역 복귀 및 부저 울림
// currentNode 기반으로 어디서든 최단 경로로 FINISH 진입
void returnToFinish() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [FINISH] FINISH 구역 복귀 기동"));

  moveToNode(9);
  turnToHeading(1);  // 동향으로 정렬

  // 메인라인을 동쪽으로 따라가며 라인 2개 통과 (FINISH 앞까지)
  int passedLines = 0;
  bool lineArmed  = true;
  int  lineStable = 0;
  while (passedLines < 2) {
    int L, C, R;
    readSensors(L, C, R);
    bool onLine = anyLine(L, C, R);
    if (onLine)  lineStable++;
    else         lineStable = 0;
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

  turnAngle(90, true);   // 동쪽 → 남쪽
  robotHeading = 2;

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < FINISH_ENTRY_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  Serial.println(F(">> [FINISH] FINISH 구역 진입 완료!"));

  // 부저 1.5초 (대회 규정: 1~2초)
  tone(BUZZER_PIN, 1000);
  delay(1500);
  noTone(BUZZER_PIN);

  prizm.setGreenLED(HIGH);
  Serial.println(F(">> [FINISH] 부저 완료. 경기 종료."));
  Serial.println(F("========================================\n"));
}

// ★ [탐색 엔진] 2개 발견 즉시 현재 구역 ID(1~4)를 반환
int qrSearchStage() {
  int randomFound = 0;

  Serial.println(F("\n--- [2구역 탐색] ---"));
  followToCrossing();
  turnAngle(90, true);
  enterZone();
  lastEntryWasForward = true;
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 2;
  }

  Serial.println(F("\n--- [4구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;
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
  lastEntryWasForward = true;
  if (scanZone(1)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 1;
  }

  Serial.println(F("\n--- [3구역 탐색] ---"));
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;
  scanZone(3);
  stopAll();
  printSearchResult();
  return 3;
}

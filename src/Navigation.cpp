/* ============================================================
 * Navigation.cpp - 거리/각도 기반 진입 및 탈출 탐색 시나리오
 * ============================================================ */
#include "Navigation.h"

#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"

// ── [1] 일반 수평선 라인 추종 ───────────────────────────────────

void followToCrossing(bool stopAtEnd) {
  // 방금 출발한 교차로를 다시 교차로로 오인하지 않도록 강제로 센서가 선을 벗어날 때까지 밀어냅니다.
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
      for (int i = 0; i < 10; i++) {
        drive(SPEED, SPEED);
        delay(5);
      }
    }
  }

  crossingArmed = true;
  crossingStable = 0;
  // 본격적인 다음 노드(교차로) 탐색 라인트레이싱
  while (true) {
    int L, C, R;
    readSensors(L, C, R);

    // 교차로 십자가를 발견하면 중심을 맞추기 위해 지정 거리만큼 더 전진
    if (detectCrossing(L, C, R)) {
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
        drive(SPEED, SPEED);
        delay(5);
      }
      if (stopAtEnd) stopAll();
      return;
    }

    // 평상시엔 앞뒤 센서 기반 고속 라인트레이싱 수행
    int RL, RC, RR;
    readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR);
    delay(5);  // 이 딜레이가 로봇의 150Hz 반응 속도를 만듭니다.
  }
}

void followToCrossing() { followToCrossing(true); }

// ── [2] 존(구역) 진입 및 횡단 (순수 이동 거리 기반) ─────────────────

void enterZone() {
  DPRINTLNF(">> [NAV] 전진으로 존 진입 시작");
  prizm.resetEncoders();
  // 라인 무시, Config.h에 지정된 거리(cm)만큼 무조건 진입
  while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_ENTER_FWD_COUNTS) {
    drive(SPEED, SPEED);
    delay(5);
  }
  stopAll();
}

void reverseEnterZone() {
  DPRINTLNF(">> [NAV] 후진으로 존 진입 시작");
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_ENTER_REV_COUNTS) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(5);
  }
  stopAll();
}

void reverseAcrossToOppositeZone() {
  DPRINTLNF(">> [NAV] 반대편 존으로 횡단 (거리 기반)");

  // 복도 교차로로 후진 탈출 후, 방향을 꺾지 않고 곧바로 맞은편 구역으로 후진 진입
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_ZONE_EXIT_REV_COUNTS) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  reverseEnterZone();
}

// ── [3] 특수 시나리오: 출발 및 복귀 기동 ──────────────────────────

void goToMainLine() {
  DPRINTLNF(">>> [START-RUN] 서향 출발 -> 12번 노드(빈 공간) -> 9-2 노드 -> 8번 노드");

  robotHeading = 270;

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_START_TO_12_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    liftDownTick();  // 리프트를 내리면서 맹주행 동시 진행
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(HEADING_12_TO_9);

  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_CROSS_ALIGN_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(270);
  followToCrossing(true);

  currentNode = 8;
  DPRINTLNF(">> [START] 8번 노드 안착 완료");
}

void returnToFinish() {
  DPRINTLNF("\n========================================");
  DPRINTLNF(">> [FINISH] 복귀 기동: 9-3 -> 12 -> START");

  moveToNode(9);

  turnToHeading(90);
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_9_TO_9_3_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(HEADING_9_3_TO_12);

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_9_3_TO_12_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
  delay(100);

  turnToHeading(HEADING_12_TO_START);
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED);
    delay(5);
  }

  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < DIST_FINISH_ENTRY_COUNTS) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();

  // 종료 세리머니
  tone(BUZZER_PIN, 1000);
  delay(1500);
  noTone(BUZZER_PIN);
  prizm.setGreenLED(HIGH);
  DPRINTLNF(">> [FINISH] 경기 종료.");
  DPRINTLNF("========================================\n");
}

// ── [4] 탐색 스테이지 (1~4구역 순회) ──────────────────────────────────

int qrSearchStage() {
  int randomFound = 0;

  // 동선 최적화를 위해 위(2구역) -> 아래(4구역) 횡단 -> 이동 -> 위(1구역) -> 아래(3구역) 순서로 탐색
  DPRINTLNF("\n--- [2구역 탐색] ---");
  enterZone();
  lastEntryWasForward = true;
  if (scanZone(2)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 2;  // 박스 2개를 일찍 다 찾으면 쓸데없는 곳은 돌지 않고 즉시 종료
  }

  DPRINTLNF("\n--- [4구역 탐색] ---");
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;
  if (scanZone(4)) randomFound++;
  if (randomFound >= 2) {
    stopAll();
    printSearchResult();
    return 4;
  }

  DPRINTLNF("\n--- [1구역 탐색] ---");
  followToCrossing();
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

  DPRINTLNF("\n--- [3구역 탐색] ---");
  reverseAcrossToOppositeZone();
  lastEntryWasForward = false;
  scanZone(3);
  stopAll();
  printSearchResult();
  return 3;
}
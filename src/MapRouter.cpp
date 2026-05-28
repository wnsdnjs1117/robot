/* ============================================================
 * MapRouter.cpp - 헤딩 완전 추적 / 진입방식 기반 탈출
 *
 * 헤딩: 로봇 앞면이 향하는 방향 (HDG_N=0, HDG_E=1, HDG_S=2, HDG_W=3)
 *
 * 맵 구조:
 *         북
 *   [1]   [2]          [5] [6]
 *    |     |            |   |
 *   [7]---[8]---[9]--[10]--[11]   ← 메인라인 (동서)
 *    |     |
 *   [3]   [4]
 *         남
 *   START는 [9] 동쪽, 메인라인 남쪽
 *
 * 구역-노드 관계:  1,3 → 7  /  2,4 → 8  /  5 → 10  /  6 → 11
 *
 * 진입 원칙 (가능하면 전진 우선):
 *   북쪽 구역(1,2,5,6): 교차로에서 북향 전진 진입
 *   남쪽 구역(3,4):     교차로에서 북향 상태로 후진 진입
 *
 * 탈출 원칙 (진입의 반대):
 *   전진 진입 → 후진 탈출  /  후진 진입 → 전진 탈출
 *   → 탈출 후 robotHeading = HDG_N 유지 (exitZone이 직접 갱신하지 않아도 됨)
 *
 * lastEntryWasForward: exitZone()이 올바른 탈출 방식을 선택하는 데 사용
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "Motion.h"

// Navigation.cpp와 순환 include 방지 — 필요한 함수만 전방 선언
void followToCrossing();
void reverseAcrossToOppositeZone();
void enterZone();
void reverseEnterZone();
void alignHeadingOnLine();

int  robotHeading       = HDG_N;
int  currentNode        = 8;
bool lastEntryWasForward = true;

// ── [내부] 헬퍼 ──────────────────────────────────────────────

static int zoneToNode(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5)              return 10;
  if (zone == 6)              return 11;
  return 8;
}

static int nodeIndex(int n) {
  if (n == 7)  return 0;
  if (n == 8)  return 1;
  if (n == 9)  return 2;
  if (n == 10) return 3;
  if (n == 11) return 4;
  return 1;
}

// 목표 헤딩으로 최소 회전(90°) 후 갱신
//   diff 1: 우회전 / diff 3: 좌회전 — 회전 방향 라인에 정렬 정지
void turnToHeading(int target) {
  int diff = (target - robotHeading + 4) % 4;
  if (diff == 0) return;
  if (diff == 1) {
    if (!turnToLine(true, TURN_LINE_MAX_DEG))
      Serial.println(F(">> [WARN] turnToHeading 우회전: 라인 미발견(한계각 정지)"));
  } else {  // diff == 3
    if (!turnToLine(false, TURN_LINE_MAX_DEG))
      Serial.println(F(">> [WARN] turnToHeading 좌회전: 라인 미발견(한계각 정지)"));
  }
  robotHeading = target;
}

// 블라인드 구간 엔코더 차동 보정 직진 → 라인 감지 즉시 정지
// 9-10, 10-11 사이의 선 없는 구간에서 사용
static void blindDriveUntilLine() {
  prizm.resetEncoders();
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) { stopAll(); break; }
    long diff = abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
    int corr = (abs(diff) <= 3) ? 0 : constrain((int)(diff / 7), -6, 6);
    drive(BLIND_SPEED - corr, BLIND_SPEED + corr);
    delay(5);
  }
}

// ── [내부] 노드 간 단일 구간 이동 ────────────────────────────
//
// 엔코더 차동 보정:  diff/7 (±6), dead-zone 3 → 미세 진동 억제
//   diff > 0: 왼쪽이 더 돔 → 오른쪽 가속
//   diff < 0: 오른쪽이 더 돔 → 왼쪽 가속
static void stepNode(int from, int to) {
  int dir = (to > from) ? HDG_E : HDG_W;
  turnToHeading(dir);

  if (from == 8 && to == 9) {
    // 라인 따라 직진하다 끊김 = 9번 도착
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (!anyLine(L, C, R)) { stopAll(); break; }
      int RL, RC, RR;
      readRearSensors(RL, RC, RR);
      lineFollowStepFull(L, C, R, RL, RC, RR);
      delay(5);
    }

  } else if (from == 9 && to == 8) {
    // 서향 직진 → 라인 감지 → followToCrossing으로 8번 정렬
    prizm.resetEncoders();
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      long diff = abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
      int corr = (abs(diff) <= 3) ? 0 : constrain((int)(diff / 7), -6, 6);
      drive(STRAIGHT_SPEED - corr, STRAIGHT_SPEED + corr);
      delay(5);
    }
    followToCrossing();

  } else if (from == 9 && to == 10) {
    // alignHeadingOnLine → 5° 좌보정 → 블라인드 → 10번 라인 CROSS_ALIGN 과전진
    // 5° 좌보정: 드리프트가 남쪽으로 치우치므로 약간 북쪽으로 조준
    alignHeadingOnLine();
    turnAngle(5, false);
    prizm.resetEncoders();
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) {
        prizm.resetEncoders();
        while (abs(prizm.readEncoderCount(1)) < CROSS_ALIGN_COUNTS) {
          drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
          delay(5);
        }
        stopAll();
        robotHeading = HDG_E;
        break;
      }
      long diff = abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
      int corr = (abs(diff) <= 3) ? 0 : constrain((int)(diff / 7), -6, 6);
      drive(BLIND_SPEED - corr, BLIND_SPEED + corr);
      delay(5);
    }

  } else if (from == 10 && to == 9) {
    // 10번 N-S선 감지 즉시 정지 → CCW 95°(서→남 ~185°)로 8-9 가로선 조준
    // followToCrossing: 우측 센서가 8-9 라인 자동 감지 후 서향 정렬 → 8번 교차로 도달
    blindDriveUntilLine();
    turnAngle(95, false);
    followToCrossing();
    robotHeading = HDG_W;
    currentNode = 8;
    Serial.println(F(">> [NAV] 10->9 via node8, continuing 8->9"));
    stepNode(8, 9);
    return;  // currentNode = to 덮어쓰기 방지

  } else if (from == 10 && to == 11) {
    blindDriveUntilLine();
    robotHeading = HDG_E;

  } else if (from == 11 && to == 10) {
    blindDriveUntilLine();
    robotHeading = HDG_W;

  } else {
    // 7↔8: 일반 라인트레이싱
    followToCrossing();
  }

  currentNode = to;
  Serial.print(F(">> [NAV] "));
  Serial.print(from);
  Serial.print(F("->"));
  Serial.println(to);
}

// ── [공개] 라우팅 핵심 3대 함수 ───────────────────────────────

// [1] 현재 노드 → 목표 노드 이동 (노드 순서: 7-8-9-10-11)
void moveToNode(int toNode) {
  static const int nodes[] = {7, 8, 9, 10, 11};
  int cur = nodeIndex(currentNode);
  int tgt = nodeIndex(toNode);
  if (cur == tgt) return;
  int step = (cur < tgt) ? 1 : -1;
  while (cur != tgt) {
    int next = cur + step;
    stepNode(nodes[cur], nodes[next]);
    cur = next;
  }
}

// [2] 구역 내부 → 교차로 노드 탈출
//   전진 진입 → 후방 센서 조향으로 후진 탈출
//   후진 진입 → 전진 탈출 (라인트레이싱)
void exitZone(int zone) {
  if (lastEntryWasForward) {
    // 후진 탈출: 후방 센서(leading) 5패턴 + 전방(trailing) 각도 보정
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < ZONE_EXIT_REV_COUNTS) {
      int RL, RC, RR;
      readRearSensors(RL, RC, RR);
      int L, C, R;
      readSensors(L, C, R);

      bool rearHasLine     = anyRearLine(RL, RC, RR);
      bool rearIsCrossing  = (RL && RC && RR);
      bool frontHasLine    = anyLine(L, C, R);
      bool frontIsCrossing = (L && C && R);

      int lsp = -BACK_SPEED, rsp = -BACK_SPEED;

      // 후방 센서 5패턴 조향
      if (rearHasLine && !rearIsCrossing) {
        if      (RL && !RC && !RR) { lsp = -(BACK_SPEED + 10); rsp = -(BACK_SPEED - 10); }
        else if (!RL && !RC && RR) { lsp = -(BACK_SPEED - 10); rsp = -(BACK_SPEED + 10); }
        else if (RL &&  RC && !RR) { lsp = -(BACK_SPEED +  5); rsp = -(BACK_SPEED -  5); }
        else if (!RL && RC &&  RR) { lsp = -(BACK_SPEED -  5); rsp = -(BACK_SPEED +  5); }
      } else if (!rearHasLine) {
        // 라인 미감지 → 엔코더 차동으로 직진 유지
        long diff = abs(prizm.readEncoderCount(1)) - abs(prizm.readEncoderCount(2));
        int corr = (abs(diff) <= 3) ? 0 : constrain((int)(diff / 7), -6, 6);
        lsp = -BACK_SPEED + corr;
        rsp = -BACK_SPEED - corr;
      }

      // 전후방 동시 감지 시 각도 정렬 보정
      if (frontHasLine && rearHasLine && !frontIsCrossing && !rearIsCrossing) {
        int angCorr = constrain((L - R) * ANGULAR_GAIN, -5, 5);
        lsp += angCorr;
        rsp -= angCorr;
      }

      drive(lsp, rsp);
      delay(5);
    }
    stopAll();
  } else {
    if (zoneToNode(zone) == 7) {
      // 노드7: T자 교차로라 followToCrossing 미감지 → 엔코더 기반 전진 탈출
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < NODE7_EXIT_COUNTS) {
        drive(SPEED, SPEED);
        delay(5);
      }
      stopAll();
    } else {
      followToCrossing();
    }
  }
  currentNode = zoneToNode(zone);
  Serial.print(F(">> [NAV] exitZone "));
  Serial.print(zone);
  Serial.print(F(" -> node "));
  Serial.println(currentNode);
}

// [3] 현재 노드 → 목표 구역 이동 + 진입
void goToZoneDirect(int zone) {
  int targetNode = zoneToNode(zone);
  Serial.print(F(">> [NAV] zone "));
  Serial.print(zone);
  Serial.print(F(" (node "));
  Serial.print(currentNode);
  Serial.print(F("->"));
  Serial.print(targetNode);
  Serial.println(F(")"));

  moveToNode(targetNode);

  // 1,2,5,6은 북쪽(HDG_N) 전진 진입 / 3,4는 남쪽(HDG_S)에서 후진 진입
  int zoneSide = (zone == 3 || zone == 4) ? HDG_S : HDG_N;

  if (robotHeading == HDG_E || robotHeading == HDG_W) {
    turnToHeading(zoneSide);
  }

  bool enterForward = (robotHeading == zoneSide);
  if (enterForward) {
    enterZone();
    lastEntryWasForward = true;
  } else {
    reverseEnterZone();
    lastEntryWasForward = false;
  }
}

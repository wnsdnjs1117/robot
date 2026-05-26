/* ============================================================
 * MapRouter.cpp - 헤딩 완전 추적 / 진입방식 기반 탈출
 *
 * 헤딩: 로봇 앞면이 향하는 방향 (0=북, 1=동, 2=남, 3=서)
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
 * 구역-노드 관계:
 *   1,3 → 7번  /  2,4 → 8번  /  5 → 10번  /  6 → 11번
 *
 * 구역은 교차로의 북쪽(1,2,5,6) 또는 남쪽(3,4)에 위치
 *
 * 진입 원칙: 가능하면 전진 우선
 *   북쪽 구역(1,2,5,6): 교차로에서 북향 전진 진입
 *   남쪽 구역(3,4):     교차로에서 북향 상태로 후진 진입
 *   → 어떤 구역이든 진입 후 헤딩 = 북향(0) (앞면이 구역 안쪽을 향함)
 *
 * 탈출 원칙: 진입과 반대
 *   전진 진입 → 후진 탈출 (교차로가 뒤에 있으므로)
 *   후진 진입 → 전진 탈출 (교차로가 앞에 있으므로)
 *   → 어떤 구역이든 탈출 후 헤딩 = 북향(0)
 *
 * lastEntryWasForward: 마지막 진입이 전진이었는지 기록
 *   → exitZone()이 올바른 탈출 방식을 선택하는 데 사용
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "Motion.h"

// Navigation.cpp와 순환 include 방지 — 필요한 함수만 전방 선언
void followToCrossing();
void reverseAcrossToOppositeZone();
void enterZone();

int robotHeading = 0;  // 로봇 앞면 방향 (goToMainLine 후 북향으로 초기화)
int currentNode = 8;   // 현재 서 있는 교차로 노드
bool lastEntryWasForward = true;  // 마지막 구역 진입이 전진이었는가

// ============================================================
// 내부 헬퍼
// ============================================================

// 구역 번호 → 해당 교차로 노드
static int zoneToNode(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5) return 10;
  if (zone == 6) return 11;
  return 8;
}

// 노드 → 배열 인덱스 (7=0, 8=1, 9=2, 10=3, 11=4)
static int nodeIndex(int n) {
  if (n == 7) return 0;
  if (n == 8) return 1;
  if (n == 9) return 2;
  if (n == 10) return 3;
  if (n == 11) return 4;
  return 1;
}

// 목표 헤딩으로 최소 회전 후 갱신
void turnToHeading(int target) {
  int diff = (target - robotHeading + 4) % 4;
  if (diff == 0) return;
  if (diff == 1) {
    turnAngle(90, true);
  }
  if (diff == 3) {
    turnAngle(90, false);
  }
  if (diff == 2) {
    turnAngle(90, true);
    turnAngle(90, true);
  }
  robotHeading = target;
}

// ============================================================
// 기본 이동 프리미티브
// ============================================================

// 라인 없는 구간 전진 점프 (9↔10, 10↔11)
// 양쪽 엔코더 절댓값 차이 보정으로 직진 유지
// (좌우 모터 부호가 반전되어 있으므로 abs로 비교)
// diff > 0: 왼쪽이 더 많이 돔 → 오른쪽 빠르게
// diff < 0: 오른쪽이 더 많이 돔 → 왼쪽 빠르게
void executeBlindRun() {
  prizm.resetEncoders();

  // 1단계: 최소 거리(400카운트)까지 엔코더 보정 직진
  while (prizm.readEncoderCount(1) < 400) {
    long diff = prizm.readEncoderCount(1) - prizm.readEncoderCount(2);
    int correction = constrain((int)diff, -5, 5);
    drive(STRAIGHT_SPEED - correction, STRAIGHT_SPEED + correction);
    delay(5);
  }

  // 2단계: 라인 찾을 때까지 계속 보정 직진
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    long diff = prizm.readEncoderCount(1) - prizm.readEncoderCount(2);
    int correction = constrain((int)diff, -5, 5);
    drive(STRAIGHT_SPEED - correction, STRAIGHT_SPEED + correction);
    delay(5);
  }

  // 3단계: 라인 발견 후 followToCrossing으로 교차로 위에 정확히 정렬
  // (비스듬히 걸친 상태로 멈추지 않고 중심 정렬 후 정지)
  followToCrossing();
}

// 노드 간 단일 구간 이동
// 7↔8: 라인트레이싱 followToCrossing
// 8→9: 라인 따라 직진 → 라인 끊김 = 9번 도착 (정지)
// 9→8: 직진하다 라인 감지 → followToCrossing으로 8번 정렬
// 9→10: 엔코더 보정 직진 → 10번 라인 감지 즉시 좌회전(북향)
// 10→9: 직진하다 라인 끊김 = 9번 도착 (정지)
// 10↔11: 엔코더 보정 직진 → 라인 감지 즉시 회전(북향)
static void stepNode(int from, int to) {
  int dir = (to > from) ? 1 : 3;
  turnToHeading(dir);

  if (from == 8 && to == 9) {
    // 라인 따라 직진하다 라인 끊김 = 9번 도착
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (!anyLine(L, C, R)) {
        stopAll();
        break;
      }
      lineFollowStep(L, C, R);
      delay(5);
    }

  } else if (from == 9 && to == 8) {
    // 서향 직진 → 라인 감지 → followToCrossing으로 8번 정렬
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      long diff = prizm.readEncoderCount(1) - prizm.readEncoderCount(2);
      int correction = constrain((int)diff, -5, 5);
      drive(STRAIGHT_SPEED - correction, STRAIGHT_SPEED + correction);
      delay(5);
    }
    followToCrossing();

  } else if (from == 9 && to == 10) {
    // 동향 엔코더 직진 → 10번 수직 라인 감지 → CROSS_ALIGN_COUNTS 과전진 → 동향 유지
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
        robotHeading = 1;  // 동향 유지 (goToZoneDirect가 필요 시 북향 전환)
        break;
      }
      long diff = prizm.readEncoderCount(1) - prizm.readEncoderCount(2);
      int correction = constrain((int)diff, -5, 5);
      drive(STRAIGHT_SPEED - correction, STRAIGHT_SPEED + correction);
      delay(5);
    }

  } else if (from == 10 && to == 9) {
    // 서향 엔코더 보정 직진 → 9번 라인 감지 = 9번 도착 (9↔10 사이 라인 없음)
    prizm.resetEncoders();
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (anyLine(L, C, R)) {  // 9번 라인(동쪽 끝) 감지 시 정지
        stopAll();
        break;
      }
      long diff = prizm.readEncoderCount(1) - prizm.readEncoderCount(2);
      int correction = constrain((int)diff, -5, 5);
      drive(STRAIGHT_SPEED - correction, STRAIGHT_SPEED + correction);
      delay(5);
    }

  } else if (from == 10 && to == 11) {
    // 동향 엔코더 직진 → 11번 수직 라인 감지 → CROSS_ALIGN_COUNTS 과전진 → 동향 유지
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
        robotHeading = 1;  // 동향 유지
        break;
      }
      long diff = prizm.readEncoderCount(1) - prizm.readEncoderCount(2);
      int correction = constrain((int)diff, -5, 5);
      drive(STRAIGHT_SPEED - correction, STRAIGHT_SPEED + correction);
      delay(5);
    }

  } else if (from == 11 && to == 10) {
    // 서향 엔코더 직진 → 10번 수직 라인 감지 → CROSS_ALIGN_COUNTS 과전진 → 서향 유지
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
        robotHeading = 3;  // 서향 유지
        break;
      }
      long diff = prizm.readEncoderCount(1) - prizm.readEncoderCount(2);
      int correction = constrain((int)diff, -5, 5);
      drive(STRAIGHT_SPEED - correction, STRAIGHT_SPEED + correction);
      delay(5);
    }

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

// ============================================================
// 기존 함수 유지 (호환성)
// ============================================================

void moveAbsoluteDirection(int targetDir) {
  int diff = (targetDir - robotHeading + 4) % 4;
  if (diff == 0) {
    followToCrossing();
  } else if (diff == 2) {
    reverseAcrossToOppositeZone();
  } else if (diff == 1) {
    turnAngle(90, true);
    robotHeading = targetDir;
    followToCrossing();
  } else if (diff == 3) {
    turnAngle(90, false);
    robotHeading = targetDir;
    followToCrossing();
  }
}

void goToNodeFromHub8(int node) {
  if (node == 1) {
    moveAbsoluteDirection(3);
    moveAbsoluteDirection(0);
  } else if (node == 2) {
    moveAbsoluteDirection(0);
  } else if (node == 3) {
    moveAbsoluteDirection(3);
    moveAbsoluteDirection(2);
  } else if (node == 4) {
    moveAbsoluteDirection(2);
  } else if (node == 5) {
    moveAbsoluteDirection(1);
    executeBlindRun();
    moveAbsoluteDirection(0);
  } else if (node == 6) {
    moveAbsoluteDirection(1);
    executeBlindRun();
    executeBlindRun();
    moveAbsoluteDirection(0);
  }
}

void returnToHub8FromNode(int node, bool cameOutForward) {
  if (node == 1 || node == 2) {
    // 전진 진입 → 후진 탈출 (엔코더 기반)
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < ZONE_EXIT_REV_COUNTS) {
      drive(-BACK_SPEED, -BACK_SPEED);
      delay(5);
    }
    stopAll();
    delay(100);
  } else if (node == 3 || node == 4) {
    followToCrossing();
  } else if (node == 5 || node == 6) {
    if (cameOutForward)
      reverseAcrossToOppositeZone();
    else
      followToCrossing();
  }
  if (node == 1 || node == 3) {
    moveAbsoluteDirection(1);
    moveAbsoluteDirection(3);
  } else if (node == 2 || node == 4) {
    moveAbsoluteDirection(3);
  } else if (node == 5) {
    moveAbsoluteDirection(3);
    executeBlindRun();
    followToCrossing();
  } else if (node == 6) {
    moveAbsoluteDirection(3);
    executeBlindRun();
    executeBlindRun();
    followToCrossing();
  }
}

// ============================================================
// 직접 라우팅 - 핵심 함수 3개
// ============================================================

// [1] 현재 노드 → 목표 노드 최단 전진 이동
// 노드 순서: 7 - 8 - 9 - 10 - 11
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
//     전진 진입 → 후진 탈출 / 후진 진입 → 전진 탈출
//     robotHeading은 변경하지 않음 — 진입 시 헤딩이 탈출 후에도 유지됨
void exitZone(int zone) {
  if (lastEntryWasForward) {
    // 전진 진입 → 후진 탈출 (엔코더 기반, 센서 미사용)
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < ZONE_EXIT_REV_COUNTS) {
      drive(-BACK_SPEED, -BACK_SPEED);
      delay(5);
    }
    stopAll();
    delay(100);
  } else {
    if (zoneToNode(zone) == 7) {
      // 노드 7은 T자 교차로라 followToCrossing 미감지 → 엔코더 기반 전진 탈출
      prizm.resetEncoders();
      while (abs(prizm.readEncoderCount(1)) < ZONE_ENTER_COUNTS) {
        drive(SPEED, SPEED);
        delay(5);
      }
      stopAll();
      delay(100);
    } else {
      // 후진 진입 → 전진 탈출 (라인트레이싱)
      followToCrossing();
    }
  }
  currentNode = zoneToNode(zone);
  Serial.print(F(">> [NAV] exitZone "));
  Serial.print(zone);
  Serial.print(F(" -> node "));
  Serial.print(currentNode);
  Serial.print(F(" heading="));
  Serial.println(robotHeading);
}

// [3] 현재 노드 → 목표 구역 이동 + 진입
//     진입 후 robotHeading = 진입 방향(zoneSide 또는 반대)
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

  // 구역 방향: 1,2,5,6=북(0), 3,4=남(2)
  int zoneSide = (zone == 3 || zone == 4) ? 2 : 0;

  // 동/서향이면 구역 방향으로 정렬 (회전 1번으로 전진 진입 가능)
  if (robotHeading == 1 || robotHeading == 3) {
    turnToHeading(zoneSide);
  }

  // 현재 헤딩이 구역 방향이면 전진, 반대면 후진
  bool enterForward = (robotHeading == zoneSide);

  if (enterForward) {
    enterZone();
    lastEntryWasForward = true;
    // robotHeading 유지 — 전진이므로 앞면 방향 그대로
    Serial.print(F(">> [NAV] 전진 진입 heading="));
    Serial.println(robotHeading);
  } else {
    // 후진 진입 (엔코더 기반)
    prizm.resetEncoders();
    while (abs(prizm.readEncoderCount(1)) < ZONE_ENTER_COUNTS) {
      drive(-BACK_SPEED, -BACK_SPEED);
      delay(5);
    }
    stopAll();
    lastEntryWasForward = false;
    // robotHeading 유지 — 후진이므로 앞면 방향 그대로
    Serial.print(F(">> [NAV] 후진 진입 heading="));
    Serial.println(robotHeading);
  }
}
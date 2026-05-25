/* ============================================================
 * MapRouter.cpp - 존 내부 기동 대응형 다이나믹 라우터
 * ============================================================ */
#include "MapRouter.h"

#include "Config.h"
#include "Motion.h"
#include "Navigation.h"

int robotHeading = 3;

// 단절 구간 직진 점프
void executeBlindRun() {
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < 400) {
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

// 다이나믹 절대 방위 기동 함수 (0:북, 1:동, 2:남, 3:서)
void moveAbsoluteDirection(int targetDir) {
  int diff = (targetDir - robotHeading + 4) % 4;

  if (diff == 0) {
    followToCrossing();
  } else if (diff == 2) {
    reverseAcrossToOppositeZone();
  } else if (diff == 1) {
    turnAngle(90, true);
    followToCrossing();
    robotHeading = targetDir;
  } else if (diff == 3) {
    turnAngle(90, false);
    followToCrossing();
    robotHeading = targetDir;
  }
}

void goToNodeFromHub8(int node) {
  if (node == 1) {
    moveAbsoluteDirection(3);  // 8➔7 서쪽 이동
    moveAbsoluteDirection(0);  // 7➔1 북쪽 진입
  } else if (node == 2) {
    moveAbsoluteDirection(0);  // 8➔2 북쪽 즉시 진입
  } else if (node == 3) {
    moveAbsoluteDirection(3);  // 8➔7 서쪽 이동
    moveAbsoluteDirection(2);  // 7➔3 남쪽 후진 진입
  } else if (node == 4) {
    moveAbsoluteDirection(2);  // 8➔4 남쪽 후진 진입
  } else if (node == 5) {
    moveAbsoluteDirection(1);  // 8➔9 동쪽 이동
    executeBlindRun();         // 9➔10 동쪽 1단 점프
    moveAbsoluteDirection(0);  // 10➔5 북쪽 진입
  } else if (node == 6) {
    moveAbsoluteDirection(1);  // 8➔9 동쪽 이동
    executeBlindRun();         // 9➔10 동쪽 1단 점프
    executeBlindRun();         // 10➔11 동쪽 2단 점프
    moveAbsoluteDirection(0);  // 11➔6 북쪽 진입
  }
}

// ★ [리팩토링 핵심] 존 내부에서 멈춘 상태의 첫 탈출까지 완벽하게 지원하는 만능
// 복귀 함수
void returnToHub8FromNode(int node, bool cameOutForward) {
  // 로봇의 현재 위치가 구역 내부(1~4)인 경우, 바라보는 방위(북쪽=0)를 기준으로
  // 탈출 기동 분기
  if (node == 1 || node == 2) {
    // 1, 2구역은 정면으로 진입했으므로 메인 트랙으로 나오기 위해 '바닥
    // 교차로선'까지 안전 후진 처리
    crossingArmed = true;
    crossingStable = 0;
    while (true) {
      int L, C, R;
      readSensors(L, C, R);
      if (detectCrossing(L, C, R)) break;
      lineFollowStepReverse(L, C, R);
      delay(5);
    }
    stopAll();
    delay(100);
  } else if (node == 3 || node == 4) {
    // 3, 4구역은 애초에 후진 관통으로 진입했으므로 헤드가 북쪽을 본 상태에서
    // '전진'으로 탈출
    followToCrossing();
  } else if (node == 5 || node == 6) {
    // 5, 6구역 고정형 탈출 프로토콜 유지
    if (cameOutForward)
      reverseAcrossToOppositeZone();
    else
      followToCrossing();
  }

  // 교차로 탈출 완수 후, 8번 허브로의 복귀 및 최종 서쪽 정렬 제어
  if (node == 1 || node == 3) {  // 현재 7번 교차로(A2) 위치
    moveAbsoluteDirection(1);    // 동쪽(오른쪽) 이동하여 8번 허브 진입
    moveAbsoluteDirection(3);    // 진입 즉시 다음 연산을 위해 제자리 서쪽 정렬
  } else if (node == 2 || node == 4) {  // 현재 8번 허브(B2) 교차로 위치 자체
    moveAbsoluteDirection(3);           // 제자리에서 서쪽 정렬 리셋
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
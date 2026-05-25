/* ============================================================
 * MissionFlow.cpp - 실시간 존 링크 체인 및 초고속 배송 스케줄러
 * ============================================================ */
#include "MissionFlow.h"

#include "BoxMap.h"
#include "MapRouter.h"
#include "Motion.h"
#include "Navigation.h"

static int currentZone = 0;

// ============================================================
// [1단계] 스타트 박스 탈출 및 1~6구역 전체 QR 탐색
// ============================================================
void executeStage1_Search() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 1] 1~6구역 전체 QR 탐색 기동"));

  goToMainLine();
  // 1~4구역 중 랜덤 박스 2개를 찾는 순간 탐색 정지 후 해당 구역 반환
  currentZone = qrSearchStage();
  robotHeading = 0;  // 북쪽 정렬

  Serial.print(F("\n>> [STAGE 1-A] 1~4구역 탐색 완료. 현 위치: "));
  Serial.print(currentZone);
  Serial.println(F("구역. 5,6구역 추가 스캔을 위해 이동합니다."));

  // --- 5번, 6번 고정 박스 필수 스캔 (초고속 횡단 다이나믹 연동) ---

  // 1. 현재 멈춘 구역에서 일단 8번 허브로 선제 탈출
  returnToHub8FromNode(currentZone, true);

  // 2. 8번(서쪽) ➔ 5번(입고) 다이렉트 주행 및 스캔
  goToNodeFromHub8(5);
  enterZone();
  delay(500);
  scanZone(5);

  // 3. 5번 ➔ 8번 허브로 안 돌아가고, 6번으로 바로 옆으로 1단 점프 횡단!
  reverseAcrossToOppositeZone();  // 5번 후진 탈출 (현재 10번 노드, 북쪽 헤딩)
  turnAngle(90, true);            // 우회전 (동쪽 헤딩)
  executeBlindRun();              // 10 ➔ 11 동쪽 점프!
  turnAngle(90, false);           // 좌회전 (북쪽 헤딩)
  enterZone();
  delay(500);
  scanZone(6);

  // 4. 6번 스캔 완료 후 8번 허브로 공식 복귀
  returnToHub8FromNode(6, true);
  currentZone = 0;  // 1단계 최종 종료, 로봇은 8번 허브(서쪽)에 완벽 대기

  Serial.println(F(">> [STAGE 1-B] 모든 구역(1~6) 스캔 완수! 8번 허브 대기."));
  Serial.println(F("========================================\n"));
}

// ============================================================
// [2단계] 1구역 1박스 제약 준수 및 제자리 스킵 배송
// ============================================================
void executeStage2_Delivery() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 2] 물리 제약(1Zone 1Box) 준수 배송 가동"));

  bool isOccupied[7];
  for (int i = 1; i <= 6; i++) {
    isOccupied[i] = boxes[i].present;  // 현재 박스가 있는 물리적 상태
  }

  int deliveredCount = 0;
  bool delivered[7] = {false};
  bool isFirstAction = true;

  while (deliveredCount < 4) {
    bool movedThisTurn = false;

    for (int zone = 1; zone <= 6; zone++) {
      if (boxes[zone].found && boxes[zone].present && !delivered[zone]) {
        int dest = boxes[zone].destination;

        // ★ [조건 1] 목적지가 현재 위치와 같으면 "제자리 유지 (Skip)"
        if (dest == zone) {
          Serial.print(F("\n[STAY] 출발지("));
          Serial.print(zone);
          Serial.println(
              F(") 박스는 이미 정답 위치에 있습니다. 이동을 생략합니다!"));
          delivered[zone] = true;
          deliveredCount++;
          movedThisTurn = true;
          break;
        }

        // ★ [조건 2] 1구역 1박스 절대 법칙 (목적지가 완전히 비어있을 때만 진입)
        else if (!isOccupied[dest]) {
          Serial.print(F("\n[ROUTE] 출발지("));
          Serial.print(zone);
          Serial.print(F(") ➔ 빈 목적지("));
          Serial.print(dest);
          Serial.println(F(") 안전 배송"));

          // (1단계에서 currentZone이 0(허브)으로 세팅되었으므로 일반 주행)
          if (isFirstAction && currentZone != 0) {
            returnToHub8FromNode(currentZone, true);
            currentZone = 0;
          }
          isFirstAction = false;

          // 출발지 픽업
          goToNodeFromHub8(zone);
          enterZone();
          delay(500);
          returnToHub8FromNode(zone, true);

          // 목적지 하차
          goToNodeFromHub8(dest);
          enterZone();
          delay(500);
          returnToHub8FromNode(dest, true);

          // 메모리 상의 공간 점유 상태 갱신 (1박스 제약 완벽 보장)
          isOccupied[zone] = false;
          isOccupied[dest] = true;
          delivered[zone] = true;
          deliveredCount++;
          movedThisTurn = true;
          break;
        }
      }
    }

    // 데드락 교착 상태 (모든 박스의 목적지에 다른 박스가 있는 꽉 막힌 상태)
    // 해결
    if (!movedThisTurn && deliveredCount < 4) {
      int stuckZone = 0, emptyZone = 0;
      for (int z = 1; z <= 6; z++) {
        if (boxes[z].found && boxes[z].present && !delivered[z]) stuckZone = z;
        if (!isOccupied[z]) emptyZone = z;  // 비어있는 임의의 구역 탐색
      }

      if (stuckZone > 0 && emptyZone > 0) {
        if (isFirstAction && currentZone != 0) {
          returnToHub8FromNode(currentZone, true);
          currentZone = 0;
        }
        isFirstAction = false;

        Serial.print(F("\n[DEADLOCK-EVADE] 교통 체증! 구역("));
        Serial.print(stuckZone);
        Serial.print(F(") ➔ 빈 구역("));
        Serial.print(emptyZone);
        Serial.println(F(") 임시 회피 대피"));

        // 대피 기동
        goToNodeFromHub8(stuckZone);
        enterZone();
        delay(500);
        returnToHub8FromNode(stuckZone, true);

        goToNodeFromHub8(emptyZone);
        enterZone();
        delay(500);
        returnToHub8FromNode(emptyZone, true);

        // 물리적 공간 점유 상태 업데이트 (여전히 1구역 1박스 유지됨)
        boxes[emptyZone].present = true;
        boxes[emptyZone].found = true;
        boxes[emptyZone].destination = boxes[stuckZone].destination;
        boxes[stuckZone].present = false;
        boxes[stuckZone].found = false;
        boxes[stuckZone].destination = 0;

        isOccupied[stuckZone] = false;
        isOccupied[emptyZone] = true;
      }
    }
  }
  Serial.println(
      F("\n>> [STAGE 2] 제자리 스킵 & 1구역 1박스 제약 기반 미션 클리어!"));
  Serial.println(F("========================================\n"));
}
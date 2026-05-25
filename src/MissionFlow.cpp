/* ============================================================
 * MissionFlow.cpp - 실시간 존 링크 체인 및 초고속 배송 스케줄러
 * ============================================================ */
#include "MissionFlow.h"

#include "BoxMap.h"
#include "MapRouter.h"
#include "Motion.h"
#include "Navigation.h"

// 1단계가 종료된 경기장 실시간 구역 위치 기억 변수 (0이면 허브, 1~4면 해당 존
// 내부)
static int currentZone = 0;

// ============================================================
// [1단계] 스타트 박스 탈출 및 2개 조기 발견 즉시 셧다운 탐색
// ============================================================
void executeStage1_Search() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 1] QR 탐색 엔진 기동"));

  goToMainLine();
  currentZone = qrSearchStage();  // ★ 2개 찾는 순간 해당 존(1~4) 내부에서
                                  // 코드가 즉시 끊기며 반환됨!

  // 구역 스캔 중 로봇의 물리적인 정면 헤딩은 예외 없이 언제나 북쪽(0)입니다.
  robotHeading = 0;

  Serial.print(F(">> [STAGE 1] 즉시 종료 완수! 로봇 정지 위치: "));
  Serial.print(currentZone);
  Serial.println(F("구역 내부 (방위: 북쪽)"));
  Serial.println(F("========================================\n"));
}

// ============================================================
// [2단계] 동적 위치 바인딩 기반 초고속 충돌 회피 배송
// ============================================================
void executeStage2_Delivery() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 2] 다이나믹 연속 링크 배송 가동"));

  bool isOccupied[7];
  for (int i = 1; i <= 6; i++) {
    isOccupied[i] = boxes[i].present;
  }

  int deliveredCount = 0;
  bool delivered[7] = {false};
  bool isFirstAction = true;  // 2단계 최초 실행 플래그

  while (deliveredCount < 4) {
    bool movedThisTurn = false;

    for (int zone = 1; zone <= 6; zone++) {
      if (boxes[zone].found && boxes[zone].present && !delivered[zone]) {
        int dest = boxes[zone].destination;

        if (!isOccupied[dest]) {
          Serial.print(F("\n[ROUTE] 출발지("));
          Serial.print(zone);
          Serial.print(F(") ➔ 목적지("));
          Serial.print(dest);
          Serial.println(F(") 최적 매핑"));

          // ★ [슈퍼 최적화: 제자리 픽업 변환 기믹]
          // 2단계 최초 실행 시, 첫 번째 배송해야 할 박스가 마침 내가 탐색을
          // 마친 구역(currentZone)과 완벽히 일치한다면?
          if (isFirstAction && currentZone == zone) {
            Serial.println(F(
                ">> [SPEED-UP] 동선 이동 생략! 현재 구역에서 즉시 상차 구동!"));
            enterZone();  // 제자리 진입 안정화 (또는 리프트 상차 메커니즘
                          // 다이렉트 연동 가능)
            delay(500);  // 상차 구동 타임

            // 상차를 완료했으므로 이제 현재 위치한 존을 빠져나가 8번 허브로
            // 복귀 정렬
            returnToHub8FromNode(zone, true);
            currentZone = 0;  // 이제 허브로 나왔으므로 존 위치값 리셋
          } else {
            // 마침 일치하지 않거나 일반 주행 루프 상황인 경우
            // 만약 로봇이 여전히 구역 내부에 갇혀 있는 초기 상태라면 선제 탈출
            // 기동을 먼저 안전하게 수행
            if (isFirstAction && currentZone != 0) {
              Serial.print(F(">> [ESCAPE] 탐색 종료지인 "));
              Serial.print(currentZone);
              Serial.println(F("구역을 선 탈출하여 허브로 집결합니다."));
              returnToHub8FromNode(currentZone, true);
              currentZone = 0;
            }

            // 표준 허브 기반 라우팅 수행
            goToNodeFromHub8(zone);
            enterZone();
            delay(500);  // 상차
            returnToHub8FromNode(zone, true);
          }

          isFirstAction = false;  // 최초 기동 오프셋 해제

          // 목적지로 자율 이송 배송
          goToNodeFromHub8(dest);
          enterZone();
          delay(500);  // 하차
          returnToHub8FromNode(dest, true);

          isOccupied[zone] = false;
          isOccupied[dest] = true;
          delivered[zone] = true;
          deliveredCount++;
          movedThisTurn = true;
          break;
        }
      }
    }

    // 데드락 교착 상태 해결 로직
    if (!movedThisTurn && deliveredCount < 4) {
      int stuckZone = 0, emptyZone = 0;
      for (int z = 1; z <= 6; z++) {
        if (boxes[z].found && boxes[z].present && !delivered[z]) stuckZone = z;
        if (!isOccupied[z]) emptyZone = z;
      }

      if (stuckZone > 0 && emptyZone > 0) {
        // 만약 데드락 회피를 쳐야 하는데 로봇이 아직 초기 존 내부에 갇혀 있는
        // 극단적 예외 상황 방어
        if (isFirstAction && currentZone != 0) {
          returnToHub8FromNode(currentZone, true);
          currentZone = 0;
        }
        isFirstAction = false;

        Serial.print(F("\n[DEADLOCK-EVADE] 구역("));
        Serial.print(stuckZone);
        Serial.print(F(") ➔ 빈 구역("));
        Serial.print(emptyZone);
        Serial.println(F(") 임시 회피"));

        goToNodeFromHub8(stuckZone);
        enterZone();
        delay(500);
        returnToHub8FromNode(stuckZone, true);

        goToNodeFromHub8(emptyZone);
        enterZone();
        delay(500);
        returnToHub8FromNode(emptyZone, true);

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
  Serial.println(F("\n>> [STAGE 2] 조기 동적 바인딩 시스템 미션 클리어!"));
  Serial.println(F("========================================\n"));
}
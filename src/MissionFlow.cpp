#include "MissionFlow.h"

#include "BoxMap.h"
#include "MapRouter.h"
#include "Motion.h"
#include "Navigation.h"

void executeStage1_Search() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 1] QR 탐색 시작"));

  goToMainLine();
  qrSearchStage();

  // 탐색 후 8번 허브 서쪽 보기 정렬
  turnAngle(90, true);
  followToCrossing();
  turnAngle(180, true);

  Serial.println(F(">> [STAGE 1] 탐색 완료! 8번 허브 초기화."));
  Serial.println(F("========================================\n"));
}

void executeStage2_Delivery() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 2] 지능형 겹침 방지 배송 시작"));

  bool isOccupied[7];
  for (int i = 1; i <= 6; i++) isOccupied[i] = boxes[i].present;

  int deliveredCount = 0;
  bool delivered[7] = {false};

  while (deliveredCount < 4) {
    bool movedThisTurn = false;

    for (int zone = 1; zone <= 6; zone++) {
      if (boxes[zone].found && boxes[zone].present && !delivered[zone]) {
        int dest = boxes[zone].destination;

        if (!isOccupied[dest]) {
          Serial.print(F("\n[TASK] 출발지("));
          Serial.print(zone);
          Serial.print(F(") ➔ 목적지("));
          Serial.print(dest);
          Serial.println(F(") 배송 시작"));

          goToNodeFromHub8(zone);
          enterZone();
          delay(500);
          returnToHub8FromNode(zone, true);

          goToNodeFromHub8(dest);
          enterZone();
          delay(500);
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

    if (!movedThisTurn && deliveredCount < 4) {
      int stuckZone = 0, emptyZone = 0;
      for (int z = 1; z <= 6; z++) {
        if (boxes[z].found && boxes[z].present && !delivered[z]) stuckZone = z;
        if (!isOccupied[z]) emptyZone = z;
      }

      if (stuckZone > 0 && emptyZone > 0) {
        Serial.print(F("\n[WARNING] 데드락! 구역("));
        Serial.print(stuckZone);
        Serial.print(F(")을 구역("));
        Serial.print(emptyZone);
        Serial.println(F(")으로 회피!"));

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
  Serial.println(F("\n>> [STAGE 2] 모든 박스 배송 완료!"));
  Serial.println(F("========================================\n"));
}
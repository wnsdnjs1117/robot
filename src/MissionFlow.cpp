/* ============================================================
 * MissionFlow.cpp - 허브 불경유 직접 라우팅 배송 스케줄러
 * ============================================================ */
#include "MissionFlow.h"

#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"
#include "Navigation.h"

void executeStage1_Search() {
  DPRINTLNF("\n========================================");
  DPRINTLNF(">> [STAGE 1] 1~6구역 전체 QR 탐색 기동");

  liftDownStart();
  goToMainLine();
  liftDownWait();

  currentNode = 8;
  robotHeading = 270;

  int foundZone = qrSearchStage();

  DPRINTF("\n>> [STAGE 1-A] 탐색 완료. 현 위치: ");
  DPRINT(foundZone);
  DPRINTLNF("구역. 5,6구역 스캔으로 이동합니다.");

  exitZone(foundZone);

  goToZoneDirect(5);
  scanZone(5);

  exitZone(5);
  goToZoneDirect(6);
  scanZone(6);

  exitZone(6);

  DPRINTLNF(">> [STAGE 1-B] 모든 구역(1~6) 스캔 완수!");
  DPRINTLNF("========================================\n");
}

void executeStage2_Delivery() {
  DPRINTLNF("\n========================================");
  DPRINTLNF(">> [STAGE 2] 직접 라우팅 배송 가동");

  bool isOccupied[7] = {false};
  for (int i = 1; i <= 6; i++) isOccupied[i] = boxes[i].present;

  int deliveredCount = 0;
  bool delivered[7] = {false};

  while (deliveredCount < 4) {
    bool movedThisTurn = false;

    for (int zone = 1; zone <= 6; zone++) {
      if (!boxes[zone].found || !boxes[zone].present || delivered[zone]) continue;
      int dest = boxes[zone].destination;

      if (dest == zone) {
        DPRINTF("\n[STAY] ");
        DPRINT(zone);
        DPRINTLNF("구역: 이미 정답 위치.");
        delivered[zone] = true;
        deliveredCount++;
        movedThisTurn = true;
        break;
      }

      if (!isOccupied[dest]) {
        DPRINTF("\n[ROUTE] ");
        DPRINT(zone);
        DPRINTF(" -> ");
        DPRINTLN(dest);

        goToZoneDirect(zone);
        liftUp();
        exitZone(zone);
        liftUpWait();

        goToZoneDirect(dest);
        liftDownUntilClear();
        exitZone(dest);
        liftDownWait();

        // ★ 상태 동기화 완벽 수정 (이중 관리 방지)
        boxes[zone].present = false;
        boxes[dest].present = true;
        isOccupied[zone] = false;
        isOccupied[dest] = true;
        delivered[zone] = true;

        deliveredCount++;
        movedThisTurn = true;
        break;
      }
    }

    if (!movedThisTurn && deliveredCount < 4) {
      int stuckZone = 0, emptyZone = 0;
      for (int z = 1; z <= 6; z++) {
        if (boxes[z].found && boxes[z].present && !delivered[z]) stuckZone = z;
        if (!isOccupied[z]) emptyZone = z;
      }
      if (stuckZone == 0 || emptyZone == 0) break;

      DPRINTF("\n[DEADLOCK] ");
      DPRINT(stuckZone);
      DPRINTF(" -> 빈 구역 ");
      DPRINTLN(emptyZone);

      goToZoneDirect(stuckZone);
      liftUp();
      exitZone(stuckZone);
      liftUpWait();

      goToZoneDirect(emptyZone);
      liftDownUntilClear();
      exitZone(emptyZone);
      liftDownWait();

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

  DPRINTLNF("\n>> [STAGE 2] 배송 완료!");
  DPRINTLNF("========================================\n");
}
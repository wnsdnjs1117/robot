/* ============================================================
 * MissionFlow.cpp - 탐색·배송 미션 스케줄러
 * ============================================================ */
#include "MissionFlow.h"
#include "BoxMap.h"
#include "Config.h"
#include "Lift.h"
#include "MapRouter.h"
#include "Motion.h"
#include "Navigation.h"

namespace {

void deliverBoxBetweenZones(int fromZone, int toZone, bool alreadyInFromZone = false) {
  if (!alreadyInFromZone) navigateToZone(fromZone);
  liftUpStart();
  liftUpWaitClear();
  moveBetweenZones(fromZone, toZone, zoneMoveOpts(false, true));
  liftDownUntilClear();
  leaveZone(toZone);
  liftDownWait();
}

// 박스 z가 '1~4존 내 · 목표도 1~4존 · 그 목표존이 빔' 조기 배송 대상인지
bool boxReadyForEarlyDelivery(int z) {
  if (z < 1 || z > 4) return false;
  if (!boxes[z].found || !boxes[z].present) return false;
  int dest = boxes[z].destination;
  return dest >= 1 && dest <= 4 && dest != z && !boxes[dest].present;
}

// from→to 이동 완료 후 전역 상태 갱신: 목적지 도착 = 제자리(STAY)
void markBoxMoved(int from, int to) {
  boxes[to].present = true;
  boxes[to].found = true;
  boxes[to].destination = to;
  boxes[from].present = false;
  boxes[from].found = false;
  boxes[from].destination = 0;
}

// 1~4존 박스 중 목표가 '빈 1~4존'인 것을 5/6 스캔 전에 먼저 배송.
// inZone: 검색 종료 시 로봇이 들어가 있는 존(없으면 0). 그 존의 박스가 대상이면
//         나가지 않고 '그 자리에서' 바로 배송해 불필요한 탈출·재진입 왕복을 없앤다.
void deliverReadyBoxesWithinZones1to4(int inZone) {
  // 1) 현재 들어가 있는 존의 박스 우선 — 재진입 없이 바로 픽업·배송
  if (boxReadyForEarlyDelivery(inZone)) {
    int dest = boxes[inZone].destination;
    deliverBoxBetweenZones(inZone, dest, /*alreadyInFromZone=*/true);
    markBoxMoved(inZone, dest);
    inZone = 0;
  }
  if (inZone >= 1 && inZone <= 4) leaveZone(inZone);
  for (int z = 1; z <= 4; z++) {
    if (!boxReadyForEarlyDelivery(z)) continue;
    deliverBoxBetweenZones(z, boxes[z].destination);
    markBoxMoved(z, boxes[z].destination);
  }
}

} // namespace

void runSearchPhase() {
  DPRINTLNF("\n========================================");
  DPRINTLNF(">> [STAGE 1] 1~6구역 전체 QR 탐색 기동");

  liftDownStart();
  driveOntoMainTrack();
  liftDownWait();

  intersectionNode = 8;
  headingDeg = 270;

  int currentZone = searchQrInZones1to4();

  DPRINTF("\n>> [STAGE 1-A] 탐색 완료. 현 위치: ");
  DPRINT(currentZone);
  DPRINTLNF("구역. 5,6구역 스캔으로 이동합니다.");

  if (countScannedBoxesInZones1to4() >= 2) {
    // 2개 모두 인식 완료 → currentZone 박스는 나가기 전에 그 자리에서 바로 배송(왕복 제거)
    deliverReadyBoxesWithinZones1to4(currentZone);
  } else {
    // 아직 2개를 못 찾음 → 먼저 재탐색(이때 currentZone을 떠남) 후 배송
    leaveZone(currentZone);
    rescanMissingQrZones1to4();
    deliverReadyBoxesWithinZones1to4(0);
  }

  beginZoneScan(5);
  navigateToZone(5);
  waitForZoneScan(5);
  for (int t = 0; !boxes[5].found && t < MAX_RESCAN_TRIES; t++) {
    leaveZone(5); navigateToZone(5); waitForZoneScan(5);
  }
  leaveZone(5);
  endZoneScan();

  beginZoneScan(6);
  navigateToZone(6);
  waitForZoneScan(6);
  for (int t = 0; !boxes[6].found && t < MAX_RESCAN_TRIES; t++) {
    leaveZone(6); navigateToZone(6); waitForZoneScan(6);
  }
  endZoneScan();

  DPRINTLNF(">> [STAGE 1-B] 모든 구역(1~6) 스캔 완수!");
  DPRINTLNF("========================================\n");
}

void runDeliveryPhase() {
  DPRINTLNF("\n========================================");
  DPRINTLNF(">> [STAGE 2] 직접 라우팅 배송 가동");

  bool zoneOccupied[7] = {false};
  for (int i = 1; i <= 6; i++) zoneOccupied[i] = boxes[i].present;

  int deliveredCount = 0;
  bool delivered[7] = {false};
  bool insideZone6 = true;

  while (deliveredCount < 4) {
    bool movedThisTurn = false;

    if (insideZone6) {
      if (boxes[6].found && boxes[6].present && !delivered[6]) {
        int dest = boxes[6].destination;

        if (dest == 6) {
          DPRINTLNF("\n[STAY] 6구역: 이미 정답 위치.");
          delivered[6] = true;
          deliveredCount++;
          movedThisTurn = true;
        }
        else if (!zoneOccupied[dest]) {
          DPRINTF("\n[ROUTE] 6 -> ");
          DPRINTLN(dest);

          deliverBoxBetweenZones(6, dest, true);
          insideZone6 = false;

          boxes[6].present = false;
          boxes[dest].present = true;
          zoneOccupied[6] = false;
          zoneOccupied[dest] = true;
          delivered[6] = true;
          deliveredCount++;
          movedThisTurn = true;
        }
      }

      if (!movedThisTurn) {
        DPRINTLNF("\n[EXIT] 다른 구역 작업을 위해 6구역에서 먼저 탈출합니다.");
        leaveZone(6);
        insideZone6 = false;
      }
      continue;
    }

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

      if (!zoneOccupied[dest]) {
        DPRINTF("\n[ROUTE] ");
        DPRINT(zone);
        DPRINTF(" -> ");
        DPRINTLN(dest);

        deliverBoxBetweenZones(zone, dest);

        boxes[zone].present = false;
        boxes[dest].present = true;
        zoneOccupied[zone] = false;
        zoneOccupied[dest] = true;
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
        if (!zoneOccupied[z]) emptyZone = z;
      }
      if (stuckZone == 0 || emptyZone == 0) break;

      DPRINTF("\n[DEADLOCK] ");
      DPRINT(stuckZone);
      DPRINTF(" -> 빈 구역 ");
      DPRINTLN(emptyZone);

      deliverBoxBetweenZones(stuckZone, emptyZone);

      boxes[emptyZone].present = true;
      boxes[emptyZone].found = true;
      boxes[emptyZone].destination = boxes[stuckZone].destination;
      boxes[stuckZone].present = false;
      boxes[stuckZone].found = false;
      boxes[stuckZone].destination = 0;
      zoneOccupied[stuckZone] = false;
      zoneOccupied[emptyZone] = true;
    }
  }

  DPRINTLNF("\n>> [STAGE 2] 배송 완료!");
  DPRINTLNF("========================================\n");
}

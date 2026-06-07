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

bool deliveryCrossesNode9(int fromZone, int toZone) {
  bool fromTop = (fromZone >= 1 && fromZone <= 4);
  bool toTop = (toZone >= 1 && toZone <= 4);
  return fromTop != toTop;
}

void deliverBoxBetweenZones(int fromZone, int toZone, bool alreadyInFromZone = false) {
  if (!alreadyInFromZone) navigateToZone(fromZone);
  bool cross9 = deliveryCrossesNode9(fromZone, toZone);

  // 박스를 들고 장애물이 있는 9번을 지날 때만 24cm, 그 외에는 12cm까지만 든다.
  liftUpStart(cross9 ? LIFT_CARRY_HIGH_CM : LIFT_CARRY_LOW_CM);
  liftUpWaitClear();

  moveBetweenZones(fromZone, toZone, zoneMoveOpts(false, true));

  liftDownUntilClear();
  leaveZone(toZone);
  liftDownWait();
}

bool boxReadyForEarlyDelivery(int z) {
  if (z < 1 || z > 4) return false;
  if (!boxes[z].found || !boxes[z].present) return false;
  int dest = boxes[z].destination;
  return dest >= 1 && dest <= 4 && dest != z && !boxes[dest].present;
}

void markBoxMoved(int from, int to) {
  boxes[to].present = true;
  boxes[to].found = true;
  boxes[to].destination = to;
  boxes[from].present = false;
  boxes[from].found = false;
  boxes[from].destination = 0;
}

void scanZoneWithRescan(int z, bool leaveAfter) {
  beginZoneScan(z);
  navigateToZone(z);
  waitForZoneScan(z);
  for (int t = 0; !boxes[z].found && t < MAX_RESCAN_TRIES; t++) {
    leaveZone(z);
    navigateToZone(z);
    waitForZoneScan(z);
  }
  if (leaveAfter) leaveZone(z);
  endZoneScan();
}

void deliverReadyBoxesWithinZones1to4(int inZone) {
  if (boxReadyForEarlyDelivery(inZone)) {
    int dest = boxes[inZone].destination;
    deliverBoxBetweenZones(inZone, dest, true);
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

// 지금 바로 배송 가능한(제자리이거나 목적지가 빈) 박스가 하나라도 있는가?
bool anyDeliverableNow(const bool zoneOccupied[7], const bool delivered[7]) {
  for (int z = 1; z <= 6; z++) {
    if (!boxes[z].found || !boxes[z].present || delivered[z]) continue;
    int d = boxes[z].destination;
    if (d == z) return true;
    if (d >= 1 && d <= 6 && !zoneOccupied[d]) return true;
  }
  return false;
}

// exclude 칸을 제외한 첫 번째 빈 칸(1~6). 없으면 0.
int firstEmptyZone(const bool zoneOccupied[7], int exclude) {
  for (int z = 1; z <= 6; z++)
    if (z != exclude && !zoneOccupied[z]) return z;
  return 0;
}

}  // namespace

void runSearchPhase() {
  liftDownStart();
  driveOntoMainTrack();
  liftDownWait();

  intersectionNode = 8;
  headingDeg = 270.0f;

  int currentZone = searchQrInZones1to4();

  if (countScannedBoxesInZones1to4() >= 2) {
    deliverReadyBoxesWithinZones1to4(currentZone);
  } else {
    leaveZone(currentZone);
    int rescanZone = rescanMissingQrZones1to4();
    deliverReadyBoxesWithinZones1to4(rescanZone);
  }

  scanZoneWithRescan(5, true);
  scanZoneWithRescan(6, false);
}

void runDeliveryPhase() {
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
          // 6번이 이미 정답(출고 제자리): 박스는 그대로 두고 로봇만 11번으로
          // 빠져나온다. (leaveZone 을 건너뛰면 로봇이 6번 존 안에 남은 채
          //  intersectionNode 만 11 로 기록돼, 다음 이동 때 존 안에서 회전해버림)
          leaveZone(6);
          delivered[6] = true;
          deliveredCount++;
          movedThisTurn = true;
        } else if (!zoneOccupied[dest]) {
          deliverBoxBetweenZones(6, dest, true);
          boxes[6].present = false;
          boxes[dest].present = true;
          zoneOccupied[6] = false;
          zoneOccupied[dest] = true;
          delivered[6] = true;
          deliveredCount++;
          movedThisTurn = true;
        } else if (!anyDeliverableNow(zoneOccupied, delivered)) {
          // 목적지가 막혔고 지금 바로 배송 가능한 박스도 없는 교착(예: 5<->6 스왑).
          // 이미 6번 안에 있으니 빈 손으로 나갔다 다시 들어오지 말고, box6 를
          // 빈 칸으로 바로 들어 옮겨 교착을 푼다(목적지는 유지, 이후 단계에서 배송).
          int tmp = firstEmptyZone(zoneOccupied, dest);
          if (tmp != 0) {
            deliverBoxBetweenZones(6, tmp, true);
            boxes[tmp].present = true;
            boxes[tmp].found = true;
            boxes[tmp].destination = dest;
            boxes[6].present = false;
            boxes[6].found = false;
            boxes[6].destination = 0;
            zoneOccupied[6] = false;
            zoneOccupied[tmp] = true;
            movedThisTurn = true;
          }
        }
      }

      if (!movedThisTurn) {
        leaveZone(6);
      }
      insideZone6 = false;
      continue;
    }

    for (int zone = 1; zone <= 6; zone++) {
      if (!boxes[zone].found || !boxes[zone].present || delivered[zone]) continue;
      int dest = boxes[zone].destination;

      if (dest == zone) {
        delivered[zone] = true;
        deliveredCount++;
        movedThisTurn = true;
        break;
      }

      if (!zoneOccupied[dest]) {
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
}

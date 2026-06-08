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

// 1<->3, 5<->6 배송만 노드 8·9(장애물)를 지나지 않으므로 낮게(LOW) 든다.
// 그 외 모든 배송은 노드 8 또는 9를 지나므로 높이(HIGH) 든다.
bool deliveryUsesLowLift(int fromZone, int toZone) {
  return (fromZone == 1 && toZone == 3) || (fromZone == 3 && toZone == 1)
      || (fromZone == 5 && toZone == 6) || (fromZone == 6 && toZone == 5);
}

void deliverBoxBetweenZones(int fromZone, int toZone, bool alreadyInFromZone = false) {
  if (!alreadyInFromZone) navigateToZone(fromZone);

  // 1<->3, 5<->6 만 장애물(노드 8·9) 없이 이동 → 낮게. 그 외엔 노드 8/9 통과 → 높이.
  bool lowLift = deliveryUsesLowLift(fromZone, toZone);
  liftUpStart(lowLift ? LIFT_CARRY_LOW_CM : LIFT_CARRY_HIGH_CM);
  liftUpWaitClear();

  // HIGH(24)로 든 경우, 장애물 노드(8/9)를 지난 뒤 이동 중 미리 14cm로 내리도록 arm.
  // (트리거: 7번으로 가는 8->7·9->7, 그리고 9->10/11 의 20cm 지점 — MapRouter)
  g_carryPreLowerArmed = !lowLift;

  moveBetweenZones(fromZone, toZone, zoneMoveOpts(false, true));

  g_carryPreLowerArmed = false;  // 트리거 못한 경로(예: 9->8 도착 등)면 여기서 해제
  cancelCarryPreLower();         // 미실행 예약이 남아있으면 취소(곧 zone에서 바닥까지 내림)
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

// 존이 연결된 교차로 노드(MapRouter 의 zoneToIntersection 과 동일 규약).
// 임시 보관 칸을 고를 때 이동 거리를 노드 번호 차이로 근사하는 데 쓴다.
int zoneNode(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5) return 10;
  if (zone == 6) return 11;
  return 8;
}

// fromZone 에서 이동 비용(노드 거리)이 가장 작은 빈 칸(1~6). 동률이면 낮은 번호.
// exclude 칸은 후보에서 제외한다. 없으면 0.
int nearestEmptyZone(const bool zoneOccupied[7], int fromZone, int exclude = 0) {
  int best = 0, bestCost = 0;
  for (int z = 1; z <= 6; z++) {
    if (z == exclude || zoneOccupied[z]) continue;
    int cost = zoneNode(z) - zoneNode(fromZone);
    if (cost < 0) cost = -cost;
    if (best == 0 || cost < bestCost) { best = z; bestCost = cost; }
  }
  return best;
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
        } else {
          // 목적지가 막혀 바로 배송할 수 없는 경우. 이미 6번 안에 있으므로 빈 손으로
          // 나갔다가 나중에 다시 6번으로 들어와 박스를 집는 낭비(나옴→재진입→나옴)를
          // 없앤다. QR 을 찍은 그 자리에서 box6 을 바로 들어 빈 칸으로 옮기고,
          // 실제 목적지 배송은 이후 일반 단계에서 처리한다(목적지는 유지).
          int tmp = nearestEmptyZone(zoneOccupied, 6, dest);
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
            delivered[tmp] = false;  // 임시 보관소로 재사용 시 stale delivered 해제
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
      int stuckZone = 0;
      for (int z = 1; z <= 6; z++) {
        if (boxes[z].found && boxes[z].present && !delivered[z]) stuckZone = z;
      }
      if (stuckZone == 0) break;
      // 마지막(최댓값) 빈 칸이 아니라, 막힌 박스에서 가장 가까운 빈 칸에 임시로 둔다.
      int emptyZone = nearestEmptyZone(zoneOccupied, stuckZone);
      if (emptyZone == 0) break;

      deliverBoxBetweenZones(stuckZone, emptyZone);

      boxes[emptyZone].present = true;
      boxes[emptyZone].found = true;
      boxes[emptyZone].destination = boxes[stuckZone].destination;
      boxes[stuckZone].present = false;
      boxes[stuckZone].found = false;
      boxes[stuckZone].destination = 0;
      zoneOccupied[stuckZone] = false;
      zoneOccupied[emptyZone] = true;
      // delivered[]는 '존' 기준이라, 박스를 내보내 비워진 칸(delivered=true)을
      // 임시 보관소로 재사용하면 새로 둔 박스가 영원히 건너뛰어진다(루프 데드락).
      // 임시로 둔 박스는 아직 목적지로 안 갔으므로 해당 칸 delivered 를 다시 false 로.
      delivered[emptyZone] = false;
    }
  }
}

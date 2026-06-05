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

  // ★ 1~4구역: 박스 2개를 모두 인식할 때까지 미발견 존 재진입 스캔
  rescanZones1to4();

  // ★ 5구역: 항상 박스가 있으므로 인식될 때까지 재진입(진입·대기·탈출 내내 스캔)
  scanArm(5); goToZoneDirect(5); scanZone(5);
  for (int t = 0; !boxes[5].found && t < MAX_RESCAN_TRIES; t++) {
    exitZone(5); goToZoneDirect(5); scanZone(5);
  }
  exitZone(5); scanDisarm();

  // ★ 6구역: 항상 박스가 있으므로 인식될 때까지 재진입(인식 후엔 존 안에 머무름)
  scanArm(6); goToZoneDirect(6); scanZone(6);
  for (int t = 0; !boxes[6].found && t < MAX_RESCAN_TRIES; t++) {
    exitZone(6); goToZoneDirect(6); scanZone(6);
  }
  scanDisarm();
  // ★ 6구역 탈출은 2단계 배송 시에 상황에 맞게 자동으로 이루어지므로 생략

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

  // ★ 핵심 수정: 1단계 스캔 종료 후 6구역 내부에 남아있는 상태를 인지
  bool isInsideZone6 = true;

  while (deliveredCount < 4) {
    bool movedThisTurn = false;

    // ─────────────────────────────────────────────────────────────
    // [우선 처리] STAGE 1 직후 6구역 내부에 있을 때의 동작
    // ─────────────────────────────────────────────────────────────
    if (isInsideZone6) {
      if (boxes[6].found && boxes[6].present && !delivered[6]) {
        int dest = boxes[6].destination;

        if (dest == 6) {
          DPRINTF("\n[STAY] 6구역: 이미 정답 위치.");
          delivered[6] = true;
          deliveredCount++;
          movedThisTurn = true;
        } 
        else if (!isOccupied[dest]) {
          // 6구역 박스를 당장 옮길 수 있는 경우! (진입 과정 없이 바로 들어올림)
          DPRINTF("\n[ROUTE] 6 -> "); 
          DPRINTLN(dest);
          
          liftUp();        // 이미 6구역 안에 있으므로 바로 들어올림
          exitZone(6);     // 그리고 바로 탈출
          liftUpWait();
          isInsideZone6 = false; // 탈출 완료 상태 업데이트
          
          goToZoneDirect(dest);
          liftDownUntilClear();
          exitZone(dest);
          liftDownWait();
          
          boxes[6].present = false;
          boxes[dest].present = true;
          isOccupied[6] = false;
          isOccupied[dest] = true;
          delivered[6] = true;
          
          deliveredCount++;
          movedThisTurn = true;
        }
      }
      
      // 6구역 박스를 당장 옮길 수 없는 상황이거나 (목적지가 찼음), 
      // 방금 6번 정답 처리를 해서 로봇만 빼내야 하는 경우
      if (!movedThisTurn) {
        DPRINTLNF("\n[EXIT] 다른 구역 작업을 위해 6구역에서 먼저 탈출합니다.");
        exitZone(6);
        isInsideZone6 = false; // 안전하게 메인 라인으로 빠져나왔음을 기록
      }
      
      // 이번 턴은 6구역 내부 처리로 마쳤으므로 다음 루프로 진행
      continue; 
    }

    // ─────────────────────────────────────────────────────────────
    // 기존의 일반적인 1~6구역 배송 라우팅 로직
    // ─────────────────────────────────────────────────────────────
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
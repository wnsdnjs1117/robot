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

// ============================================================
// [1단계] 스타트 박스 탈출 및 1~6구역 전체 QR 탐색
// ============================================================
void executeStage1_Search() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 1] 1~6구역 전체 QR 탐색 기동"));

  // 리프트 초기화: 하강 시작 → goToMainLine()과 동시 진행
  liftDownStart();
  goToMainLine();
  liftDownWait();

  // goToMainLine() 완료 시 로봇은 8번 허브(메인라인) 위에 정렬됨
  currentNode = 8;
  robotHeading = HDG_N;

  // 1~4구역 탐색 (qrSearchStage 내부에서 currentNode/robotHeading 갱신 불필요
  // — 탐색 완료 시 어느 구역 내부에 있으므로 아래서 exitZone으로 탈출)
  int foundZone = qrSearchStage();
  // foundZone: 탐색이 멈춘 구역 (1~4)

  Serial.print(F("\n>> [STAGE 1-A] 탐색 완료. 현 위치: "));
  Serial.print(foundZone);
  Serial.println(F("구역. 5,6구역 스캔으로 이동합니다."));

  // 현재 구역에서 탈출 → currentNode 갱신
  exitZone(foundZone);

  // ── 5구역 스캔 ──────────────────────────────────────────
  // 현재 노드(7 or 8)에서 10번 노드로 직접 이동 → 5구역 진입
  goToZoneDirect(5);
  scanZone(5);

  // ── 6구역 스캔 (5구역 탈출 후 바로 11번으로 이동) ────────
  exitZone(5);  // 10번 노드로 탈출
  goToZoneDirect(6);
  scanZone(6);

  // 6구역 탈출 → currentNode = 11
  exitZone(6);

  Serial.println(F(">> [STAGE 1-B] 모든 구역(1~6) 스캔 완수!"));
  Serial.println(F("========================================\n"));
}

// ============================================================
// [2단계] 1구역 1박스 제약 준수 직접 라우팅 배송
// ============================================================
void executeStage2_Delivery() {
  Serial.println(F("\n========================================"));
  Serial.println(F(">> [STAGE 2] 직접 라우팅 배송 가동"));

  bool isOccupied[7] = {false};
  for (int i = 1; i <= 6; i++) isOccupied[i] = boxes[i].present;

  int deliveredCount = 0;
  bool delivered[7] = {false};

  while (deliveredCount < 4) {
    bool movedThisTurn = false;

    for (int zone = 1; zone <= 6; zone++) {
      if (!boxes[zone].found || !boxes[zone].present || delivered[zone])
        continue;
      int dest = boxes[zone].destination;

      // 제자리 유지
      if (dest == zone) {
        Serial.print(F("\n[STAY] "));
        Serial.print(zone);
        Serial.println(F("구역: 이미 정답 위치."));
        delivered[zone] = true;
        deliveredCount++;
        movedThisTurn = true;
        break;
      }

      // 목적지가 비어 있으면 픽업 → 직접 이동 → 하차
      if (!isOccupied[dest]) {
        Serial.print(F("\n[ROUTE] "));
        Serial.print(zone);
        Serial.print(F(" ➔ "));
        Serial.println(dest);

        // ── 픽업 ─────────────────────────────────────────
        goToZoneDirect(zone);  // 현재 노드 → 출발 구역 직접 이동 + 진입
        liftUp();              // 24cm 상승 (15cm 이상 → 주행 허가)
        exitZone(zone);        // 구역 탈출 → currentNode 갱신

        // ── 하차 ─────────────────────────────────────────
        goToZoneDirect(dest);  // 현재 노드 → 목적 구역 직접 이동 + 진입
        liftDown();            // 0cm 하강 (10cm 이하 → 주행 허가)
        exitZone(dest);        // 구역 탈출 → currentNode 갱신

        isOccupied[zone] = false;
        isOccupied[dest] = true;
        delivered[zone] = true;
        deliveredCount++;
        movedThisTurn = true;
        break;
      }
    }

    // 데드락 해소
    if (!movedThisTurn && deliveredCount < 4) {
      int stuckZone = 0, emptyZone = 0;
      for (int z = 1; z <= 6; z++) {
        if (boxes[z].found && boxes[z].present && !delivered[z]) stuckZone = z;
        if (!isOccupied[z]) emptyZone = z;
      }
      if (stuckZone == 0 || emptyZone == 0) break;  // 안전 탈출

      Serial.print(F("\n[DEADLOCK] "));
      Serial.print(stuckZone);
      Serial.print(F(" ➔ 빈 구역 "));
      Serial.println(emptyZone);

      goToZoneDirect(stuckZone);
      liftUp();
      exitZone(stuckZone);

      goToZoneDirect(emptyZone);
      liftDown();
      exitZone(emptyZone);

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

  Serial.println(F("\n>> [STAGE 2] 배송 완료!"));
  Serial.println(F("========================================\n"));
}

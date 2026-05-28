/* ============================================================
 * BoxMap.cpp - 제자리 유지(Stay)가 포함된 안전 랜덤 시뮬레이터.
 * ============================================================ */
#include "BoxMap.h"

#include <Arduino.h>

BoxInfo boxes[7];

static void printZoneName(int z) {
  if (z == ZONE_IN)
    DPRINTF("입고(5)");
  else if (z == ZONE_OUT)
    DPRINTF("출고(6)");
  else {
    DPRINT(z);
    DPRINTF("구역");
  }
}

void setupRandomLayout() {
  for (int i = 0; i < 7; i++) {
    boxes[i].present = false;
    boxes[i].found = false;
    boxes[i].destination = 0;
  }
  randomSeed(analogRead(A0));

  // 1. 고정 박스 배치 (5번 입고, 6번 출고)
  boxes[ZONE_IN].present = true;
  boxes[ZONE_OUT].present = true;

  // 2. 1~4구역 중 랜덤하게 2곳 배치 (a, b)
  int a = random(1, 5);
  int b;
  do {
    b = random(1, 5);
  } while (b == a);

  boxes[a].present = true;
  boxes[b].present = true;

  // 3. 목적지 배열 셔플 (제자리 유지 가능)
  int dests[4];
  int all[6] = {1, 2, 3, 4, 5, 6};

  for (int i = 5; i > 0; i--) {
    int r = random(0, i + 1);
    int temp = all[i];
    all[i] = all[r];
    all[r] = temp;
  }

  dests[0] = all[0];
  dests[1] = all[1];
  dests[2] = all[2];
  dests[3] = all[3];

  // 4. 목적지 확정 데이터 바인딩
  boxes[a].destination = dests[0];
  boxes[b].destination = dests[1];
  boxes[ZONE_IN].destination = dests[2];
  boxes[ZONE_OUT].destination = dests[3];

  // 5. 시리얼 모니터 정답지 출력 (★ 문법 에러 전면 수정 완료)
  DPRINTLNF("\n===== [시뮬레이션: 이번 판 정답지] =====");

  DPRINTF("  랜덤 박스 [");
  printZoneName(a);
  DPRINTF("] -> 목적지: ");
  DPRINTLN(dests[0]);
  DPRINTF("  랜덤 박스 [");
  printZoneName(b);
  DPRINTF("] -> 목적지: ");
  DPRINTLN(dests[1]);
  DPRINTF("  고정 박스 [입고(5)] -> 목적지: ");
  DPRINTLN(dests[2]);
  DPRINTF("  고정 박스 [출고(6)] -> 목적지: ");
  DPRINTLN(dests[3]);

  DPRINTLNF("========================================");
}

bool scanZone(int zone) {
  DPRINTF(">> [SCAN] ");
  printZoneName(zone);
  if (boxes[zone].present) {
    boxes[zone].found = true;
    DPRINTF(" -> 발견! (목적지: ");
    DPRINT(boxes[zone].destination);
    DPRINTLNF(")");
    return true;
  } else {
    DPRINTLNF(" -> 비어 있음");
    return false;
  }
}

void printSearchResult() {
  DPRINTLNF("\n========== [현재까지 확정된 QR 정보] ==========");
  for (int z = 1; z <= 6; z++) {
    if (boxes[z].found) {
      DPRINTF("  구역 ");
      DPRINT(z);
      DPRINTF(" -> 목적지: ");
      DPRINTLN(boxes[z].destination);
    }
  }
  DPRINTLNF("==============================================");
}
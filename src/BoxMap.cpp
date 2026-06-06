/* ============================================================
 * BoxMap.cpp - 제자리 유지(Stay)가 포함된 안전 랜덤 시뮬레이터.
 * ============================================================ */
#include "BoxMap.h"

#include <Arduino.h>

#include "Config.h"
#include "Motion.h"   // beep()
#if !QR_SIMULATION
#include "HuskyQR.h"  // 실제 QR 리더
#endif

BoxInfo boxes[7];

// ── QR 연속 스캔 상태 ──
volatile int g_scanTargetZone = 0;

void scanArm(int zone) { 
#if !QR_SIMULATION
  HuskyQR::flush(); // 새 구역 스캔 전 이전 구역의 버퍼된 데이터 비우기
#endif
  g_scanTargetZone = zone; 
}

void scanDisarm() { 
  g_scanTargetZone = 0; 
}

void scanTick() {
  int z = g_scanTargetZone;
  if (z == 0 || boxes[z].found) return;          // 비활성 또는 이미 인식
  static unsigned long last = 0;
  if (millis() - last < SCAN_POLL_MS) return;    // 스티어링 루프 보호용 throttle
  last = millis();
#if QR_SIMULATION
  // 가상: 박스 존재를 곧 인식으로 간주(목적지는 setupRandomLayout가 채움)
  if (boxes[z].present) { boxes[z].found = true; beep(120); }
#else
  int id = HuskyQR::readBoxId();
  if (id >= 1 && id <= 6) {
    boxes[z].found = true;
    boxes[z].present = true;
    boxes[z].destination = id;   // 읽은 QR ID가 곧 목적지 존
    beep(120);
  }
#endif
}

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

  boxes[ZONE_IN].present = true;
  boxes[ZONE_OUT].present = true;

  int a = random(1, 5);
  int b;
  do {
    b = random(1, 5);
  } while (b == a);

  boxes[a].present = true;
  boxes[b].present = true;

  // 목적지 배열 셔플 (제자리 유지 가능)
  int all[6] = {1, 2, 3, 4, 5, 6};
  for (int i = 5; i > 0; i--) {
    int r = random(0, i + 1);
    int temp = all[i];
    all[i] = all[r];
    all[r] = temp;
  }

  // 목적지 확정 데이터 바인딩 (불필요한 중간 배열 제거)
  boxes[a].destination = all[0];
  boxes[b].destination = all[1];
  boxes[ZONE_IN].destination = all[2];
  boxes[ZONE_OUT].destination = all[3];

  DPRINTLNF("\n===== [시뮬레이션: 이번 판 정답지] =====");
  DPRINTF("  랜덤 박스 [");
  printZoneName(a);
  DPRINTF("] -> 목적지: ");
  DPRINTLN(all[0]);
  DPRINTF("  랜덤 박스 [");
  printZoneName(b);
  DPRINTF("] -> 목적지: ");
  DPRINTLN(all[1]);
  DPRINTF("  고정 박스 [입고(5)] -> 목적지: ");
  DPRINTLN(all[2]);
  DPRINTF("  고정 박스 [출고(6)] -> 목적지: ");
  DPRINTLN(all[3]);
  DPRINTLNF("========================================");
}

// 존에서 정지한 채 최대 SCAN_DWELL_MS 동안 QR을 계속 스캔(인식되면 조기 종료).
// 진입/탈출 모션 중에도 scanTick()이 돌므로 여기 도착 전에 이미 래치됐을 수 있다.
// disarm은 호출자(탐색 헬퍼)가 탈출까지 끝난 뒤 수행 → 탈출 중에도 계속 스캔.
bool scanZone(int zone) {
  DPRINTF(">> [SCAN] ");
  printZoneName(zone);
  scanArm(zone);
  unsigned long end = millis() + SCAN_DWELL_MS;
  while (millis() < end && !boxes[zone].found) { scanTick(); }
  if (boxes[zone].found) {
    DPRINTF(" -> 발견! (목적지: ");
    DPRINT(boxes[zone].destination);
    DPRINTLNF(")");
    return true;
  }
  DPRINTLNF(" -> 미인식");
  return false;
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
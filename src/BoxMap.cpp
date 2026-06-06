/* ============================================================
 * BoxMap.cpp - 박스 데이터 및 QR 스캔 상태
 * ============================================================ */
#include "BoxMap.h"
#include "Config.h"
#include "Motion.h"
#if !QR_SIMULATION
#include "HuskyQR.h"
#endif

BoxInfo boxes[7];
volatile int activeScanZone = 0;

void beginZoneScan(int zone) {
#if !QR_SIMULATION
  HuskyQR::flush();
#endif
  activeScanZone = zone;
}

void endZoneScan() {
  activeScanZone = 0;
}

void pollZoneScan() {
  int zone = activeScanZone;
  if (zone == 0 || boxes[zone].found) return;

  static unsigned long lastPollMs = 0;
  if (millis() - lastPollMs < SCAN_POLL_MS) return;
  lastPollMs = millis();

  int fl, fc, fr, rl, rc, rr;
  readFrontLineSensors(fl, fc, fr);
  readRearLineSensors(rl, rc, rr);
  if (frontOnLine(fl, fc, fr) || rearOnLine(rl, rc, rr)) return;

#if QR_SIMULATION
  if (boxes[zone].present) { boxes[zone].found = true; playBeep(BUZZER_QR_FOUND_MS); }
#else
  int id = HuskyQR::readBoxId();
  if (id >= 1 && id <= 6) {
    boxes[zone].found = true;
    boxes[zone].present = true;
    boxes[zone].destination = id;
    playBeep(BUZZER_QR_FOUND_MS);
  }
#endif
}

#if ROBOT_DEBUG
static void printZoneLabel(int zone) {
  if (zone == ZONE_IN) DPRINTF("입고(5)");
  else if (zone == ZONE_OUT) DPRINTF("출고(6)");
  else { DPRINT(zone); DPRINTF("구역"); }
}
#endif

void setupRandomLayout() {
  for (int i = 0; i < 7; i++) {
    boxes[i].present = false;
    boxes[i].found = false;
    boxes[i].destination = 0;
  }
  randomSeed(analogRead(A0));

  boxes[ZONE_IN].present = true;
  boxes[ZONE_OUT].present = true;

  int zoneA = random(1, 5);
  int zoneB;
  do { zoneB = random(1, 5); } while (zoneB == zoneA);
  boxes[zoneA].present = true;
  boxes[zoneB].present = true;

  int destPool[6] = {1, 2, 3, 4, 5, 6};
  for (int i = 5; i > 0; i--) {
    int j = random(0, i + 1);
    int tmp = destPool[i]; destPool[i] = destPool[j]; destPool[j] = tmp;
  }

  boxes[zoneA].destination = destPool[0];
  boxes[zoneB].destination = destPool[1];
  boxes[ZONE_IN].destination = destPool[2];
  boxes[ZONE_OUT].destination = destPool[3];

#if ROBOT_DEBUG
  DPRINTLNF("\n===== [시뮬레이션: 이번 판 정답지] =====");
  DPRINTF("  랜덤 박스 ["); printZoneLabel(zoneA);
  DPRINTF("] -> 목적지: "); DPRINTLN(destPool[0]);
  DPRINTF("  랜덤 박스 ["); printZoneLabel(zoneB);
  DPRINTF("] -> 목적지: "); DPRINTLN(destPool[1]);
  DPRINTF("  고정 박스 [입고(5)] -> 목적지: "); DPRINTLN(destPool[2]);
  DPRINTF("  고정 박스 [출고(6)] -> 목적지: "); DPRINTLN(destPool[3]);
  DPRINTLNF("========================================");
#endif
}

bool waitForZoneScan(int zone) {
#if ROBOT_DEBUG
  DPRINTF(">> [SCAN] ");
  printZoneLabel(zone);
#endif
  beginZoneScan(zone);
  unsigned long deadline = millis() + SCAN_DWELL_MS;
  while (millis() < deadline && !boxes[zone].found) pollZoneScan();
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
#if ROBOT_DEBUG
  DPRINTLNF("\n========== [현재까지 확정된 QR 정보] ==========");
  for (int z = 1; z <= 6; z++) {
    if (boxes[z].found) {
      DPRINTF("  구역 "); DPRINT(z);
      DPRINTF(" -> 목적지: "); DPRINTLN(boxes[z].destination);
    }
  }
  DPRINTLNF("==============================================");
#endif
}

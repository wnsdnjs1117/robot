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
  if (boxes[zone].present) { boxes[zone].found = true; startBeep(BUZZER_QR_FOUND_MS); }
#else
  int id = HuskyQR::readBoxId();
  int dest = 0;
  for (int i = 0; i < 6; i++) {          // 읽은 ID에 해당하는 목적지(i+1) 찾기
    if (DEST_HUSKY_ID[i] == id) { dest = i + 1; break; }
  }
  if (dest >= 1 && dest <= 6) {
    boxes[zone].found = true;
    boxes[zone].present = true;
    boxes[zone].destination = dest;
    startBeep(BUZZER_QR_FOUND_MS);
  }
#endif
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
}

bool waitForZoneScan(int zone) {
  beginZoneScan(zone);
  unsigned long deadline = millis() + SCAN_DWELL_MS;
  while (millis() < deadline && !boxes[zone].found) pollZoneScan();
  return boxes[zone].found;
}

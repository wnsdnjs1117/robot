/* ============================================================
 * BoxMap.h - 박스·QR 스캔 상태
 * ============================================================ */
#ifndef BOXMAP_H
#define BOXMAP_H

const int ZONE_IN  = 5;  // 입고 구역
const int ZONE_OUT = 6;  // 출고 구역

struct BoxInfo {
  bool present;      // 박스 존재 여부
  bool found;        // QR 인식 완료
  int  destination;  // 목적지 구역 (1~6)
};

extern BoxInfo boxes[7];

void setupRandomLayout();
bool waitForZoneScan(int zone);

extern volatile int activeScanZone; // 1~6: 스캔 중인 구역 / 0: 비활성
void beginZoneScan(int zone);
void endZoneScan();
void pollZoneScan();

#endif

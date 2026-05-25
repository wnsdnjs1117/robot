/* ============================================================
 * BoxMap.h - 가상 랜덤 박스 데이터 구조 선언
 * ============================================================ */
#ifndef BOXMAP_H
#define BOXMAP_H

const int ZONE_IN = 5;
const int ZONE_OUT = 6;

struct BoxInfo {
  bool present;
  bool found;
  int destination;
};

extern BoxInfo boxes[7];

void setupRandomLayout();
bool scanZone(int zone);
int knownBoxCount();
void printSearchResult();

#endif
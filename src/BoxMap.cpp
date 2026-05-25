#include "BoxMap.h"

#include <Arduino.h>

BoxInfo boxes[7];

static void printZoneName(int z) {
  if (z == ZONE_IN)
    Serial.print(F("입고"));
  else if (z == ZONE_OUT)
    Serial.print(F("출고"));
  else {
    Serial.print(z);
    Serial.print(F("구역"));
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
  boxes[ZONE_IN].found = true;
  boxes[ZONE_OUT].present = true;
  boxes[ZONE_OUT].found = true;

  int a = random(1, 5);
  int b;
  do {
    b = random(1, 5);
  } while (b == a);

  boxes[a].present = true;
  boxes[b].present = true;

  int activeZones[4] = {a, b, ZONE_IN, ZONE_OUT};
  int dests[4];
  bool valid = false;

  while (!valid) {
    int all[6] = {1, 2, 3, 4, 5, 6};
    for (int i = 0; i < 6; i++) {
      int r = random(0, 6);
      int temp = all[i];
      all[i] = all[r];
      all[r] = temp;
    }
    valid = true;
    for (int i = 0; i < 4; i++) {
      dests[i] = all[i];
      if (dests[i] == activeZones[i]) {
        valid = false;
        break;
      }
    }
  }

  boxes[a].destination = dests[0];
  boxes[b].destination = dests[1];
  boxes[ZONE_IN].destination = dests[2];
  boxes[ZONE_OUT].destination = dests[3];

  Serial.println(F("\n===== [이번 판 박스 배치 정답] ====="));
  Serial.print(F("  랜덤: "));
  printZoneName(a);
  Serial.print(F(", "));
  printZoneName(b);
  Serial.println();
  Serial.println(F("  고정: 입고, 출고"));
  Serial.println(F("========================================"));
}

bool scanZone(int zone) {
  Serial.print(F(">> [SCAN] "));
  printZoneName(zone);
  if (boxes[zone].present) {
    boxes[zone].found = true;
    Serial.print(F(" -> 발견! (목적지 "));
    Serial.print(boxes[zone].destination);
    Serial.println(F(")"));
    return true;
  } else {
    Serial.println(F(" -> 비어 있음"));
    return false;
  }
}

int knownBoxCount() {
  int n = 0;
  for (int z = 1; z <= 6; z++)
    if (boxes[z].found) n++;
  return n;
}

void printSearchResult() {
  Serial.println(F("\n========== [탐색 완료] =========="));
  for (int z = 1; z <= 6; z++) {
    if (boxes[z].found) {
      Serial.print(F("  박스: "));
      printZoneName(z);
      Serial.print(F("  (목적지 "));
      Serial.print(boxes[z].destination);
      Serial.println(F(")"));
    }
  }
  Serial.println(F("================================"));
}
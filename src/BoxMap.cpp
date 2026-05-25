/* ============================================================
 * BoxMap.cpp - 박스 위치 관리 + QR 스캔 시뮬레이션 구현
 * ============================================================ */
#include "BoxMap.h"

#include <Arduino.h>

BoxInfo boxes[7];

// 구역 이름 출력 헬퍼
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

// QR 목적지(이동 위치) 랜덤 부여 - 자기 구역은 제외 (후속 단계용 placeholder)
static void assignDest(int zone) {
  int d;
  do {
    d = random(1, 7);
  } while (d == zone);  // 1~6 중 자기 구역 제외
  boxes[zone].destination = d;
}

void setupRandomLayout() {
  // 0) 초기화
  for (int i = 0; i < 7; i++) {
    boxes[i].present = false;
    boxes[i].found = false;
    boxes[i].destination = 0;
  }

  // 1) 입고(5)/출고(6) 고정 박스 - 로봇이 이미 안다고 처리(found=true)
  boxes[ZONE_IN].present = true;
  boxes[ZONE_IN].found = true;
  boxes[ZONE_OUT].present = true;
  boxes[ZONE_OUT].found = true;

  // 2) 1~4구역 중 랜덤하게 서로 다른 2곳 선택
  randomSeed(analogRead(A0));  // 미사용 아날로그핀(A0) 노이즈로 시드
  int a = random(1, 5);        // 1~4
  int b;
  do {
    b = random(1, 5);
  } while (b == a);
  boxes[3].present = true;
  boxes[4].present = true;

  // 3) 모든 박스에 QR 목적지 부여 (시뮬레이션)
  assignDest(a);
  assignDest(b);
  assignDest(ZONE_IN);
  assignDest(ZONE_OUT);

  // 4) 정답지 출력 (디버그용 - 로봇은 1~4 위치를 모름)
  Serial.println(F("\n===== [이번 판 박스 배치 / 정답지] ====="));
  Serial.print(F("  랜덤 박스: "));
  printZoneName(a);
  Serial.print(F(", "));
  printZoneName(b);
  Serial.println();
  Serial.println(F("  입고/출고: 고정 (이미 알고 있음)"));
  Serial.println(F("  >> 로봇은 1~4구역 위치를 모른 채 탐색 시작"));
  Serial.println(F("========================================"));
}

bool scanZone(int zone) {
  Serial.print(F(">> [SCAN] "));
  printZoneName(zone);
  if (boxes[zone].present) {
    boxes[zone].found = true;
    Serial.print(F(" -> 박스 발견! (QR 목적지 "));
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
  Serial.print(F("  총 확정 박스: "));
  Serial.print(knownBoxCount());
  Serial.println(F(" / 4"));
  Serial.println(F("================================"));
}

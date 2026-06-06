/* ============================================================
 * HuskyQR.cpp - HuskyLens 공식 라이브러리(HUSKYLENS) 래퍼
 * request()로 결과를 받아 첫 블록의 학습 ID(1~6)를 목적지로 반환한다.
 * 통신 타임아웃을 짧게 잡아 카메라 무응답이 주행 루프를 막지 않게 한다.
 * ============================================================ */
#include "HuskyQR.h"

#include <Arduino.h>
#include <Wire.h>
#include "HUSKYLENS.h"

static HUSKYLENS huskylens;

namespace HuskyQR {

void begin() {
  // Wire는 PRIZM(PrizmBegin)이 이미 시작함. 여기선 노크 핸드셰이크만 시도.
  huskylens.setTimeOutDuration(30);   // 기본 100ms → 30ms (주행 루프 보호)
  for (int i = 0; i < 5; i++) {       // 카메라 부팅 지연 대비 짧은 재시도(비치명적)
    if (huskylens.begin(Wire)) break;
    delay(50);
  }
}

int readBoxId() {
  if (!huskylens.request()) return 0;   // 통신 실패/무응답
  while (huskylens.available()) {
    HUSKYLENSResult r = huskylens.read();
    if (r.command == COMMAND_RETURN_BLOCK && r.ID >= 1 && r.ID <= 6) {
      return (int)r.ID;
    }
  }
  return 0;
}

void flush() {
  // 이전 구역 스캔 시 버퍼에 남아있던 데이터를 모두 읽어 폐기
  if (huskylens.request()) {
    while (huskylens.available()) {
      huskylens.read();
    }
  }
}

} // namespace HuskyQR
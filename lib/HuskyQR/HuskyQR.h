/* ============================================================
 * HuskyQR.h - HuskyLens(I2C) 최소 QR/태그 ID 리더
 *   인식된 블록의 학습 ID(1~6)를 그대로 반환한다. (텍스트 파싱 없음)
 *   주의: Wire.begin()은 호출하지 않는다 — PRIZM(PrizmBegin)이 이미 버스를 연다.
 * ============================================================ */
#ifndef HUSKYQR_H
#define HUSKYQR_H

namespace HuskyQR {
  void begin();      // 초기화(버스는 PRIZM가 이미 시작). 현재는 no-op 자리.
  int  readBoxId();  // 1회 요청·파싱 → 인식 블록 ID 1..6, 없거나 실패 시 0
}

#endif

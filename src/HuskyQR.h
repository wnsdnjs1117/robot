/* ============================================================
 * HuskyQR.h - HuskyLens(공식 라이브러리) 래퍼
 * 인식된 블록의 학습 ID(1~6)를 그대로 목적지로 반환한다. (텍스트 파싱 없음)
 * 주의: Wire.begin()은 호출하지 않는다 — PRIZM(PrizmBegin)이 이미 버스를 연다.
 * ============================================================ */
#ifndef HUSKYQR_H
#define HUSKYQR_H

namespace HuskyQR {
  void begin();      // 노크 핸드셰이크(버스는 PRIZM가 이미 시작)
  int  readBoxId();  // 1회 요청·파싱 → 인식 블록 ID 1..6, 없거나 실패 시 0
  void flush();      // 카메라 버퍼 비우기 (이전 스캔 잔재 방지)
}

#endif
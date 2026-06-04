/* ============================================================
 * LiftTest.h - 리프트 단독 테스트 모드
 *   Config.h 에서 LIFT_TEST_MODE 1 로 설정하면 활성화
 *
 *   u = liftUp()        (5cm 블로킹 후 배경 상승)
 *   d = liftDownStart() (배경 하강)
 *   s = 즉시 정지
 *
 *   시리얼 출력 (200ms): encL/encR | spdL/spdR(tick/10ms) | hL/hR(cm)
 * ============================================================ */
#ifndef LIFT_TEST_H
#define LIFT_TEST_H

#if LIFT_TEST_MODE

#include "Config.h"
#include "Lift.h"

// ── 상태 ─────────────────────────────────────────────────────
static enum { LT_IDLE, LT_UP, LT_DOWN } _ltState = LT_IDLE;
static long          _ltPrevEncL  = 0, _ltPrevEncR  = 0;
static long          _ltSpdL      = 0, _ltSpdR      = 0;
static unsigned long _ltLastTick  = 0;
static unsigned long _ltLastPrint = 0;

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();
  exc.controllerEnable(EXP_ID);
  { unsigned long _t = millis(); while (millis() - _t < 10) { } }  // delay 없이 settle
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  Serial.println(F("=== LIFT TEST ==="));
  Serial.println(F("u=상승  d=하강  s=정지"));
  Serial.println(F("encL/R | spdL/R(tick/10ms) | hL/R(cm)"));
}

void loop() {
  // ── 키 입력 ────────────────────────────────────────────────
  if (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'u' || c == 'U') {
      Serial.println(F("[UP]"));
      _ltState = LT_UP;
      liftUp();   // 5cm 까지 블로킹, 이후 liftUpTick() 으로 계속
    }
    else if (c == 'd' || c == 'D') {
      Serial.println(F("[DOWN]"));
      _ltState = LT_DOWN;
      liftDownStart();
    }
    else if (c == 's' || c == 'S') {
      Serial.println(F("[STOP]"));
      _ltState = LT_IDLE;
      exc.setMotorPowers(EXP_ID, 0, 0);
    }
  }

  // ── 리프트 틱 (실제 로봇과 동일한 함수 호출) ──────────────
  if (_ltState == LT_UP)   liftUpTick();
  if (_ltState == LT_DOWN) liftDownTick();

  unsigned long now = millis();

  // ── 10ms 주기: 속도 계산 ──────────────────────────────────
  if (now - _ltLastTick >= LIFT_TICK_INTERVAL_MS) {
    long encL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long encR = exc.readEncoderCount(EXP_ID, LIFT_R);
    _ltSpdL   = encL - _ltPrevEncL;
    _ltSpdR   = encR - _ltPrevEncR;
    _ltPrevEncL = encL;
    _ltPrevEncR = encR;
    _ltLastTick = now;
  }

  // ── 200ms 주기: 시리얼 출력 ───────────────────────────────
  if (now - _ltLastPrint >= 200) {
    long encL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long encR = exc.readEncoderCount(EXP_ID, LIFT_R);

    Serial.print(F("enc:"));  Serial.print(encL);
    Serial.print(F("/"));     Serial.print(encR);
    Serial.print(F(" spd:")); Serial.print(_ltSpdL);
    Serial.print(F("/"));     Serial.print(_ltSpdR);
    Serial.print(F(" h:"));   Serial.print(heightL, 2);
    Serial.print(F("/"));     Serial.println(heightR, 2);

    _ltLastPrint = now;
  }
}

#endif // LIFT_TEST_MODE
#endif // LIFT_TEST_H

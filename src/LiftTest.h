/* ============================================================
 * LiftTest.h - 리프트 단독 테스트 모드
 *   Config.h 에서 LIFT_TEST_MODE 1 로 설정하면 활성화
 *   s = 정지 / u = 상승 / d = 하강
 *   시리얼 출력 (200ms): encL encR | spdL spdR | hL hR
 * ============================================================ */
#ifndef LIFT_TEST_H
#define LIFT_TEST_H

#if LIFT_TEST_MODE

#include "Config.h"
#include "Lift.h"

// ── 테스트 전용 상태 ─────────────────────────────────────────
static char     _ltMode      = 's';
static long     _ltPrevEncL  = 0, _ltPrevEncR  = 0;
static long     _ltSpeedL    = 0, _ltSpeedR    = 0;
static float    _ltHeightL   = 0, _ltHeightR   = 0;
static unsigned long _ltLastTick  = 0;
static unsigned long _ltLastPrint = 0;

// ── 방향 상수 (Lift.cpp와 동일) ──────────────────────────────
static const int _LT_DIR_L =  1;
static const int _LT_DIR_R = -1;

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();
  exc.controllerEnable(EXP_ID);
  delay(10);
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  Serial.println(F("=== LIFT TEST MODE ==="));
  Serial.println(F("u=상승  d=하강  s=정지"));
  Serial.println(F("[encL/encR | spdL/spdR(tick/10ms) | hL/hR(cm)]"));
}

void loop() {
  // ── 키 입력 ────────────────────────────────────────────────
  if (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'u': case 'U':
        _ltMode = 'u';
        Serial.println(F("[UP]"));
        break;
      case 'd': case 'D':
        _ltMode = 'd';
        Serial.println(F("[DOWN]"));
        break;
      case 's': case 'S':
        _ltMode = 's';
        exc.setMotorPowers(EXP_ID, 0, 0);
        Serial.println(F("[STOP]"));
        break;
      default: break;
    }
  }

  unsigned long now = millis();

  // ── 10ms 주기: 엔코더 → 속도 → 높이 갱신 ─────────────────
  if (now - _ltLastTick >= LIFT_TICK_INTERVAL_MS) {
    long encL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long encR = exc.readEncoderCount(EXP_ID, LIFT_R);

    _ltSpeedL  = (encL - _ltPrevEncL) * _LT_DIR_L;
    _ltSpeedR  = (encR - _ltPrevEncR) * _LT_DIR_R;
    _ltHeightL += (float)_ltSpeedL / LIFT_COUNTS_PER_CM;
    _ltHeightR += (float)_ltSpeedR / LIFT_COUNTS_PER_CM;

    _ltPrevEncL = encL;
    _ltPrevEncR = encR;
    _ltLastTick = now;
  }

  // ── 모터 출력 ─────────────────────────────────────────────
  if      (_ltMode == 'u') exc.setMotorPowers(EXP_ID,  LIFT_UP_POWER, -LIFT_UP_POWER);
  else if (_ltMode == 'd') exc.setMotorPowers(EXP_ID, -LIFT_DOWN_POWER, LIFT_DOWN_POWER);

  // ── 200ms 주기: 시리얼 출력 ───────────────────────────────
  if (now - _ltLastPrint >= 200) {
    long encL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long encR = exc.readEncoderCount(EXP_ID, LIFT_R);

    Serial.print(F("enc:"));  Serial.print(encL);
    Serial.print(F("/"));     Serial.print(encR);
    Serial.print(F(" spd:")); Serial.print(_ltSpeedL);
    Serial.print(F("/"));     Serial.print(_ltSpeedR);
    Serial.print(F(" h:"));   Serial.print(_ltHeightL, 2);
    Serial.print(F("/"));     Serial.println(_ltHeightR, 2);

    _ltLastPrint = now;
  }
}

#endif // LIFT_TEST_MODE
#endif // LIFT_TEST_H

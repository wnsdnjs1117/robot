/* ============================================================
 * Lift.cpp - 듀얼 리프트 높이 기반 제어 (좌우 독립 정지 및 실시간 동기화)
 * 상승: 각 모터가 24cm 도달 시 개별 정지 (20cm 이상 시 좌/우 독립 감속)
 * 하강: 각 모터가 설정 이하 진입 후 비례 타이머 만료 시 개별 정지 (좌/우 독립 감속)
 * 동기화: 양쪽 구동 중일 때 높이 편차를 기반으로 파워 실시간 보정
 * 보정: 리프트 계산 높이가 음수(< 0)가 될 경우 0으로 강제 클램핑 처리
 * ============================================================ */
#include "Lift.h"
#include "Config.h"

static const int DIR_L =  1;
static const int DIR_R = -1;

EXPANSION exc;
const int EXP_ID = 1;
const int LIFT_L = 1;
const int LIFT_R = 2;

float heightL = 0;
float heightR = 0;

static bool liftUpRunning   = false;
static bool liftDownRunning = false;

// 하강 — 좌우 독립 타이머
static bool          nearFloorL = false, nearFloorR = false;
static bool          stoppedL   = false, stoppedR   = false;
static unsigned long nearFloorStartL = 0, nearFloorStartR = 0;
static unsigned long nearFloorDurL   = 0, nearFloorDurR   = 0;

static long          prevEncL     = 0, prevEncR     = 0;
static unsigned long lastTickTime = 0;

// ── 내부 헬퍼 ────────────────────────────────────────────────

static void resetState() {
  prevEncL     = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  prevEncR     = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;
  lastTickTime = 0;
}

static void updateHeight() {
  unsigned long now = millis();
  if (now - lastTickTime < LIFT_TICK_INTERVAL_MS) return;

  long curL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  long curR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;

  heightL += (float)(curL - prevEncL) / LIFT_COUNTS_PER_CM;
  heightR += (float)(curR - prevEncR) / LIFT_COUNTS_PER_CM;

  // 높이가 음수 수치로 내려가면 0으로 클램핑
  if (heightL < 0.0f) heightL = 0.0f;
  if (heightR < 0.0f) heightR = 0.0f;

  prevEncL = curL;
  prevEncR = curR;
  lastTickTime = now;
}

static unsigned long calcNearFloorDur(float h) {
  float remaining = h - (-2.0f);
  float refDist   = LIFT_NEAR_FLOOR_CM - (-2.0f);  // 7cm
  return (unsigned long)(remaining / refDist * LIFT_FLOOR_TIME_MS);
}

// ── 상승 (nonblocking: liftUpStart → liftUpWaitClear(5cm) → 주행 중 liftUpTick) ──

void liftUpStart() {
  resetState();
  liftUpRunning = true;
}

bool liftUpClearReached() {
  return heightL >= LIFT_UP_CLEAR_CM && heightR >= LIFT_UP_CLEAR_CM;
}

void liftUpWaitClear() {
  while (!liftUpClearReached()) {
    liftUpTick();
    liftDownTick();
  }
}

static int liftUpBasePower(float h, bool done, int slowPwr) {
  if (done) return 0;
  return (h >= LIFT_UP_SLOW_ZONE_CM) ? slowPwr : LIFT_UP_POWER;
}

void liftUpTick() {
  if (!liftUpRunning) return;
  updateHeight();

  bool doneL = (heightL >= LIFT_MAX_HEIGHT_CM);
  bool doneR = (heightR >= LIFT_MAX_HEIGHT_CM);
  int basePwrL = liftUpBasePower(heightL, doneL, LIFT_UP_SLOW_POWER_L);
  int basePwrR = liftUpBasePower(heightR, doneR, LIFT_UP_SLOW_POWER_R);

  int syncOffset = 0;
  if (basePwrL > 0 && basePwrR > 0) {
    syncOffset = (int)((heightL - heightR) * LIFT_SYNC_GAIN);
  }

  int pwrL = basePwrL - syncOffset;
  int pwrR = basePwrR + syncOffset;

  if (basePwrL > 0 && pwrL < 5) pwrL = 5;
  if (basePwrR > 0 && pwrR < 5) pwrR = 5;
  if (basePwrL == 0) pwrL = 0;
  if (basePwrR == 0) pwrR = 0;

  exc.setMotorPowers(EXP_ID, pwrL, -pwrR);

  if (doneL && doneR) liftUpRunning = false;
}

void liftUpWait() {
  if (!liftUpRunning) return;
  lastTickTime = 0;
  DPRINTLNF(">> [LIFT] 상승 완료 대기 중...");
  while (liftUpRunning) {
    liftUpTick();
  }
  DPRINTLNF(">> [LIFT] 상승 확인 완료");
}

// ── 하강 ─────────────────────────────────────────────────────

void liftDown() {
  liftDownStart();
  liftDownWait();
}

void liftDownStart() {
  resetState();
  nearFloorL = nearFloorR = false;
  stoppedL   = stoppedR   = false;
  liftDownRunning = true;
  DPRINTLNF(">> [LIFT] 하강 시작");
}

void liftDownTick() {
  if (!liftDownRunning) return;
  updateHeight();

  unsigned long now = millis();

  if (!nearFloorL && heightL <= LIFT_NEAR_FLOOR_CM) {
    nearFloorL      = true;
    nearFloorDurL   = calcNearFloorDur(heightL);
    nearFloorStartL = now;
    DPRINTLNF(">> [LIFT-L] 바닥 근접 — 타이머 시작");
  }
  if (!nearFloorR && heightR <= LIFT_NEAR_FLOOR_CM) {
    nearFloorR      = true;
    nearFloorDurR   = calcNearFloorDur(heightR);
    nearFloorStartR = now;
    DPRINTLNF(">> [LIFT-R] 바닥 근접 — 타이머 시작");
  }

  if (nearFloorL && !stoppedL && (now - nearFloorStartL >= nearFloorDurL)) {
    stoppedL = true;
    DPRINTLNF(">> [LIFT-L] 하강 완료");
  }
  if (nearFloorR && !stoppedR && (now - nearFloorStartR >= nearFloorDurR)) {
    stoppedR = true;
    DPRINTLNF(">> [LIFT-R] 하강 완료");
  }

  int basePwrL = stoppedL ? 0 : ((heightL <= LIFT_DOWN_SLOW_ZONE_CM) ? LIFT_DOWN_SLOW_POWER_L : LIFT_DOWN_POWER);
  int basePwrR = stoppedR ? 0 : ((heightR <= LIFT_DOWN_SLOW_ZONE_CM) ? LIFT_DOWN_SLOW_POWER_R : LIFT_DOWN_POWER);

  int syncOffset = 0;
  if (basePwrL > 0 && basePwrR > 0) {
    syncOffset = (int)((heightL - heightR) * LIFT_SYNC_GAIN);
  }

  int pwrL = basePwrL + syncOffset;
  int pwrR = basePwrR - syncOffset;

  if (basePwrL > 0 && pwrL < 5) pwrL = 5;
  if (basePwrR > 0 && pwrR < 5) pwrR = 5;
  if (basePwrL == 0) pwrL = 0;
  if (basePwrR == 0) pwrR = 0;

  exc.setMotorPowers(EXP_ID, -pwrL, pwrR);

  if (stoppedL && stoppedR) {
    exc.resetEncoder(EXP_ID, LIFT_L);
    exc.resetEncoder(EXP_ID, LIFT_R);
    heightL = 0.0f;
    heightR = 0.0f;
    liftDownRunning = false;
    DPRINTLNF(">> [LIFT] 하강 완전 완료");
  }
}

void liftDownWait() {
  if (!liftDownRunning) return;
  lastTickTime = 0;
  DPRINTLNF(">> [LIFT] 착지 대기 중...");
  while (liftDownRunning) {
    liftDownTick();
  }
  DPRINTLNF(">> [LIFT] 착지 확인 완료");
}

void liftDownUntilClear() {
  liftDownStart();
  while (heightL > LIFT_DOWN_CLEAR_CM || heightR > LIFT_DOWN_CLEAR_CM) {
    liftDownTick();
  }
  DPRINTLNF(">> [LIFT] 주행 허가 기준 높이 도달 (배경 하강 계속)");
}


// ── [신규 추가] 내장형 리프트 단독 테스트 모드 ───────────────────
#if LIFT_TEST_MODE

// 이름 있는 enum으로 선언 (문법 호환성 강화)
enum LiftTestState { LT_IDLE, LT_UP, LT_DOWN };
static LiftTestState _ltState = LT_IDLE;

static long          _ltPrevEncL  = 0, _ltPrevEncR  = 0;
static long          _ltSpdL      = 0, _ltSpdR      = 0;
static unsigned long _ltLastTick  = 0;
static unsigned long _ltLastPrint = 0;

// 메인 main.cpp 등에서 이 모드가 활성화되었을 때 호출하여 강제 루프를 돌릴 수 있는 함수입니다.
void runLiftTestMode() {
  // 최초 1회 초기화 시뮬레이션
  static bool initDone = false;
  if (!initDone) {
    Serial.begin(9600);
    exc.controllerEnable(EXP_ID);
    { unsigned long _t = millis(); while (millis() - _t < 10) { } }  // delay 없이 settle
    exc.resetEncoder(EXP_ID, LIFT_L);
    exc.resetEncoder(EXP_ID, LIFT_R);
    Serial.println(F("=== LIFT INTERNAL TEST MODE ACTIVE ==="));
    Serial.println(F("u=상승  d=하강  s=정지"));
    Serial.println(F("encL/R | spdL/R(tick/10ms) | hL/R(cm)"));
    initDone = true;
  }

  // ── 키 입력 처리 ──
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'u' || c == 'U') {
      Serial.println(F("[UP]"));
      _ltState = LT_UP;
      liftUpStart();
    }
    else if (c == 'd' || c == 'D') {
      Serial.println(F("[DOWN]"));
      _ltState = LT_DOWN;
      liftDownStart();
    }
    else if (c == 's' || c == 'S') {
      Serial.println(F("[STOP]"));
      _ltState = LT_IDLE;
      liftUpRunning = false;
      liftDownRunning = false;
      exc.setMotorPowers(EXP_ID, 0, 0);
    }
  }

  // ── 리프트 상태 업데이트 ──
  if (_ltState == LT_UP)   liftUpTick();
  if (_ltState == LT_DOWN) liftDownTick();

  unsigned long now = millis();

  // ── 10ms 주기: 속도 계산 ──
  if (now - _ltLastTick >= LIFT_TICK_INTERVAL_MS) {
    long encL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long encR = exc.readEncoderCount(EXP_ID, LIFT_R);
    _ltSpdL   = encL - _ltPrevEncL;
    _ltSpdR   = encR - _ltPrevEncR;
    _ltPrevEncL = encL;
    _ltPrevEncR = encR;
    _ltLastTick = now;
  }

  // ── 200ms 주기: 시리얼 모니터 실시간 출력 ──
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
#endif
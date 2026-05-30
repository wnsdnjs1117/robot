/* ============================================================
 * Lift.cpp - 듀얼 리프트 높이 기반 제어 (좌우 독립 정지)
 *   상승: 각 모터가 20cm 도달 시 개별 정지
 *   하강: 각 모터가 5cm 이하 진입 후 비례 타이머 만료 시 개별 정지
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

  prevEncL = curL;
  prevEncR = curR;
  lastTickTime = now;
}

static unsigned long calcNearFloorDur(float h) {
  float remaining = h - (-2.0f);
  float refDist   = LIFT_NEAR_FLOOR_CM - (-2.0f);  // 7cm
  return (unsigned long)(remaining / refDist * LIFT_FLOOR_TIME_MS);
}

// ── 상승 ─────────────────────────────────────────────────────

void liftUp() {
  resetState();
  liftUpRunning = false;

  // LIFT_UP_CLEAR_CM 까지 블로킹 — 먼저 도달한 쪽은 그 자리서 대기
  do {
    int pwrL = (heightL >= LIFT_UP_CLEAR_CM) ? 0 : LIFT_UP_POWER;
    int pwrR = (heightR >= LIFT_UP_CLEAR_CM) ? 0 : LIFT_UP_POWER;
    exc.setMotorPowers(EXP_ID, pwrL, -pwrR);
    delay(LIFT_TICK_INTERVAL_MS);
    updateHeight();
  } while (heightL < LIFT_UP_CLEAR_CM || heightR < LIFT_UP_CLEAR_CM);

  liftUpRunning = true;
  DPRINTLNF(">> [LIFT] 주행 허가 기준 높이 통과 (배경 상승 계속)");
}

void liftUpTick() {
  if (!liftUpRunning) return;
  updateHeight();

  bool doneL = (heightL >= LIFT_MAX_HEIGHT_CM);
  bool doneR = (heightR >= LIFT_MAX_HEIGHT_CM);

  int pwrL = doneL ? 0 : LIFT_UP_POWER;
  int pwrR = doneR ? 0 : LIFT_UP_POWER;
  exc.setMotorPowers(EXP_ID, pwrL, -pwrR);

  if (doneL && doneR) {
    liftUpRunning = false;
    DPRINTLNF(">> [LIFT] 상승 완료");
  }
}

void liftUpWait() {
  if (!liftUpRunning) return;
  lastTickTime = 0;
  DPRINTLNF(">> [LIFT] 상승 완료 대기 중...");
  while (liftUpRunning) {
    liftUpTick();
    delay(LIFT_TICK_INTERVAL_MS);
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

  // 좌측 — 5cm 이하 진입 시 타이머 시작
  if (!nearFloorL && heightL <= LIFT_NEAR_FLOOR_CM) {
    nearFloorL      = true;
    nearFloorDurL   = calcNearFloorDur(heightL);
    nearFloorStartL = now;
    DPRINTLNF(">> [LIFT-L] 바닥 근접 — 타이머 시작");
  }
  // 우측 — 5cm 이하 진입 시 타이머 시작
  if (!nearFloorR && heightR <= LIFT_NEAR_FLOOR_CM) {
    nearFloorR      = true;
    nearFloorDurR   = calcNearFloorDur(heightR);
    nearFloorStartR = now;
    DPRINTLNF(">> [LIFT-R] 바닥 근접 — 타이머 시작");
  }

  // 타이머 만료 → 개별 정지
  if (nearFloorL && !stoppedL && (now - nearFloorStartL >= nearFloorDurL)) {
    stoppedL = true;
    DPRINTLNF(">> [LIFT-L] 하강 완료");
  }
  if (nearFloorR && !stoppedR && (now - nearFloorStartR >= nearFloorDurR)) {
    stoppedR = true;
    DPRINTLNF(">> [LIFT-R] 하강 완료");
  }

  int pwrL = stoppedL ? 0 : LIFT_DOWN_POWER;
  int pwrR = stoppedR ? 0 : LIFT_DOWN_POWER;
  exc.setMotorPowers(EXP_ID, -pwrL, pwrR);

  // 양쪽 모두 완료
  if (stoppedL && stoppedR) {
    exc.resetEncoder(EXP_ID, LIFT_L);
    exc.resetEncoder(EXP_ID, LIFT_R);
    heightL = 0;
    heightR = 0;
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
    delay(LIFT_TICK_INTERVAL_MS);
  }
  DPRINTLNF(">> [LIFT] 착지 확인 완료");
}

void liftDownUntilClear() {
  liftDownStart();
  while (heightL > LIFT_DOWN_CLEAR_CM || heightR > LIFT_DOWN_CLEAR_CM) {
    liftDownTick();
    delay(LIFT_TICK_INTERVAL_MS);
  }
  DPRINTLNF(">> [LIFT] 주행 허가 기준 높이 도달 (배경 하강 계속)");
}

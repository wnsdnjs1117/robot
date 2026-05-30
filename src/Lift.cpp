/* ============================================================
 * Lift.cpp - 듀얼 리프트 높이 기반 제어
 *   상승: heightL 또는 heightR >= 20cm 시 정지
 *   하강: height <= 5cm 진입 후 LIFT_FLOOR_TIME_MS 경과 시 정지
 * ============================================================ */
#include "Lift.h"
#include "Config.h"

static const int DIR_L =  1;  // 양수 = 상승
static const int DIR_R = -1;

EXPANSION exc;
const int EXP_ID = 1;
const int LIFT_L = 1;
const int LIFT_R = 2;

float heightL = 0;
float heightR = 0;

static bool liftUpRunning    = false;
static bool liftDownRunning  = false;
static bool nearFloorMode    = false;   // 5cm 이하 진입 후 타이머 하강 중
static unsigned long nearFloorStartTime = 0;

static long prevEncL = 0;
static long prevEncR = 0;
static unsigned long lastTickTime = 0;

// ── 내부 헬퍼 ────────────────────────────────────────────────

static void resetState() {
  prevEncL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  prevEncR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;
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

// ── 상승 ─────────────────────────────────────────────────────

void liftUp() {
  resetState();
  liftUpRunning = false;

  // LIFT_UP_CLEAR_CM 도달까지 블로킹
  do {
    exc.setMotorPowers(EXP_ID, LIFT_UP_POWER, -LIFT_UP_POWER);
    delay(LIFT_TICK_INTERVAL_MS);
    updateHeight();
  } while (heightL < LIFT_UP_CLEAR_CM && heightR < LIFT_UP_CLEAR_CM);

  liftUpRunning = true;
  DPRINTLNF(">> [LIFT] 주행 허가 기준 높이 통과 (배경 상승 계속)");
}

void liftUpTick() {
  if (!liftUpRunning) return;
  updateHeight();

  if (heightL >= LIFT_MAX_HEIGHT_CM || heightR >= LIFT_MAX_HEIGHT_CM) {
    exc.setMotorPowers(EXP_ID, 0, 0);
    liftUpRunning = false;
    DPRINTLNF(">> [LIFT] 상승 완료");
    return;
  }
  exc.setMotorPowers(EXP_ID, LIFT_UP_POWER, -LIFT_UP_POWER);
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
  nearFloorMode = false;
  liftDownRunning = true;
  DPRINTLNF(">> [LIFT] 하강 시작");
}

void liftDownTick() {
  if (!liftDownRunning) return;
  updateHeight();

  // [1단계] 5cm 이하 진입 시 타이머 시작
  if (!nearFloorMode && (heightL <= LIFT_NEAR_FLOOR_CM || heightR <= LIFT_NEAR_FLOOR_CM)) {
    nearFloorMode = true;
    nearFloorStartTime = millis();
    DPRINTLNF(">> [LIFT] 바닥 근접 — 타이머 하강 시작");
  }

  // [2단계] 타이머 만료 시 정지
  if (nearFloorMode && (millis() - nearFloorStartTime >= LIFT_FLOOR_TIME_MS)) {
    exc.setMotorPowers(EXP_ID, 0, 0);
    exc.resetEncoder(EXP_ID, LIFT_L);
    exc.resetEncoder(EXP_ID, LIFT_R);
    heightL = 0;
    heightR = 0;
    liftDownRunning = false;
    DPRINTLNF(">> [LIFT] 하강 완료");
    return;
  }

  exc.setMotorPowers(EXP_ID, -LIFT_DOWN_POWER, LIFT_DOWN_POWER);
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

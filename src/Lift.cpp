/* ============================================================
 * Lift.cpp - 듀얼 리프트 높이 기반 제어 (좌우 독립 정지 및 실시간 동기화)
 * 상승: 각 모터가 24cm 도달 시 개별 정지 (20cm 이상 시 파워 20으로 감속)
 * 하강: 각 모터가 5cm 이하 진입 후 비례 타이머 만료 시 개별 정지 (파워 10으로 감속)
 * 동기화: 양쪽이 모두 구동 중일 때 높이 편차를 기반으로 파워를 실시간 보정, 한쪽 정지 시 해제
 * 보정: 리프트 계산 높이가 음수(< 0)가 될 경우 0으로 강제 클램핑 처리 및 모터별 수동 밸런스 비율 적용
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

// ── 상승 ─────────────────────────────────────────────────────

void liftUp() {
  resetState();
  liftUpRunning = false;

  // LIFT_UP_CLEAR_CM 까지 블로킹 — 먼저 도달한 쪽은 그 자리서 대기
  do {
    // 1. 높이에 따른 독립 기본 타겟 파워 결정
    int basePwrL = (heightL >= LIFT_UP_CLEAR_CM) ? 0 : ((heightL >= LIFT_UP_SLOW_ZONE_CM) ? LIFT_UP_SLOW_POWER : LIFT_UP_POWER);
    int basePwrR = (heightR >= LIFT_UP_CLEAR_CM) ? 0 : ((heightR >= LIFT_UP_SLOW_ZONE_CM) ? LIFT_UP_SLOW_POWER : LIFT_UP_POWER);

    // 2. 동기화 보정량 계산 (둘 다 살아있을 때만 적용, 한쪽 정지 시 0)
    int syncOffset = 0;
    if (basePwrL > 0 && basePwrR > 0) {
      syncOffset = (int)((heightL - heightR) * LIFT_SYNC_GAIN);
    }

    // 3. 최종 출력 파워 산출 (왼쪽이 더 높으면 왼쪽 파워 감속, 오른쪽 가속)
    int pwrL = basePwrL - syncOffset;
    int pwrR = basePwrR + syncOffset;

    // 모터 멈춤 및 역회전 방지용 최소 파워 클램핑 제한
    if (basePwrL > 0 && pwrL < 5) pwrL = 5;
    if (basePwrR > 0 && pwrR < 5) pwrR = 5;
    if (basePwrL == 0) pwrL = 0;
    if (basePwrR == 0) pwrR = 0;

    // 모터 자체의 하드웨어 출력 한계 비율 추가 보정
    int finalPwrL = (int)(pwrL * LIFT_LEFT_MOTOR_RATIO);
    int finalPwrR = (int)(pwrR * LIFT_RIGHT_MOTOR_RATIO);
    if (basePwrL > 0 && finalPwrL < 5) finalPwrL = 5;
    if (basePwrR > 0 && finalPwrR < 5) finalPwrR = 5;
    if (basePwrL == 0) finalPwrL = 0;
    if (basePwrR == 0) finalPwrR = 0;

    exc.setMotorPowers(EXP_ID, finalPwrL, -finalPwrR);
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

  // 1. 높이에 따른 독립 기본 타겟 파워 결정 (20cm 이상 진입 시 파워 감속)
  int basePwrL = doneL ? 0 : ((heightL >= LIFT_UP_SLOW_ZONE_CM) ? LIFT_UP_SLOW_POWER : LIFT_UP_POWER);
  int basePwrR = doneR ? 0 : ((heightR >= LIFT_UP_SLOW_ZONE_CM) ? LIFT_UP_SLOW_POWER : LIFT_UP_POWER);

  // 2. 동기화 보정량 계산 (둘 다 구동 중일 때만 동작, 한쪽 정지 시 자동 해제)
  int syncOffset = 0;
  if (basePwrL > 0 && basePwrR > 0) {
    syncOffset = (int)((heightL - heightR) * LIFT_SYNC_GAIN);
  }

  // 3. 최종 출력 파워 계산
  int pwrL = basePwrL - syncOffset;
  int pwrR = basePwrR + syncOffset;

  // 최소 파워 하한선 제한 처리
  if (basePwrL > 0 && pwrL < 5) pwrL = 5;
  if (basePwrR > 0 && pwrR < 5) pwrR = 5;
  if (basePwrL == 0) pwrL = 0;
  if (basePwrR == 0) pwrR = 0;

  // 모터 자체의 하드웨어 출력 한계 비율 추가 보정
  int finalPwrL = (int)(pwrL * LIFT_LEFT_MOTOR_RATIO);
  int finalPwrR = (int)(pwrR * LIFT_RIGHT_MOTOR_RATIO);
  if (basePwrL > 0 && finalPwrL < 5) finalPwrL = 5;
  if (basePwrR > 0 && finalPwrR < 5) finalPwrR = 5;
  if (basePwrL == 0) finalPwrL = 0;
  if (basePwrR == 0) finalPwrR = 0;

  exc.setMotorPowers(EXP_ID, finalPwrL, -finalPwrR);

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

  // 1. 높이에 따른 독립 기본 타겟 파워 결정 (5cm 이하 진입 시 파워 감속)
  int basePwrL = stoppedL ? 0 : ((heightL <= LIFT_DOWN_SLOW_ZONE_CM) ? LIFT_DOWN_SLOW_POWER : LIFT_DOWN_POWER);
  int basePwrR = stoppedR ? 0 : ((heightR <= LIFT_DOWN_SLOW_ZONE_CM) ? LIFT_DOWN_SLOW_POWER : LIFT_DOWN_POWER);

  // 2. 동기화 보정량 계산 (하강 제어)
  int syncOffset = 0;
  if (basePwrL > 0 && basePwrR > 0) {
    syncOffset = (int)((heightL - heightR) * LIFT_SYNC_GAIN);
  }

  // 3. 최종 출력 파워 계산 (하강 부호는 최종 구동 함수 인자에서 처리)
  int pwrL = basePwrL + syncOffset;
  int pwrR = basePwrR - syncOffset;

  // 최소 파워 하한선 제한 처리
  if (basePwrL > 0 && pwrL < 5) pwrL = 5;
  if (basePwrR > 0 && pwrR < 5) pwrR = 5;
  if (basePwrL == 0) pwrL = 0;
  if (basePwrR == 0) pwrR = 0;

  // 모터 자체의 하드웨어 출력 한계 비율 추가 보정
  int finalPwrL = (int)(pwrL * LIFT_LEFT_MOTOR_RATIO);
  int finalPwrR = (int)(pwrR * LIFT_RIGHT_MOTOR_RATIO);
  if (basePwrL > 0 && finalPwrL < 5) finalPwrL = 5;
  if (basePwrR > 0 && finalPwrR < 5) finalPwrR = 5;
  if (basePwrL == 0) finalPwrL = 0;
  if (basePwrR == 0) finalPwrR = 0;

  exc.setMotorPowers(EXP_ID, -finalPwrL, finalPwrR);

  // 양쪽 모두 완료
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
/* ============================================================
 * Lift.cpp - 초고속 반응형 듀얼 리프트 동기 제어 (Moving Average 적용)
 * ============================================================ */
#include "Lift.h"
#include <Arduino.h>
#include "Config.h"

const int DIR_L = 1;
const int DIR_R = -1;

EXPANSION exc;
const int EXP_ID = 1;
const int LIFT_L = 1;
const int LIFT_R = 2;

float floatPowerL = 30.0f;
float floatPowerR = 30.0f;
int powerL = 30;
int powerR = 30;

float heightL = 0;
float heightR = 0;
bool isStalledL = false;
bool isStalledR = false;
bool isMaxReachedL = false;
bool isMaxReachedR = false;

static bool liftUpRunning = false;
static bool liftDownRunning = false;

int lowSpeedCounterL = 0;
int lowSpeedCounterR = 0;
int hardJamCounterL = 0;
int hardJamCounterR = 0;

bool readyToDetectStallL = false;
bool readyToDetectStallR = false;
int cruiseCounterL = 0;
int cruiseCounterR = 0;

unsigned long moveStartTime = 0;
unsigned long lastCheckTime = 0;
long prevCurL = 0;
long prevCurR = 0;

long histL[100];
long histR[100];
int histIdx = 0;
int histSize = 10;  

static void liftResetState(int startPowerL, int startPowerR) {
  isStalledL = false;
  isStalledR = false;
  isMaxReachedL = false;
  isMaxReachedR = false;
  lowSpeedCounterL = 0;
  lowSpeedCounterR = 0;
  hardJamCounterL = 0;
  hardJamCounterR = 0;
  
  readyToDetectStallL = false;
  readyToDetectStallR = false;
  cruiseCounterL = 0;
  cruiseCounterR = 0;

  moveStartTime = millis();
  lastCheckTime = 0;

  floatPowerL = (float)startPowerL;
  floatPowerR = (float)startPowerR;
  powerL = startPowerL;
  powerR = startPowerR;

  prevCurL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  prevCurR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;

  histSize = 100 / LIFT_TICK_INTERVAL_MS;
  if (histSize < 1) histSize = 1;
  if (histSize > 100) histSize = 100;

  for (int i = 0; i < histSize; i++) {
    histL[i] = prevCurL;
    histR[i] = prevCurR;
  }
  histIdx = 0;
}

static void updateLiftControlLogic(bool isBlockingPhase) {
  unsigned long currentTime = millis();

  if (currentTime - lastCheckTime >= LIFT_TICK_INTERVAL_MS) {
    long curL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
    long curR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;

    long stepL = curL - prevCurL;
    long stepR = curR - prevCurR;
    heightL += (float)stepL / LIFT_COUNTS_PER_CM;
    heightR += (float)stepR / LIFT_COUNTS_PER_CM;
    prevCurL = curL;
    prevCurR = curR;

    long oldestL = histL[histIdx];
    long oldestR = histR[histIdx];
    long diffL = abs(curL - oldestL);
    long diffR = abs(curR - oldestR);

    histL[histIdx] = curL;
    histR[histIdx] = curR;
    histIdx = (histIdx + 1) % histSize;

    int currentTargetSpeed = LIFT_TARGET_SPEED;
    int currentMaxPower = LIFT_MAX_POWER;

    if (heightL >= 20.0 || heightR >= 20.0) {
      currentTargetSpeed = 70;
      currentMaxPower = 20;
    }

    if (heightL >= LIFT_MAX_HEIGHT_CM) {
      heightL = LIFT_MAX_HEIGHT_CM;
      isMaxReachedL = true;
    }
    if (heightR >= (LIFT_MAX_HEIGHT_CM + LIFT_RIGHT_OFFSET_CM)) {
      heightR = LIFT_MAX_HEIGHT_CM + LIFT_RIGHT_OFFSET_CM;
      isMaxReachedR = true;
    }

    float k_inc = (float)LIFT_TICK_INTERVAL_MS / 100.0f; 

    if (!isMaxReachedL && !isMaxReachedR) {
      if (diffL < currentTargetSpeed) floatPowerL += k_inc;
      else if (diffL > currentTargetSpeed) floatPowerL -= k_inc;

      if (diffR < currentTargetSpeed) floatPowerR += k_inc;
      else if (diffR > currentTargetSpeed) floatPowerR -= k_inc;

      float heightError = heightL - (heightR - LIFT_RIGHT_OFFSET_CM);
      if (heightError > 0.1) {
        floatPowerL -= k_inc;
        floatPowerR += k_inc;
      } else if (heightError < -0.1) {
        floatPowerR -= k_inc;
        floatPowerL += k_inc;
      }
    } else {
      if (!isMaxReachedL) floatPowerL = 20.0f;
      if (!isMaxReachedR) floatPowerR = 20.0f;
    }

    floatPowerL = constrain(floatPowerL, 10.0f, (float)currentMaxPower);
    floatPowerR = constrain(floatPowerR, 10.0f, (float)currentMaxPower);
    powerL = (int)floatPowerL;
    powerR = (int)floatPowerR;

    lastCheckTime = currentTime;

    if (isBlockingPhase && heightL >= LIFT_UP_CLEAR_CM && heightR >= LIFT_UP_CLEAR_CM) {
      liftUpRunning = true;
      return;
    }
  }

  int outPowerL = isMaxReachedL ? 125 : powerL;
  int outPowerR = isMaxReachedR ? -125 : -powerR;
  exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);

  if (isMaxReachedL && isMaxReachedR) {
    exc.setMotorPowers(EXP_ID, 125, -125);
    liftUpRunning = false;
  }
}

void liftUp() {
  liftUpRunning = false;
  DPRINTLNF(">> [LIFT] 상승 시작 (지정 높이 도달 시 주행 시작)");
  liftResetState(30, 30);

  while (!liftUpRunning) {
    updateLiftControlLogic(true);
    if (!liftUpRunning && isMaxReachedL && isMaxReachedR) break;
    delay(5);  
  }
  DPRINTLNF(">> [LIFT] 주행 허가 기준 높이 통과 (배경 상승 계속)");
}

void liftUpTick() {
  if (!liftUpRunning) return;
  updateLiftControlLogic(false);
  if (!liftUpRunning) {
    DPRINTLNF(">> [LIFT] 최고 높이 상승 완료");
  }
}

void liftUpWait() {
  if (!liftUpRunning) return;
  DPRINTLNF(">> [LIFT] 상승 완료 대기 중...");
  lastCheckTime = 0;
  while (liftUpRunning) {
    liftUpTick();
    delay(5);
  }
  DPRINTLNF(">> [LIFT] 상승 확인 완료");
}

void liftDownStart();
void liftDownWait();

void liftDown() {
  liftDownStart();
  liftDownWait();
}

void liftDownStart() {
  DPRINTLNF(">> [LIFT] 하강 시작 (논블로킹 — 주행과 동시)");
  liftResetState(20, 20);
  liftDownRunning = true;
}

void liftDownTick() {
  if (!liftDownRunning) return;

  unsigned long currentTime = millis();

  if (currentTime - lastCheckTime >= LIFT_TICK_INTERVAL_MS) {
    long curL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
    long curR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;

    long stepL = curL - prevCurL;
    long stepR = curR - prevCurR;
    heightL += (float)stepL / LIFT_COUNTS_PER_CM;
    heightR += (float)stepR / LIFT_COUNTS_PER_CM;
    if (heightL < 0) heightL = 0;
    if (heightR < 0) heightR = 0;
    prevCurL = curL;
    prevCurR = curR;

    long oldestL = histL[histIdx];
    long oldestR = histR[histIdx];
    long diffL = abs(curL - oldestL);  
    long diffR = abs(curR - oldestR);

    histL[histIdx] = curL;
    histR[histIdx] = curR;
    histIdx = (histIdx + 1) % histSize;

    unsigned long elapsed = currentTime - moveStartTime;

    // [페이즈 1] Hard Jam 밀착 감지
    if (elapsed > LIFT_GRACE_PERIOD_MS && elapsed <= LIFT_HARD_JAM_PHASE_MS) {
      if (diffL <= LIFT_HARD_JAM_THRESHOLD) {
        if (++hardJamCounterL >= LIFT_HARD_JAM_CONFIRM_COUNT && !isStalledL) {
          isStalledL = true; heightL = 0;
          DPRINTLNF(">> [LIFT] 왼쪽 즉시 정지 (이미 바닥)");
        }
      } else hardJamCounterL = 0;

      if (diffR <= LIFT_HARD_JAM_THRESHOLD) {
        if (++hardJamCounterR >= LIFT_HARD_JAM_CONFIRM_COUNT && !isStalledR) {
          isStalledR = true; heightR = 0;
          DPRINTLNF(">> [LIFT] 오른쪽 즉시 정지 (이미 바닥)");
        }
      } else hardJamCounterR = 0;
    }

    // ★ [수정 완료] Config.h의 비율(Ratio) 및 대기 카운트 연동
    float targetCruise = (float)LIFT_TARGET_SPEED * LIFT_CRUISE_SPEED_RATIO;
    
    if (!readyToDetectStallL) {
      if (diffL >= targetCruise) {
        if (++cruiseCounterL >= LIFT_CRUISE_CONFIRM_COUNT) { 
          readyToDetectStallL = true;
          DPRINTLNF(">> [LIFT] 왼쪽 정상 하강 속도 진입 완료 (바닥 탐지 활성화)");
        }
      } else cruiseCounterL = 0; 
      
      if (elapsed > 1000) readyToDetectStallL = true; 
    }

    if (!readyToDetectStallR) {
      if (diffR >= targetCruise) {
        if (++cruiseCounterR >= LIFT_CRUISE_CONFIRM_COUNT) {
          readyToDetectStallR = true;
          DPRINTLNF(">> [LIFT] 오른쪽 정상 하강 속도 진입 완료 (바닥 탐지 활성화)");
        }
      } else cruiseCounterR = 0;
      
      if (elapsed > 1000) readyToDetectStallR = true; 
    }

    // PID 제어부
    float k_inc = (float)LIFT_TICK_INTERVAL_MS / 100.0f;

    if (!isStalledL && !isStalledR) {
      if (diffL < LIFT_TARGET_SPEED) floatPowerL += k_inc;
      else if (diffL > LIFT_TARGET_SPEED) floatPowerL -= k_inc;

      if (diffR < LIFT_TARGET_SPEED) floatPowerR += k_inc;
      else if (diffR > LIFT_TARGET_SPEED) floatPowerR -= k_inc;

      float heightError = heightL - (heightR - LIFT_RIGHT_OFFSET_CM);
      if (heightError > 0.1) {
        floatPowerL += k_inc; floatPowerR -= k_inc;
      } else if (heightError < -0.1) {
        floatPowerR += k_inc; floatPowerL -= k_inc;
      }
    } else {
      if (!isStalledL) floatPowerL = 20.0f;
      if (!isStalledR) floatPowerR = 20.0f;
    }

    floatPowerL = constrain(floatPowerL, 10.0f, (float)LIFT_MAX_POWER);
    floatPowerR = constrain(floatPowerR, 10.0f, (float)LIFT_MAX_POWER);
    powerL = (int)floatPowerL;
    powerR = (int)floatPowerR;

    // [페이즈 2] 정상 바닥 착지 감지
    if (readyToDetectStallL) {
      if (diffL < LIFT_DOWN_STALL_THRESHOLD) {
        if (++lowSpeedCounterL >= LIFT_DOWN_EMERGENCY_COUNT && !isStalledL) {
          isStalledL = true; heightL = 0;
          DPRINTLNF(">> [LIFT] 왼쪽 바닥 착지 확인");
        }
      } else lowSpeedCounterL = 0;
    }

    if (readyToDetectStallR) {
      if (diffR < LIFT_DOWN_STALL_THRESHOLD) {
        if (++lowSpeedCounterR >= LIFT_DOWN_EMERGENCY_COUNT && !isStalledR) {
          isStalledR = true; heightR = 0;
          DPRINTLNF(">> [LIFT] 오른쪽 바닥 착지 확인");
        }
      } else lowSpeedCounterR = 0;
    }

    lastCheckTime = currentTime;

    if (isStalledL && isStalledR) {
      exc.setMotorPowers(EXP_ID, -125, 125);
      exc.resetEncoder(EXP_ID, LIFT_L);
      exc.resetEncoder(EXP_ID, LIFT_R);
      heightL = 0; heightR = 0;
      liftDownRunning = false;
      DPRINTLNF(">> [LIFT] 하강 완전 종료");
      return;
    }
  }

  int outPowerL = isStalledL ? -125 : -powerL;
  int outPowerR = isStalledR ? 125 : powerR;
  exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);
}

void liftDownWait() {
  if (!liftDownRunning) return;
  DPRINTLNF(">> [LIFT] 착지 대기 중...");
  lastCheckTime = 0;
  while (liftDownRunning) {
    liftDownTick();
    delay(5);
  }
  DPRINTLNF(">> [LIFT] 착지 확인 완료");
}

void liftDownUntilClear() {
  liftDownStart();
  // ★ [수정 완료] 하강 허용 높이가 1cm로 변경되었습니다!
  while (heightL > LIFT_DOWN_CLEAR_CM || heightR > LIFT_DOWN_CLEAR_CM) {
    liftDownTick();
    delay(5);
  }
  DPRINTLNF(">> [LIFT] 주행 허가 기준 높이 도달 (배경 하강 계속)");
}
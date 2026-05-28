/* ============================================================
 * Lift.cpp - 듀얼 리프트 동기 제어 (개별 비상 정지 포함)
 * ============================================================ */
#include "Lift.h"

#include <Arduino.h>

#include "Config.h"

// [엔코더 방향 설정]
const int DIR_L = 1;
const int DIR_R = -1;  // 우측 리프트 방향 반전 보정

EXPANSION exc;
const int EXP_ID = 1;
const int LIFT_L = 1;
const int LIFT_R = 2;

// ========================================================================
// ★ [사용자 설정 파라미터 제어 구역]
// ========================================================================
const float MAX_HEIGHT_LIMIT = 24.0;  // 기준 최대 상승 제한 높이 (cm)
const float RIGHT_OFFSET = 0.6;       // 우측 리프트 추가 상승 오차 조정값 (cm)

const int DEFAULT_TARGET_SPEED = 220;  // 기본 목표 속도 (220)
const int DOWN_STALL_THRESHOLD = 100;  // 하강 시 정상 스톨 감지 기준 속도 (100으로 변경)
const int DEFAULT_MAX_POWER = 50;      // 기본 최대 파워 제한 (60)
const float LIFT_COUNTS_PER_CM = 200.0;

// 비상 정지 기준 상수
const int EMERGENCY_SPEED_LIMIT = 60;         // 비상 정지 유발 속도 (60 이하)
const int UP_EMERGENCY_DURATION_COUNT = 10;   // 상승 시 걸림 판정 시간 (1초 - 무거운 짐 대응)
const int DOWN_EMERGENCY_DURATION_COUNT = 5;  // 하강 시 걸림 판정 시간 (0.5초 - 바닥 뚜둑 소리 최소화)
// ========================================================================

// 글로벌 초기값
int powerL = 30;
int powerR = 30;

float heightL = 0;
float heightR = 0;
bool isAccelDone = false;
bool isStalledL = false;
bool isStalledR = false;

// 상승 한계 도달 플래그
bool isMaxReachedL = false;
bool isMaxReachedR = false;

static bool liftUpRunning = false;

// 저속 상태 유지를 카운트하기 위한 누적 변수
int lowSpeedCounterL = 0;
int lowSpeedCounterR = 0;

unsigned long moveStartTime = 0;
unsigned long lastCheckTime = 0;
long prevCountL = 0;
long prevCountR = 0;

// ── 내부 헬퍼 ────────────────────────────────────────────────
static void liftResetState(int startPowerL, int startPowerR) {
  isAccelDone = false;
  isStalledL = false;
  isStalledR = false;
  isMaxReachedL = false;
  isMaxReachedR = false;
  lowSpeedCounterL = 0;
  lowSpeedCounterR = 0;
  moveStartTime = millis();
  lastCheckTime = 0;
  prevCountL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  prevCountR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;
  powerL = startPowerL;
  powerR = startPowerR;
}

// ============================================================
// liftUp() : 리프트를 MAX_HEIGHT_LIMIT(24cm)까지 올린다.
//            heightL >= 15.0cm 가 될 때까지 블로킹 후 반환.
//            (로봇은 15cm 이상 든 이후에야 주행 가능)
// ============================================================
void liftUp() {
  liftUpRunning = false;
  Serial.println(F(">> [LIFT] 상승 시작 (15cm 도달 시 즉시 반환)"));

  liftResetState(30, 30);

  while (true) {
    unsigned long currentTime = millis();
    long rawL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long rawR = exc.readEncoderCount(EXP_ID, LIFT_R);
    long curL = rawL * DIR_L;
    long curR = rawR * DIR_R;

    if (currentTime - lastCheckTime >= 100) {
      long dL = curL - prevCountL;
      long dR = curR - prevCountR;
      heightL += (float)dL / LIFT_COUNTS_PER_CM;
      heightR += (float)dR / LIFT_COUNTS_PER_CM;
      long diffL = abs(dL);
      long diffR = abs(dR);

      int currentTargetSpeed = DEFAULT_TARGET_SPEED;
      int currentMaxPower = DEFAULT_MAX_POWER;

      // 20cm 이상 → 감속
      if (heightL >= 20.0 || heightR >= 20.0) {
        currentTargetSpeed = 70;
        currentMaxPower = 20;
      }

      // 한계 도달 감시
      if (heightL >= MAX_HEIGHT_LIMIT) {
        heightL = MAX_HEIGHT_LIMIT;
        isMaxReachedL = true;
      }
      if (heightR >= (MAX_HEIGHT_LIMIT + RIGHT_OFFSET)) {
        heightR = MAX_HEIGHT_LIMIT + RIGHT_OFFSET;
        isMaxReachedR = true;
      }

      // 속도 추종 + 좌우 높이 편차 보정
      if (!isStalledL && !isStalledR && !isMaxReachedL && !isMaxReachedR) {
        if (diffL < currentTargetSpeed)
          powerL++;
        else if (diffL > currentTargetSpeed)
          powerL--;
        if (diffR < currentTargetSpeed)
          powerR++;
        else if (diffR > currentTargetSpeed)
          powerR--;

        float heightError = heightL - (heightR - RIGHT_OFFSET);
        if (heightError > 0.1) {
          powerL--;
          powerR++;
        } else if (heightError < -0.1) {
          powerR--;
          powerL++;
        }
      } else {
        if (!isStalledL && !isMaxReachedL) powerL = 20;
        if (!isStalledR && !isMaxReachedR) powerR = 20;
      }

      powerL = constrain(powerL, 10, currentMaxPower);
      powerR = constrain(powerR, 10, currentMaxPower);

      // 비상 저속 감시 (상승 중 걸림 감지)
      if (currentTime - moveStartTime > 200) {
        if (diffL <= EMERGENCY_SPEED_LIMIT) {
          if (++lowSpeedCounterL >= UP_EMERGENCY_DURATION_COUNT && !isStalledL) {
            isStalledL = true;
            Serial.println(F(">> [LIFT] 왼쪽 비상 정지 (상승 중 저속)"));
          }
        } else {
          lowSpeedCounterL = 0;
        }
        if (diffR <= EMERGENCY_SPEED_LIMIT) {
          if (++lowSpeedCounterR >= UP_EMERGENCY_DURATION_COUNT && !isStalledR) {
            isStalledR = true;
            Serial.println(F(">> [LIFT] 오른쪽 비상 정지 (상승 중 저속)"));
          }
        } else {
          lowSpeedCounterR = 0;
        }
      }

      prevCountL = curL;
      prevCountR = curR;
      lastCheckTime = currentTime;

      // LIFT_UP_CLEAR_CM 도달 → 논블로킹으로 전환 후 즉시 반환 (모터 계속 상승)
      if (heightL >= LIFT_UP_CLEAR_CM && heightR >= LIFT_UP_CLEAR_CM) {
        liftUpRunning = true;
        Serial.println(F(">> [LIFT] 주행 허가 (15cm 도달, 배경 상승 계속)"));
        return;
      }

      Serial.print(F("  [UP] L="));
      Serial.print(heightL, 1);
      Serial.print(F("cm R="));
      Serial.print(heightR, 1);
      Serial.println(F("cm"));
    }

    // 모터 출력
    int outPowerL = (isStalledL || isMaxReachedL) ? 125 : powerL;
    int outPowerR = (isStalledR || isMaxReachedR) ? -125 : -powerR;
    exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);

    // 비상: 양쪽 스톨/한계 → 브레이크
    if ((isMaxReachedL || isStalledL) && (isMaxReachedR || isStalledR)) {
      exc.setMotorPowers(EXP_ID, 125, -125);
      Serial.println(F(">> [LIFT] 비상 정지 (상승 중)"));
      return;
    }

    delay(10);
  }
}

void liftDownStart();
void liftDownWait();

// liftDown() : 블로킹 하강
void liftDown() {
  liftDownStart();
  liftDownWait();
}

// ── 논블로킹 상승 API ────────────────────────────────────────

void liftUpTick() {
  if (!liftUpRunning) return;

  unsigned long currentTime = millis();
  long curL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  long curR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;

  if (currentTime - lastCheckTime >= 100) {
    long dL = curL - prevCountL;
    long dR = curR - prevCountR;
    heightL += (float)dL / LIFT_COUNTS_PER_CM;
    heightR += (float)dR / LIFT_COUNTS_PER_CM;
    long diffL = abs(dL);
    long diffR = abs(dR);

    int currentTargetSpeed = DEFAULT_TARGET_SPEED;
    int currentMaxPower = DEFAULT_MAX_POWER;
    if (heightL >= 20.0 || heightR >= 20.0) {
      currentTargetSpeed = 70;
      currentMaxPower = 20;
    }

    if (heightL >= MAX_HEIGHT_LIMIT) {
      heightL = MAX_HEIGHT_LIMIT;
      isMaxReachedL = true;
    }
    if (heightR >= (MAX_HEIGHT_LIMIT + RIGHT_OFFSET)) {
      heightR = MAX_HEIGHT_LIMIT + RIGHT_OFFSET;
      isMaxReachedR = true;
    }

    if (!isStalledL && !isStalledR && !isMaxReachedL && !isMaxReachedR) {
      if (diffL < currentTargetSpeed)
        powerL++;
      else if (diffL > currentTargetSpeed)
        powerL--;
      if (diffR < currentTargetSpeed)
        powerR++;
      else if (diffR > currentTargetSpeed)
        powerR--;
      float heightError = heightL - (heightR - RIGHT_OFFSET);
      if (heightError > 0.1) {
        powerL--;
        powerR++;
      } else if (heightError < -0.1) {
        powerR--;
        powerL++;
      }
    } else {
      if (!isStalledL && !isMaxReachedL) powerL = 20;
      if (!isStalledR && !isMaxReachedR) powerR = 20;
    }
    powerL = constrain(powerL, 10, currentMaxPower);
    powerR = constrain(powerR, 10, currentMaxPower);

    if (currentTime - moveStartTime > 200) {
      if (diffL <= EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterL >= UP_EMERGENCY_DURATION_COUNT && !isStalledL) {
          isStalledL = true;
          Serial.println(F(">> [LIFT] 왼쪽 비상 정지 (상승 중)"));
        }
      } else {
        lowSpeedCounterL = 0;
      }
      if (diffR <= EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterR >= UP_EMERGENCY_DURATION_COUNT && !isStalledR) {
          isStalledR = true;
          Serial.println(F(">> [LIFT] 오른쪽 비상 정지 (상승 중)"));
        }
      } else {
        lowSpeedCounterR = 0;
      }
    }

    prevCountL = curL;
    prevCountR = curR;
    lastCheckTime = currentTime;
  }

  int outPowerL = (isStalledL || isMaxReachedL) ? 125 : powerL;
  int outPowerR = (isStalledR || isMaxReachedR) ? -125 : -powerR;
  exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);

  if ((isMaxReachedL || isStalledL) && (isMaxReachedR || isStalledR)) {
    exc.setMotorPowers(EXP_ID, 125, -125);
    liftUpRunning = false;
    Serial.println(F(">> [LIFT] 상승 완료 (24cm) — 논블로킹"));
  }
}

void liftUpWait() {
  if (!liftUpRunning) return;
  Serial.println(F(">> [LIFT] 상승 완료 대기 중..."));
  prevCountL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  prevCountR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;
  lastCheckTime = 0;
  while (liftUpRunning) {
    liftUpTick();
    delay(10);
  }
  Serial.println(F(">> [LIFT] 상승 확인 완료"));
}

// ── 논블로킹 하강 3단계 API ───────────────────────────────────

static bool liftDownRunning = false;

void liftDownStart() {
  Serial.println(F(">> [LIFT] 하강 시작 (논블로킹 — 주행과 동시)"));
  liftResetState(30, 30);
  liftDownRunning = true;
}

// 매 루프 틱마다 호출: 하강 모터 제어 1사이클 수행
void liftDownTick() {
  if (!liftDownRunning) return;

  unsigned long currentTime = millis();
  long rawL = exc.readEncoderCount(EXP_ID, LIFT_L);
  long rawR = exc.readEncoderCount(EXP_ID, LIFT_R);
  long curL = rawL * DIR_L;
  long curR = rawR * DIR_R;

  if (currentTime - lastCheckTime >= 100) {
    long dL = curL - prevCountL;
    long dR = curR - prevCountR;
    heightL += (float)dL / LIFT_COUNTS_PER_CM;
    heightR += (float)dR / LIFT_COUNTS_PER_CM;
    if (heightL < 0) heightL = 0;
    if (heightR < 0) heightR = 0;
    long diffL = abs(dL);
    long diffR = abs(dR);

    // 속도 추종 + 좌우 편차 보정 (하강)
    if (!isStalledL && !isStalledR) {
      if (diffL < DEFAULT_TARGET_SPEED)
        powerL++;
      else if (diffL > DEFAULT_TARGET_SPEED)
        powerL--;
      if (diffR < DEFAULT_TARGET_SPEED)
        powerR++;
      else if (diffR > DEFAULT_TARGET_SPEED)
        powerR--;
      float heightError = heightL - (heightR - RIGHT_OFFSET);
      if (heightError > 0.1) {
        powerL++;
        powerR--;
      } else if (heightError < -0.1) {
        powerR++;
        powerL--;
      }
    } else {
      if (!isStalledL) powerL = 20;
      if (!isStalledR) powerR = 20;
    }
    powerL = constrain(powerL, 10, DEFAULT_MAX_POWER);
    powerR = constrain(powerR, 10, DEFAULT_MAX_POWER);

    // 가속 완료 확인
    if (!isAccelDone && currentTime - moveStartTime > 200 && diffL >= DEFAULT_TARGET_SPEED * 0.9) isAccelDone = true;

    // 바닥 스톨 감지 (정상: 가속 후 감속) - 마찰 오작동 방지를 위해 DOWN_STALL_THRESHOLD 적용
    if (isAccelDone) {
      if (diffL < DOWN_STALL_THRESHOLD && !isStalledL) {
        isStalledL = true;
        heightL = 0;
      }
      if (diffR < DOWN_STALL_THRESHOLD && !isStalledR) {
        isStalledR = true;
        heightR = 0;
      }
    }

    // 비상 정지: DOWN_EMERGENCY_DURATION_COUNT 적용 (빠른 감지로 뚜둑 소리 방지)
    if (currentTime - moveStartTime > 200) {
      if (diffL < EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterL >= DOWN_EMERGENCY_DURATION_COUNT && !isStalledL) {
          isStalledL = true;
          heightL = 0;
          Serial.println(F(">> [LIFT] 왼쪽 비상 정지 (이미 바닥)"));
        }
      } else {
        lowSpeedCounterL = 0;
      }
      if (diffR < EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterR >= DOWN_EMERGENCY_DURATION_COUNT && !isStalledR) {
          isStalledR = true;
          heightR = 0;
          Serial.println(F(">> [LIFT] 오른쪽 비상 정지 (이미 바닥)"));
        }
      } else {
        lowSpeedCounterR = 0;
      }
    }

    prevCountL = curL;
    prevCountR = curR;
    lastCheckTime = currentTime;

    // 양쪽 착지 → 브레이크 + 영점 리셋
    if (isStalledL && isStalledR) {
      exc.setMotorPowers(EXP_ID, -125, 125);
      exc.resetEncoder(EXP_ID, LIFT_L);
      exc.resetEncoder(EXP_ID, LIFT_R);
      heightL = 0;
      heightR = 0;
      liftDownRunning = false;
      Serial.println(F(">> [LIFT] 하강 완료 (0cm) — 논블로킹"));
      return;
    }
  }

  // 모터 출력
  int outPowerL = isStalledL ? -125 : -powerL;
  int outPowerR = isStalledR ? 125 : powerR;
  exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);
}

// 하강이 완전히 끝날 때까지 블로킹 대기
void liftDownWait() {
  if (!liftDownRunning) return;
  Serial.println(F(">> [LIFT] 착지 대기 중..."));
  // prevCount 재동기화: 틱 공백 구간 동안의 누적 이동 반영
  prevCountL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  prevCountR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;
  lastCheckTime = 0;
  while (liftDownRunning) {
    liftDownTick();
    delay(10);
  }
  Serial.println(F(">> [LIFT] 착지 확인 완료"));
}

// 10cm 이하 도달 시 즉시 반환 (착지는 liftDownWait로 완료)
void liftDownUntilClear() {
  liftDownStart();
  while (heightL > LIFT_DOWN_CLEAR_CM || heightR > LIFT_DOWN_CLEAR_CM) {
    liftDownTick();
    delay(10);
  }
  Serial.println(F(">> [LIFT] 주행 허가 (10cm 이하)"));
}
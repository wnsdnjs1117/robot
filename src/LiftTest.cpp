/* ========================================================================
 * LiftTest.cpp - 리프트 개별 독립 비상 정지(2초 저속 감시) 완벽판
 * ======================================================================== */
#include "LiftTest.h"

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
const float RIGHT_OFFSET     = 0.6;   // 우측 리프트 추가 상승 오차 조정값 (cm)

const int DEFAULT_TARGET_SPEED = 220;  // 기본 목표 속도
const int STALL_THRESHOLD      = 140;
const int DEFAULT_MAX_POWER    = 50;   // 기본 최대 파워 제한
const float COUNTS_PER_CM     = 200.0;

// 비상 정지 기준 상수
const int EMERGENCY_SPEED_LIMIT    = 60;   // 비상 정지 유발 속도 (60 이하)
const int EMERGENCY_DURATION_COUNT = 10;   // 100ms × 10회 = 1000ms (1초)
// ========================================================================

// 글로벌 상태 변수
int powerL = 30;
int powerR = 30;

float heightL = 0;
float heightR = 0;
bool isAccelDone   = false;
bool isStalledL    = false;
bool isStalledR    = false;
bool isMaxReachedL = false;
bool isMaxReachedR = false;

int  lowSpeedCounterL = 0;
int  lowSpeedCounterR = 0;

unsigned long moveStartTime = 0;
unsigned long lastCheckTime = 0;
long prevCountL = 0;
long prevCountR = 0;

// 논블로킹 상승/하강 진행 중 플래그
static bool liftUpRunning   = false;
static bool liftDownRunning = false;

// ============================================================
// 내부 헬퍼: 리프트 상태 변수 초기화
// ============================================================
static void liftResetState(int startPowerL, int startPowerR) {
  liftUpRunning    = false;
  liftDownRunning  = false;
  isAccelDone      = false;
  isStalledL       = false;
  isStalledR       = false;
  isMaxReachedL    = false;
  isMaxReachedR    = false;
  lowSpeedCounterL = 0;
  lowSpeedCounterR = 0;
  moveStartTime    = millis();
  lastCheckTime    = 0;
  prevCountL       = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
  prevCountR       = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;
  powerL           = startPowerL;
  powerR           = startPowerR;
}

// ============================================================
// liftUpStart() : 논블로킹 상승 시작 (즉시 반환)
// ============================================================
void liftUpStart() {
  Serial.println(F(">> [LIFT] 상승 시작 (논블로킹)"));
  liftResetState(30, 30);
  liftUpRunning = true;
}

// ============================================================
// liftUpTick() : 매 루프 틱마다 호출 — 상승 모터 제어 1사이클
// ============================================================
void liftUpTick() {
  if (!liftUpRunning) return;

  unsigned long currentTime = millis();
  long rawL = exc.readEncoderCount(EXP_ID, LIFT_L);
  long rawR = exc.readEncoderCount(EXP_ID, LIFT_R);
  long curL = rawL * DIR_L;
  long curR = rawR * DIR_R;

  if (currentTime - lastCheckTime >= 100) {
    long dL = curL - prevCountL;
    long dR = curR - prevCountR;
    heightL += (float)dL / COUNTS_PER_CM;
    heightR += (float)dR / COUNTS_PER_CM;
    long diffL = abs(dL);
    long diffR = abs(dR);

    int currentTargetSpeed = DEFAULT_TARGET_SPEED;
    int currentMaxPower    = DEFAULT_MAX_POWER;

    // 20cm 이상 → 감속
    if (heightL >= 20.0 || heightR >= 20.0) {
      currentTargetSpeed = 70;
      currentMaxPower    = 20;
    }

    // 한계 도달 감시
    if (heightL >= MAX_HEIGHT_LIMIT) {
      heightL = MAX_HEIGHT_LIMIT;
      if (!isMaxReachedL) { isMaxReachedL = true; Serial.println(F(">> [LIFT] 왼쪽 상한 도달")); }
    }
    if (heightR >= (MAX_HEIGHT_LIMIT + RIGHT_OFFSET)) {
      heightR = MAX_HEIGHT_LIMIT + RIGHT_OFFSET;
      if (!isMaxReachedR) { isMaxReachedR = true; Serial.println(F(">> [LIFT] 오른쪽 상한 도달")); }
    }

    // 속도 추종 + 좌우 높이 편차 보정
    if (!isStalledL && !isStalledR && !isMaxReachedL && !isMaxReachedR) {
      if (diffL < currentTargetSpeed) powerL++;
      else if (diffL > currentTargetSpeed) powerL--;
      if (diffR < currentTargetSpeed) powerR++;
      else if (diffR > currentTargetSpeed) powerR--;

      float heightError = heightL - (heightR - RIGHT_OFFSET);
      if (heightError > 0.1)       { powerL--; powerR++; }
      else if (heightError < -0.1) { powerR--; powerL++; }
    } else {
      if (!isStalledL && !isMaxReachedL) powerL = 20;
      if (!isStalledR && !isMaxReachedR) powerR = 20;
    }

    powerL = constrain(powerL, 10, currentMaxPower);
    powerR = constrain(powerR, 10, currentMaxPower);

    // 비상 저속 감시 (상승 중 걸림 감지)
    if (currentTime - moveStartTime > 200) {
      if (diffL <= EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterL >= EMERGENCY_DURATION_COUNT && !isStalledL) {
          isStalledL = true;
          Serial.println(F(">> [LIFT] 왼쪽 비상 정지 (상승 중 저속)"));
        }
      } else { lowSpeedCounterL = 0; }

      if (diffR <= EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterR >= EMERGENCY_DURATION_COUNT && !isStalledR) {
          isStalledR = true;
          Serial.println(F(">> [LIFT] 오른쪽 비상 정지 (상승 중 저속)"));
        }
      } else { lowSpeedCounterR = 0; }
    }

    prevCountL    = curL;
    prevCountR    = curR;
    lastCheckTime = currentTime;

    // 양쪽 모두 완료 → 브레이크 + 플래그 해제
    if ((isMaxReachedL || isStalledL) && (isMaxReachedR || isStalledR)) {
      exc.setMotorPowers(EXP_ID, 125, -125);
      liftUpRunning = false;
      Serial.println(F(">> [LIFT] 상승 완료 (24cm) — 논블로킹"));
      return;
    }
  }

  // 매 틱 모터 출력
  int outPowerL = (isStalledL || isMaxReachedL) ?  125 :  powerL;
  int outPowerR = (isStalledR || isMaxReachedR) ? -125 : -powerR;
  exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);
}

// ============================================================
// liftUpWait() : 상승이 완전히 끝날 때까지 블로킹 대기
// ============================================================
void liftUpWait() {
  if (!liftUpRunning) return;
  Serial.println(F(">> [LIFT] 24cm 도달 대기 중..."));
  while (liftUpRunning) { liftUpTick(); delay(10); }
  Serial.println(F(">> [LIFT] 24cm 도달 확인 완료"));
}

// ============================================================
// liftUp() : ★ 양쪽이 15cm 이상 오르면 반환 (빠른 이동 허가)
//            liftUpRunning = true 유지 → exitZone 내 liftActiveTick이 24cm까지 완료
// ============================================================
void liftUp() {
  Serial.println(F(">> [LIFT] 상승 시작 (목표: 24cm, 주행 해제: 15cm)"));
  liftUpStart();
  while (heightL < 15.0 || heightR < 15.0) {
    liftUpTick();
    delay(10);
  }
  Serial.println(F(">> [LIFT] 주행 허가 (15cm 이상) — 리프트 계속 상승 중"));
}

// ============================================================
// 논블로킹 하강 — 3단계 API
//   liftDownStart() → 이동 루프에서 liftDownTick() → liftDownWait()
// ============================================================

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
    heightL += (float)dL / COUNTS_PER_CM;
    heightR += (float)dR / COUNTS_PER_CM;
    if (heightL < 0) heightL = 0;
    if (heightR < 0) heightR = 0;
    long diffL = abs(dL);
    long diffR = abs(dR);

    // 속도 추종 + 좌우 편차 보정 (하강)
    if (!isStalledL && !isStalledR) {
      if (diffL < DEFAULT_TARGET_SPEED) powerL++;
      else if (diffL > DEFAULT_TARGET_SPEED) powerL--;
      if (diffR < DEFAULT_TARGET_SPEED) powerR++;
      else if (diffR > DEFAULT_TARGET_SPEED) powerR--;

      float heightError = heightL - (heightR - RIGHT_OFFSET);
      if (heightError > 0.1)       { powerL++; powerR--; }
      else if (heightError < -0.1) { powerR++; powerL--; }
    } else {
      if (!isStalledL) powerL = 20;
      if (!isStalledR) powerR = 20;
    }
    powerL = constrain(powerL, 10, DEFAULT_MAX_POWER);
    powerR = constrain(powerR, 10, DEFAULT_MAX_POWER);

    // 가속 완료 확인
    if (!isAccelDone && currentTime - moveStartTime > 200 &&
        diffL >= DEFAULT_TARGET_SPEED * 0.9)
      isAccelDone = true;

    // 바닥 스톨 감지 (정상: 가속 후 급감속)
    if (isAccelDone) {
      if (diffL < STALL_THRESHOLD && !isStalledL) { isStalledL = true; heightL = 0; }
      if (diffR < STALL_THRESHOLD && !isStalledR) { isStalledR = true; heightR = 0; }
    }

    // 비상 정지: 1초 연속 저속 → 이미 바닥으로 간주
    if (currentTime - moveStartTime > 200) {
      if (diffL < EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterL >= EMERGENCY_DURATION_COUNT && !isStalledL) {
          isStalledL = true; heightL = 0;
          Serial.println(F(">> [LIFT] 왼쪽 비상 정지 (이미 바닥)"));
        }
      } else { lowSpeedCounterL = 0; }
      if (diffR < EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterR >= EMERGENCY_DURATION_COUNT && !isStalledR) {
          isStalledR = true; heightR = 0;
          Serial.println(F(">> [LIFT] 오른쪽 비상 정지 (이미 바닥)"));
        }
      } else { lowSpeedCounterR = 0; }
    }

    prevCountL    = curL;
    prevCountR    = curR;
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
  int outPowerR = isStalledR ?  125 :  powerR;
  exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);
}

// 하강이 완전히 끝날 때까지 블로킹 대기
void liftDownWait() {
  if (!liftDownRunning) return;
  Serial.println(F(">> [LIFT] 착지 대기 중..."));
  while (liftDownRunning) { liftDownTick(); delay(10); }
  Serial.println(F(">> [LIFT] 착지 확인 완료"));
}

// ============================================================
// liftDown() : ★ 양쪽이 10cm 이하로 내려오면 반환 (빠른 이동 허가)
//              liftDownRunning = true 유지 → exitZone 내 liftActiveTick이 0cm까지 완료
// ============================================================
void liftDown() {
  Serial.println(F(">> [LIFT] 하강 시작 (목표: 0cm, 주행 해제: 10cm)"));
  liftDownStart();
  while (heightL > 10.0 || heightR > 10.0) {
    liftDownTick();
    delay(10);
  }
  Serial.println(F(">> [LIFT] 주행 허가 (10cm 이하) — 리프트 계속 하강 중"));
}

// ============================================================
// liftActiveTick() : 이동 루프에서 호출 — 진행 중인 상승/하강 1틱 처리
//                   (followToCrossing, exitZone 등 이동 루프 내부에서 사용)
// ============================================================
void liftActiveTick() {
  if      (liftUpRunning)   liftUpTick();
  else if (liftDownRunning) liftDownTick();
}

// ============================================================
// runLiftStallTest() : 수동 테스트용 (핀 5 HIGH=상승, LOW=하강)
// ============================================================
void runLiftStallTest() {
  exc.controllerEnable(EXP_ID);
  delay(10);
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  Serial.begin(9600);
  Serial.print(">> [SYSTEM] 제어 시스템 가동 (우측 오프셋: ");
  Serial.print(RIGHT_OFFSET);
  Serial.println("cm, 리프트 개별 독립 비상 정지 모드)");

  int lastState = -1;

  while (true) {
    int sensorState = digitalRead(5);
    unsigned long currentTime = millis();
    long rawL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long rawR = exc.readEncoderCount(EXP_ID, LIFT_R);
    long curL = rawL * DIR_L;
    long curR = rawR * DIR_R;

    if (currentTime - lastCheckTime >= 100) {
      long dL = curL - prevCountL;
      long dR = curR - prevCountR;
      heightL += (float)dL / COUNTS_PER_CM;
      heightR += (float)dR / COUNTS_PER_CM;
      long diffL = abs(dL);
      long diffR = abs(dR);

      int currentTargetSpeed = DEFAULT_TARGET_SPEED;
      int currentMaxPower    = DEFAULT_MAX_POWER;

      if (sensorState == HIGH) {
        if (heightL >= 20.0 || heightR >= 20.0) {
          currentTargetSpeed = 70; currentMaxPower = 20;
        }
        if (heightL >= MAX_HEIGHT_LIMIT) {
          heightL = MAX_HEIGHT_LIMIT;
          if (!isMaxReachedL) { isMaxReachedL = true; Serial.println(">> [LIMIT] 왼쪽 제한"); }
        }
        if (heightR >= (MAX_HEIGHT_LIMIT + RIGHT_OFFSET)) {
          heightR = MAX_HEIGHT_LIMIT + RIGHT_OFFSET;
          if (!isMaxReachedR) { isMaxReachedR = true; Serial.println(">> [LIMIT] 오른쪽 제한"); }
        }
      }

      if (!isStalledL && !isStalledR && !isMaxReachedL && !isMaxReachedR) {
        if (diffL < currentTargetSpeed) powerL++;
        else if (diffL > currentTargetSpeed) powerL--;
        if (diffR < currentTargetSpeed) powerR++;
        else if (diffR > currentTargetSpeed) powerR--;
        float heightError = heightL - (heightR - RIGHT_OFFSET);
        if (sensorState == HIGH) {
          if (heightError > 0.1)       { powerL--; powerR++; }
          else if (heightError < -0.1) { powerR--; powerL++; }
        } else {
          if (heightError > 0.1)       { powerL++; powerR--; }
          else if (heightError < -0.1) { powerR++; powerL--; }
        }
      } else {
        if (!isStalledL && !isMaxReachedL) powerL = 20;
        if (!isStalledR && !isMaxReachedR) powerR = 20;
      }

      if (powerL > currentMaxPower) powerL = currentMaxPower;
      if (powerR > currentMaxPower) powerR = currentMaxPower;
      if (powerL < 10) powerL = 10;
      if (powerR < 10) powerR = 10;

      if (currentTime - moveStartTime > 200) {
        if (diffL <= EMERGENCY_SPEED_LIMIT) {
          lowSpeedCounterL++;
          if (lowSpeedCounterL >= EMERGENCY_DURATION_COUNT && !isStalledL) {
            isStalledL = true; heightL = 0; Serial.println(">> [EMERGENCY] 왼쪽 비상 정지");
          }
        } else { lowSpeedCounterL = 0; }
        if (diffR <= EMERGENCY_SPEED_LIMIT) {
          lowSpeedCounterR++;
          if (lowSpeedCounterR >= EMERGENCY_DURATION_COUNT && !isStalledR) {
            isStalledR = true; heightR = 0; Serial.println(">> [EMERGENCY] 오른쪽 비상 정지");
          }
        } else { lowSpeedCounterR = 0; }
      }

      if (sensorState == LOW) {
        if (!isAccelDone && currentTime - moveStartTime > 200 &&
            diffL >= currentTargetSpeed * 0.9)
          isAccelDone = true;
        if (isAccelDone) {
          if (diffL < STALL_THRESHOLD && !isStalledL) { isStalledL = true; heightL = 0; }
          if (diffR < STALL_THRESHOLD && !isStalledR) { isStalledR = true; heightR = 0; }
        }
      }

      Serial.print("L: Hgt="); Serial.print(heightL, 1);
      Serial.print("cm Spd="); Serial.print(diffL);
      Serial.print(" Pwr=");   Serial.print(powerL);
      Serial.print(" | R: Hgt="); Serial.print(heightR, 1);
      Serial.print("cm Spd=");    Serial.print(diffR);
      Serial.print(" Pwr=");      Serial.println(powerR);

      prevCountL    = curL;
      prevCountR    = curR;
      lastCheckTime = currentTime;
    }

    if (sensorState == HIGH) {
      if (lastState != HIGH) {
        isAccelDone = false; isStalledL = false; isStalledR = false;
        isMaxReachedL = false; isMaxReachedR = false;
        lowSpeedCounterL = 0; lowSpeedCounterR = 0;
        moveStartTime = millis(); lastState = HIGH;
        powerL = 30; powerR = 30;
      }
      int outPowerL = isStalledL || isMaxReachedL ?  125 :  powerL;
      int outPowerR = isStalledR || isMaxReachedR ? -125 : -powerR;
      exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);
    } else {
      if (lastState != LOW) {
        isAccelDone = false; isStalledL = false; isStalledR = false;
        isMaxReachedL = false; isMaxReachedR = false;
        lowSpeedCounterL = 0; lowSpeedCounterR = 0;
        moveStartTime = millis(); lastState = LOW;
        powerL = 30; powerR = 30;
      }
      int outPowerL = isStalledL ? -125 : -powerL;
      int outPowerR = isStalledR ?  125 :  powerR;
      exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);
    }

    delay(10);
  }
}

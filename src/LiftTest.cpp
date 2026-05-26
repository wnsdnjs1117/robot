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
const float RIGHT_OFFSET = 0.6;       // 우측 리프트 추가 상승 오차 조정값 (cm)

const int DEFAULT_TARGET_SPEED = 220;  // 기본 목표 속도 (220)
const int STALL_THRESHOLD = 140;
const int DEFAULT_MAX_POWER = 50;  // 기본 최대 파워 제한 (60)
const float COUNTS_PER_CM = 200.0;

// 비상 정지 기준 상수
const int EMERGENCY_SPEED_LIMIT = 60;     // 비상 정지 유발 속도 (60 이하)
const int EMERGENCY_DURATION_COUNT = 10;  // 100ms * 10회 = 1000ms (1초)
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

// 저속 상태 유지를 카운트하기 위한 누적 변수
int lowSpeedCounterL = 0;
int lowSpeedCounterR = 0;

unsigned long moveStartTime = 0;
unsigned long lastCheckTime = 0;
long prevCountL = 0;
long prevCountR = 0;

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

      // 높이 계산
      heightL += (float)dL / COUNTS_PER_CM;
      heightR += (float)dR / COUNTS_PER_CM;

      long diffL = abs(dL);
      long diffR = abs(dR);

      // 동적 제어 변수
      int currentTargetSpeed = DEFAULT_TARGET_SPEED;
      int currentMaxPower = DEFAULT_MAX_POWER;

      if (sensorState == HIGH) {  // 상승 중일 때
        // 20cm 넘으면 감속 모드 진입
        if (heightL >= 20.0 || heightR >= 20.0) {
          currentTargetSpeed = 70;
          currentMaxPower = 20;
        }

        // 왼쪽 정지 감시 (기준 높이)
        if (heightL >= MAX_HEIGHT_LIMIT) {
          heightL = MAX_HEIGHT_LIMIT;
          if (!isMaxReachedL) {
            isMaxReachedL = true;
            Serial.println(">> [LIMIT] 왼쪽 리프트 제한 도달 -> 정지(125)");
          }
        }

        // 오른쪽 정지 감시 (기준 높이 + 상단 설정 오차값)
        if (heightR >= (MAX_HEIGHT_LIMIT + RIGHT_OFFSET)) {
          heightR = MAX_HEIGHT_LIMIT + RIGHT_OFFSET;
          if (!isMaxReachedR) {
            isMaxReachedR = true;
            Serial.print(">> [LIMIT] 오른쪽 리프트 제한 도달 (");
            Serial.print(heightR, 1);
            Serial.println("cm) -> 정지(125)");
          }
        }
      }

      // [실시간 속도 + 높이 차이 통합 보정 알고리즘]
      // ★ 한쪽이라도 정지(Stall/Max) 상태가 되면 동기화(보정)를 끄고 남은
      // 리프트만 독립 구동합니다.
      if (!isStalledL && !isStalledR && !isMaxReachedL && !isMaxReachedR) {
        // 1단계: 개별 타겟 속도 추종
        if (diffL < currentTargetSpeed)
          powerL++;
        else if (diffL > currentTargetSpeed)
          powerL--;
        if (diffR < currentTargetSpeed)
          powerR++;
        else if (diffR > currentTargetSpeed)
          powerR--;

        // 2단계: 양쪽 실시간 높이 편차 계산 및 추격 제어
        float heightError = heightL - (heightR - RIGHT_OFFSET);

        if (sensorState == HIGH) {  // [상승 중]
          if (heightError > 0.1) {
            powerL--;
            powerR++;
          } else if (heightError < -0.1) {
            powerR--;
            powerL++;
          }
        } else {  // [하강 중]
          if (heightError > 0.1) {
            powerL++;
            powerR--;
          } else if (heightError < -0.1) {
            powerR++;
            powerL--;
          }
        }
      } else {
        // 한쪽이 먼저 멈춘 경우, 아직 멈추지 않은 쪽은 안전 속도 유지를 위해
        // 기본 파워(20) 제공
        if (!isStalledL && !isMaxReachedL) powerL = 20;
        if (!isStalledR && !isMaxReachedR) powerR = 20;
      }

      // 파워 안전 제한 조건
      if (powerL > currentMaxPower) powerL = currentMaxPower;
      if (powerR > currentMaxPower) powerR = currentMaxPower;
      if (powerL < 10) powerL = 10;
      if (powerR < 10) powerR = 10;

      // [★ 핵심 수정]: 리프트별로 2초 저속 유지를 따로 체크하여 독립 정지 처리
      if (currentTime - moveStartTime > 200) {
        // 1. 왼쪽 리프트 독립 감시
        if (diffL <= EMERGENCY_SPEED_LIMIT) {
          lowSpeedCounterL++;
          if (lowSpeedCounterL >= EMERGENCY_DURATION_COUNT && !isStalledL) {
            isStalledL = true;
            heightL = 0;  // 바닥 안착 영점 조정
            Serial.println(
                ">> [EMERGENCY] 왼쪽 리프트만 2초 저속 감지 -> 개별 비상 정지");
          }
        } else {
          lowSpeedCounterL = 0;
        }

        // 2. 오른쪽 리프트 독립 감시
        if (diffR <= EMERGENCY_SPEED_LIMIT) {
          lowSpeedCounterR++;
          if (lowSpeedCounterR >= EMERGENCY_DURATION_COUNT && !isStalledR) {
            isStalledR = true;
            heightR = 0;  // 바닥 안착 영점 조정
            Serial.println(
                ">> [EMERGENCY] 오른쪽 리프트만 2초 저속 감지 -> 개별 비상 "
                "정지");
          }
        } else {
          lowSpeedCounterR = 0;
        }
      }

      // 기존 하강 부하 감시 알고리즘 (동시 정지 유발 방지를 위해 개별 분리
      // 처리)
      if (sensorState == LOW) {
        if (!isAccelDone && (currentTime - moveStartTime > 200 &&
                             diffL >= currentTargetSpeed * 0.9))
          isAccelDone = true;

        if (isAccelDone) {
          if (diffL < STALL_THRESHOLD && !isStalledL) {
            isStalledL = true;
            heightL = 0;
          }
          if (diffR < STALL_THRESHOLD && !isStalledR) {
            isStalledR = true;
            heightR = 0;
          }
        }
      }

      Serial.print("L: Hgt=");
      Serial.print(heightL, 1);
      Serial.print("cm Spd=");
      Serial.print(diffL);
      Serial.print(" Pwr=");
      Serial.print(powerL);
      Serial.print(" | R: Hgt=");
      Serial.print(heightR, 1);
      Serial.print("cm Spd=");
      Serial.print(diffR);
      Serial.print(" Pwr=");
      Serial.println(powerR);

      prevCountL = curL;
      prevCountR = curR;
      lastCheckTime = currentTime;
    }

    if (sensorState == HIGH) {  // 상승 명령 제어
      if (lastState != HIGH) {
        isAccelDone = false;
        isStalledL = false;
        isStalledR = false;
        isMaxReachedL = false;
        isMaxReachedR = false;
        lowSpeedCounterL = 0;
        lowSpeedCounterR = 0;
        moveStartTime = millis();
        lastState = HIGH;

        powerL = 30;
        powerR = 30;
      }

      // ★ 조건 검사를 통해 정지된 리프트만 125 브레이크 인가
      int outPowerL = isStalledL || isMaxReachedL ? 125 : powerL;
      int outPowerR = isStalledR || isMaxReachedR ? -125 : -powerR;

      exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);

    } else {  // 하강 명령 제어
      if (lastState != LOW) {
        isAccelDone = false;
        isStalledL = false;
        isStalledR = false;
        isMaxReachedL = false;
        isMaxReachedR = false;
        lowSpeedCounterL = 0;
        lowSpeedCounterR = 0;
        moveStartTime = millis();
        lastState = LOW;

        powerL = 30;
        powerR = 30;
      }

      // ★ 조건 검사를 통해 정지된 리프트만 하강 브레이크 인가
      int outPowerL = isStalledL ? -125 : -powerL;
      int outPowerR = isStalledR ? 125 : powerR;

      exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);
    }

    delay(10);
  }
}

// ============================================================
// 내부 헬퍼: 리프트 상태 변수를 초기화하고 모터 방향을 세팅
// ============================================================
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
  Serial.println(F(">> [LIFT] 상승 시작 (목표: 24cm, 주행 해제: 15cm)"));

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
      heightL += (float)dL / COUNTS_PER_CM;
      heightR += (float)dR / COUNTS_PER_CM;
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
          if (++lowSpeedCounterL >= EMERGENCY_DURATION_COUNT && !isStalledL) {
            isStalledL = true;
            Serial.println(F(">> [LIFT] 왼쪽 비상 정지 (상승 중 저속)"));
          }
        } else {
          lowSpeedCounterL = 0;
        }
        if (diffR <= EMERGENCY_SPEED_LIMIT) {
          if (++lowSpeedCounterR >= EMERGENCY_DURATION_COUNT && !isStalledR) {
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

    // 양쪽 모두 한계 도달 or 비상 정지 → 모터 브레이크 후 탈출
    if ((isMaxReachedL || isStalledL) && (isMaxReachedR || isStalledR)) {
      exc.setMotorPowers(EXP_ID, 125, -125);
      Serial.println(F(">> [LIFT] 상승 완료 (24cm)"));
      break;
    }

    // ★ 15cm 이상 올라간 순간부터 함수 반환 가능 — 여기선 끝까지 올리므로 계속
    // 진행
    delay(10);
  }

  // 15cm 미만이면 추가 대기 (안전망 — 정상 흐름에서는 발생 안 함)
  while (heightL < 15.0 && heightR < 15.0) {
    delay(50);
  }
  Serial.println(F(">> [LIFT] 주행 허가 (15cm 이상 확인)"));
}

// ============================================================
// liftDown() : 리프트를 바닥(0cm)까지 내린다.
//              heightL <= 10.0cm 가 될 때까지 블로킹 후 반환.
//              (로봇은 10cm 이하로 내린 이후에야 주행 가능)
// ============================================================
void liftDown() {
  Serial.println(F(">> [LIFT] 하강 시작 (목표: 0cm, 주행 해제: 10cm)"));

  liftResetState(30, 30);

  bool driveReleased = false;  // 10cm 이하 도달 여부 추적

  while (true) {
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
      // 바닥 아래로 내려가지 않도록 클램프
      if (heightL < 0) heightL = 0;
      if (heightR < 0) heightR = 0;
      long diffL = abs(dL);
      long diffR = abs(dR);

      // 10cm 이하 도달 → 주행 해제 로그 1회 출력
      if (!driveReleased && heightL <= 10.0 && heightR <= 10.0) {
        driveReleased = true;
        Serial.println(F(">> [LIFT] 주행 허가 (10cm 이하 확인)"));
      }

      // 속도 추종 + 좌우 높이 편차 보정 (하강 방향)
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

      // 가속 완료 확인 (하강 스톨 감지용)
      if (!isAccelDone && currentTime - moveStartTime > 200 &&
          diffL >= DEFAULT_TARGET_SPEED * 0.9)
        isAccelDone = true;

      // 하강 스톨 감지 (바닥 도달)
      if (isAccelDone) {
        if (diffL < STALL_THRESHOLD && !isStalledL) {
          isStalledL = true;
          heightL = 0;
          Serial.println(F(">> [LIFT] 왼쪽 바닥 도달"));
        }
        if (diffR < STALL_THRESHOLD && !isStalledR) {
          isStalledR = true;
          heightR = 0;
          Serial.println(F(">> [LIFT] 오른쪽 바닥 도달"));
        }
      }

      prevCountL = curL;
      prevCountR = curR;
      lastCheckTime = currentTime;

      Serial.print(F("  [DOWN] L="));
      Serial.print(heightL, 1);
      Serial.print(F("cm R="));
      Serial.print(heightR, 1);
      Serial.println(F("cm"));
    }

    // 모터 출력 (하강 방향)
    int outPowerL = isStalledL ? -125 : -powerL;
    int outPowerR = isStalledR ? 125 : powerR;
    exc.setMotorPowers(EXP_ID, outPowerL, outPowerR);

    // 양쪽 모두 바닥 도달 → 브레이크 후 탈출
    if (isStalledL && isStalledR) {
      exc.setMotorPowers(EXP_ID, -125, 125);
      // 엔코더 영점 리셋 (다음 liftUp() 기준점)
      exc.resetEncoder(EXP_ID, LIFT_L);
      exc.resetEncoder(EXP_ID, LIFT_R);
      heightL = 0;
      heightR = 0;
      Serial.println(F(">> [LIFT] 하강 완료 (0cm)"));
      break;
    }

    delay(10);
  }
}
// ============================================================
// 논블로킹 하강 — 3단계 API
// 사용법:
//   liftDownStart();       // 하강 시작 (즉시 반환)
//   while (주행 루프) {
//     drive(...);
//     liftDownTick();      // 매 틱마다 호출
//     delay(5);
//   }
//   liftDownWait();        // 완전 착지까지 블로킹 대기
// ============================================================

static bool liftDownRunning = false;  // 논블로킹 하강 진행 중 플래그

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
    if (!isAccelDone && currentTime - moveStartTime > 200 &&
        diffL >= DEFAULT_TARGET_SPEED * 0.9)
      isAccelDone = true;

    // 바닥 스톨 감지 (정상: 가속 후 감속)
    if (isAccelDone) {
      if (diffL < STALL_THRESHOLD && !isStalledL) {
        isStalledL = true;
        heightL = 0;
      }
      if (diffR < STALL_THRESHOLD && !isStalledR) {
        isStalledR = true;
        heightR = 0;
      }
    }

    // 비상 정지: 가속 완료 여부와 무관하게
    // 시작 200ms 후에도 속도가 EMERGENCY_SPEED_LIMIT 미만으로
    // EMERGENCY_DURATION_COUNT(10회=1초) 연속이면 이미 바닥으로 간주
    if (currentTime - moveStartTime > 200) {
      if (diffL < EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterL >= EMERGENCY_DURATION_COUNT && !isStalledL) {
          isStalledL = true;
          heightL = 0;
          Serial.println(F(">> [LIFT] 왼쪽 비상 정지 (이미 바닥)"));
        }
      } else {
        lowSpeedCounterL = 0;
      }
      if (diffR < EMERGENCY_SPEED_LIMIT) {
        if (++lowSpeedCounterR >= EMERGENCY_DURATION_COUNT && !isStalledR) {
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
  while (liftDownRunning) {
    liftDownTick();
    delay(10);
  }
  Serial.println(F(">> [LIFT] 착지 확인 완료"));
}
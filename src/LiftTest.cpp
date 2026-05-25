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
const int STALL_THRESHOLD = 120;
const int DEFAULT_MAX_POWER = 60;  // 기본 최대 파워 제한 (60)
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
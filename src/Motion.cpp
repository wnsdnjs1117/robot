/* ============================================================
 * Motion.cpp - 후진 조향 완전 제거 (무조건 일직선 후진) 버전
 * ============================================================ */
#include "Motion.h"

#include "Config.h"

// 엔코더 기반 폐루프 속도 제어
void drive(int l, int r) {
  l = constrain(l, -100, 100);
  r = constrain(r, -100, 100);
  prizm.setMotorSpeeds(-(l * 7), r * 7);  // DPS 단위 변환(*7) 및 M1 반전
}

// 하드웨어 전자 브레이크 (급정지 명령)
void stopAll() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  delay(200);
}

// 제자리 좌회전
void spinLeft90() {
  prizm.resetEncoders();
  drive(-SPIN_SPEED, SPIN_SPEED);
  while (abs(prizm.readEncoderCount(1)) < SPIN_90_COUNTS) {
  }
  stopAll();
}

// 제자리 우회전
void spinRight90() {
  prizm.resetEncoders();
  drive(SPIN_SPEED, -SPIN_SPEED);
  while (abs(prizm.readEncoderCount(1)) < SPIN_90_COUNTS) {
  }
  stopAll();
}

// 단순 직진 후진
void reverseStraight(int counts) {
  prizm.resetEncoders();
  drive(-BACK_SPEED, -BACK_SPEED);
  while (abs(prizm.readEncoderCount(1)) < counts) {
  }
  stopAll();
}

// 센서 값 읽기
void readSensors(int& L, int& C, int& R) {
  L = digitalRead(SENSOR_LEFT);
  C = digitalRead(SENSOR_CENTER);
  R = digitalRead(SENSOR_RIGHT);
  if (INVERT_SENSORS) {
    L = !L;
    C = !C;
    R = !R;
  }
}

bool anyLine(int L, int C, int R) { return (L == 1 || C == 1 || R == 1); }

// 전진 라인트레이싱 (기존 감도 및 조향 유지)
void lineFollowStep(int L, int C, int R) {
  if (L == 1 && C == 0 && R == 0) {
    drive(SPEED - 20, SPEED + 10);
    lastSensorState = 1;
  } else if (L == 1 && C == 1 && R == 0) {
    drive(SPEED - 10, SPEED + 5);
    lastSensorState = 1;
  } else if (L == 0 && C == 0 && R == 1) {
    drive(SPEED + 10, SPEED - 20);
    lastSensorState = 2;
  } else if (L == 0 && C == 1 && R == 1) {
    drive(SPEED + 5, SPEED - 10);
    lastSensorState = 2;
  } else if (C == 1) {
    if (lastSensorState == 1) {
      drive(SPEED + 4, SPEED - 4);
      lastSensorState = 0;
    } else if (lastSensorState == 2) {
      drive(SPEED - 4, SPEED + 4);
      lastSensorState = 0;
    } else {
      drive(SPEED, SPEED);
    }
  } else if (L == 0 && C == 0 && R == 0) {
    if (lastSensorState == 1)
      drive(SPEED - 16, SPEED + 6);
    else if (lastSensorState == 2)
      drive(SPEED + 6, SPEED - 16);
    else
      drive(SPEED, SPEED);
  } else {
    drive(SPEED, SPEED);
  }
}

// ★ [대폭 수정] 후진 시 조향 완전 삭제
// 센서값(L, C, R)에 관계없이 무조건 완전한 일직선 속도로만 밀고 나갑니다.
void lineFollowStepReverse(int L, int C, int R) {
  drive(-BACK_SPEED, -BACK_SPEED);
}

// 교차로 감지 (연속 확정 방식)
bool detectCrossing(int L, int C, int R) {
  bool isCross = (L == 1 && C == 1 && R == 1);
  if (isCross)
    crossingStable++;
  else
    crossingStable = 0;

  if (!isCross) {
    crossingArmed = true;
    return false;
  }
  if (crossingArmed && crossingStable >= CROSS_CONFIRM) {
    crossingArmed = false;
    return true;
  }
  return false;
}
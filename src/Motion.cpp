/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 비례식 각도 제어 구현부
 * ============================================================ */
#include "Motion.h"
#include "Config.h"
#include "MapRouter.h" 
#include "Lift.h" // ★ 논블로킹 리프트 제어를 위해 추가

bool enableEdgeSteering = false;

// ── [0] Non-Blocking 비동기 대기 함수 ───────────────────────────

void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    liftUpTick();
    liftDownTick();
    delay(5); // 백그라운드 스케줄링 최소 양보
  }
}

// ── [1] 모터 구동 및 정지 (전 구간 엔코더 독립 속도 제어 적용) ──

void drive(int l, int r) {
  // 명령값이 0이면 즉시 전력 차단
  if (l == 0 && r == 0) {
    prizm.setMotorSpeeds(0, 0);
    return;
  }

  static unsigned long lastTime = 0;
  static long lastEncL = 0;
  static long lastEncR = 0;
  
  static int outL = 0;
  static int outR = 0;
  static int lastReqL = 0;
  static int lastReqR = 0;

  unsigned long now = millis();
  unsigned long dt = now - lastTime;

  // 사용자의 방향/속도 명령(예: 직진->회전)이 바뀌면 보정값을 리셋하여 즉각 반응
  if (l != lastReqL || r != lastReqR) {
    outL = l;
    outR = r;
  }
  lastReqL = l;
  lastReqR = r;

  // 20ms 주기 샘플링 제어
  if (dt >= 20) { 
    long encL = prizm.readEncoderCount(1);
    long encR = prizm.readEncoderCount(2);

    // 20ms 동안의 틱 변화량(속도) 절댓값 산출 (엔코더 부호 꼬임 완벽 방지)
    long curVelL_mag = abs(encL - lastEncL);
    long curVelR_mag = abs(encR - lastEncR);

    // 목표 속도 크기
    float targetVelL_mag = abs(l) * VELOCITY_TARGET_FACTOR;
    float targetVelR_mag = abs(r) * VELOCITY_TARGET_FACTOR;

    // 목표와의 오차
    float errL = targetVelL_mag - curVelL_mag;
    float errR = targetVelR_mag - curVelR_mag;

    // Feed-Forward(기본 출력) + P 제어 (오차 보정)
    int outL_mag = abs(l) + (int)(errL * VELOCITY_KP);
    int outR_mag = abs(r) + (int)(errR * VELOCITY_KP);

    // 주행 원래의 방향(부호) 복원
    outL = (l >= 0) ? outL_mag : -outL_mag;
    outR = (r >= 0) ? outR_mag : -outR_mag;

    // 최대 한계 제한(안전장치)
    outL = constrain(outL, l - VELOCITY_MAX_CORRECTION, l + VELOCITY_MAX_CORRECTION);
    outR = constrain(outR, r - VELOCITY_MAX_CORRECTION, r + VELOCITY_MAX_CORRECTION);

    lastTime = now;
    lastEncL = encL;
    lastEncR = encR;
  }

  // ★ 하드웨어 기계적 직진 오차 개별 오프셋 적용
  int finalL = outL;
  int finalR = outR;
  
  if (finalL > 0 && finalR > 0) {
    // 전진할 때는 속도를 더해줌
    finalL += MOTOR_OFFSET_L; 
    finalR += MOTOR_OFFSET_R;
  } else if (finalL < 0 && finalR < 0) {
    // 후진할 때는 (음수이므로) 속도를 빼서 절댓값을 키워줌
    finalL -= MOTOR_OFFSET_L; 
    finalR -= MOTOR_OFFSET_R;
  }
  
  finalL = constrain(finalL, -100, 100);
  finalR = constrain(finalR, -100, 100);
  
  prizm.setMotorSpeeds(-(finalL * 7), (finalR * 7));
}

void stopAll() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  
  safeDelay(50);
}

// ── [2] 거리 기반 제자리 회전 ──────────────────────────────────────

void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * degrees);
  long brakePoint = targetCounts - SPIN_BRAKE_LEAD;

  while (true) {
    long pos = (abs(prizm.readEncoderCount(1)) + abs(prizm.readEncoderCount(2))) / 2;
    if (pos >= brakePoint) break;

    if (degrees == 90 && pos > (targetCounts * 0.8f)) {
      int L, C, R;
      readSensors(L, C, R);

      if (isRight && L == 1) {
        DPRINTLNF(">> [TURN] 우회전 오버슈트 방지! (좌센서 감지)");
        break;
      }
      if (!isRight && R == 1) {
        DPRINTLNF(">> [TURN] 좌회전 오버슈트 방지! (우센서 감지)");
        break;
      }
    }

    int spd = SPIN_SPEED;
    if (isRight) drive(spd, -spd);
    else drive(-spd, spd);

    liftUpTick();
    liftDownTick();
    delay(5);
  }
  stopAll();
}

// ── [3] 라인 센서 읽기 ──────────────────────────────────────

void readSensors(int& L, int& C, int& R) {
  L = digitalRead(SENSOR_LEFT);
  C = digitalRead(SENSOR_CENTER);
  R = digitalRead(SENSOR_RIGHT);
  if (INVERT_SENSORS) { L = !L; C = !C; R = !R; }
}

void readRearSensors(int& RL, int& RC, int& RR) {
  RL = (analogRead(SENSOR_REAR_LEFT) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RC = (analogRead(SENSOR_REAR_CENTER) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RR = (analogRead(SENSOR_REAR_RIGHT) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
}

bool anyLine(int L, int C, int R) { return (L == 1 || C == 1 || R == 1); }
bool anyRearLine(int RL, int RC, int RR) { return RL || RC || RR; }

// ── [4] 다중 센서 라인 트레이싱 제어 (전진) ─────────────────────────

void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  int lsp = SPEED, rsp = SPEED;

  bool ignoreLeftEdge = true;
  bool ignoreRightEdge = true;

  if (enableEdgeSteering) {
    if (robotHeading == 0 || robotHeading == 360) {
      ignoreLeftEdge = false; 
    } else if (robotHeading == 180) {
      ignoreRightEdge = false; 
    }
  }

  if (FL && FC && FR) {
    lsp = SPEED; rsp = SPEED; lastSensorState = 0;
  } 
  else if (ignoreLeftEdge && FL && FC) {
    lsp = SPEED; rsp = SPEED; lastSensorState = 0;
  }
  else if (ignoreRightEdge && FC && FR) {
    lsp = SPEED; rsp = SPEED; lastSensorState = 0;
  }
  else if (FL && FC) {
    lsp = SPEED - 20; rsp = SPEED + 10;
    lastSensorState = 1;
  } 
  else if (FC && FR) {
    lsp = SPEED + 10; rsp = SPEED - 20;
    lastSensorState = 2;
  } 
  else if (FL && !FC && !FR) {
    lsp = SPEED - 20; rsp = SPEED + 10;
    lastSensorState = 1;
  } 
  else if (!FL && !FC && FR) {
    lsp = SPEED + 10; rsp = SPEED - 20;
    lastSensorState = 2;
  } 
  else if (FC) {
    lsp = SPEED; rsp = SPEED;
    lastSensorState = 0;
  } 
  else {
    if (lastSensorState == 1) {
      lsp = SPEED - 20; rsp = SPEED + 6;
    } else if (lastSensorState == 2) {
      lsp = SPEED + 6; rsp = SPEED - 20;
    } else {
      lsp = SPEED / 2; rsp = SPEED / 2;
    }
  }

  bool frontIsCrossing = (FL && FC) || (FC && FR);
  bool rearIsCrossing = (RL && RC) || (RC && RR);

  if (!frontIsCrossing && !rearIsCrossing) {
    if (RL && !RC && !RR) {
      lsp += REAR_ALIGN_GAIN; rsp -= REAR_ALIGN_GAIN;
    } else if (!RL && !RC && RR) {
      lsp -= REAR_ALIGN_GAIN; rsp += REAR_ALIGN_GAIN;
    }
  }

  drive(constrain(lsp, -100, 100), constrain(rsp, -100, 100));
}

// ── [5] 다중 센서 라인 트레이싱 제어 (후진) ─────────────────────────

void reverseLineFollowStep(int RL, int RC, int RR) {
  int lsp = -BACK_SPEED;
  int rsp = -BACK_SPEED;

  bool ignoreLeftEdge = true;   
  bool ignoreRightEdge = true;  

  if (enableEdgeSteering) {
    if (robotHeading == 0 || robotHeading == 360) {
      ignoreRightEdge = false; 
    } else if (robotHeading == 180) {
      ignoreLeftEdge = false;  
    }
  }

  if (RL && RC && RR) {
    lsp = -BACK_SPEED; rsp = -BACK_SPEED;
  }
  else if (ignoreLeftEdge && RL && RC) {
    lsp = -BACK_SPEED; rsp = -BACK_SPEED;
  }
  else if (ignoreRightEdge && RC && RR) {
    lsp = -BACK_SPEED; rsp = -BACK_SPEED;
  }
  else if (RL && RC) {
    lsp = -BACK_SPEED + BACK_STEER_WEAK; 
    rsp = -BACK_SPEED - BACK_STEER_WEAK; 
  } 
  else if (RC && RR) {
    lsp = -BACK_SPEED - BACK_STEER_WEAK;
    rsp = -BACK_SPEED + BACK_STEER_WEAK;
  } 
  else if (RL && !RC && !RR) {
    lsp = -BACK_SPEED + BACK_STEER_STRONG; 
    rsp = -BACK_SPEED - BACK_STEER_STRONG; 
  } 
  else if (!RL && !RC && RR) {
    lsp = -BACK_SPEED - BACK_STEER_STRONG;
    rsp = -BACK_SPEED + BACK_STEER_STRONG;
  } 
  else if (RC) {
    lsp = -BACK_SPEED; rsp = -BACK_SPEED;
  }
  else {
    lsp = -BACK_SPEED / 2; rsp = -BACK_SPEED / 2;
  }
  
  drive(constrain(lsp, -100, 100), constrain(rsp, -100, 100));
}
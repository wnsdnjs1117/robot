/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 비례식 각도 제어 구현부
 * ============================================================ */
#include "Motion.h"
#include "Config.h"
#include "MapRouter.h" 
#include "Lift.h" // ★ 논블로킹 리프트 제어를 위해 추가

bool enableEdgeSteering = false;

// ── [1] 모터 구동 및 정지 ────────────────────────────────────────

void drive(int l, int r) {
  if (l > 0 && r > 0) {
    l -= DRIVE_BIAS;
    r += DRIVE_BIAS;
  } else if (l < 0 && r < 0) {
    l += DRIVE_BIAS;
    r -= DRIVE_BIAS;
  }
  l = constrain(l, -100, 100);
  r = constrain(r, -100, 100);
  prizm.setMotorSpeeds(-(l * 7), r * 7);
}

void stopAll() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  delay(50);
}

// ── [2] 거리 기반 제자리 회전 ──────────────────────────────────────

void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * degrees);
  long brakePoint = targetCounts - SPIN_BRAKE_LEAD;
  const int minSpd = 10;

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

    float ratio = (float)pos / (float)targetCounts;
    int spd;
    if (ratio < 0.25f) {
      spd = minSpd + (int)((SPIN_SPEED - minSpd) * (ratio / 0.25f));
    } else if (ratio < 0.70f) {
      spd = SPIN_SPEED;
    } else if (ratio < 0.90f) {
      spd = minSpd + (int)((SPIN_SPEED - minSpd) * ((0.90f - ratio) / 0.20f));
    } else {
      spd = minSpd;
    }

    if (isRight) drive(spd, -spd);
    else drive(-spd, spd);

    // ★ [논블로킹 제어] 회전 중에도 리프트의 상태를 지속 감시 및 감속/정지 처리
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
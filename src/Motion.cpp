/* ============================================================
 * Motion.cpp - 듀얼 PID 조향 및 무중단 가감속 이동 (가감속 30% 비율 적용)
 * ============================================================ */
#include "Motion.h"
#include "Config.h"
#include "MapRouter.h" 
#include "Lift.h" 
#include "TestMode.h" // 디버그 키 감지용
#include <math.h>

bool enableEdgeSteering = false;

// ── [0] Non-Blocking 비동기 대기 ──
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    liftUpTick(); liftDownTick();
    checkDebugKey();
  }
}

// ── [1] 모터 구동 및 급정지 ───────────────
void drive(int l, int r) {
  checkDebugKey();
  
  if (l == 0 && r == 0) { prizm.setMotorSpeeds(0, 0); return; }
  
  static unsigned long lastTime = 0;
  static long lastEncL = 0;
  static long lastEncR = 0;
  static int outL = 0, outR = 0, lastReqL = 0, lastReqR = 0;

  unsigned long now = millis();
  unsigned long dt = now - lastTime;

  if (l != lastReqL || r != lastReqR) { outL = l; outR = r; }
  lastReqL = l; lastReqR = r;

  if (dt >= 20) { 
    long encL = prizm.readEncoderCount(1);
    long encR = prizm.readEncoderCount(2);

    long curVelL_mag = labs(encL - lastEncL);
    long curVelR_mag = labs(encR - lastEncR);

    float targetVelL_mag = abs(l) * VELOCITY_TARGET_FACTOR;
    float targetVelR_mag = abs(r) * VELOCITY_TARGET_FACTOR;

    float errL = targetVelL_mag - curVelL_mag;
    float errR = targetVelR_mag - curVelR_mag;

    int outL_mag = abs(l) + (int)(errL * VELOCITY_KP);
    int outR_mag = abs(r) + (int)(errR * VELOCITY_KP);

    outL = (l >= 0) ? outL_mag : -outL_mag;
    outR = (r >= 0) ? outR_mag : -outR_mag;

    outL = constrain(outL, l - VELOCITY_MAX_CORRECTION, l + VELOCITY_MAX_CORRECTION);
    outR = constrain(outR, r - VELOCITY_MAX_CORRECTION, r + VELOCITY_MAX_CORRECTION);

    lastTime = now; lastEncL = encL; lastEncR = encR;
  }

  int finalL = outL; int finalR = outR;
  
  float speedMag = abs(finalL);
  if (speedMag < 20.0) speedMag = 20.0;
  if (speedMag > 100.0) speedMag = 100.0;
  
  float compLow = 1.021; 
  float compHigh = 0.985; 
  float comp = compLow + (compHigh - compLow) * ((speedMag - 20.0) / 80.0);
  
  if (finalL > 0) {
    finalL = (int)(finalL * comp + 0.5);
  } else if (finalL < 0) {
    finalL = (int)(finalL * comp - 0.5);
  }
  
  if (finalL > 0 && finalR > 0) { finalL += MOTOR_OFFSET_L; finalR += MOTOR_OFFSET_R; } 
  else if (finalL < 0 && finalR < 0) { finalL -= MOTOR_OFFSET_L; finalR -= MOTOR_OFFSET_R; }
  
  prizm.setMotorSpeeds(-(constrain(finalL, -100, 100) * 7), constrain(finalR, -100, 100) * 7);
}

void stopAll() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  safeDelay(80); 
  prizm.setMotorSpeeds(0, 0);
}

// ── [2] ★ 제자리 칼각 회전 (스마트 보정 적용) ───────────
void turnAngle(int degrees, bool isRight) {
  stopAll(); 
  safeDelay(40);
  
  float absDeg = fabs((float)degrees);
  
  // ★ 스마트 각도 보정 (Dynamic Overshoot Compensation)
  // 회전량에 비례하여 미끄러짐(관성)을 선형적으로 예측하여 빼줍니다.
  // 예: 90도 회전 시 2.5도 보정, 9도 회전 시 0.25도 보정
  float slipCompensation = (absDeg / 90.0) * 2.5; 
  float compDeg = absDeg - slipCompensation;
  
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * compDeg);
  if (targetCounts <= 0) return;

  long rampCounts = (long)(targetCounts * 0.3f);
  if (rampCounts < 1) rampCounts = 1; 

  long startL = prizm.readEncoderCount(1);
  long startR = prizm.readEncoderCount(2);

  while (true) {
    long dL = labs(prizm.readEncoderCount(1) - startL);
    long dR = labs(prizm.readEncoderCount(2) - startR);
    long pos = (dL + dR) / 2;
    
    long error = targetCounts - pos; 
    if (error <= 0) break; 
    
    float spd_accel = SPIN_SPEED;
    float spd_decel = SPIN_SPEED;

    if (pos < rampCounts) {
      spd_accel = 20.0 + (SPIN_SPEED - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0));
    }
    if (error < rampCounts) {
      spd_decel = 20.0 + (SPIN_SPEED - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0));
    }
    
    int spd = (int)min(spd_accel, spd_decel);

    if (spd > SPIN_SPEED) spd = SPIN_SPEED;
    if (spd < 20) spd = 20;

    if (isRight) drive(spd, -spd);
    else drive(-spd, spd);

    liftUpTick(); liftDownTick();
  }
  
  stopAll(); 
}

// ── [3] 센서 읽기 ───────────
void readSensors(int& L, int& C, int& R) {
  L = digitalRead(SENSOR_LEFT); C = digitalRead(SENSOR_CENTER); R = digitalRead(SENSOR_RIGHT);
  if (INVERT_SENSORS) { L = !L; C = !C; R = !R; }
}

void readRearSensors(int& RL, int& RC, int& RR) {
  RL = (analogRead(SENSOR_REAR_LEFT) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RC = (analogRead(SENSOR_REAR_CENTER) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
  RR = (analogRead(SENSOR_REAR_RIGHT) >= REAR_SENSOR_THRESHOLD) ? 1 : 0;
}

bool anyLine(int L, int C, int R) { return (L == 1 || C == 1 || R == 1); }
bool anyRearLine(int RL, int RC, int RR) { return RL || RC || RR; }

// ── [4] 전진 라인 트레이싱 ───────────
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR, int baseSpeed) {
  static float integral = 0;
  static float lastPosF = 0;

  int lsp = baseSpeed;
  int rsp = baseSpeed;

  int posF = 0, posR = 0;
  if (FL && !FC && !FR) { posF = -2; lastSensorState = 1; }
  else if (FL && FC && !FR) { posF = -1; lastSensorState = 1; }
  else if (!FL && FC && !FR) { posF = 0; lastSensorState = 0; }
  else if (!FL && FC && FR) { posF = 1; lastSensorState = 2; }
  else if (!FL && !FC && FR) { posF = 2; lastSensorState = 2; }
  else if (FL && FC && FR) { posF = 0; lastSensorState = 0; } 
  else { posF = (lastSensorState == 1) ? -3 : ((lastSensorState == 2) ? 3 : 0); } 

  if (RL && !RC && !RR) posR = -2;
  else if (RL && RC && !RR) posR = -1;
  else if (!RL && RC && !RR) posR = 0;
  else if (!RL && RC && RR) posR = 1;
  else if (!RL && !RC && RR) posR = 2;
  else posR = 0; 

  float error = posF;
  integral += error;
  float derivative = error - lastPosF;

  bool isHard = (abs(error) >= 2);
  float current_KP = isHard ? LINE_KP_FWD_HARD : LINE_KP_FWD_SOFT;

  float steer = (error * current_KP) + (integral * LINE_KI) + (derivative * LINE_KD);

  lsp += (int)round(steer);
  rsp -= (int)round(steer);

  float alignVal = 0;
  if ((FL || FC || FR) && (RL || RC || RR)) {
    if (!(FL && FC && FR) && !(RL && RC && RR)) { 
      float alignError = posF - posR; 
      alignVal = alignError * LINE_ALIGN_GAIN;
      lsp += (int)round(alignVal); 
      rsp -= (int)round(alignVal);
    }
  }

  if (FL == 1 && FC == 0 && FR == 0 && RL == 1 && RC == 0 && RR == 0) {
    lsp -= EDGE_SYNC_GAIN; rsp += EDGE_SYNC_GAIN;
  }
  else if (FL == 0 && FC == 0 && FR == 1 && RL == 0 && RC == 0 && RR == 1) {
    lsp += EDGE_SYNC_GAIN; rsp -= EDGE_SYNC_GAIN;
  }

  lastPosF = error;
  drive(lsp, rsp);
}

void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  lineFollowStepFull(FL, FC, FR, RL, RC, RR, SPEED);
}

// ── [5] 후진 라인 트레이싱 ───────────
void reverseLineFollowStep(int RL, int RC, int RR, int FL, int FC, int FR, int baseSpeed) {
  static float integralRev = 0;
  static float lastPosR = 0;

  int lsp = -baseSpeed;
  int rsp = -baseSpeed;

  int posR = 0, posF = 0;
  if (RL && !RC && !RR) { posR = -2; lastSensorState = 1; }
  else if (RL && RC && !RR) { posR = -1; lastSensorState = 1; }
  else if (!RL && RC && !RR) { posR = 0; lastSensorState = 0; }
  else if (!RL && RC && RR) { posR = 1; lastSensorState = 2; }
  else if (!RL && !RC && RR) { posR = 2; lastSensorState = 2; }
  else if (RL && RC && RR) { posR = 0; lastSensorState = 0; }
  else { posR = (lastSensorState == 1) ? -3 : ((lastSensorState == 2) ? 3 : 0); } 

  if (FL && !FC && !FR) posF = -2;
  else if (FL && FC && !FR) posF = -1;
  else if (!FL && FC && !FR) posF = 0;
  else if (!FL && FC && FR) posF = 1;
  else if (!FL && !FC && FR) posF = 2;
  else posF = 0; 

  float error = posR;
  integralRev += error;
  float derivative = error - lastPosR;

  bool isHard = (abs(error) >= 2);
  float current_KP = isHard ? LINE_KP_REV_HARD : LINE_KP_REV_SOFT;

  float steer = (error * current_KP) + (integralRev * LINE_KI) + (derivative * LINE_KD);

  lsp -= (int)round(steer);
  rsp += (int)round(steer);

  float alignVal = 0;
  if ((RL || RC || RR) && (FL || FC || FR)) {
    if (!(RL && RC && RR) && !(FL && FC && FR)) {
      float alignError = posR - posF;
      alignVal = alignError * LINE_ALIGN_GAIN;
      lsp -= (int)round(alignVal); 
      rsp += (int)round(alignVal);
    }
  }

  if (RL == 1 && RC == 0 && RR == 0 && FL == 1 && FC == 0 && FR == 0) {
    lsp += EDGE_SYNC_GAIN; rsp -= EDGE_SYNC_GAIN;
  }
  else if (RL == 0 && RC == 0 && RR == 1 && FL == 0 && FC == 0 && FR == 1) {
    lsp -= EDGE_SYNC_GAIN; rsp += EDGE_SYNC_GAIN;
  }

  lastPosR = error;
  drive(lsp, rsp);
}

void reverseLineFollowStep(int RL, int RC, int RR, int FL, int FC, int FR) {
  reverseLineFollowStep(RL, RC, RR, FL, FC, FR, BACK_SPEED);
}

// ── [6] ★ S-커브 이동 함수들 (전체 거리의 30% 가감속 적용) ───────────
void driveStraightSmooth(float cm, int maxSpd) {
  long startEnc = labs(prizm.readEncoderCount(1));
  long targetCounts = CM(cm);
  if (targetCounts <= 0) return;
  
  long rampCounts = (long)(targetCounts * 0.3f);
  if (rampCounts < 1) rampCounts = 1;

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long pos = labs(currentEnc - startEnc);
    long error = targetCounts - pos;
    if (error <= 0) break; 
    
    float spd_accel = maxSpd;
    float spd_decel = maxSpd;
    
    if (pos < rampCounts) {
      spd_accel = 20.0 + (maxSpd - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0));
    }
    if (error < rampCounts) {
      spd_decel = 20.0 + (maxSpd - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0));
    }
    
    int currentSpd = (int)min(spd_accel, spd_decel);
    currentSpd = constrain(currentSpd, 20, maxSpd);
    
    drive(currentSpd, currentSpd);
    liftUpTick(); liftDownTick();
  }
  stopAll();
}

void lineFollowSmooth(float cm, int maxSpd) {
  long startEnc = labs(prizm.readEncoderCount(1));
  long targetCounts = CM(cm);
  if (targetCounts <= 0) return;
  
  long rampCounts = (long)(targetCounts * 0.3f);
  if (rampCounts < 1) rampCounts = 1;

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long pos = labs(currentEnc - startEnc);
    long error = targetCounts - pos;
    if (error <= 0) break;
    
    float spd_accel = maxSpd;
    float spd_decel = maxSpd;
    
    if (pos < rampCounts) {
      spd_accel = 20.0 + (maxSpd - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0));
    }
    if (error < rampCounts) {
      spd_decel = 20.0 + (maxSpd - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0));
    }
    
    int currentSpd = (int)min(spd_accel, spd_decel);
    currentSpd = constrain(currentSpd, 20, maxSpd);
    
    int FL, FC, FR, RL, RC, RR;
    readSensors(FL, FC, FR); readRearSensors(RL, RC, RR);
    
    lineFollowStepFull(FL, FC, FR, RL, RC, RR, currentSpd);
    liftUpTick(); liftDownTick();
  }
  stopAll();
}

void reverseLineFollowSmooth(float cm, int maxSpd) {
  long startEnc = labs(prizm.readEncoderCount(1));
  long targetCounts = CM(cm);
  if (targetCounts <= 0) return;
  
  long rampCounts = (long)(targetCounts * 0.3f);
  if (rampCounts < 1) rampCounts = 1;

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long pos = labs(currentEnc - startEnc);
    long error = targetCounts - pos;
    if (error <= 0) break;
    
    float spd_accel = maxSpd;
    float spd_decel = maxSpd;
    
    if (pos < rampCounts) {
      spd_accel = 20.0 + (maxSpd - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0));
    }
    if (error < rampCounts) {
      spd_decel = 20.0 + (maxSpd - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0));
    }
    
    int currentSpd = (int)min(spd_accel, spd_decel);
    currentSpd = constrain(currentSpd, 20, maxSpd);
    
    int FL, FC, FR, RL, RC, RR;
    readSensors(FL, FC, FR); readRearSensors(RL, RC, RR);
    
    reverseLineFollowStep(RL, RC, RR, FL, FC, FR, currentSpd);
    liftUpTick(); liftDownTick();
  }
  stopAll();
}

void driveExtraDecel(float cm, int startSpd) {
  long startEnc = labs(prizm.readEncoderCount(1));
  long targetCounts = CM(cm);
  if (targetCounts <= 0) { stopAll(); return; }

  int absStart = abs(startSpd);

  while (true) {
    long currentEnc = labs(prizm.readEncoderCount(1));
    long pos = labs(currentEnc - startEnc);
    if (pos >= targetCounts) break;

    float progress = (float)pos / targetCounts;
    int currentMag = 20 + (absStart - 20) * cos(progress * (PI / 2.0));
    
    if (startSpd < 0) currentMag = -currentMag;
    
    drive(currentMag, currentMag);
    liftUpTick(); liftDownTick();
  }
  stopAll(); 
}
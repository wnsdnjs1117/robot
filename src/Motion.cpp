/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 제어 (가장자리 평행 이탈 방지 조향 적용)
 * ============================================================ */
#include "Motion.h"
#include "Config.h"
#include "MapRouter.h" 
#include "Lift.h" 

bool enableEdgeSteering = false;

// ── [0] Non-Blocking 비동기 대기 ───────────────────────────
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    liftUpTick(); liftDownTick(); delay(5); 
  }
}

// ── [1] 모터 구동 및 급정지(Brake & Release) ───────────────
void drive(int l, int r) {
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

    long curVelL_mag = abs(encL - lastEncL);
    long curVelR_mag = abs(encR - lastEncR);

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
  
  if (finalL > 0 && finalR > 0) { finalL += MOTOR_OFFSET_L; finalR += MOTOR_OFFSET_R; } 
  else if (finalL < 0 && finalR < 0) { finalL -= MOTOR_OFFSET_L; finalR -= MOTOR_OFFSET_R; }
  
  prizm.setMotorSpeeds(-(constrain(finalL, -100, 100) * 7), constrain(finalR, -100, 100) * 7);
}

void stopAll() {
  // 1. 모터에 125(급정지/Brake) 명령 전달하여 관성을 칼같이 잡음
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  
  // 2. 급정지가 물리적으로 걸릴 아주 짧은 시간만 대기 (60ms)
  safeDelay(60); 
  
  // 3. 바로 제동을 풀고 자연스러운 상태(Coast)로 전환 (다음 동작으로 즉각 이어짐)
  prizm.setMotorSpeeds(0, 0);
}

// ── [2] ★ 제자리 칼각 회전 ─────────────────────────────────
void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  safeDelay(40);
  
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * degrees);
  long brakePoint = targetCounts - SPIN_BRAKE_LEAD;

  while (true) {
    long pos = (abs(prizm.readEncoderCount(1)) + abs(prizm.readEncoderCount(2))) / 2;
    if (pos >= brakePoint) break; 

    if (pos > (targetCounts * 0.6f)) {
      int L, C, R; readSensors(L, C, R);
      if (isRight && L == 1) break;
      if (!isRight && R == 1) break;
    }

    int spd = SPIN_SPEED;
    if (isRight) drive(spd, -spd);
    else drive(-spd, spd);

    liftUpTick(); liftDownTick(); delay(5);
  }
  // 스핀 턴 완료 후 1회 급제동 및 즉각 해제 (추가 딜레이 완전히 삭제)
  stopAll(); 
}

// ── [3] 센서 읽기 ──────────────────────────────────────────
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

// ── [4] 전진 라인 트레이싱 ─────────────────────────────────
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  int lsp = SPEED, rsp = SPEED;

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

  if (posF == -2) { lsp -= 20; rsp += 10; }
  else if (posF == -1) { lsp -= 10; rsp += 5; }
  else if (posF == 1) { lsp += 5; rsp -= 10; }  
  else if (posF == 2) { lsp += 20; rsp -= 10; }
  else if (posF == -3) { lsp -= 20; rsp += 6; } 
  else if (posF == 3) { lsp += 6; rsp -= 20; }  

  if ((FL || FC || FR) && (RL || RC || RR)) {
    if (!(FL && FC && FR) && !(RL && RC && RR)) { 
      int diff = posF - posR;
      lsp += (diff * REAR_ALIGN_GAIN); 
      rsp -= (diff * REAR_ALIGN_GAIN);
    }
  }

  if (FL == 1 && FC == 0 && FR == 0 && RL == 1 && RC == 0 && RR == 0) {
    lsp -= EDGE_SYNC_GAIN; rsp += EDGE_SYNC_GAIN;
  }
  else if (FL == 0 && FC == 0 && FR == 1 && RL == 0 && RC == 0 && RR == 1) {
    lsp += EDGE_SYNC_GAIN; rsp -= EDGE_SYNC_GAIN;
  }

  drive(lsp, rsp);
}

// ── [5] 후진 라인 트레이싱 ─────────────────────────────────
void reverseLineFollowStep(int RL, int RC, int RR, int FL, int FC, int FR) {
  int lsp = -BACK_SPEED, rsp = -BACK_SPEED;

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

  if (posR == -2) { lsp += 20; rsp -= 10; }
  else if (posR == -1) { lsp += 10; rsp -= 5; } 
  else if (posR == 1) { lsp -= 5; rsp += 10; }  
  else if (posR == 2) { lsp -= 20; rsp += 10; }
  else if (posR == -3) { lsp += 20; rsp -= 6; } 
  else if (posR == 3) { lsp -= 6; rsp += 20; } 

  if ((RL || RC || RR) && (FL || FC || FR)) {
    if (!(RL && RC && RR) && !(FL && FC && FR)) {
      int diff = posR - posF;
      lsp -= (diff * REAR_ALIGN_GAIN); 
      rsp += (diff * REAR_ALIGN_GAIN);
    }
  }

  if (RL == 1 && RC == 0 && RR == 0 && FL == 1 && FC == 0 && FR == 0) {
    lsp += EDGE_SYNC_GAIN; rsp -= EDGE_SYNC_GAIN;
  }
  else if (RL == 0 && RC == 0 && RR == 1 && FL == 0 && FC == 0 && FR == 1) {
    lsp -= EDGE_SYNC_GAIN; rsp += EDGE_SYNC_GAIN;
  }

  drive(lsp, rsp);
}
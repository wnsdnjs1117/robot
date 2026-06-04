/* ============================================================
 * Motion.cpp - 하드웨어 구동 및 제어 (가장자리 평행 이탈 방지 조향 적용)
 * ============================================================ */
#include "Motion.h"
#include "Config.h"
#include "MapRouter.h" 
#include "Lift.h" 
#include <math.h>

bool enableEdgeSteering = false;

// ── [0] Non-Blocking 비동기 대기 (delay() 미사용, millis 기반 + 리프트 틱) ──
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    liftUpTick(); liftDownTick();
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
  
  // ★ 물리적 모터 비선형 편차 보정 (속도 구간별 맞춤 보정)
  // "느릴 때는 오른쪽이 빠르고, 빠를 때는 왼쪽이 빠름" 현상을 해결하기 위한 동적 배율
  float speedMag = abs(finalL);
  if (speedMag < 20.0) speedMag = 20.0;
  if (speedMag > 100.0) speedMag = 100.0;
  
  // 1. 속도가 20(최저 속도)일 때의 왼쪽 모터 보정치
  // 오른쪽이 빠르므로 왼쪽을 5% 증폭 (1.05) - 필요시 조절하세요!
  float compLow = 1.021; 
  
  // 2. 속도가 100(최고 속도)일 때의 왼쪽 모터 보정치
  // 왼쪽이 빠르므로 왼쪽을 3% 감소 (0.97) - 필요시 조절하세요!
  float compHigh = 0.985; 
  
  // 현재 속도에 맞춰 compLow와 compHigh 사이의 값을 자연스럽게 섞어줌 (선형 보간)
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
  safeDelay(60); 
  prizm.setMotorSpeeds(0, 0);
}

// ── [2] ★ 제자리 칼각 회전 (절대 거리 기반 초스무스 가감속 + 오차 보정 적용) ───────────
void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  safeDelay(40);
  
  float absDeg = fabs((float)degrees);
  float compDeg = absDeg;
  
  // 실측 데이터 기반 수학적 보정식 적용 (90도=92도, 720도=715도 기준)
  if (absDeg >= 3.0) {
    compDeg = (absDeg - 3.0) * 1.011236; 
  } else {
    compDeg = absDeg * (90.0 / 92.0); 
  }
  
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * compDeg);
  if (targetCounts <= 0) return;

  // 짧은 거리 급제동 방지: 고정된 각도(30도)를 기준으로 감속
  long rampCounts = (long)((SPIN_90_COUNTS / 90.0) * 30.0);

  while (true) {
    long pos = (abs(prizm.readEncoderCount(1)) + abs(prizm.readEncoderCount(2))) / 2;
    long error = targetCounts - pos; 
    if (error <= 0) break; 
    
    float spd_accel = SPIN_SPEED;
    float spd_decel = SPIN_SPEED;

    // 출발 직후 30도 동안 부드럽게 가속
    if (pos < rampCounts) {
      spd_accel = 20.0 + (SPIN_SPEED - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0));
    }
    // 도착 직전 30도 동안 부드럽게 감속
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

// ── [3] 센서 읽기 (디바운스 + 히스테리시스 필터 적용) ───────────────────────
static int filterDigital(int ch, int raw) {
  static int     stable[6] = {0, 0, 0, 0, 0, 0};
  static int     cand[6]   = {0, 0, 0, 0, 0, 0};
  static uint8_t cnt[6]    = {0, 0, 0, 0, 0, 0};

  if (raw == stable[ch]) { cand[ch] = raw; cnt[ch] = 0; return stable[ch]; }
  if (raw == cand[ch]) {
    if (++cnt[ch] >= SENSOR_FILTER_SAMPLES) { stable[ch] = raw; cnt[ch] = 0; }
  } else {
    cand[ch] = raw; cnt[ch] = 1;
  }
  return stable[ch];
}

void readSensors(int& L, int& C, int& R) {
  int rl = digitalRead(SENSOR_LEFT), rc = digitalRead(SENSOR_CENTER), rr = digitalRead(SENSOR_RIGHT);
  if (INVERT_SENSORS) { rl = !rl; rc = !rc; rr = !rr; }
  L = filterDigital(0, rl);
  C = filterDigital(1, rc);
  R = filterDigital(2, rr);
}

void readRearSensors(int& RL, int& RC, int& RR) {
  static int prev[3] = {0, 0, 0};
  int a[3] = { analogRead(SENSOR_REAR_LEFT),
               analogRead(SENSOR_REAR_CENTER),
               analogRead(SENSOR_REAR_RIGHT) };
  int raw[3];
  for (int i = 0; i < 3; i++) {
    if (prev[i]) raw[i] = (a[i] <= REAR_SENSOR_THRESHOLD - REAR_SENSOR_HYSTERESIS) ? 0 : 1;
    else         raw[i] = (a[i] >= REAR_SENSOR_THRESHOLD + REAR_SENSOR_HYSTERESIS) ? 1 : 0;
    prev[i] = raw[i];
  }
  RL = filterDigital(3, raw[0]);
  RC = filterDigital(4, raw[1]);
  RR = filterDigital(5, raw[2]);
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
/* ============================================================
 * Motion.cpp - 물리 편차 보정 및 듀얼(Dual) PID 라인트레이싱 (전/후진 독립 조향)
 * ============================================================ */
#include "Motion.h"
#include "Config.h"
#include "MapRouter.h" 
#include "Lift.h" 
#include <math.h>

bool enableEdgeSteering = false;

// ============================================================
// ★ [전진용 PID 조향 값]
// ============================================================
float LINE_KP_FWD_SOFT = 0.6;    // 유지
float LINE_KP_FWD_HARD = 3.5;    // 유지

// ============================================================
// ★ [후진용 PID 조향 값]
// ============================================================
float LINE_KP_REV_SOFT = 0.4;    // 유지
float LINE_KP_REV_HARD = 1.2;    // 1.2 -> 1.0 (후진 강 조향도 살짝 더 깎음)

// ============================================================
// ★ [공통 보조 제어 값] - 진동(Chattering) 해결을 위해 대폭 수정!
// ============================================================
float LINE_KI = 0.0;         // 유지
float LINE_KD = 5.0;         // ★ 10.0 -> 1.0 으로 대폭 낮춤! (좌우 떨림의 완벽한 주범)
float LINE_ALIGN_GAIN = 1.5; // ★ 1.5 -> 1.0 으로 낮춤 (차체를 억지로 펴려는 힘을 줄임)

// ── [0] Non-Blocking 비동기 대기 ──
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    liftUpTick(); liftDownTick();
  }
}

// ── [1] 모터 구동 및 급정지 (동적 편차 보정 유지) ───────────────
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
  
  // ★ 물리적 모터 비선형 편차 보정 
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
  safeDelay(60); 
  prizm.setMotorSpeeds(0, 0);
}

// ── [2] ★ 제자리 칼각 회전 ───────────
void turnAngle(int degrees, bool isRight) {
  prizm.resetEncoders();
  safeDelay(40);
  
  float absDeg = fabs((float)degrees);
  float compDeg = absDeg;
  
  if (absDeg >= 3.0) {
    compDeg = (absDeg - 3.0) * 1.011236; 
  } else {
    compDeg = absDeg * (90.0 / 92.0); 
  }
  
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * compDeg);
  if (targetCounts <= 0) return;

  long rampCounts = (long)((SPIN_90_COUNTS / 90.0) * 30.0);

  while (true) {
    long pos = (abs(prizm.readEncoderCount(1)) + abs(prizm.readEncoderCount(2))) / 2;
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
void lineFollowStepFull(int FL, int FC, int FR, int RL, int RC, int RR) {
  static float integral = 0;
  static float lastPosF = 0;

  int lsp = SPEED;
  int rsp = SPEED;

  // 1. 에러 도출
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

  // 2. 전방 센서 기반 조향 PID 제어 (★ 전진용 조향 값 적용)
  float error = posF;
  integral += error;
  float derivative = error - lastPosF;

  bool isHard = (abs(error) >= 2);
  float current_KP = isHard ? LINE_KP_FWD_HARD : LINE_KP_FWD_SOFT;
  const char* steerMode = isHard ? "FWD-HARD" : "FWD-SOFT";

  float steer = (error * current_KP) + (integral * LINE_KI) + (derivative * LINE_KD);

  lsp += (int)round(steer);
  rsp -= (int)round(steer);

  // 3. 선과 평행을 맞추는 조향 (앞/뒤 센서 오차 비교)
  float alignVal = 0;
  if ((FL || FC || FR) && (RL || RC || RR)) {
    if (!(FL && FC && FR) && !(RL && RC && RR)) { 
      float alignError = posF - posR; 
      alignVal = alignError * LINE_ALIGN_GAIN;
      lsp += (int)round(alignVal); 
      rsp -= (int)round(alignVal);
    }
  }

  // 4. 모서리 동기화 제어
  if (FL == 1 && FC == 0 && FR == 0 && RL == 1 && RC == 0 && RR == 0) {
    lsp -= EDGE_SYNC_GAIN; rsp += EDGE_SYNC_GAIN;
  }
  else if (FL == 0 && FC == 0 && FR == 1 && RL == 0 && RC == 0 && RR == 1) {
    lsp += EDGE_SYNC_GAIN; rsp -= EDGE_SYNC_GAIN;
  }

  lastPosF = error;

  // ★ 50ms 간격 시리얼 모니터링
  static unsigned long lastPrintFwd = 0;
  if (millis() - lastPrintFwd > 50) {
    Serial.print("[전진] 센서(전):"); Serial.print(FL); Serial.print(FC); Serial.print(FR);
    Serial.print(" Err:"); Serial.print(posF);
    Serial.print(" | ["); Serial.print(steerMode); Serial.print("]");
    Serial.print(" KP:"); Serial.print(current_KP);
    Serial.print(" | Steer:"); Serial.print(steer);
    Serial.print(" | 출력(L):"); Serial.print(lsp);
    Serial.print(" (R):"); Serial.println(rsp);
    lastPrintFwd = millis();
  }

  drive(lsp, rsp);
}

// ── [5] 후진 라인 트레이싱 ───────────
void reverseLineFollowStep(int RL, int RC, int RR, int FL, int FC, int FR) {
  static float integralRev = 0;
  static float lastPosR = 0;

  int lsp = -BACK_SPEED;
  int rsp = -BACK_SPEED;

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

  // 2. 후방 센서 기반 조향 PID 제어 (★ 후진용 독립 조향 값 적용!)
  float error = posR;
  integralRev += error;
  float derivative = error - lastPosR;

  bool isHard = (abs(error) >= 2);
  float current_KP = isHard ? LINE_KP_REV_HARD : LINE_KP_REV_SOFT;
  const char* steerMode = isHard ? "REV-HARD" : "REV-SOFT";

  float steer = (error * current_KP) + (integralRev * LINE_KI) + (derivative * LINE_KD);

  lsp -= (int)round(steer); // 후진 시 조향 부호 반전
  rsp += (int)round(steer);

  // 3. 선과 평행을 맞추는 조향 (뒤/앞 센서 오차 비교)
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

  // ★ 50ms 간격 시리얼 모니터링
  static unsigned long lastPrintRev = 0;
  if (millis() - lastPrintRev > 50) {
    Serial.print("[후진] 센서(후):"); Serial.print(RL); Serial.print(RC); Serial.print(RR);
    Serial.print(" Err:"); Serial.print(posR);
    Serial.print(" | ["); Serial.print(steerMode); Serial.print("]");
    Serial.print(" KP:"); Serial.print(current_KP);
    Serial.print(" | Steer:"); Serial.print(steer);
    Serial.print(" | 출력(L):"); Serial.print(lsp);
    Serial.print(" (R):"); Serial.println(rsp);
    lastPrintRev = millis();
  }

  drive(lsp, rsp);
}
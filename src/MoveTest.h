/* ============================================================
 * MoveTest.h - 시리얼 명령으로 이동/회전 단독 테스트 모드
 * Config.h 에서 MOVE_TEST_MODE 1 로 설정하면 활성화
 *
 * [시리얼 명령]  (9600bps, 줄 끝에 개행)
 * m <cm> <speed>   : 직진 이동.  예) m 50 30  → 50cm 를 속도 30으로 전진
 * m -20 40 → 20cm 를 속도 40으로 후진
 * t <deg> <speed>  : 제자리 회전. 예) t -90 30 → 속도 30으로 반시계(좌) 90도
 * t 90 30  → 속도 30으로 시계(우) 90도
 * ============================================================ */
#ifndef MOVE_TEST_H
#define MOVE_TEST_H

#if MOVE_TEST_MODE

#include "Config.h"
#include "Motion.h"

static char    _mtBuf[24];
static uint8_t _mtLen = 0;

static void _mtWait(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) { }
}

static void _mtBrake() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  _mtWait(80);
  prizm.setMotorSpeeds(0, 0);
}

// ★ 직진/후진 이동 (절대 거리 기반 스무스)
static void _mtMove(float cm, int speed) {
  int max_spd = constrain(abs(speed), 1, 100);
  int dir = (cm >= 0) ? 1 : -1; 
  float absCm = fabs(cm);
  float compCm = absCm;
  
  // 이동 오차 보정
  if (absCm >= 2.0) {
    compCm = (absCm - 1.0) * 1.0; 
  }
  
  long targetCounts = CM(compCm);
  if (targetCounts <= 0) return; 
  
  // ★ 짧은 거리 급제동 방지: 고정된 15cm 폭을 기준으로 감속합니다.
  long rampCounts = CM(15.0);
  
  prizm.resetEncoders(); _mtWait(40);
  
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1)) + labs(prizm.readEncoderCount(2))) / 2;
    long error = targetCounts - pos;
    if (error <= 0) break;
    
    float spd_accel = max_spd;
    float spd_decel = max_spd;

    // 출발 직후 15cm 동안 부드럽게 가속
    if (pos < rampCounts) {
      spd_accel = 20.0 + (max_spd - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0));
    }
    // 도착 직전 15cm 동안 부드럽게 감속
    if (error < rampCounts) {
      spd_decel = 20.0 + (max_spd - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0));
    }
    
    int spd = (int)min(spd_accel, spd_decel);
    
    if (spd > max_spd) spd = max_spd;
    if (spd < 20) spd = 20;

    drive(spd * dir, spd * dir);
  }
  
  _mtBrake();
}

// ★ 제자리 회전 (절대 거리 기반 스무스)
static void _mtTurn(float deg, int speed) {
  int max_spd = constrain(abs(speed), 1, 100);
  bool right = (deg >= 0);
  
  float absDeg = fabs(deg);
  float compDeg = absDeg;
  
  // 수학적 오차 보정
  if (absDeg >= 3.0) {
    compDeg = (absDeg - 3.0) * 1.011236; 
  } else {
    compDeg = absDeg * (90.0 / 92.0);
  }
  
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * compDeg);
  if (targetCounts <= 0) return;
  
  // ★ 짧은 거리 급제동 방지: 비율이 아닌 30도 기준으로 가감속
  long rampCounts = (long)((SPIN_90_COUNTS / 90.0) * 30.0);
  
  prizm.resetEncoders(); _mtWait(40);
  
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1)) + labs(prizm.readEncoderCount(2))) / 2;
    long error = targetCounts - pos;
    if (error <= 0) break;
    
    float spd_accel = max_spd;
    float spd_decel = max_spd;

    // 가속 구간
    if (pos < rampCounts) {
      spd_accel = 20.0 + (max_spd - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0));
    }
    // 감속 구간
    if (error < rampCounts) {
      spd_decel = 20.0 + (max_spd - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0));
    }
    
    int spd = (int)min(spd_accel, spd_decel);
    
    if (spd > max_spd) spd = max_spd;
    if (spd < 20) spd = 20;

    if (right) drive(spd, -spd);
    else       drive(-spd, spd);
  }
  
  _mtBrake();
}

static void _mtHelp() {
  Serial.println(F("=== MOVE TEST ==="));
  Serial.println(F("m <cm> <speed>  : 이동  (m 50 30=50cm 전진 / m -20 40=20cm 후진)"));
  Serial.println(F("t <deg> <speed> : 회전  (t -90 30=반시계90 / t 90 30=시계90)"));
}

static void _mtExec(char* line) {
  char* tok = strtok(line, " \t");
  if (!tok) return;
  char cmd = tok[0];
  char* p1 = strtok(NULL, " \t");
  char* p2 = strtok(NULL, " \t");
  if (!p1 || !p2) { Serial.println(F("[오류] 형식: m/t <값> <속도>")); _mtHelp(); return; }

  float v1 = atof(p1);
  int   v2 = atoi(p2);

  if (cmd == 'm' || cmd == 'M') {
    Serial.print(F(">> MOVE ")); Serial.print(v1);
    Serial.print(F("cm @ spd ")); Serial.println(v2);
    _mtMove(v1, v2);
    Serial.print(F("   done. enc(1)=")); Serial.println(prizm.readEncoderCount(1));
  } else if (cmd == 't' || cmd == 'T') {
    Serial.print(F(">> TURN ")); Serial.print(v1);
    Serial.print(F("deg @ spd ")); Serial.println(v2);
    _mtTurn(v1, v2);
    Serial.print(F("   done. ")); Serial.print(v1 >= 0 ? F("CW(우)") : F("CCW(좌)"));
    Serial.println();
  } else {
    Serial.println(F("[오류] 알 수 없는 명령")); _mtHelp();
  }
}

void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();
  prizm.resetEncoders();
  _mtHelp();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (_mtLen > 0) { _mtBuf[_mtLen] = '\0'; _mtExec(_mtBuf); _mtLen = 0; }
    } else if (_mtLen < sizeof(_mtBuf) - 1) {
      _mtBuf[_mtLen++] = c;
    }
  }
}

#endif // MOVE_TEST_MODE
#endif // MOVE_TEST_H
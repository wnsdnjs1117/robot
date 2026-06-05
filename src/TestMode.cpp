/* ============================================================
 * TestMode.cpp - 센서, 리프트, 이동/회전 통합 테스트 모드 구현
 * ============================================================ */
#include "Config.h"

// Config.h 에서 RUN_TEST_MODE가 1일 때만 빌드됨 (용량 완벽 절약)
#if RUN_TEST_MODE == 1

#include "TestMode.h"
#include "Motion.h"
#include "Lift.h"
#include "HuskyQR.h"

// ── [1] 센서 테스트 모드 ───────────────────────────────────────
static unsigned long _stLastPrint = 0;

static void setupSensorTest() {
  Serial.println(F("Front(D): L C R   |   Rear(A): L C R"));
  _stLastPrint = millis();
}

static bool loopSensorTest() {
  if (Serial.available()) {
    char c = Serial.peek();
    if (c == 'q' || c == 'Q') {
      Serial.read(); 
      return false;  
    }
  }

  unsigned long now = millis();
  if (now - _stLastPrint >= 200) {
    int frontL = digitalRead(SENSOR_LEFT);
    int frontC = digitalRead(SENSOR_CENTER);
    int frontR = digitalRead(SENSOR_RIGHT);

    int rearL = analogRead(SENSOR_REAR_LEFT);
    int rearC = analogRead(SENSOR_REAR_CENTER);
    int rearR = analogRead(SENSOR_REAR_RIGHT);

    Serial.print(F("Front(D): "));
    Serial.print(frontL); Serial.print(F(" "));
    Serial.print(frontC); Serial.print(F(" "));
    Serial.print(frontR); 
    
    Serial.print(F("   |   Rear(A): "));
    Serial.print(rearL); Serial.print(F("\t"));
    Serial.print(rearC); Serial.print(F("\t"));
    Serial.println(rearR);

    _stLastPrint = now;
  }
  return true;
}

// ── [2] 리프트 테스트 모드 ───────────────────────────────────────
static enum { LT_IDLE, LT_UP, LT_DOWN } _ltState = LT_IDLE;
static long          _ltPrevEncL  = 0, _ltPrevEncR  = 0;
static long          _ltSpdL      = 0, _ltSpdR      = 0;
static unsigned long _ltLastTick  = 0;
static unsigned long _ltLastPrint = 0;

static void setupLiftTest() {
  Serial.println(F("u=상승  d=하강  s=정지"));
  Serial.println(F("encL/R | spdL/R(tick/10ms) | hL/R(cm)"));
  _ltState = LT_IDLE;
  _ltLastTick = millis();
  _ltLastPrint = millis();
}

static bool loopLiftTest() {
  if (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'q' || c == 'Q') {
      exc.setMotorPowers(EXP_ID, 0, 0);
      _ltState = LT_IDLE;
      return false; 
    }
    else if (c == 'u' || c == 'U') {
      Serial.println(F("[UP]"));
      _ltState = LT_UP;
      liftUp();   
    }
    else if (c == 'd' || c == 'D') {
      Serial.println(F("[DOWN]"));
      _ltState = LT_DOWN;
      liftDownStart();
    }
    else if (c == 's' || c == 'S') {
      Serial.println(F("[STOP]"));
      _ltState = LT_IDLE;
      exc.setMotorPowers(EXP_ID, 0, 0);
    }
  }

  if (_ltState == LT_UP)   liftUpTick();
  if (_ltState == LT_DOWN) liftDownTick();

  unsigned long now = millis();

  if (now - _ltLastTick >= LIFT_TICK_INTERVAL_MS) {
    long encL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long encR = exc.readEncoderCount(EXP_ID, LIFT_R);
    _ltSpdL   = encL - _ltPrevEncL;
    _ltSpdR   = encR - _ltPrevEncR;
    _ltPrevEncL = encL;
    _ltPrevEncR = encR;
    _ltLastTick = now;
  }

  if (now - _ltLastPrint >= 200) {
    long encL = exc.readEncoderCount(EXP_ID, LIFT_L);
    long encR = exc.readEncoderCount(EXP_ID, LIFT_R);

    Serial.print(F("enc:"));  Serial.print(encL);
    Serial.print(F("/"));     Serial.print(encR);
    Serial.print(F(" spd:")); Serial.print(_ltSpdL);
    Serial.print(F("/"));     Serial.print(_ltSpdR);
    Serial.print(F(" h:"));   Serial.print(heightL, 2);
    Serial.print(F("/"));     Serial.println(heightR, 2);

    _ltLastPrint = now;
  }
  return true;
}

// ── [3] 이동 및 회전 테스트 모드 ──────────────────────────────────
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

static void _mtMove(float cm, int speed) {
  int max_spd = constrain(abs(speed), 1, 100);
  int dir = (cm >= 0) ? 1 : -1; 
  float absCm = fabs(cm);
  float compCm = absCm;
  
  if (absCm >= 2.0) { compCm = (absCm - 1.0) * 1.0; }
  long targetCounts = CM(compCm);
  if (targetCounts <= 0) return; 
  
  long rampCounts = (long)(targetCounts * 0.3f);
  if (rampCounts < 1) rampCounts = 1;

  prizm.resetEncoders(); _mtWait(40);
  
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1)) + labs(prizm.readEncoderCount(2))) / 2;
    long error = targetCounts - pos;
    if (error <= 0) break;
    
    float spd_accel = max_spd;
    float spd_decel = max_spd;

    if (pos < rampCounts) { spd_accel = 20.0 + (max_spd - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0)); }
    if (error < rampCounts) { spd_decel = 20.0 + (max_spd - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0)); }
    
    int spd = (int)min(spd_accel, spd_decel);
    if (spd > max_spd) spd = max_spd;
    if (spd < 20) spd = 20;

    drive(spd * dir, spd * dir);
  }
  _mtBrake();
}

static void _mtTurn(float deg, int speed) {
  int max_spd = constrain(abs(speed), 1, 100);
  bool right = (deg >= 0);
  
  float absDeg = fabs(deg);
  
  // ★ 스마트 각도 보정 (테스트 모드에도 동일하게 반영)
  float slipCompensation = (absDeg / 90.0) * 2.5; 
  float compDeg = absDeg - slipCompensation;
  
  long targetCounts = (long)((SPIN_90_COUNTS / 90.0) * compDeg);
  if (targetCounts <= 0) return;
  
  long rampCounts = (long)(targetCounts * 0.3f);
  if (rampCounts < 1) rampCounts = 1;

  prizm.resetEncoders(); _mtWait(40);
  
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1)) + labs(prizm.readEncoderCount(2))) / 2;
    long error = targetCounts - pos;
    if (error <= 0) break;
    
    float spd_accel = max_spd;
    float spd_decel = max_spd;

    if (pos < rampCounts) { spd_accel = 20.0 + (max_spd - 20.0) * sin(((float)pos / rampCounts) * (PI / 2.0)); }
    if (error < rampCounts) { spd_decel = 20.0 + (max_spd - 20.0) * sin(((float)error / rampCounts) * (PI / 2.0)); }
    
    int spd = (int)min(spd_accel, spd_decel);
    if (spd > max_spd) spd = max_spd;
    if (spd < 20) spd = 20;

    if (right) drive(spd, -spd);
    else       drive(-spd, spd);
  }
  _mtBrake();
}

static void _mtHelp() {
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
    Serial.print(F("   done. ")); Serial.println(v1 >= 0 ? F("CW(우)") : F("CCW(좌)"));
  } else {
    Serial.println(F("[오류] 알 수 없는 명령")); _mtHelp();
  }
}

static void setupMoveTest() {
  prizm.resetEncoders();
  _mtLen = 0;
  _mtHelp();
}

static bool loopMoveTest() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'q' || c == 'Q') {
      return false;
    }
    if (c == '\n' || c == '\r') {
      if (_mtLen > 0) { _mtBuf[_mtLen] = '\0'; _mtExec(_mtBuf); _mtLen = 0; }
    } else if (_mtLen < sizeof(_mtBuf) - 1) {
      _mtBuf[_mtLen++] = c;
    }
  }
  return true;
}

// ── [4] QR(HuskyLens) 인식 테스트 모드 ──────────────────────────────
static unsigned long _qrLastPrint = 0;

static void setupQrTest() {
  HuskyQR::begin();   // Wire는 PrizmBegin()이 이미 시작함
  Serial.println(F("HuskyLens QR 인식 테스트 — QR을 카메라에 비추세요."));
  Serial.println(F("각 박스 QR이 ID 1~6으로 학습돼 있어야 합니다. (q=종료)"));
  _qrLastPrint = millis();
}

static bool loopQrTest() {
  if (Serial.available()) {
    char c = Serial.peek();
    if (c == 'q' || c == 'Q') { Serial.read(); return false; }
  }
  unsigned long now = millis();
  if (now - _qrLastPrint >= 200) {
    int id = HuskyQR::readBoxId();
    if (id >= 1 && id <= 6) {
      Serial.print(F("인식 ID(목적지): ")); Serial.println(id);
      beep(120);
    } else {
      Serial.println(F("...(인식 없음)"));
    }
    _qrLastPrint = now;
  }
  return true;
}

// ── [5] 통합 테스트 메뉴 ───────────────────────────────────────────
void runTestMenu() {
  stopAll(); 
  
  Serial.println(F("\n=================================="));
  Serial.println(F("      로봇 통합 테스트 모드"));
  Serial.println(F("=================================="));
  Serial.println(F("[c] 변경  |  [e] 선택/실행  |  [q] 종료"));
  
  int mode = 0;
  const char* names[] = {"[1] 센서 테스트", "[2] 리프트 테스트", "[3] 이동/회전 테스트", "[4] QR 인식 테스트"};
  const int MODE_COUNT = 4;
  Serial.print(F("\n▶ 현재 선택: ")); Serial.println(names[mode]);

  while(true) {
    if(Serial.available()) {
      char c = Serial.read();
      if(c == 'c' || c == 'C') {
        mode = (mode + 1) % MODE_COUNT;
        Serial.print(F("▶ 현재 선택: ")); Serial.println(names[mode]);
      }
      else if(c == 'e' || c == 'E') {
        Serial.println(F("\n----------------------------------"));
        Serial.print(F(" 실행 중: ")); Serial.println(names[mode]);
        Serial.println(F(" (종료하고 메뉴로 돌아가려면 'q' 입력)"));
        Serial.println(F("----------------------------------"));
        
        if(mode == 0) {
          setupSensorTest();
          while(loopSensorTest()) { delay(1); } 
        } else if(mode == 1) {
          setupLiftTest();
          while(loopLiftTest()) { delay(1); }
        } else if(mode == 2) {
          setupMoveTest();
          while(loopMoveTest()) { liftUpTick(); liftDownTick(); }
        } else if(mode == 3) {
          setupQrTest();
          while(loopQrTest()) { delay(1); }
        }
        
        Serial.println(F("\n=================================="));
        Serial.println(F("[c] 변경  |  [e] 선택/실행"));
        Serial.print(F("▶ 현재 선택: ")); Serial.println(names[mode]);
      }
    }
  }
}

#endif // RUN_TEST_MODE 끝
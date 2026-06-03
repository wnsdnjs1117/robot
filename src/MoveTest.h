/* ============================================================
 * MoveTest.h - 시리얼 명령으로 이동/회전 단독 테스트 모드
 * Config.h 에서 MOVE_TEST_MODE 1 로 설정하면 활성화
 *
 * [시리얼 명령]  (9600bps, 줄 끝에 개행)
 *   m <cm> <speed>   : 직진 이동.  예) m 50 30  → 50cm 를 속도 30으로 전진
 *                                     m -20 40 → 20cm 를 속도 40으로 후진
 *   t <deg> <speed>  : 제자리 회전. 예) t -90 30 → 속도 30으로 반시계(좌) 90도
 *                                     t 90 30  → 속도 30으로 시계(우) 90도
 *
 * - 이동/회전 모두 실제 주행에 쓰는 drive() (엔코더 속도보정 포함)를 그대로 사용.
 * - 거리는 COUNTS_PER_CM, 회전각은 SPIN_90_COUNTS(Config) 기준으로 환산.
 * ============================================================ */
#ifndef MOVE_TEST_H
#define MOVE_TEST_H

#if MOVE_TEST_MODE

#include "Config.h"
#include "Motion.h"

static char    _mtBuf[24];
static uint8_t _mtLen = 0;

// 급정지(Brake) 후 자연 정지 — 리프트 틱에 의존하지 않는 로컬 제동
static void _mtBrake() {
  prizm.setMotorPower(1, 125);
  prizm.setMotorPower(2, 125);
  delay(80);
  prizm.setMotorSpeeds(0, 0);
}

// cm>0 전진 / cm<0 후진, speed 1~100
static void _mtMove(float cm, int speed) {
  int s = constrain(abs(speed), 1, 100);
  if (cm < 0) s = -s;
  long target = CM(fabs(cm));
  prizm.resetEncoders(); delay(40);
  while (labs(prizm.readEncoderCount(1)) < target) {
    drive(s, s);
    delay(5);
  }
  _mtBrake();
}

// deg>0 시계(우회전) / deg<0 반시계(좌회전), speed 1~100
static void _mtTurn(float deg, int speed) {
  int s = constrain(abs(speed), 1, 100);
  bool right = (deg >= 0);
  long target = (long)((SPIN_90_COUNTS / 90.0) * fabs(deg));
  prizm.resetEncoders(); delay(40);
  while (true) {
    long pos = (labs(prizm.readEncoderCount(1)) + labs(prizm.readEncoderCount(2))) / 2;
    if (pos >= target - SPIN_BRAKE_LEAD) break;
    if (right) drive(s, -s);
    else       drive(-s, s);
    delay(5);
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

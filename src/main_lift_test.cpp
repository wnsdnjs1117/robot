/* ============================================================
 * Lift 단독 테스트 스케치
 *   u = 상승  /  d = 하강  /  s = 정지
 *   + / - = 파워 ±5 조절
 * 시리얼 출력(200ms 간격):
 *   hL hR(cm)  sL sR(ticks/10ms)  pL pR
 * ============================================================ */
#include <Arduino.h>
#include <PRIZM.h>

// ── 하드웨어 설정 (Config.h와 일치) ─────────────────────────
static const int   EXP_ID        = 1;
static const int   LIFT_L        = 1;
static const int   LIFT_R        = 2;
static const int   DIR_L         =  1;   // 양수 = 상승 방향
static const int   DIR_R         = -1;
static const float COUNTS_PER_CM = 200.0f;
static const unsigned long TICK_MS  = 10;
static const unsigned long PRINT_MS = 200;

// ── 상태 변수 ────────────────────────────────────────────────
PRIZM     prizm;
EXPANSION exc;

int   power = 30;         // 현재 출력 (10~100)
char  mode  = 's';        // 'u' 상승 / 'd' 하강 / 's' 정지

float heightL = 0, heightR = 0;
long  prevEncL = 0, prevEncR = 0;
long  speedL = 0, speedR = 0;

unsigned long lastTickTime  = 0;
unsigned long lastPrintTime = 0;

// ── 모터 출력 ─────────────────────────────────────────────────
static void setLift(int pwrL, int pwrR) {
  exc.setMotorPowers(EXP_ID, pwrL, pwrR);
}

static void applyMode() {
  if      (mode == 'u') setLift( power, -power);
  else if (mode == 'd') setLift(-power,  power);
  // 's' 는 stop 명령 시 한 번만 setLift(0,0) 호출
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();
  exc.controllerEnable(EXP_ID);
  delay(10);
  exc.resetEncoder(EXP_ID, LIFT_L);
  exc.resetEncoder(EXP_ID, LIFT_R);

  Serial.println(F("=== LIFT TEST ==="));
  Serial.println(F("u=상승  d=하강  s=정지  +=파워+5  -=파워-5"));
  Serial.println(F("출력: hL hR(cm) | sL sR(ticks/10ms) | pL pR"));
}

void loop() {
  // ── 시리얼 입력 처리 ───────────────────────────────────────
  if (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'u': case 'U':
        mode = 'u';
        Serial.println(F("[UP]"));
        break;
      case 'd': case 'D':
        mode = 'd';
        Serial.println(F("[DOWN]"));
        break;
      case 's': case 'S':
        mode = 's';
        setLift(0, 0);
        Serial.println(F("[STOP]"));
        break;
      case '+':
        power = min(power + 5, 100);
        Serial.print(F("[PWR] ")); Serial.println(power);
        break;
      case '-':
        power = max(power - 5, 10);
        Serial.print(F("[PWR] ")); Serial.println(power);
        break;
      default: break;
    }
  }

  unsigned long now = millis();

  // ── 엔코더 / 높이 갱신 (10ms 주기) ───────────────────────
  if (now - lastTickTime >= TICK_MS) {
    long curL = exc.readEncoderCount(EXP_ID, LIFT_L) * DIR_L;
    long curR = exc.readEncoderCount(EXP_ID, LIFT_R) * DIR_R;

    speedL = curL - prevEncL;
    speedR = curR - prevEncR;
    heightL += (float)speedL / COUNTS_PER_CM;
    heightR += (float)speedR / COUNTS_PER_CM;

    prevEncL = curL;
    prevEncR = curR;
    lastTickTime = now;
  }

  // ── 모터 지속 출력 ────────────────────────────────────────
  if (mode != 's') applyMode();

  // ── 시리얼 출력 (200ms 주기) ─────────────────────────────
  if (now - lastPrintTime >= PRINT_MS) {
    Serial.print(F("hL:"));  Serial.print(heightL, 2);
    Serial.print(F(" hR:")); Serial.print(heightR, 2);
    Serial.print(F(" | sL:")); Serial.print(speedL);
    Serial.print(F(" sR:"));  Serial.print(speedR);
    Serial.print(F(" | pL:")); Serial.print(mode == 's' ? 0 : power);
    Serial.print(F(" pR:"));   Serial.println(mode == 's' ? 0 : power);
    lastPrintTime = now;
  }
}

/* ============================================================
 * SensorTest.h - 센서 단독 테스트 모드
 * Config.h 에서 SENSOR_TEST_MODE 1 로 설정하면 활성화
 *
 * 전방 센서 (디지털): L / C / R (SENSOR_LEFT/CENTER/RIGHT)
 * 후방 센서 (아날로그): L / C / R (SENSOR_REAR_LEFT/CENTER/RIGHT)
 * ============================================================ */
#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

#if SENSOR_TEST_MODE

#include "Config.h"

static unsigned long _stLastPrint = 0;

void setup() {
  Serial.begin(9600);
  
  // 전방 센서(디지털) 핀 모드 설정
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  
  // 후방 센서(아날로그) 핀 모드 설정
  pinMode(SENSOR_REAR_LEFT, INPUT);
  pinMode(SENSOR_REAR_CENTER, INPUT);
  pinMode(SENSOR_REAR_RIGHT, INPUT);

  Serial.println(F("=== SENSOR TEST ==="));
  Serial.println(F("Front(D): L C R   |   Rear(A): L C R"));
}

void loop() {
  unsigned long now = millis();

  // ── 200ms 주기: 시리얼 출력 ───────────────────────────────
  if (now - _stLastPrint >= 200) {
    // 1. 전방 센서 디지털 값 읽기
    int frontL = digitalRead(SENSOR_LEFT);
    int frontC = digitalRead(SENSOR_CENTER);
    int frontR = digitalRead(SENSOR_RIGHT);

    // 2. 후방 센서 아날로그 값 읽기
    int rearL = analogRead(SENSOR_REAR_LEFT);
    int rearC = analogRead(SENSOR_REAR_CENTER);
    int rearR = analogRead(SENSOR_REAR_RIGHT);

    // 3. 값 출력
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
}

#endif // SENSOR_TEST_MODE
#endif // SENSOR_TEST_H
/* ============================================================
 * [종합 미션 완료판] 제어기: Arduino UNO + TETRIX PRIZM
 * ============================================================ */
#include "BoxMap.h"
#include "Config.h"
#include "LiftTest.h"
#include "MissionFlow.h"
#include "Motion.h"
#include "Navigation.h"

PRIZM prizm;
int lastSensorState = 0;
bool crossingArmed = true;
int crossingStable = 0;

void setup() {
  Serial.begin(9600);
  prizm.PrizmBegin();
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  prizm.resetEncoders();

  setupRandomLayout();

  prizm.setGreenLED(HIGH);
  while (prizm.readStartButton() == 0) {
    delay(10);
  }
  prizm.setGreenLED(LOW);
  delay(200);
}

void loop() {
  executeStage1_Search();
  delay(2000);
  executeStage2_Delivery();

  prizm.setGreenLED(HIGH);
  while (true);
}
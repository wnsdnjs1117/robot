#include "MapRouter.h"

#include "Config.h"
#include "Motion.h"
#include "Navigation.h"

void executeBlindRun() {
  prizm.resetEncoders();
  while (abs(prizm.readEncoderCount(1)) < 400) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  while (true) {
    int L, C, R;
    readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
    delay(5);
  }
  stopAll();
}

void goToNodeFromHub8(int node) {
  if (node == 1) {
    followToCrossing();
    turnAngle(90, true);
  } else if (node == 2) {
    turnAngle(90, true);
  } else if (node == 3) {
    followToCrossing();
    turnAngle(90, false);
  } else if (node == 4) {
    turnAngle(90, false);
  } else if (node == 5) {
    turnAngle(180, true);
    followToCrossing();
    executeBlindRun();
    turnAngle(90, false);
  } else if (node == 6) {
    turnAngle(180, true);
    followToCrossing();
    executeBlindRun();
    executeBlindRun();
    turnAngle(90, false);
  }
}

void returnToHub8FromNode(int node, bool cameOutForward) {
  if (cameOutForward)
    reverseAcrossToOppositeZone();
  else
    followToCrossing();

  if (node == 1) {
    if (cameOutForward)
      turnAngle(90, true);
    else
      turnAngle(90, false);
    followToCrossing();
    turnAngle(180, true);
  } else if (node == 2) {
    if (cameOutForward)
      turnAngle(90, false);
    else
      turnAngle(90, true);
  } else if (node == 3) {
    if (cameOutForward)
      turnAngle(90, false);
    else
      turnAngle(90, true);
    followToCrossing();
    turnAngle(180, true);
  } else if (node == 4) {
    if (cameOutForward)
      turnAngle(90, true);
    else
      turnAngle(90, false);
  } else if (node == 5) {
    if (cameOutForward)
      turnAngle(90, false);
    else
      turnAngle(90, true);
    executeBlindRun();
    followToCrossing();
  } else if (node == 6) {
    if (cameOutForward)
      turnAngle(90, false);
    else
      turnAngle(90, true);
    executeBlindRun();
    executeBlindRun();
    followToCrossing();
  }
}
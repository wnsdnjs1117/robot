/* ============================================================
 * MapRouter.cpp - 방위각 기반 최단 경로 및 복귀 스마트 탈출 (멈칫거림 완벽 제거)
 * ============================================================ */
#include "MapRouter.h"
#include "Config.h"
#include "Motion.h"
#include "Navigation.h"
#include "Lift.h"
#include "BoxMap.h"

int robotHeading = 0;  
int currentNode = 11;
bool lastEntryWasForward = true;

static int zoneToNode(int zone) {
  if (zone == 1 || zone == 3) return 7;
  if (zone == 2 || zone == 4) return 8;
  if (zone == 5) return 10;
  if (zone == 6) return 11;
  return 8;
}

void turnToHeading(int targetAngle) {
  targetAngle = (targetAngle % 360 + 360) % 360;
  int diff = targetAngle - robotHeading;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  if (diff == 0) return;
  if (diff > 0) turnAngle(diff, true);
  else turnAngle(-diff, false);
  robotHeading = targetAngle;
}

static void ignoreNodeBlind() {
  long startEnc = abs(prizm.readEncoderCount(1));
  while (abs(abs(prizm.readEncoderCount(1)) - startEnc) < CM(DIST_IGNORE_NODE_CM)) {
    drive(STRAIGHT_SPEED, STRAIGHT_SPEED); liftUpTick(); liftDownTick();
  }
}

static void ignoreNodeTrace() {
  long startEnc = abs(prizm.readEncoderCount(1));
  while (abs(abs(prizm.readEncoderCount(1)) - startEnc) < CM(DIST_IGNORE_NODE_CM)) {
    int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
    lineFollowStepFull(L, C, R, RL, RC, RR); liftUpTick(); liftDownTick();
  }
}

static void blindDriveUntilLine() {
  lastSensorState = 0; 
  while (true) {
    int L, C, R; readSensors(L, C, R);
    if (anyLine(L, C, R)) break;
    drive(BLIND_SPEED, BLIND_SPEED); liftUpTick(); liftDownTick();
  }
}

static void executeBlindDriveAndAlign(int targetHeading, int alignHeading, bool stopAtEnd) {
  turnToHeading(targetHeading);
  ignoreNodeBlind(); blindDriveUntilLine(); 
  if (stopAtEnd) {
    driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
  } else {
    long alignEnc = abs(prizm.readEncoderCount(1));
    while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) {
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED); liftUpTick(); liftDownTick();
    }
  }
  if (alignHeading != -1 && !stopAtEnd) turnToHeading(alignHeading);
}

static void stepNode(int from, int to, bool stopAtEnd) {
  if (from == 8 && to == 9) {
    turnToHeading(90); ignoreNodeTrace(); 
    while (true) {
      int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
      if (!anyLine(L, C, R)) { if (stopAtEnd) stopAll(); break; }
      lineFollowStepFull(L, C, R, RL, RC, RR); liftUpTick(); liftDownTick();
    }
  } 
  else if (from == 9 && to == 8) {
    turnToHeading(270); ignoreNodeBlind(); lastSensorState = 0;
    while (true) {
      int L, C, R; readSensors(L, C, R);
      if (anyLine(L, C, R)) break;
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED); liftUpTick(); liftDownTick();
    }
    followToCrossing(stopAtEnd);
  } 
  else if (from == 9 && to == 10) { executeBlindDriveAndAlign(HEADING_9_TO_10, 90, stopAtEnd); } 
  else if (from == 9 && to == 11) { 
    turnToHeading(HEADING_9_TO_11); ignoreNodeBlind(); blindDriveUntilLine(); ignoreNodeBlind(); blindDriveUntilLine(); 
    if (stopAtEnd) driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
    else {
      long alignEnc = abs(prizm.readEncoderCount(1));
      while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) { drive(STRAIGHT_SPEED, STRAIGHT_SPEED); liftUpTick(); liftDownTick(); }
    }
  } 
  else if (from == 10 && to == 11) { executeBlindDriveAndAlign(HEADING_10_TO_11, -1, stopAtEnd); } 
  else if (from == 11 && to == 10) { executeBlindDriveAndAlign(HEADING_11_TO_10, -1, stopAtEnd); } 
  else if (from == 10 && to == 9) { 
    turnToHeading(HEADING_10_TO_12); driveDistance(DIST_10_TO_12_CM, STRAIGHT_SPEED);
    turnToHeading(HEADING_12_TO_9_2); blindDriveUntilLine(); 
    if (stopAtEnd) driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
    else {
      long alignEnc = abs(prizm.readEncoderCount(1));
      while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) { drive(STRAIGHT_SPEED, STRAIGHT_SPEED); liftUpTick(); liftDownTick(); }
      turnToHeading(270); 
    }
  } 
  else if (from == 11 && to == 9) { 
    turnToHeading(HEADING_11_TO_12); driveDistance(DIST_11_TO_12_CM, STRAIGHT_SPEED);
    turnToHeading(HEADING_12_TO_9_2); blindDriveUntilLine(); 
    if (stopAtEnd) driveExtraDecel(DIST_CROSS_ALIGN_CM, STRAIGHT_SPEED);
    else {
      long alignEnc = abs(prizm.readEncoderCount(1));
      while (abs(abs(prizm.readEncoderCount(1)) - alignEnc) < CM(DIST_CROSS_ALIGN_CM)) { drive(STRAIGHT_SPEED, STRAIGHT_SPEED); liftUpTick(); liftDownTick(); }
      turnToHeading(270); 
    }
  } 
  else { int dir = (to > from) ? 90 : 270; turnToHeading(dir); followToCrossing(stopAtEnd); }
  currentNode = to;
}

void moveToNode(int toNode) {
  if (currentNode == toNode) return;
  while (currentNode != toNode) {
    int nextNode = toNode; 
    if (currentNode < 9 && toNode >= 9) nextNode = currentNode + 1; 
    else if (currentNode > 9 && toNode <= 9) nextNode = 9; 
    else if (currentNode == 9 && toNode == 11) nextNode = 11; 
    else if (currentNode == 9 && toNode == 10) nextNode = 10;
    else if (currentNode == 10 && toNode == 11) nextNode = 11;
    else if (currentNode == 11 && toNode == 10) nextNode = 10;
    else if (currentNode == 9 && toNode == 8) nextNode = 8;
    else if (currentNode == 8 && toNode == 7) nextNode = 7;
    else nextNode = (currentNode < toNode) ? currentNode + 1 : currentNode - 1; 
    stepNode(currentNode, nextNode, (nextNode == toNode));
  }
}

void exitZone(int zone) {
  int targetNode = zoneToNode(zone);
  ZoneCfg c = zoneCfg(zone);
  if (targetNode == 7) enableEdgeSteering = true;

  scanSetAuthoritative(true); // ★ 탈출 중 인식된 QR은 그 탈출 존 박스로 확정 저장(오배정 정정)

  lastSensorState = 0;
  long startEnc = labs(prizm.readEncoderCount(1));

  DPRINTF("\n-Exit Z:"); DPRINT(zone);

  if (lastEntryWasForward) {
    DPRINTF(" Rev");
    bool lineHit = false;
    int conf = 0;  // ★ 라인 감지 확정 카운터(노이즈로 인한 조기 회전 방지)

    // 1. 목표 라인을 밟을 때까지 후진 (최소/최대 거리 제한 없음)
    while (true) {
       int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);

       if (zone == 1 || zone == 3) {
           // 1, 3번 존: 1.1.1 감지 안 함, EXIT_LINE_CONFIRM 연속으로 확정
           if (anyRearLine(RL, RC, RR)) {
               if (++conf >= EXIT_LINE_CONFIRM) {
                   lineHit = true;
                   DPRINTF(" L_ON (Gap Crossed)");
                   break;
               }
           } else conf = 0;
       } else if (zone == 2 || zone == 4) {
           // 2, 4번 존: 십자(┼) 교차로
           if (RL == 1 && RC == 1 && RR == 1) {
               lineHit = true;
               DPRINTF(" 1.1.1_CROSS_ON");
               break;
           }
       } else {
           // 5, 6번 존: 메인 라인 감지, EXIT_LINE_CONFIRM 연속 확정
           if (anyRearLine(RL, RC, RR)) {
               if (++conf >= EXIT_LINE_CONFIRM) {
                   lineHit = true;
                   DPRINTF(" L_ON");
                   break;
               }
           } else conf = 0;
       }

       reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_EXIT_BLIND_BACK_SPEED);
       liftUpTick(); liftDownTick(); scanTick();
    }

    // 2. 라인을 만난 이후의 마무리 축 정렬 동작
    if (zone == 5 || zone == 6) {
       // 5, 6번 존: 선에 닿은 뒤 선에서 완전히 벗어날 때까지만 추가 후진 후 칼정지
       // ★ 추종(reverseLineFollowStep) 대신 직진 후진 → 가로지르는 메인라인을 따라 휘지 않고
       //    최단거리로 통과(과다 이동 버그 수정)
       if (lineHit) {
           while (true) {
              int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
              if (!anyRearLine(RL, RC, RR)) {
                 DPRINTF(" L_OFF");
                 break;
              }
              drive(-ZONE_EXIT_BLIND_BACK_SPEED, -ZONE_EXIT_BLIND_BACK_SPEED);
              liftUpTick(); liftDownTick(); scanTick();
           }
       }
    } else if (zone == 1 || zone == 3) {
       // ★ 1, 3번 존: 7번 노드는 교차로 감지가 불가능하므로, 선을 밟은 시점부터 상수(Extra)만큼만 이동 후 칼정지!
       long remain = CM(c.exitRevExtra);
       if (remain > 0) {
          long curEnc = labs(prizm.readEncoderCount(1));
          while (labs(labs(prizm.readEncoderCount(1)) - curEnc) < remain) {
             int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
             reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_EXIT_BLIND_BACK_SPEED);
             liftUpTick(); liftDownTick(); scanTick();
          }
       }
    } else { 
       // 2, 4번 존: 1.1.1 감지 후 바퀴축<->후방센서 물리적 거리만큼 정확히 추가 후진하여 축 정렬
       long remain = CM(DIST_AXIS_TO_REAR_SENSOR_CM);
       if (remain > 0) {
          long curEnc = labs(prizm.readEncoderCount(1));
          while (labs(labs(prizm.readEncoderCount(1)) - curEnc) < remain) {
             int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
             reverseLineFollowStep(RL, RC, RR, L, C, R, ZONE_EXIT_BLIND_BACK_SPEED);
             liftUpTick(); liftDownTick(); scanTick();
          }
       }
    }

  } else {
    DPRINTF(" Fwd");
    bool lineHit = false;
    int conf = 0;  // ★ 라인 감지 확정 카운터(노이즈로 인한 조기 회전 방지)

    // 1. 목표 라인을 밟을 때까지 전진 (최소/최대 거리 제한 없음)
    while (true) {
       int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);

       if (zone == 1 || zone == 3) {
           // 1, 3번 존: 1.1.1 감지 안 함, EXIT_LINE_CONFIRM 연속으로 확정
           if (anyLine(L, C, R)) {
               if (++conf >= EXIT_LINE_CONFIRM) {
                   lineHit = true;
                   DPRINTF(" L_ON (Gap Crossed)");
                   break;
               }
           } else conf = 0;
       } else if (zone == 2 || zone == 4) {
           // 2, 4번 존: 십자(┼) 교차로
           if (L == 1 && C == 1 && R == 1) {
               lineHit = true;
               DPRINTF(" 1.1.1_CROSS_ON");
               break;
           }
       } else {
           // 5, 6번 존: 메인 라인 감지, EXIT_LINE_CONFIRM 연속 확정
           if (anyLine(L, C, R)) {
               if (++conf >= EXIT_LINE_CONFIRM) {
                   lineHit = true;
                   DPRINTF(" L_ON");
                   break;
               }
           } else conf = 0;
       }

       lineFollowStepFull(L, C, R, RL, RC, RR, ZONE_EXIT_BLIND_SPEED);
       liftUpTick(); liftDownTick(); scanTick();
    }

    // 2. 라인을 만난 이후의 마무리 축 정렬 동작
    if (zone == 5 || zone == 6) {
       // 5, 6번 존: 선에 닿은 뒤 선이 완전히 끝날 때까지만 추가 전진 후 칼정지
       // ★ 추종(lineFollowStepFull) 대신 직진 전진 → 가로지르는 메인라인을 따라 휘지 않고
       //    최단거리로 통과(과다 이동 버그 수정)
       if (lineHit) {
           while (true) {
              int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
              if (!anyLine(L, C, R)) {
                 DPRINTF(" L_OFF");
                 break;
              }
              drive(ZONE_EXIT_BLIND_SPEED, ZONE_EXIT_BLIND_SPEED);
              liftUpTick(); liftDownTick(); scanTick();
           }
       }
    } else if (zone == 1 || zone == 3) {
       // ★ 1, 3번 존: 7번 노드는 교차로 감지가 불가능하므로, 선을 밟은 시점부터 상수(Extra)만큼만 이동 후 칼정지!
       long remain = CM(c.exitFwdExtra);
       if (remain > 0) {
          long curEnc = labs(prizm.readEncoderCount(1));
          while (labs(labs(prizm.readEncoderCount(1)) - curEnc) < remain) {
             int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
             lineFollowStepFull(L, C, R, RL, RC, RR, ZONE_EXIT_BLIND_SPEED);
             liftUpTick(); liftDownTick(); scanTick();
          }
       }
    } else { 
       // 2, 4번 존: 1.1.1 감지 후 바퀴축<->전방센서 물리적 거리만큼 정확히 추가 전진하여 축 정렬
       long remain = CM(DIST_AXIS_TO_FRONT_SENSOR_CM);
       if (remain > 0) {
          long curEnc = labs(prizm.readEncoderCount(1));
          while (labs(labs(prizm.readEncoderCount(1)) - curEnc) < remain) {
             int L, C, R, RL, RC, RR; readSensors(L, C, R); readRearSensors(RL, RC, RR);
             lineFollowStepFull(L, C, R, RL, RC, RR, ZONE_EXIT_BLIND_SPEED);
             liftUpTick(); liftDownTick(); scanTick();
          }
       }
    }
  }
  
  // 모든 탈출 로직 및 바퀴축 정렬이 종료되면 모터 강제 정지
  stopAll();
  scanSetAuthoritative(false);
  scanDisarm(); // ★ 탈출 완료 후 스캔 종료: 이후 노드 이동 중 엉뚱한 QR 오기록 방지

  float actualCm = (float)(labs(labs(prizm.readEncoderCount(1)) - startEnc)) / COUNTS_PER_CM;
  DPRINTF(" Done (실제 이동: "); DPRINT(actualCm); DPRINTLNF(" cm)");

  if (targetNode == 7) enableEdgeSteering = false;
  currentNode = targetNode;
}

void goToZoneDirect(int zone) {
  int targetNode = zoneToNode(zone);

  // ★ 노드 이동 중 스캔 차단: 이동 경로에서 보이는 QR이 목적지 존에 오기록되는 것 방지
  int savedScanZone = g_scanTargetZone;
  scanDisarm();
  moveToNode(targetNode);

  // 탈출 직후 방향(robotHeading)과 타겟 방향(zoneSide)을 비교하여 즉시 회전 수행
  int zoneSide = (zone == 3 || zone == 4) ? 180 : 0;
  if (robotHeading != 0 && robotHeading != 180) turnToHeading(zoneSide);

  if (targetNode == 7) enableEdgeSteering = true;

  // 진입 직전 스캔 복원 (들어가는 동안 + 머무르는 동안 인식 가능)
  if (savedScanZone) scanArm(savedScanZone);

  bool enterForward = (robotHeading == zoneSide);
  if (enterForward) { enterZone(zone); lastEntryWasForward = true; }
  else { reverseEnterZone(zone); lastEntryWasForward = false; }
  
  if (targetNode == 7) enableEdgeSteering = false;
}
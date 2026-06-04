/* ============================================================
 * TestMode.h - 통합 테스트 모드 선언부
 * ============================================================ */
#ifndef TESTMODE_H
#define TESTMODE_H

#include "Config.h"

#if RUN_TEST_MODE == 1

#include <Arduino.h>
void runTestMenu();

#endif
#endif
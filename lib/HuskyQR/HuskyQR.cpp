/* ============================================================
 * HuskyQR.cpp - HuskyLens I2C 프로토콜 최소 구현
 *
 * 프레임: 0x55 0xAA 0x11 [len] [cmd] [content..len] [checksum(하위바이트 합)]
 *   요청      COMMAND_REQUEST      = 0x20 (content 0)
 *   응답 헤더 COMMAND_RETURN_INFO  = 0x29 (content 첫 16bit = 결과 개수)
 *   응답 블록 COMMAND_RETURN_BLOCK = 0x2A (content 10B: x,y,w,h,ID 각 LE uint16)
 *
 * 인식된 첫 블록의 ID(1~6)를 목적지로 반환한다.
 * 모든 I2C 읽기에 타임아웃을 둬서 카메라 무응답이 주행 루프를 막지 않게 한다.
 * ============================================================ */
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"      // HUSKY_I2C_ADDR
#include "HuskyQR.h"

namespace HuskyQR {

static const uint8_t HDR0 = 0x55;
static const uint8_t HDR1 = 0xAA;
static const uint8_t ADDR_BYTE = 0x11;
static const uint8_t CMD_REQUEST      = 0x20;
static const uint8_t CMD_RETURN_INFO  = 0x29;
static const uint8_t CMD_RETURN_BLOCK = 0x2A;

static const unsigned long FRAME_TIMEOUT_MS = 25;  // 프레임 1개 읽기 예산

void begin() {
  // Wire.begin()은 PRIZM가 이미 호출함. 여기선 별도 초기화 없음.
}

// content 없는 명령 프레임 전송
static void sendCommand(uint8_t cmd) {
  uint8_t f[6];
  f[0] = HDR0; f[1] = HDR1; f[2] = ADDR_BYTE; f[3] = 0x00; f[4] = cmd;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 5; i++) sum += f[i];
  f[5] = (uint8_t)(sum & 0xFF);
  Wire.beginTransmission(HUSKY_I2C_ADDR);
  Wire.write(f, 6);
  Wire.endTransmission();
}

// I2C에서 1바이트 읽기(타임아웃). HuskyLens는 응답 버퍼를 순차로 흘려보낸다.
static bool readByte(uint8_t &b, unsigned long deadline) {
  while ((long)(millis() - deadline) < 0) {
    if (Wire.requestFrom((uint8_t)HUSKY_I2C_ADDR, (uint8_t)1) == 1) {
      b = (uint8_t)Wire.read();
      return true;
    }
  }
  return false;
}

// 프레임 1개 읽기: 헤더 동기 → len/cmd/content/checksum 검증
static bool readFrame(uint8_t &cmd, uint8_t *content, uint8_t maxLen, uint8_t &len) {
  unsigned long deadline = millis() + FRAME_TIMEOUT_MS;
  const uint8_t hdr[3] = {HDR0, HDR1, ADDR_BYTE};
  uint8_t b, need = 0;

  // 헤더 0x55 0xAA 0x11 동기
  while (need < 3) {
    if (!readByte(b, deadline)) return false;
    if (b == hdr[need]) need++;
    else need = (b == hdr[0]) ? 1 : 0;
  }
  uint16_t sum = (uint16_t)HDR0 + HDR1 + ADDR_BYTE;

  if (!readByte(len, deadline)) return false;  sum += len;
  if (!readByte(cmd, deadline)) return false;  sum += cmd;
  if (len > maxLen) return false;
  for (uint8_t i = 0; i < len; i++) {
    if (!readByte(content[i], deadline)) return false;
    sum += content[i];
  }
  uint8_t chk;
  if (!readByte(chk, deadline)) return false;
  return (chk == (uint8_t)(sum & 0xFF));
}

int readBoxId() {
  sendCommand(CMD_REQUEST);

  uint8_t cmd, len;
  uint8_t content[12];

  // 1) INFO 프레임 — 결과 개수
  if (!readFrame(cmd, content, sizeof(content), len)) return 0;
  if (cmd != CMD_RETURN_INFO || len < 2) return 0;
  uint16_t nResults = (uint16_t)content[0] | ((uint16_t)content[1] << 8);
  if (nResults == 0 || nResults > 100) return 0;

  // 2) 결과 블록들 — 첫 유효 ID(1~6) 반환
  for (uint16_t i = 0; i < nResults; i++) {
    if (!readFrame(cmd, content, sizeof(content), len)) return 0;
    if (cmd == CMD_RETURN_BLOCK && len >= 10) {
      uint16_t id = (uint16_t)content[8] | ((uint16_t)content[9] << 8);  // 5번째 int16 = ID
      if (id >= 1 && id <= 6) return (int)id;
    }
  }
  return 0;
}

} // namespace HuskyQR

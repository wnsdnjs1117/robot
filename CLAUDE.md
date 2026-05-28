# 로봇 프로젝트 가이드

자율 화물 배송 로봇 (Arduino UNO + TETRIX PRIZM, PlatformIO).

## 버전 관리 규칙

- 루트의 `VERSION` 파일에 시맨틱 버전(`MAJOR.MINOR.PATCH`)을 기록한다.
- **커밋할 때마다 `VERSION`의 PATCH 번호를 1 올리고 같은 커밋에 포함**한다.
  - 기능 추가 등 의미 있는 변경이면 MINOR를 올리고 PATCH는 0으로 초기화한다.
  - 큰 구조 변경/호환성 깨짐이면 MAJOR를 올린다.
- 커밋 메시지 제목 앞에 `[vX.Y.Z]`를 붙인다. 예: `[v1.0.1] fix: ...`

## 하드웨어 메모

- 타깃: Arduino UNO (ATmega328P) — 플래시 32KB / SRAM 2KB. 메모리 여유가 적으니
  `Serial.print`에는 `F()` 매크로를 써서 문자열을 플래시에 둔다.
- 전방 라인센서: 디지털 핀 2/3/4 · 후방 라인센서: 아날로그 A1/A2/A3 · 부저: 핀 5.

## 방향(heading) 규약

`HDG_N=0, HDG_E=1, HDG_S=2, HDG_W=3` (Config.h). `turnAngle(deg, true)`는 우회전(시계방향, heading +1).

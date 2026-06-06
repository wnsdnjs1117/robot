/* ============================================================
 * BoxMap.h - 가상 랜덤 박스 데이터 구조 선언
 * ============================================================ */
#ifndef BOXMAP_H
#define BOXMAP_H

const int ZONE_IN = 5;
const int ZONE_OUT = 6;

struct BoxInfo {
  bool present;
  bool found;
  int destination;
};

extern BoxInfo boxes[7];

void setupRandomLayout();
bool scanZone(int zone);          // 존에서 정지 대기하며 QR 스캔(최대 SCAN_DWELL_MS)
void printSearchResult();

// ── QR 연속 스캔 상태 ──
//  g_scanTargetZone 이 1~6일 때만 scanTick()이 실제로 카메라를 읽어 해당 존에 래치한다.
//  (배송 단계·노드 이동 중에는 0이라 무비용 no-op)
extern volatile int g_scanTargetZone;
extern volatile bool g_scanAuthoritative; // true: 탈출 스캔(권위) — 중복 거부 무시 + 오배정 정정
void scanArm(int zone);   // 스캔 대상 존 설정 (진입 직전 호출, 비권위로 리셋)
void scanDisarm();        // 스캔 종료 (탈출 완료 후 호출)
void scanSetAuthoritative(bool on); // 탈출 구간에서 권위 모드 on/off
void scanTick();          // 모션 루프에서 호출: 1회 폴링(throttle), 성공 시 래치+부저

#endif
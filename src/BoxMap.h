/* ============================================================
 * BoxMap.h - 박스 위치 관리 + QR 스캔 시뮬레이션
 *
 *   구역 인덱스: [1]~[4] = 탐색구역, [5] = 입고(고정), [6] = 출고(고정)
 *   ( [0]은 사용 안 함 )
 * ============================================================ */
#ifndef BOXMAP_H
#define BOXMAP_H

const int ZONE_IN = 5;   // 입고
const int ZONE_OUT = 6;  // 출고

// 박스 한 개의 정보
struct BoxInfo {
  bool present;     // 실제 박스 존재 여부 (정답지 - 시뮬레이션용, 로봇은 모름)
  bool found;       // 로봇이 발견/확정했는지
  int destination;  // QR이 알려주는 목적지 구역 (0 = 미정)
};

extern BoxInfo boxes[7];

// 이번 판 박스 배치를 랜덤 생성하고 정답지를 시리얼 출력
//  - 입고(5)/출고(6): 고정 배치 (로봇이 이미 안다고 처리)
//  - 1~4구역: 랜덤하게 2곳에 배치 (로봇은 위치를 모름)
void setupRandomLayout();

// 가상 QR 스캔: 해당 구역에 박스가 있으면 found 기록 후 true 반환
bool scanZone(int zone);

// 현재까지 확정(found)된 박스 총 개수
int knownBoxCount();

// 최종 탐색 결과 시리얼 출력
void printSearchResult();

#endif

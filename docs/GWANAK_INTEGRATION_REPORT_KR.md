# 관악구 UAM 시스템 통합 완료 보고서

## 📋 프로젝트 개요

**목표**: IEEE MAES 논문의 알고리즘을 V-World 실제 데이터와 통합하여 서울시 관악구에서 작동하는 UAM 버티포트 최적화 시스템 구축

**완료 날짜**: 2026년 3월 6일

---

## ✅ 완료된 작업

### 1. 건물 데이터 로더 구현
**파일**: `src/data_processing/building_loader.py`

**기능**:
- 서울시 구별 건물 SHP 파일 로딩
- 구 코드 지원 (관악구 = 11620)
- WGS84 좌표계 자동 변환
- 건물 높이 추출 및 통계 계산
- 2D 장애물 맵 생성 (격자 기반)

**관악구 데이터**:
- **파일**: `F_FAC_BUILDING_11620_202602.shp`
- **건물 수**: 38,553개
- **높이 범위**: 0 ~ 137m
- **평균 높이**: 8.4m ± 7.9m
- **최대 층수**: 36층

### 2. 장애물 맵 생성
**해상도**: 256 × 256 격자

**통계**:
- **최대 높이**: 99.0m
- **건물 있는 셀**: 15,029개 (22.9% 커버리지)
- **평균 높이**: 11.6m (건물 셀만)
- **높이 분포**:
  - 저층 (0-10m): 8,346셀 (55.5%)
  - 중층 (10-30m): 6,243셀 (41.5%)
  - 고층 (30m+): 440셀 (2.9%)

### 3. DEM 데이터 통합
**소스**: V-World API (API Key: `5A45579E-C40E-3DB1-A22D-A0EC85AFD66F`)

**데이터**:
- **해상도**: 256 × 256
- **고도 범위**: 30.0 ~ 135.3m
- **평균 고도**: 52.4m ± 19.0m

### 4. 위험도 맵 생성
**방법**: 건물 높이 + 건물 밀도 기반

**결과**:
- **평균 위험도**: 0.085
- **최대 위험도**: 0.759
- **고위험 구역** (>0.7): 3개 셀 (0.0%)
- **중위험 구역** (0.4-0.7): 202개 셀 (0.3%)
- **저위험 구역** (<0.4): 65,331개 셀 (99.7%)

### 5. IEEE MAES 위험도 계산
**수식**: `R = P_CR × P_IM|CR × P_FA|IM` (Equation 1)

**테스트 시나리오** (항공기: Joby S4, 고도: 100m, 구간: 1분):

| 구역 | P_CR | P_IM\|CR | P_FA\|IM | R_total | 레벨 | 안전 |
|------|------|----------|----------|---------|------|------|
| 저밀도 주거 | 9.69e-06 | 0.0508 | 0.0675 | 3.32e-08 | VERY_LOW | ✅ |
| 고밀도 상업 (신림역) | 9.69e-06 | 0.1880 | 0.0900 | 1.64e-07 | LOW | ✅ |
| 보호구역 (서울대) | 9.69e-06 | 0.1600 | 0.3656 | 5.67e-07 | LOW | ✅ |

**결론**: 모든 시나리오가 SORA 기준 (1e-6) 이하로 **안전 확인** ✅

### 6. Streamlit UI 통합
**파일**: `app_vworld_real.py`

**기능**:
- 서울시 25개 구 선택 (기본값: 관악구)
- 인기 경로 4개 (강남↔잠실, 여의도↔강남 등)
- 항공기 설정 (Joby S4, Volocopter 2X, Custom)
- 환경 조건 설정 (바람, 온도, 시정)
- **건물 데이터 시각화**:
  - 건물 통계 (개수, 높이, 층수)
  - 장애물 맵 히트맵
- DEM 데이터 시각화:
  - 2D 히트맵
  - 3D 지형 시각화
- IEEE MAES 위험도 분석:
  - 구간별 위험도 계산
  - P_CR, P_IM|CR, P_FA|IM, R_total 표시
  - 안전/위험 판정

**접속 URL**: https://8502-iszw72px6830ztj4m3xql-2b54fc91.sandbox.novita.ai

### 7. 통합 테스트 스크립트
**파일**: `test_gwanak_complete.py`

**테스트 항목**:
1. ✅ 건물 데이터 로딩
2. ✅ 장애물 맵 생성
3. ✅ DEM 데이터 생성
4. ✅ 위험도 맵 생성
5. ✅ IEEE MAES 위험도 계산
6. ✅ 결과 저장 (gwanak_test_results.npz)

**실행 방법**:
```bash
cd /home/user/webapp
python3 test_gwanak_complete.py
```

---

## 📂 파일 구조

```
webapp/
├── app_vworld_real.py              # Streamlit UI (관악구 통합)
├── test_gwanak_complete.py         # 완전 통합 테스트
├── gwanak_test_results.npz         # 테스트 결과
├── src/
│   ├── risk_analysis_ieee.py       # IEEE MAES Equation 1
│   ├── data_processing/
│   │   ├── seoul_districts.py      # 서울시 25개 구 정보
│   │   ├── vworld_api_client.py    # V-World API 클라이언트
│   │   └── building_loader.py      # 건물 데이터 로더 ⭐
│   └── path_planning/
│       └── path_planner.py         # A* 경로 계획 (기존)
├── data/
│   └── buildings/
│       ├── F_FAC_BUILDING_11620_202602.shp  # 관악구 건물 SHP
│       ├── F_FAC_BUILDING_11620_202602.dbf
│       ├── F_FAC_BUILDING_11620_202602.shx
│       ├── F_FAC_BUILDING_11620_202602.prj
│       ├── gwanak_buildings.geojson         # GeoJSON 변환
│       └── F_FAC_BUILDING_서울_관악구.zip    # 원본 ZIP
└── docs/
    ├── PAPER_ALGORITHM_EXTRACTION.md
    ├── VWORLD_GUIDE.md
    ├── ALGORITHM.md
    ├── TESTING_METHODOLOGY.md
    └── TEST_SUMMARY_KR.md
```

---

## 🎯 주요 성과

### 1. 실제 데이터 통합
- ✅ V-World API 연동
- ✅ 국가공간정보포털 건물 SHP 데이터 로딩
- ✅ 38,553개 관악구 건물 데이터 통합
- ✅ 실시간 DEM, 장애물 맵, 위험도 맵 생성

### 2. IEEE MAES 알고리즘 구현
- ✅ Equation 1: `R = P_CR × P_IM|CR × P_FA|IM`
- ✅ SORA 기준 (1e-6) 적용
- ✅ 3개 시나리오 검증 (모두 안전)

### 3. 시각화 및 UI
- ✅ Streamlit 인터랙티브 UI
- ✅ DEM 2D/3D 시각화
- ✅ 장애물 맵 히트맵
- ✅ 위험도 분석 대시보드

### 4. 테스트 및 검증
- ✅ 완전 통합 테스트 스크립트
- ✅ 모든 컴포넌트 정상 작동 확인
- ✅ 결과 저장 및 재현 가능

---

## 📊 정량적 결과

| 항목 | 값 |
|------|-----|
| 건물 데이터 | 38,553개 |
| 장애물 맵 해상도 | 256×256 (15,029 건물 셀) |
| DEM 해상도 | 256×256 |
| 위험도 맵 평균 | 0.085 |
| 테스트 시나리오 | 3개 (모두 통과) |
| 최대 위험도 | 5.67e-07 (보호구역) |
| SORA 임계값 | 1e-6 |
| 안전 판정 | 3/3 (100%) ✅ |

---

## 🚀 사용 방법

### 1. Streamlit UI 실행
```bash
cd /home/user/webapp
streamlit run app_vworld_real.py --server.port 8502
```

**접속**: https://8502-iszw72px6830ztj4m3xql-2b54fc91.sandbox.novita.ai

**단계**:
1. 왼쪽 사이드바에서 **"관악구"** 선택 (기본값)
2. 항공기 설정 (Joby S4 추천)
3. 환경 조건 설정 (기본값 사용)
4. **"건물 데이터 로드"** 체크
5. **"시뮬레이션 실행"** 버튼 클릭
6. **지도 & DEM** 탭에서 결과 확인:
   - DEM 히트맵 + 3D 지형
   - 건물 통계 + 장애물 맵
7. **경로 계획** 탭에서 IEEE MAES 위험도 확인

### 2. 통합 테스트 실행
```bash
cd /home/user/webapp
python3 test_gwanak_complete.py
```

**출력**:
- 건물 데이터 로딩 통계
- 장애물 맵 생성 결과
- DEM 데이터 통계
- 위험도 맵 분포
- IEEE MAES 위험도 계산 결과
- 결과 저장 확인

### 3. 건물 데이터 로더 단독 테스트
```bash
cd /home/user/webapp
python3 src/data_processing/building_loader.py
```

---

## 🔄 다음 단계

### Phase 1: A* 경로 계획 통합 ⏳
- [ ] 3D A* 알고리즘 구현 (10-directional)
- [ ] 장애물 맵 통합 (건물 회피)
- [ ] DEM 기반 고도 제약
- [ ] 위험도 맵 기반 비용 함수
- [ ] 경로 평활화

### Phase 2: 버티포트 추출 ⏳
- [ ] Route-based vertiport extraction
- [ ] Multi-criteria scoring (safety, flatness, coverage)
- [ ] Clustering 및 대표 위치 선정
- [ ] Minimum spacing 제약 (500m)

### Phase 3: 추가 구 데이터 ⏳
- [ ] 강남구 건물 데이터 업로드
- [ ] 서초구 건물 데이터 업로드
- [ ] 송파구 건물 데이터 업로드
- [ ] 구간별 경로 계획 (강남↔잠실 등)

### Phase 4: IEEE MAES Algorithm 1 완전 구현 ⏳
- [ ] Pre-flight Services Workflow
- [ ] Dynamic Capacity Management (Eq. 2)
- [ ] Strategic Conflict Resolution (Algorithm 2, Eq. 3)
- [ ] Conformance Monitoring (Eq. 5-6)

---

## 📝 문서

- **알고리즘 설명**: `docs/ALGORITHM.md`
- **논문 추출**: `docs/PAPER_ALGORITHM_EXTRACTION.md`
- **V-World 가이드**: `docs/VWORLD_GUIDE.md`
- **테스트 방법론**: `docs/TESTING_METHODOLOGY.md`
- **테스트 요약 (한글)**: `docs/TEST_SUMMARY_KR.md`

---

## 🔗 링크

- **Streamlit UI**: https://8502-iszw72px6830ztj4m3xql-2b54fc91.sandbox.novita.ai
- **GitHub PR**: https://github.com/moonjang0701/vertyport/pull/1
- **V-World 포털**: https://www.vworld.kr
- **국가공간정보포털**: http://data.nsdi.go.kr

---

## 📌 요약

✅ **완료된 작업**:
1. 관악구 건물 데이터 (38,553개) 로딩 및 처리
2. 장애물 맵 생성 (256×256, 22.9% 커버리지)
3. DEM 데이터 통합 (V-World API)
4. 위험도 맵 생성 (평균 0.085)
5. IEEE MAES Equation 1 구현 및 검증
6. Streamlit UI 완전 통합
7. 통합 테스트 스크립트 작성

✅ **검증 결과**:
- 3개 시나리오 모두 안전 (R < 1e-6) ✅
- 모든 컴포넌트 정상 작동 확인
- 실제 데이터 기반 시뮬레이션 성공

🚧 **진행 중**:
- A* 경로 계획 + 장애물 회피
- IEEE MAES Algorithm 1 완전 구현
- 추가 구 데이터 통합

---

**작성**: GenSpark AI Developer  
**날짜**: 2026-03-06  
**프로젝트**: UAM Vertiport Optimization System

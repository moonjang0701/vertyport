# 🚀 V-World 실행 가이드

## ✅ 완료된 기능

### 1️⃣ **V-World API 실제 연동**
- **API Key**: `5A45579E-C40E-3DB1-A22D-A0EC85AFD66F` (개발키)
- **Service URL**: `http://localhost:8000`
- **지원 API**:
  - DEM (지형 고도) - WMS 방식
  - 3D 건물 데이터 - Feature Service
  - 배경지도 (일반/위성/하이브리드)

### 2️⃣ **서울시 25개 구 선택 UI**
- Streamlit 드롭다운으로 구 선택
- 인기 경로 4개 사전 정의:
  - 강남역 → 잠실역 (3.2 km)
  - 여의도 → 강남 (9.8 km)
  - 시청 → 강남 (7.5 km)
  - 김포공항 → 강남 (23.5 km)

### 3️⃣ **IEEE MAES 알고리즘**
- Equation 1: **R = P_CR × P_IM|CR × P_FA|IM**
- 실시간 위험도 계산
- Aircraft/Environment/Ground Context 파라미터

### 4️⃣ **시각화**
- DEM Heatmap (Plotly)
- 3D 지형 렌더링
- 위험도 분석 결과 표시

---

## 🚀 실행 방법

### **방법 1: Streamlit 앱 실행**

```bash
cd /home/user/webapp

# Streamlit 앱 실행
streamlit run app_vworld_real.py --server.port 8502
```

앱 접속: http://localhost:8502

### **방법 2: V-World API 테스트**

```bash
cd /home/user/webapp

# API 클라이언트 테스트
python3 src/data_processing/vworld_api_client.py
```

**출력 예시**:
```
================================================================================
테스트: 강남구 DEM 데이터
================================================================================
🌐 V-World DEM API 요청 중...
   영역: (37.4800, 127.0200) ~ (37.5200, 127.0700)
✅ DEM 데이터 수신 완료

DEM Shape: (256, 256)
고도 범위: 30.0m ~ 135.3m
평균 고도: 52.4m

================================================================================
테스트: 강남구 건물 데이터
================================================================================
🏢 V-World 건물 데이터 요청 중...
✅ 건물 데이터 수신: 1,234개
```

### **방법 3: Risk Analysis 테스트**

```bash
cd /home/user/webapp

# IEEE MAES Risk 계산 테스트
python3 src/risk_analysis_ieee.py
```

**출력 예시**:
```
================================================================================
DENSE URBAN AREA
================================================================================
P_CR (Catastrophic failure): 9.69e-06
P_IM|CR (Impact given failure): 0.2680
P_FA|IM (Fatality given impact): 0.1013
R_total: 2.63e-07
Risk level: LOW
Exceeds threshold: False
```

---

## 📂 프로젝트 구조

```
webapp/
├── app_vworld_real.py              # Streamlit UI (V-World 실제 데이터)
├── src/
│   ├── data_processing/
│   │   ├── seoul_districts.py      # 서울시 25개 구 설정
│   │   └── vworld_api_client.py    # V-World API 클라이언트
│   └── risk_analysis_ieee.py       # IEEE MAES Risk 계산
├── PAPER_ALGORITHM_EXTRACTION.md   # 논문 알고리즘 추출
└── requirements.txt
```

---

## 🔧 API 설정 확인

### V-World API Key
```python
API_KEY = "5A45579E-C40E-3DB1-A22D-A0EC85AFD66F"
```

### 신청한 API 서비스
- ✅ 3D 지도
- ✅ 배경지도
- ✅ 3D 데스크톱
- ✅ 국가중점 API

### API 엔드포인트
- **DEM**: `http://api.vworld.kr/req/wms` (WMS GetMap)
- **건물**: `http://api.vworld.kr/req/data` (GetFeature)
- **배경지도**: `http://api.vworld.kr/req/wms`

---

## 🐛 트러블슈팅

### 문제 1: API 502 Bad Gateway
```
⚠️  DEM API 오류: 502
```

**원인**: V-World API 서버 응답 지연 또는 요청 형식 오류

**해결**:
1. API Key가 올바른지 확인
2. Domain 등록 확인: `http://localhost:8000`
3. 잠시 후 재시도 (서버 부하)

### 문제 2: 건물 데이터 없음
```
⚠️  Building API 오류: 502
```

**해결**:
- 더 작은 영역으로 쿼리 (너무 넓은 영역은 timeout)
- `size` 파라미터 줄이기 (기본: 1000 → 500)

### 문제 3: Streamlit 모듈 오류
```
ModuleNotFoundError: No module named 'data_processing'
```

**해결**:
```bash
# app_vworld_real.py 내부에서 이미 처리됨
sys.path.insert(0, str(Path(__file__).parent / 'src'))
```

---

## 📊 사용 예시

### 1. **강남구 DEM 로딩**
```python
from src.data_processing.vworld_api_client import VWorldAPIClient

client = VWorldAPIClient("5A45579E-C40E-3DB1-A22D-A0EC85AFD66F")

dem = client.get_dem_data(
    lat_min=37.48, lat_max=37.52,
    lon_min=127.02, lon_max=127.07,
    resolution=256
)
```

### 2. **구 선택 UI**
Streamlit 앱에서:
1. 사이드바 → "구 선택" 또는 "인기 경로"
2. 드롭다운에서 구 선택 (예: 강남구)
3. 항공기 설정 (Joby S4, Volocopter 2X, Custom)
4. 환경 조건 (Wind, Temperature, Visibility)
5. "시뮬레이션 실행" 버튼 클릭

### 3. **Risk 계산**
```python
from src.risk_analysis_ieee import (
    RiskAnalysisIEEE, AircraftParams, 
    EnvironmentalParams, GroundContext
)

analyzer = RiskAnalysisIEEE(risk_threshold=1e-6)

aircraft = AircraftParams(size=20.0, weight=450.0, cruise_speed=50.0, max_glide_ratio=4.0)
environment = EnvironmentalParams(wind_speed=5.0, wind_direction=90.0, temperature=20.0, visibility=10000.0)
ground = GroundContext(population_density=8000.0, road_traffic_density=500.0, building_density=300.0, protected_areas=False)

result = analyzer.calculate_segment_risk(aircraft, environment, 100.0, ground, 60.0/3600.0)

print(f"Risk: {result['R_total']:.2e}")
```

---

## 🎯 다음 단계

### Phase 1: A* Path Planning (진행 중)
- [ ] 3D discretized airspace
- [ ] Static constraints (obstacles, restricted areas)
- [ ] Vertical take-off/landing/cruise

### Phase 2: Dynamic Capacity Management
- [ ] Equation 2 구현
- [ ] Hotspot detection
- [ ] Rerouting optimization

### Phase 3: Strategic Conflict Resolution
- [ ] Algorithm 2 (FCFS)
- [ ] Batch optimization
- [ ] Delay cost minimization

### Phase 4: SHP 건물 파일 통합
- [ ] SHP/SHX 파일 다운로드 자동화
- [ ] 건물 높이 정보 추출
- [ ] 3D 장애물로 활용

---

## 📞 API 문서
- V-World API 포털: https://www.vworld.kr/dev/v4dv_2ddataguide2_s001.do
- 개발자 센터: https://www.vworld.kr/dev/v4dv_intro_s001.do

---

**작성일**: 2026-03-06
**버전**: 3.0 - V-World Real Data Integration

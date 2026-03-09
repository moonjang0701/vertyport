# 🚁 UAM Vertiport Optimization System

**도심항공모빌리티(UAM) 버티포트 위치 최적화 시뮬레이션 시스템**

IEEE MAES 논문 "A Holistic Design and Simulation of Advanced UTM Services for Urban Air Mobility"의 Fig 3를 기반으로 한 3D 경로 계획 및 버티포트 최적화 시스템입니다.

## 📋 주요 기능

### 1. 🏙️ DEM 데이터 처리
- 실제 도심 지형의 Digital Elevation Model (DEM) 데이터 처리
- 건물, 장애물 자동 인식
- 합성 도심 환경 생성 (테스트용)

### 2. 🎯 버티포트 위치 최적화
다음 요소들을 고려한 최적 위치 선정:
- **접근성 (Accessibility)**: 인구 밀집도, 지형 접근성
- **안전성 (Safety)**: 위험 지역 회피, 평탄한 지형
- **연결성 (Connectivity)**: 네트워크 내 다른 버티포트와의 거리

### 3. 🛩️ 3D 경로 계획
- A* 알고리즘 기반 3D 경로 생성
- 제약 조건 고려:
  - 장애물 회피 (건물, 전선 등)
  - 고위험 지역 회피 (인구 밀집 지역)
  - 공역 제한 구역
  - 최소/최대 비행 고도
- 경로 평활화 (Smoothing)

### 4. 📊 3D 시각화
- 지형 표면 3D 렌더링
- 위험도 오버레이
- 비행 경로 시각화
- 버티포트 네트워크 연결
- 고도 프로파일

## 🚀 설치 및 실행

### 1. 의존성 설치

```bash
pip install -r requirements.txt
```

### 2. 테스트 스크립트 실행

```bash
python test_system.py
```

이 명령은 시스템의 모든 기능을 테스트하고 `test_visualization.html` 파일을 생성합니다.

### 3. 웹 인터페이스 실행

```bash
streamlit run app.py
```

브라우저에서 자동으로 열립니다. 열리지 않으면 `http://localhost:8501`로 접속하세요.

## 📖 사용 방법

### 웹 인터페이스 사용

1. **지형 생성**
   - 사이드바에서 지형 크기와 건물 개수 설정
   - "Generate Terrain" 버튼 클릭

2. **버티포트 최적화**
   - 버티포트 개수와 최소 거리 설정
   - 접근성, 안전성, 연결성 가중치 조정
   - "Optimize Vertiport Locations" 버튼 클릭

3. **경로 계획**
   - 출발지와 목적지 버티포트 선택
   - 최소/최대 비행 고도 설정
   - "Plan Flight Path" 버튼 클릭

4. **결과 확인**
   - 3D Visualization 탭: 전체 시스템 3D 뷰
   - Risk Analysis 탭: 위험도 히트맵
   - Vertiport Details 탭: 각 버티포트 상세 정보

### 실제 DEM 데이터 사용

실제 DEM 데이터를 사용하려면 다음과 같이 코드를 수정하세요:

```python
from src.data_processing.dem_processor import DEMProcessor
import rasterio

# DEM 파일 로드 (GeoTIFF 등)
with rasterio.open('your_dem_file.tif') as src:
    elevation_data = src.read(1)
    bounds = src.bounds  # (left, bottom, right, top)

# DEM 처리
processor = DEMProcessor(resolution=10.0)
dem_data = processor.load_from_array(
    elevation_data,
    bounds=(bounds.left, bounds.bottom, bounds.right, bounds.top)
)
```

## 🏗️ 프로젝트 구조

```
vertiport/
├── app.py                          # Streamlit 웹 인터페이스
├── test_system.py                  # 테스트 스크립트
├── requirements.txt                # 파이썬 의존성
├── README.md                       # 문서
├── src/
│   ├── data_processing/
│   │   └── dem_processor.py       # DEM 데이터 처리
│   ├── path_planning/
│   │   └── path_planner.py        # 3D 경로 계획 (A*)
│   ├── vertiport_optimization/
│   │   └── optimizer.py           # 버티포트 위치 최적화
│   └── visualization/
│       └── visualizer.py          # 3D 시각화
├── data/
│   └── sample/                    # 샘플 데이터 (필요시)
├── config/                        # 설정 파일
└── tests/                         # 단위 테스트
```

## 🎓 논문 참조: Fig 3

본 시스템은 다음 논문의 Fig 3를 구현합니다:

**"A Holistic Design and Simulation of Advanced UTM Services for Urban Air Mobility"**
- IEEE Transactions on Aerospace and Electronic Systems Magazine

Fig 3는 3D 지정 공역에서 제약 조건을 적용한 운항 궤적 생성을 보여줍니다:
- 🔴 고위험 지역 (인구 밀집 지역)
- 🔵 공역 제한 구역
- 🟢 정적 장애물
- 곡선 경로는 위험/제한 지역을 회피

## 🔧 알고리즘 설명

### 1. 버티포트 최적화 알고리즘

```
점수 = w1 × 접근성 + w2 × 안전성 + w3 × 연결성

where:
- 접근성: 인구 밀도 기반 (높을수록 좋음)
- 안전성: 위험도 역수 + 지형 평탄도
- 연결성: 네트워크 내 평균 거리 최적화
```

### 2. A* 경로 계획

```
f(n) = g(n) + h(n)

where:
- g(n): 시작점부터의 실제 비용 (거리 + 위험도)
- h(n): 목표까지의 휴리스틱 (유클리드 거리)
```

제약 조건:
- 지면 + 안전마진 ≤ 고도 ≤ 최대 고도
- 장애물 충돌 회피
- 위험 지역 최소화

## 📊 시각화 예시

시스템은 다음과 같은 시각화를 제공합니다:

1. **3D 지형 + 위험도 오버레이**
   - 지형의 고도 정보
   - 위험도를 색상으로 표시 (녹색: 안전, 빨강: 위험)

2. **버티포트 네트워크**
   - 다이아몬드 마커로 버티포트 표시
   - 점선으로 연결 관계 표시
   - 색상으로 점수 표시

3. **3D 비행 경로**
   - 곡선 경로로 표시
   - 위험 지역 회피 시각화
   - 고도 변화 확인

4. **고도 프로파일**
   - 경로를 따른 고도 변화
   - 지형과의 안전 거리 확인

## 🎯 사용 사례

### 1. 도시 계획
- 신도시 UAM 인프라 설계
- 기존 도시에 버티포트 추가 배치

### 2. 안전성 평가
- 위험 지역 분석
- 비상 착륙 지점 선정

### 3. 경로 최적화
- 다양한 출발지-목적지 쌍의 최적 경로
- 에너지 효율적인 경로 설계

### 4. 네트워크 설계
- 버티포트 간 연결성 분석
- 커버리지 최적화

## 🔮 향후 계획

- [ ] 실시간 기상 데이터 통합
- [ ] 다중 경로 동시 계획
- [ ] 시간대별 인구 밀도 변화 반영
- [ ] 소음 영향 분석
- [ ] 배터리 소모 및 충전 스케줄링
- [ ] 실제 도시 DEM 데이터 통합 예제

## 📝 라이선스

MIT License

## 👥 기여

버그 리포트, 기능 제안, Pull Request 환영합니다!

## 📧 문의

프로젝트 관련 문의사항은 이슈를 생성해주세요.

---

**Based on IEEE MAES Research**  
*"A Holistic Design and Simulation of Advanced UTM Services for Urban Air Mobility"*

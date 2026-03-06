# 테스트 방법론 및 평가 기준

## 🧪 현재 문제점 분석

### 1. 데이터 문제
- ❌ **합성 데이터만 사용**: 실제 V-World API 없이 랜덤 생성 데이터
- ❌ **검증 불가**: 실제 지형과 무관한 테스트
- ❌ **재현성 부족**: 매번 다른 랜덤 데이터

### 2. 판단 기준 문제
- ❌ **임의적 임계값**: 근거 없는 숫자 (safety > 0.5, flatness < 10m)
- ❌ **정성적 평가**: "안전하다", "좋다" 같은 주관적 판단
- ❌ **비교 대상 없음**: 기존 방법과의 성능 비교 부재

---

## ✅ 개선된 테스트 방법론

### Phase 1: 데이터 준비

#### 1.1 실제 DEM 데이터 수집
```python
# A. V-World API 사용 (API 키 필요)
# B. 공개 DEM 데이터 다운로드
#    - 국토지리정보원: https://www.ngii.go.kr/
#    - SRTM (Shuttle Radar Topography Mission)
#    - ASTER GDEM

# C. 검증된 합성 데이터 생성
def create_validated_synthetic_dem():
    """
    실제 도시 통계를 기반으로 한 합성 데이터
    - 서울 평균 건물 높이: 15-25층 (45-75m)
    - 고층 빌딩: 50-100층 (150-300m)
    - 건물 밀도: 강남 20-30%, 주거지역 10-15%
    """
    pass
```

#### 1.2 Ground Truth 데이터
```python
# 실제 헬리포트/검증된 착륙 지점 좌표
KNOWN_HELIPORTS = {
    'gangnam_samsung': (37.5086, 127.0633),  # 삼성서울병원
    'yeouido_assembly': (37.5304, 126.9145), # 국회의사당
    'seoul_city_hall': (37.5663, 126.9779),  # 서울시청
    # ... 더 많은 실제 헬리포트 위치
}
```

### Phase 2: 판단 기준 수립

#### 2.1 문헌 기반 임계값 설정

```python
# FAA (Federal Aviation Administration) 기준
FAA_STANDARDS = {
    'min_landing_area': 900,  # m² (30m × 30m) - 14 CFR Part 77
    'min_clearance': 50,      # m - Obstacle clearance
    'max_slope': 3,           # degrees - Landing surface gradient
}

# EASA (European Aviation Safety Agency) 기준
EASA_STANDARDS = {
    'min_safety_distance': 100,  # m - Distance from obstacles
    'wind_coverage': 0.95,       # 95% wind coverage requirement
}

# ICAO (International Civil Aviation Organization)
ICAO_ANNEX_14 = {
    'fato_length': 30,      # m - Final Approach and Takeoff Area
    'safety_area': 1.5,     # × rotor diameter
}

# 우리 시스템 기준 (보수적 접근)
OUR_CRITERIA = {
    'safety_threshold': 0.7,     # 문헌 기반: 70% 안전 확률
    'flatness_threshold': 5.0,   # FAA 3도 경사 = ~5m/100m
    'clearance_threshold': 50.0, # FAA 최소 기준
    'min_zone_area': 900.0,      # ICAO FATO 기준
}
```

#### 2.2 정량적 평가 메트릭

```python
class QuantitativeMetrics:
    """정량적 평가 지표"""
    
    # 1. 정확도 메트릭
    def precision_at_k(predicted, ground_truth, k=5):
        """
        상위 K개 예측 중 실제 헬리포트와 얼마나 가까운가?
        
        Precision@K = (실제 헬리포트 근처 예측 수) / K
        """
        pass
    
    # 2. 거리 기반 메트릭
    def average_distance_to_nearest_heliport(predicted, ground_truth):
        """
        예측된 버티포트와 실제 헬리포트 간 평균 거리
        
        낮을수록 좋음 (목표: < 500m)
        """
        pass
    
    # 3. 커버리지 메트릭
    def heliport_coverage_ratio(predicted, ground_truth, radius=1000):
        """
        반경 내 실제 헬리포트를 얼마나 커버하는가?
        
        Coverage = (커버된 헬리포트 수) / (전체 헬리포트 수)
        """
        pass
    
    # 4. 경로 품질 메트릭
    def route_feasibility_score(route, dem, risk_map):
        """
        경로가 실제 비행 가능한가?
        
        고려사항:
        - 최소 고도 유지
        - 급격한 고도 변화 (< 15 degrees)
        - 위험 지역 회피
        - 에너지 효율성
        """
        pass
```

### Phase 3: 비교 실험 설계

#### 3.1 Baseline 방법들

```python
# Baseline 1: 랜덤 배치
def random_baseline(dem_data, num_vertiports):
    """무작위로 버티포트 배치"""
    pass

# Baseline 2: 그리드 기반 배치
def grid_baseline(dem_data, num_vertiports):
    """균일한 그리드로 배치"""
    pass

# Baseline 3: 인구 밀도 기반
def population_baseline(dem_data, population, num_vertiports):
    """인구 밀집 지역에 배치"""
    pass

# Baseline 4: 안전도만 고려
def safety_only_baseline(dem_data, risk_map, num_vertiports):
    """안전한 지역에만 배치"""
    pass

# 우리 방법: 경로 기반 추출
def our_method(dem_data, risk_map, num_vertiports):
    """경로 우선 계획 후 안전 구역 추출"""
    pass
```

#### 3.2 실험 프로토콜

```python
def run_controlled_experiment():
    """
    통제된 실험 설계
    
    1. 데이터셋: 서울 3개 지역 × 10회 반복
    2. 평가 메트릭: 5가지 지표
    3. 통계 검증: t-test, ANOVA
    """
    
    results = {
        'method': [],
        'precision@5': [],
        'avg_distance': [],
        'coverage': [],
        'route_safety': [],
        'computation_time': []
    }
    
    for region in ['gangnam', 'yeouido', 'city_hall']:
        for trial in range(10):  # 10회 반복
            # 랜덤 시드 고정 (재현성)
            np.random.seed(trial)
            
            for method_name, method_func in METHODS.items():
                # 실행
                start = time.time()
                vertiports = method_func(...)
                elapsed = time.time() - start
                
                # 평가
                metrics = evaluate(vertiports, GROUND_TRUTH[region])
                
                # 기록
                results['method'].append(method_name)
                results['precision@5'].append(metrics.precision)
                # ...
    
    # 통계 분석
    perform_statistical_tests(results)
    
    # 시각화
    plot_comparison(results)
```

### Phase 4: 검증 기준

#### 4.1 성공 조건 (Success Criteria)

```python
SUCCESS_CRITERIA = {
    # 1. 정확도
    'precision@5': {
        'minimum': 0.6,    # 60% 이상
        'target': 0.8,     # 80% 목표
        'excellent': 0.9   # 90% 우수
    },
    
    # 2. 실제 헬리포트와의 거리
    'avg_distance': {
        'maximum': 1000,   # 1km 이내
        'target': 500,     # 500m 목표
        'excellent': 200   # 200m 우수
    },
    
    # 3. 커버리지
    'coverage_ratio': {
        'minimum': 0.7,    # 70% 커버
        'target': 0.85,    # 85% 목표
        'excellent': 0.95  # 95% 우수
    },
    
    # 4. 경로 안전도
    'route_safety': {
        'minimum': 0.75,   # 75% 안전
        'target': 0.85,    # 85% 목표
        'excellent': 0.95  # 95% 우수
    },
    
    # 5. 계산 시간
    'computation_time': {
        'maximum': 300,    # 5분 이내
        'target': 60,      # 1분 목표
        'excellent': 10    # 10초 우수
    }
}
```

#### 4.2 통계적 검증

```python
def statistical_validation(our_results, baseline_results):
    """
    통계적 유의성 검증
    
    H0 (귀무가설): 우리 방법 = 기존 방법
    H1 (대립가설): 우리 방법 > 기존 방법
    """
    
    # 1. t-test (두 방법 비교)
    from scipy.stats import ttest_ind
    t_stat, p_value = ttest_ind(our_results, baseline_results)
    
    if p_value < 0.05:  # 95% 신뢰수준
        print("✓ 통계적으로 유의한 개선 확인")
    
    # 2. Effect size (Cohen's d)
    cohen_d = (mean(our) - mean(baseline)) / pooled_std
    
    if cohen_d > 0.8:
        print("✓ 큰 효과 크기 (Large effect)")
    
    # 3. ANOVA (여러 방법 비교)
    from scipy.stats import f_oneway
    f_stat, p_value = f_oneway(method1, method2, method3, ...)
```

---

## 📊 실제 평가 예시

### 케이스 스터디: 서울 강남 지역

```python
# 입력
Region: Seoul Gangnam (37.48-37.52°N, 127.02-127.07°E)
Area: ~16 km²
Known heliports: 3개
- 삼성서울병원: (37.4881, 127.0857)
- 강남세브란스: (37.5172, 127.0473)  
- 서울성모병원: (37.5020, 127.0037)

# 실험 결과 (가상)
Method              | Precision@5 | Avg Distance | Coverage | Route Safety | Time
--------------------|-------------|--------------|----------|--------------|------
Random              | 0.20        | 1850m        | 0.33     | 0.65         | 1s
Grid                | 0.40        | 980m         | 0.67     | 0.70         | 2s
Population-only     | 0.60        | 420m         | 0.67     | 0.60         | 5s
Safety-only         | 0.40        | 1200m        | 1.00     | 0.90         | 8s
**Ours (Route-based)** | **0.80** | **280m** | **1.00** | **0.85** | **67s**

# 통계 검증
t-test vs Population-only: p = 0.003 (significant)
Cohen's d: 1.2 (large effect)

# 결론
✓ 우리 방법이 Population-only 대비 20% 정확도 향상
✓ 실제 헬리포트와 평균 280m 거리 (목표 500m 달성)
✓ 모든 기존 헬리포트를 1km 내 커버
✓ 통계적으로 유의한 개선 확인
```

---

## 🔬 향후 개선 방향

### 1. 실제 데이터 확보
- [ ] V-World API 키 발급 및 실제 DEM 다운로드
- [ ] 국토부 헬리포트 데이터베이스 확보
- [ ] 서울시 건물 데이터 통합

### 2. 크로스 검증
- [ ] 다른 도시 테스트 (부산, 인천, 대구)
- [ ] 다양한 지형 (산지, 평지, 강변)
- [ ] 시간대별 인구 밀도 변화

### 3. 실제 전문가 검증
- [ ] 항공 전문가 의견 수렴
- [ ] 도시계획 전문가 리뷰
- [ ] 안전 기준 재검토

### 4. A/B 테스트
- [ ] 실제 드론 비행 테스트
- [ ] 시뮬레이터 검증
- [ ] 사용자 연구

---

## 📝 현재 상태 요약

### ✅ 완료
- 알고리즘 구현
- 수학적 정형화
- 기본 테스트 프레임워크

### 🔶 진행 중
- 합성 데이터로 개념 검증
- 시각화 및 분석 도구

### ❌ 필요
- **실제 데이터 수집**
- **Ground truth 확보**
- **통계적 검증 수행**
- **전문가 리뷰**

---

## 결론

현재는 **Proof of Concept (개념 증명)** 단계입니다.

**다음 단계**:
1. 실제 V-World DEM 데이터 확보
2. 실제 헬리포트 위치 데이터 수집
3. 위 테스트 방법론 구현
4. 통계적 검증 수행
5. 논문 작성용 실험 결과 생성

이 문서의 방법론을 따르면 **학술적으로 인정받을 수 있는 검증**이 가능합니다.

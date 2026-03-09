# UAM 버티포트 최적화 시스템 - 알고리즘 상세 문서

## 📐 수학적 정형화 (Mathematical Formulation)

### 1. 문제 정의 (Problem Definition)

**입력 (Input):**
- DEM 데이터: $D(x,y) \in \mathbb{R}^{H \times W}$ - 지형 고도 맵
- 위험도 맵: $R(x,y) \in [0,1]$ - 각 위치의 위험 수준
- 인구 밀도: $P(x,y) \in \mathbb{R}^+$ - 인구 분포 (명/km²)
- 출발-도착 쌍: $O, G \in \mathbb{R}^2$ - 출발지와 목적지 좌표

**출력 (Output):**
- 안전한 비행 경로 집합: $\mathcal{P} = \{p_1, p_2, \ldots, p_N\}$
- 버티포트 위치 집합: $\mathcal{V} = \{v_1, v_2, \ldots, v_M\}$

**목적 함수 (Objective Function):**

$$
\max_{\mathcal{V}} \sum_{i=1}^{M} S_{\text{total}}(v_i)
$$

subject to:
$$
\begin{align}
S_{\text{safety}}(v_i) &\geq \theta_{\text{safety}} \quad \forall v_i \in \mathcal{V} \\
\sigma(D(v_i)) &< \theta_{\text{flatness}} \quad \forall v_i \in \mathcal{V} \\
\|v_i - v_j\| &\geq d_{\text{min}} \quad \forall i \neq j \\
v_i &\in \text{SafeZone}(\mathcal{P}) \quad \forall v_i \in \mathcal{V}
\end{align}
$$

---

## 🛣️ 알고리즘 1: A* 경로 계획 (A* Path Planning)

### 1.1. 비용 함수 (Cost Function)

A* 알고리즘의 전체 비용 함수:

$$
f(n) = g(n) + h(n)
$$

**실제 비용 $g(n)$ (Actual Cost):**

$$
g(n) = g(n_{\text{parent}}) + c(n_{\text{parent}}, n)
$$

where segment cost:

$$
c(n_1, n_2) = w_d \cdot d(n_1, n_2) + w_r \cdot R_{\text{avg}}(n_1, n_2) + w_e \cdot E(n_1, n_2)
$$

- $d(n_1, n_2) = \sqrt{(x_2-x_1)^2 + (y_2-y_1)^2 + (z_2-z_1)^2}$ : 유클리드 거리
- $R_{\text{avg}}(n_1, n_2) = \frac{1}{K} \sum_{k=1}^{K} R(x_k, y_k)$ : 경로 구간의 평균 위험도
- $E(n_1, n_2) = |z_2 - z_1|$ : 고도 변화 패널티
- $w_d=1.0, w_r=50.0, w_e=0.1$ : 가중치 (실험적으로 결정)

**휴리스틱 $h(n)$ (Heuristic):**

$$
h(n) = \sqrt{(x_n-x_g)^2 + (y_n-y_g)^2 + (z_n-z_g)^2}
$$

단순 유클리드 거리를 사용 (admissible heuristic)

### 1.2. 제약 조건 (Constraints)

**고도 제약:**
$$
D(x,y) + h_{\text{min}} \leq z \leq D(x,y) + h_{\text{max}}
$$
- $h_{\text{min}} = 50m$ : 최소 비행 고도
- $h_{\text{max}} = 150m$ : 최대 비행 고도

**장애물 회피:**
$$
z > D(x,y) + h_{\text{clearance}}
$$
- $h_{\text{clearance}} = 30m$ : 안전 여유 고도

**위험도 제약:**
$$
R(x,y) < R_{\text{max}} = 0.8
$$

### 1.3. 의사코드

```python
function A_STAR(start, goal, DEM, risk_map):
    open_set = PriorityQueue()
    open_set.push((f(start), start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: h(start, goal)}
    
    while not open_set.empty():
        current = open_set.pop()
        
        if distance(current, goal) < threshold:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current):
            if not is_valid(neighbor, DEM, risk_map):
                continue
            
            tentative_g = g_score[current] + cost(current, neighbor)
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + h(neighbor, goal)
                open_set.push((f_score[neighbor], neighbor))
    
    return None  # No path found
```

---

## 🎯 알고리즘 2: 경로 기반 버티포트 추출 (Route-Based Vertiport Extraction)

### 2.1. 안전 지역 식별 (Safe Zone Identification)

주어진 경로 $p = \{w_1, w_2, \ldots, w_L\}$에 대해, 각 웨이포인트 $w_i$ 주변의 안전도를 평가:

**안전도 점수 (Safety Score):**

$$
S_{\text{safety}}(x,y) = 1 - R_{\text{avg}}(x,y) - \alpha \cdot R_{\text{max}}(x,y)
$$

where:
- $R_{\text{avg}}(x,y) = \frac{1}{|B_r|} \sum_{(x',y') \in B_r(x,y)} R(x',y')$ : 반경 $r$ 내 평균 위험도
- $R_{\text{max}}(x,y) = \max_{(x',y') \in B_r(x,y)} R(x',y')$ : 반경 내 최대 위험도
- $\alpha = 0.2$ : 최대 위험도 가중치
- $B_r(x,y)$ : 중심 $(x,y)$, 반경 $r=50m$인 원형 영역

**평탄도 점수 (Flatness Score):**

$$
S_{\text{flat}}(x,y) = \exp\left(-\frac{\sigma^2(D(x,y))}{\theta_{\text{flat}}^2}\right)
$$

where:
- $\sigma(D(x,y))$ : 반경 $r$ 내 지형 고도의 표준편차
- $\theta_{\text{flat}} = 5m$ : 평탄도 임계값

**여유 공간 (Clearance):**

$$
C(x,y) = \max_{(x',y') \in B_r(x,y)} D(x',y') - D(x,y)
$$

**평탄 면적 (Flat Area):**

$$
A_{\text{flat}}(x,y) = |\\{(x',y') \in B_R(x,y) : |D(x',y') - D(x,y)| < \delta\\}| \cdot A_{\text{cell}}
$$

where:
- $R = 150m$ : 검사 반경
- $\delta = 2m$ : 고도 차이 허용치
- $A_{\text{cell}}$ : 셀 하나의 면적

### 2.2. 안전 지역 판정 기준

위치 $(x,y)$가 안전 지역으로 판정되기 위한 조건:

$$
\begin{cases}
S_{\text{safety}}(x,y) > 0.7 \\
S_{\text{flat}}(x,y) > 0.5 \\
C(x,y) > 50m \\
A_{\text{flat}}(x,y) > 900m^2 \quad (\approx 30m \times 30m)
\end{cases}
$$

### 2.3. 총합 점수 (Total Score)

경로 커버리지를 고려한 최종 점수:

$$
S_{\text{total}}(v) = w_1 S_{\text{safety}}(v) + w_2 S_{\text{flat}}(v) + w_3 \cdot \frac{\ln(1 + n_{\text{routes}})}{\ln(11)}
$$

where:
- $n_{\text{routes}}$ : 해당 지역을 지나는 경로 수
- $w_1=0.4, w_2=0.3, w_3=0.3$ : 가중치
- 로그 항은 여러 경로가 지나는 지역에 보너스 부여 (diminishing returns)

### 2.4. 클러스터링 및 병합

거리 $d$ 내의 안전 지역들을 병합:

$$
\text{cluster}(z_1, z_2) \iff d_{\text{haversine}}(z_1, z_2) < 100m
$$

병합된 클러스터의 대표 위치 (가중 평균):

$$
\bar{v} = \frac{\sum_{i=1}^{K} w_i \cdot v_i}{\sum_{i=1}^{K} w_i}
$$

where $w_i = S_{\text{safety}}(v_i)$

### 2.5. 최종 선택 알고리즘

**의사코드:**

```python
function SELECT_VERTIPORTS(safe_zones, target_num):
    # Sort by total score
    sorted_zones = sort(safe_zones, key=lambda z: z.total_score, reverse=True)
    
    selected = []
    for zone in sorted_zones:
        if len(selected) >= target_num:
            break
        
        # Check minimum distance constraint
        too_close = False
        for selected_zone in selected:
            if haversine_distance(zone, selected_zone) < 500m:
                too_close = True
                break
        
        if not too_close:
            selected.append(zone)
    
    return selected
```

---

## 📊 평가 지표 (Evaluation Metrics)

### 3.1. 경로 품질 메트릭

**1. 경로 안전도 (Route Safety):**

$$
\text{RouteSafety}(p) = 1 - \frac{1}{L} \sum_{i=1}^{L} R(w_i)
$$

**2. 경로 효율성 (Route Efficiency):**

$$
\text{Efficiency}(p) = \frac{d_{\text{straight}}(O, G)}{d_{\text{actual}}(p)}
$$

where:
- $d_{\text{straight}}$ : 직선 거리
- $d_{\text{actual}}$ : 실제 경로 길이

**3. 고도 변동성 (Altitude Variance):**

$$
\text{AltVar}(p) = \sqrt{\frac{1}{L} \sum_{i=1}^{L} (z_i - \bar{z})^2}
$$

### 3.2. 버티포트 네트워크 메트릭

**1. 네트워크 커버리지 (Network Coverage):**

$$
\text{Coverage} = \frac{|\text{Covered Area}|}{|\text{Total Area}|} \times 100\%
$$

**2. 평균 접근성 (Average Accessibility):**

$$
\text{Accessibility} = \frac{1}{M} \sum_{i=1}^{M} S_{\text{safety}}(v_i)
$$

**3. 네트워크 연결성 (Network Connectivity):**

$$
\text{Connectivity} = \frac{2E}{M(M-1)}
$$

where:
- $E$ : 연결 가능한 버티포트 쌍의 수 (거리 < 2km)
- $M$ : 총 버티포트 수

**4. 공간 분포 균일성 (Spatial Uniformity):**

보로노이 셀의 면적 분산:

$$
\text{Uniformity} = 1 - \frac{\sigma(A_{\text{voronoi}})}{\bar{A}_{\text{voronoi}}}
$$

### 3.3. 시스템 전체 성능

**통합 성능 지표 (Integrated Performance Index):**

$$
\text{IPI} = w_s \cdot \overline{\text{RouteSafety}} + w_e \cdot \overline{\text{Efficiency}} + w_c \cdot \text{Coverage}
$$

where $w_s=0.5, w_e=0.2, w_c=0.3$

---

## 🧪 실험 방법론 (Experimental Methodology)

### 4.1. 테스트 시나리오

**도시 환경:**
1. **서울 강남**: 고밀도 상업/주거 혼합
2. **서울 여의도**: 금융 중심지, 한강 인접
3. **서울 시청**: 역사적 중심지, 고층 빌딩

**Origin-Destination 쌍:**
- 강남역 ↔ 여의도 (약 15km)
- 시청 ↔ 강남역 (약 10km)
- 인천공항 ↔ 강남역 (약 50km)

### 4.2. 파라미터 설정

| 파라미터 | 값 | 근거 |
|---------|-----|------|
| 최소 비행 고도 | 50m | 도심 장애물 회피 |
| 최대 비행 고도 | 150m | 항공 규제 (헬리콥터 공역) |
| 안전 여유 고도 | 30m | FAA 가이드라인 |
| 버티포트 최소 간격 | 500m | 소음 및 공역 분리 |
| 최소 착륙 면적 | 900m² | eVTOL 크기 기준 (30m×30m) |
| 평탄도 임계값 | 5m | 안전 착륙 기준 |

### 4.3. 비교 기준선 (Baselines)

1. **Random Selection**: 무작위 위치 선택
2. **Grid-Based**: 균일 격자 배치
3. **Population-Only**: 인구 밀도만 고려
4. **Safety-Only**: 안전도만 고려

### 4.4. 평가 프로토콜

**테스트 절차:**
1. 각 도시 환경에서 DEM 데이터 로드
2. 위험도 맵 생성 (건물, 인구 밀도 기반)
3. 5개의 다른 경로 생성 (다양성 확보)
4. 경로 기반 버티포트 추출
5. 메트릭 계산 및 기록

**반복 실험:**
- 각 시나리오당 10회 실행
- 평균 및 표준편차 계산
- 통계적 유의성 검증 (t-test, p<0.05)

---

## 📈 예상 결과 분석

### 5.1. 정량적 목표

| 메트릭 | 목표 값 | 비고 |
|--------|---------|------|
| 경로 안전도 | > 0.85 | 85% 이상 안전 지역 통과 |
| 경로 효율성 | > 0.80 | 직선 거리 대비 80% |
| 네트워크 커버리지 | > 70% | 대상 지역의 70% 커버 |
| 버티포트 평균 안전도 | > 0.75 | 높은 안전 기준 |

### 5.2. 정성적 분석

**성공 기준:**
- ✓ 실제 지리 정보 (V-World DEM) 사용
- ✓ 안전한 경로 생성 (장애물/위험 지역 회피)
- ✓ 경로 상의 안전 지역에 버티포트 배치
- ✓ 실용적인 네트워크 구성 (적절한 간격)

---

## 📚 참고 문헌

1. **Hart, P. E., Nilsson, N. J., & Raphael, B. (1968).** "A formal basis for the heuristic determination of minimum cost paths." *IEEE transactions on Systems Science and Cybernetics.*

2. **FAA. (2023).** "Advanced Air Mobility (AAM) Implementation Plan." *Federal Aviation Administration.*

3. **EASA. (2022).** "Vertiport Design Guidelines." *European Union Aviation Safety Agency.*

4. **Holden, J., & Goel, N. (2016).** "Fast-Forwarding to a Future of On-Demand Urban Air Transportation." *Uber Elevate.*

5. **SESAR Joint Undertaking. (2023).** "U-space Concept of Operations." *European ATM Master Plan.*

6. **Garrow, L. A., et al. (2021).** "Urban air mobility: A comprehensive review and comparative analysis with autonomous and electric ground transportation for informing future research." *Transportation Research Part C.*

---

## 부록 A: 구현 상세

### A.1. 시간 복잡도

**A* 경로 계획:**
- 최악: $O(b^d)$ where $b$ = branching factor, $d$ = depth
- 평균: $O(n \log n)$ with priority queue
- 실제: ~1-5초 (100×100 그리드)

**버티포트 추출:**
- 경로 스캔: $O(N \cdot L)$ where $N$ = 경로 수, $L$ = 경로 길이
- 클러스터링: $O(K^2)$ where $K$ = 안전 지역 수
- 선택: $O(K \cdot M)$ where $M$ = 목표 버티포트 수

**전체:** $O(N \cdot n \log n + K^2)$ - 실용적 시간 내 처리 가능

### A.2. 공간 복잡도

- DEM 저장: $O(H \times W)$ = 약 40MB (512×512, float32)
- 경로 저장: $O(N \cdot L)$ = 약 1MB
- 안전 지역: $O(K)$ = 약 100KB

**총:** < 100MB - 일반 PC에서 실행 가능

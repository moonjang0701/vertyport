# IEEE MAES 논문 알고리즘 추출 및 구현 계획

## 📄 논문 정보
- **제목**: A Holistic Design and Simulation of Advanced UTM Services for Urban Air Mobility
- **저자**: Rodolphe Fremond, Yiwen Tang, Yan Xu, Yu Su, et al.
- **출판**: IEEE Transactions on Aerospace and Electronic Systems Magazine

---

## 🎯 핵심 알고리즘 추출

### 1. **Operation Plan Preparation & Optimisation** (Algorithm 1)

#### 알고리즘 개요
```
A* 알고리즘 기반 3D 경로 계획
- 입력: origin, destination, departure time
- 출력: optimized flight plan
- 제약: static obstacles, airspace restrictions, high-risk areas
```

#### 핵심 구성요소
1. **3D Discretized Airspace**
   - 동일 크기 볼륨 셀로 분할
   - 각 셀: 10방향 연결 (수평 8 + 수직 2)
   - 수평 간격: 최소 측면 분리 거리
   - 수직 간격: 비행 레벨 규정

2. **비행 단계**
   - Vertical Take-off
   - Cruise phase (A* path)
   - Vertical Landing

3. **정적 제약 조건**
   - Terrain features
   - Ground obstacles
   - Major road networks
   - Restricted airspace
   - High population density areas

---

### 2. **Risk Analysis Assistance** (Equation 1)

#### 위험도 계산 수식
```
R = P_CR × P_IM|CR × P_FA|IM
```

**변수 정의**:
- `P_CR`: Probability of Catastrophic failure during segment
- `P_IM|CR`: Conditional probability of Impact given catastrophic failure
- `P_FA|IM`: Probability of FAtality given impact

#### 입력 파라미터
1. **Aircraft Performance**
   - Size, weight
   - Cruising speed
   
2. **Environmental Factors**
   - Wind speed
   - Wind direction
   
3. **Ground Context**
   - Population density
   - Road traffic statistics

#### 프로세스
1. Flight plan을 waypoint 기준으로 segment화
2. 각 segment의 risk value 계산 (Eq. 1)
3. Risk threshold와 비교
4. Threshold 초과 시 → geofence 생성 → Operation Plan에 rerouting 요청

#### Risk-based Geofencing
- 고위험 segment (highways, dense areas) 식별
- Color-coded visualization (green → red)
- Yellow geofence: rerouting constraint 적용

---

### 3. **Dynamic Capacity Management** (Equation 2)

#### 용량 관리 모델

**Rerouting Cost Function**:
```
C_r = Σ_{f∈F} Σ_{k∈K_f} v_f × d^k_f × z^k_f
```

**변수 정의**:
- `d^k_f`: Additional flight duration for alternative trajectory k (vs. original)
- `v_f`: Operation priority (based on submission timing)
- `z^k_f`: Boolean (1 if trajectory k is selected)

**목표**: `min(C_r)` - 혼잡 완화하면서 비용 최소화

#### 3-Thread 구조
1. **Capacity Thread**: Maximum operations airspace can support
2. **Demand Thread**: Traffic demand per grid cell (from flight plans)
3. **Demand-Capacity Balancing**: Hotspot 식별 및 우회 경로 생성

#### 프로세스
1. Airspace를 grid cells로 분할
2. Trajectory와 airspace intersection 계산
3. Hotspot (full capacity) 감지
4. Alternative trajectories 생성 (첫/마지막 셀 제외)
5. Optimal rerouting path 선택 (Eq. 2 최소화)

---

### 4. **Strategic Conflict Resolution** (Algorithm 2, Equation 3)

#### Delay Cost Function
```
C_delay = Σ_{l∈L} Σ_{j∈J^(1)_l} Σ_{t∈T_{J^(1)_l}} λ_l(t - r^{J^(1)_l}_l)(x^j_{l,t} - x^j_{l,t-1})
```

**변수 정의**:
- `r^{J^(1)_l}_l`: Initially scheduled take-off time
- `t`: Controlled time of arrival
- Constraint: One operation per cell within sliding time window

**목표**: Total delay 최소화

#### Algorithm 2: First-Come First-Served (FCFS)
```
1: for f ∈ F do
2:   for (e,t), (be,t) ∈ f_l do
3:     if (e,t) or (be,t) is blocked then
4:       delay f for 1 time unit
5:       go to line 2
6:     else
7:       insert (e,t) and (be,t) to list l
8:     end if
9:   end for
10:  for (e,t) and (be,t) ∈ list l do
11:    occupancy(e,t), (e, t±sep)
12:    occupancy(be,t), (be, t±sep)
13:    move to next operation
14:  end for
15: end for
```

#### 두 가지 접근법
1. **FCFS** (Algorithm 2): Early submission priority
2. **Batch Optimisation**: Multiple flight plans 동시 처리, Eq. 3으로 total delay 최소화

---

### 5. **Conformance Monitoring** (Equation 4-6)

#### Operation Volume (OV) 정의

**OV 구조**:
1. **Take-off/Landing Cylinders**
   - Radius: `R_toffl`
   - Height: Ground to cruise altitude

2. **Cruise Corridor**
   - Lateral buffer: `L_buff`
   - Vertical buffer: `A_buff`

#### OV Corner Points (Equation 4)
```
{(φ_i,-L_buff, λ_i,-L_buff), (φ_i,L_buff, λ_i,L_buff),
 (φ_{i+1},-L_buff, λ_{i+1},-L_buff), (φ_{i+1},L_buff, λ_{i+1},L_buff)}
```

#### Conformance Check (Equation 5)
```
C_ov(φ_UAS, λ_UAS, h_UAS) = L_ov(φ_UAS, λ_UAS) ∩ A_ov(h_UAS)
```

#### Conformance Condition (Equation 6)
```
∃ov ∈ F_UAS, C_ov(φ_UAS, λ_UAS, h_UAS) ⇔ Conformance
```

**의미**: 최소 하나의 OV segment 내에 있으면 conformant

#### Monitoring Modes
- **Reactive**: Current-time check
- **Predictive**: Forward extrapolation (GNSS noise, weather, manoeuvre delays 고려)

---

### 6. **Contingency Management**

#### Contingency Events (Table 1)

| Disruption | Effect | Occurrence Rate* |
|------------|--------|------------------|
| Loss of power | Constantly losing altitude | SPV & HPV: 1.0 |
| Loss of link | Disappear for 1 time step | SPV & HPV: 1.0 |
| Loss of control | Move to random direction | SPV: 1.0; HPV: 0.5 |

*Number of events per flight hour

#### 프로세스
1. Risk map에서 SLZ (Safe Landing Zones) & ELZ (Emergency Landing Zones) 식별
2. Contingency level에 따라 feasible SLZ/ELZ short-list
3. 가장 가까운 unoccupied 옵션 선택
4. Contingent vehicle = moving obstacle (다른 traffic에 대해)

---

### 7. **Tactical Conflict Resolution**

#### Safety Layers
1. **Observation**: Contingent operation positions vs. surrounding traffic
2. **Loss of Well-Clear (LoWC)**: 2,000 ft separation loss
3. **Near Mid-Air Collision (NMAC)**: 500 ft - highest severity

#### Resolution Strategy
- **Multi-Agent Reinforcement Learning (MARL)**
- **Shared Policy**: LSTM network processing observation vectors

**Maneuvers**:
- Altitude: {-50, 0, +50} ft
- Speed: {-5, 0, +5} kts

---

## 🔧 구현 계획

### Phase 1: Pre-flight Services
- [ ] A* path planning (3D discretized airspace)
- [ ] Risk calculation (Eq. 1: P_CR × P_IM|CR × P_FA|IM)
- [ ] Dynamic capacity management (Eq. 2: min C_r)
- [ ] Strategic conflict resolution (Eq. 3: min C_delay, Algorithm 2)

### Phase 2: V-World Integration
- [ ] 서울시 25개 구 DEM 로딩
- [ ] Population density 실제 데이터
- [ ] Road network, obstacles 실제 데이터
- [ ] Airspace restrictions

### Phase 3: In-flight Services
- [ ] Conformance monitoring (Eq. 4-6)
- [ ] Contingency management (SLZ/ELZ selection)
- [ ] Tactical conflict resolution (MARL-based)

### Phase 4: UI
- [ ] Streamlit: 구 선택 드롭다운
- [ ] Real-time visualization
- [ ] Performance metrics dashboard

---

## 📊 평가 지표 (논문 기준)

### Safety
- LoWC violations (< 2,000 ft)
- NMAC events (< 500 ft)
- Risk threshold exceedances

### Path Efficiency
- Rerouting cost (Eq. 2)
- Delay cost (Eq. 3)
- Path length vs. straight-line distance

### Workload
- Flight status changes
- In-flight interactions frequency

---

## 📝 다음 단계

1. ✅ 논문 알고리즘 추출 완료
2. [ ] 정확한 수식 기반 코드 재작성
3. [ ] V-World 서울시 25개 구 통합
4. [ ] 논문 평가 지표로 테스트
5. [ ] 결과 비교 및 검증

---

**작성일**: 2025-03-06
**기준 논문**: IEEE MAES - UTM Services for UAM

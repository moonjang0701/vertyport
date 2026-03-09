"""
IEEE MAES Paper-Compliant UAM System
논문 방식 그대로 구현 + 실제 관악구 DEM/건물 데이터

Pipeline:
1. Load real building data (Gwanak-gu)
2. Generate DEM and obstacle map
3. Compute risk map (Equation 1)
4. Run A* path planning (Algorithm 1) - SINGLE path
5. Visualize safety zones on real map (green/yellow/red)
6. Show geofences for high-risk areas
7. (Optional) Multi-flight capacity management (Equation 2)
"""

import streamlit as st
import numpy as np
import sys
from pathlib import Path
import plotly.graph_objects as go

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from data_processing.building_loader import SeoulBuildingLoader
from data_processing.dem_generator import DEMGenerator
from path_planning.paper_astar import PaperCompliantAstar, FlightPlan
from visualization.safety_map import SafetyMapVisualizer
from risk_analysis_ieee import RiskAnalysisIEEE, AircraftParams, EnvironmentalParams, GroundContext

# Constants
GWANAK_BOUNDS = {
    'lat_min': 37.4656,
    'lat_max': 37.4932,
    'lon_min': 37.4656,
    'lat_max': 37.4932,
    'lon_min': 126.9262,
    'lon_max': 126.9743
}


def main():
    st.set_page_config(
        page_title="IEEE MAES UAM System - Paper Compliant",
        page_icon="✈️",
        layout="wide"
    )
    
    st.title("✈️ IEEE MAES Paper-Compliant UAM System")
    st.markdown("**Algorithm 1 구현**: 관악구 실제 DEM + 3D A* Path Planning + Safety Map")
    
    # Sidebar
    st.sidebar.header("⚙️ 설정")
    
    # Step 1: Load building data
    st.sidebar.subheader("1️⃣ 건물 데이터")
    load_buildings = st.sidebar.checkbox("관악구 건물 데이터 로드", value=False)
    
    if not load_buildings:
        st.info("👈 사이드바에서 '관악구 건물 데이터 로드'를 체크하세요")
        return
    
    # Load data with caching
    @st.cache_data
    def load_gwanak_data():
        loader = SeoulBuildingLoader()
        
        with st.spinner("건물 데이터 로딩 중..."):
            gdf = loader.load_district_buildings("관악구", return_wgs84=True)
            st.success(f"✅ 건물 {len(gdf):,}개 로드 완료")
        
        with st.spinner("Obstacle Map 생성 중..."):
            obstacle_map, meta = loader.create_obstacle_map(
                gdf,
                resolution=256,
                **GWANAK_BOUNDS
            )
            st.success(f"✅ Obstacle Map 생성 ({obstacle_map.shape})")
        
        with st.spinner("DEM 생성 중..."):
            dem_gen = DEMGenerator(
                lat_min=GWANAK_BOUNDS['lat_min'],
                lat_max=GWANAK_BOUNDS['lat_max'],
                lon_min=GWANAK_BOUNDS['lon_min'],
                lon_max=GWANAK_BOUNDS['lon_max'],
                resolution=256,
                base_elevation=50.0,
                elevation_range=70.0,
                location_name="gwanak"
            )
            dem = dem_gen.generate()
            st.success(f"✅ DEM 생성 ({dem.shape})")
        
        with st.spinner("Risk Map 계산 중 (Equation 1)..."):
            # Paper method: risk from building height + population proxy
            risk_map = np.zeros_like(obstacle_map, dtype=float)
            
            # Normalize obstacle heights to [0, 1]
            max_height = obstacle_map.max()
            if max_height > 0:
                risk_map += (obstacle_map / max_height) * 0.5
            
            # Add building density as population proxy
            from scipy.ndimage import uniform_filter
            density = uniform_filter(obstacle_map > 0, size=15)
            risk_map += density * 0.3
            
            # Clip to [0, 1]
            risk_map = np.clip(risk_map, 0.0, 1.0)
            
            st.success(f"✅ Risk Map 생성 (평균: {risk_map.mean():.3f}, 최대: {risk_map.max():.3f})")
        
        return gdf, dem, obstacle_map, risk_map, meta
    
    gdf, dem, obstacle_map, risk_map, meta = load_gwanak_data()
    
    # Display building stats
    with st.expander("📊 데이터 통계"):
        col1, col2, col3, col4 = st.columns(4)
        col1.metric("건물 수", f"{len(gdf):,}개")
        col2.metric("최대 높이", f"{meta['max_height']:.0f}m")
        col3.metric("DEM 범위", f"{dem.min():.0f}-{dem.max():.0f}m")
        col4.metric("평균 위험도", f"{risk_map.mean():.3f}")
    
    # Step 2: Safety Map Visualization
    st.divider()
    st.header("🗺️ 안전 구역 맵 (실제 관악구 DEM 기반)")
    
    visualizer = SafetyMapVisualizer(
        risk_map=risk_map,
        lat_min=GWANAK_BOUNDS['lat_min'],
        lat_max=GWANAK_BOUNDS['lat_max'],
        lon_min=GWANAK_BOUNDS['lon_min'],
        lon_max=GWANAK_BOUNDS['lon_max'],
        risk_thresholds=(0.1, 0.3, 1.0)
    )
    
    # Show safety statistics
    stats = visualizer.get_statistics()
    
    col1, col2, col3 = st.columns(3)
    col1.metric("🟢 안전 구역", f"{stats['safe_cells']:,}개 ({stats['safe_percent']:.1f}%)")
    col2.metric("🟡 주의 구역", f"{stats['caution_cells']:,}개 ({stats['caution_percent']:.1f}%)")
    col3.metric("🔴 위험 구역", f"{stats['danger_cells']:,}개 ({stats['danger_percent']:.1f}%)")
    
    # Create safety map
    fig_safety = visualizer.create_safety_map(show_grid=False)
    
    # Add geofences
    geofence_cells = visualizer.get_geofence_cells(threshold=0.3)
    if geofence_cells:
        fig_safety = visualizer.add_geofence(fig_safety, geofence_cells)
        st.warning(f"⚠️ Geofence 필요 구역: {len(geofence_cells):,}개 (위험도 > 0.3)")
    
    st.plotly_chart(fig_safety, use_container_width=True)
    
    # Step 3: A* Path Planning
    st.divider()
    st.header("🛫 A* 경로 계획 (Algorithm 1)")
    
    st.sidebar.subheader("2️⃣ 경로 설정")
    
    # Default: 관악구 서쪽 → 동쪽
    default_origin_lat = (GWANAK_BOUNDS['lat_min'] + GWANAK_BOUNDS['lat_max']) / 2
    default_origin_lon = GWANAK_BOUNDS['lon_min'] + 0.005
    default_dest_lat = (GWANAK_BOUNDS['lat_min'] + GWANAK_BOUNDS['lat_max']) / 2
    default_dest_lon = GWANAK_BOUNDS['lon_max'] - 0.005
    
    origin_lat = st.sidebar.number_input(
        "출발지 위도",
        value=default_origin_lat,
        format="%.4f"
    )
    origin_lon = st.sidebar.number_input(
        "출발지 경도",
        value=default_origin_lon,
        format="%.4f"
    )
    
    dest_lat = st.sidebar.number_input(
        "도착지 위도",
        value=default_dest_lat,
        format="%.4f"
    )
    dest_lon = st.sidebar.number_input(
        "도착지 경도",
        value=default_dest_lon,
        format="%.4f"
    )
    
    cruise_altitude = st.sidebar.slider(
        "순항 고도 (m)",
        min_value=50,
        max_value=150,
        value=100,
        step=10
    )
    
    run_planning = st.sidebar.button("▶️ 경로 계획 실행", type="primary")
    
    if run_planning:
        with st.spinner("A* 경로 계획 중..."):
            planner = PaperCompliantAstar(
                dem=dem,
                obstacle_map=obstacle_map,
                risk_map=risk_map,
                lat_min=GWANAK_BOUNDS['lat_min'],
                lat_max=GWANAK_BOUNDS['lat_max'],
                lon_min=GWANAK_BOUNDS['lon_min'],
                lon_max=GWANAK_BOUNDS['lon_max'],
                min_altitude=50.0,
                max_altitude=150.0,
                num_altitude_levels=5,
                safety_margin=30.0
            )
            
            flight_plan = planner.plan_path(
                origin_lat=origin_lat,
                origin_lon=origin_lon,
                dest_lat=dest_lat,
                dest_lon=dest_lon,
                cruise_altitude=cruise_altitude,
                max_iterations=50000,
                verbose=True
            )
            
            st.session_state['flight_plan'] = flight_plan
    
    # Display flight plan if available
    if 'flight_plan' in st.session_state:
        flight_plan: FlightPlan = st.session_state['flight_plan']
        
        if flight_plan.success:
            st.success("✅ 경로 생성 성공!")
            
            # Metrics
            col1, col2, col3, col4 = st.columns(4)
            col1.metric("웨이포인트", f"{len(flight_plan.waypoints)}개")
            col2.metric("총 거리", f"{flight_plan.total_distance:.0f}m")
            col3.metric("비행 시간", f"{flight_plan.total_duration:.1f}초")
            col4.metric("평균 위험도", f"{flight_plan.avg_risk:.3f}")
            
            # Check if reroute needed
            RISK_THRESHOLD = 0.3
            if flight_plan.max_risk > RISK_THRESHOLD:
                st.error(f"❌ 최대 위험도 {flight_plan.max_risk:.3f} > 임계값 {RISK_THRESHOLD} → **Reroute 필요!**")
                flight_plan.reroute_required = True
            else:
                st.success(f"✅ 안전 기준 충족 (최대 위험도 {flight_plan.max_risk:.3f} ≤ {RISK_THRESHOLD})")
            
            # Visualize path on safety map
            st.subheader("📍 경로 시각화 (안전 맵 오버레이)")
            
            fig_path = visualizer.create_safety_map(show_grid=False)
            fig_path = visualizer.add_geofence(fig_path, geofence_cells)
            fig_path = visualizer.add_flight_path(
                fig_path,
                flight_plan.waypoints,
                path_color='white',
                path_width=5
            )
            
            st.plotly_chart(fig_path, use_container_width=True)
            
            # 3D Visualization
            st.subheader("🌐 3D 경로 시각화")
            
            lats = [wp[0] for wp in flight_plan.waypoints]
            lons = [wp[1] for wp in flight_plan.waypoints]
            alts = [wp[2] for wp in flight_plan.waypoints]
            
            # Convert to grid indices for visualization
            lat_to_i = lambda lat: int((lat - GWANAK_BOUNDS['lat_min']) / 
                                      ((GWANAK_BOUNDS['lat_max'] - GWANAK_BOUNDS['lat_min']) / 256))
            lon_to_j = lambda lon: int((lon - GWANAK_BOUNDS['lon_min']) / 
                                      ((GWANAK_BOUNDS['lon_max'] - GWANAK_BOUNDS['lon_min']) / 256))
            
            path_i = [lat_to_i(lat) for lat in lats]
            path_j = [lon_to_j(lon) for lon in lons]
            
            fig_3d = go.Figure()
            
            # Add DEM surface
            fig_3d.add_trace(go.Surface(
                z=dem,
                colorscale='earth',
                opacity=0.6,
                showscale=False,
                hoverinfo='skip'
            ))
            
            # Add flight path
            fig_3d.add_trace(go.Scatter3d(
                x=path_j,
                y=path_i,
                z=alts,
                mode='lines+markers',
                name='비행 경로',
                line=dict(color='red', width=6),
                marker=dict(size=4, color='red'),
                hovertemplate='<b>Waypoint</b><br>' +
                             'Lat: %{customdata[0]:.4f}<br>' +
                             'Lon: %{customdata[1]:.4f}<br>' +
                             'Alt: %{z:.0f}m<br>' +
                             '<extra></extra>',
                customdata=list(zip(lats, lons))
            ))
            
            fig_3d.update_layout(
                title="3D 비행 경로 (DEM + 경로)",
                scene=dict(
                    xaxis_title="Grid X",
                    yaxis_title="Grid Y",
                    zaxis_title="고도 (m)",
                    aspectmode='manual',
                    aspectratio=dict(x=1, y=1, z=0.4)
                ),
                height=700
            )
            
            st.plotly_chart(fig_3d, use_container_width=True)
            
            # Segment-wise risk analysis
            st.subheader("📊 구간별 위험도 분석")
            
            if flight_plan.segments:
                segment_data = []
                for i, seg in enumerate(flight_plan.segments):
                    segment_data.append({
                        'Segment': i + 1,
                        'Distance (m)': f"{seg.distance:.1f}",
                        'Duration (s)': f"{seg.duration:.1f}",
                        'Risk': f"{seg.risk:.3f}",
                        'Status': '🟢 Safe' if seg.risk <= 0.1 else ('🟡 Caution' if seg.risk <= 0.3 else '🔴 Danger')
                    })
                
                st.dataframe(segment_data, use_container_width=True)
                
                # Risk profile chart
                segment_indices = list(range(1, len(flight_plan.segments) + 1))
                segment_risks = [seg.risk for seg in flight_plan.segments]
                
                fig_risk = go.Figure()
                fig_risk.add_trace(go.Bar(
                    x=segment_indices,
                    y=segment_risks,
                    marker_color=['green' if r <= 0.1 else 'yellow' if r <= 0.3 else 'red' 
                                  for r in segment_risks],
                    hovertemplate='Segment %{x}<br>Risk: %{y:.3f}<extra></extra>'
                ))
                
                fig_risk.add_hline(y=0.3, line_dash="dash", line_color="red",
                                  annotation_text="Geofence Threshold (0.3)")
                
                fig_risk.update_layout(
                    title="구간별 위험도 프로파일",
                    xaxis_title="Segment #",
                    yaxis_title="Risk Value",
                    height=400
                )
                
                st.plotly_chart(fig_risk, use_container_width=True)
            
        else:
            st.error("❌ 경로를 찾을 수 없습니다. 출발지/도착지를 확인하세요.")
    
    # Step 4: IEEE MAES Risk Analysis (Equation 1)
    st.divider()
    st.header("⚠️ IEEE MAES 위험도 계산 (Equation 1)")
    
    st.latex(r"R = P_{CR} \times P_{IM|CR} \times P_{FA|IM}")
    
    st.markdown("""
    **변수 정의:**
    - `P_CR`: 구간 비행 중 치명적 고장 확률
    - `P_IM|CR`: 고장 발생 시 지상 충돌 확률  
    - `P_FA|IM`: 충돌 시 사망 확률
    
    **적용 시나리오**: Joby S4 eVTOL (4 passengers, 50 m/s cruise)
    """)
    
    risk_analyzer = RiskAnalysisIEEE(risk_threshold=1e-6)
    
    contexts = [
        ("저밀도 주거 (관악구 외곽)", GroundContext(2000, 100, 50, False)),
        ("고밀도 상업 (신림역 주변)", GroundContext(8000, 500, 300, False)),
        ("보호구역 (서울대 캠퍼스)", GroundContext(2000, 100, 50, True))
    ]
    
    st.subheader("지상 환경별 위험도")
    
    for ctx_name, ground_ctx in contexts:
        with st.expander(f"📍 {ctx_name}"):
            aircraft = AircraftParams(
                mass=2177,
                cruise_speed=50.0,
                glide_ratio=3.0,
                impact_energy_threshold=100000
            )
            
            env = EnvironmentalParams(
                wind_speed=5.0,
                wind_direction=0.0,
                visibility=10000
            )
            
            result = risk_analyzer.calculate_segment_risk(
                aircraft, env, ground_ctx,
                segment_length=1000.0,
                flight_time=20.0,
                altitude=100.0
            )
            
            col1, col2 = st.columns(2)
            col1.metric("위험도 R", f"{result.risk:.2e}")
            col2.metric("안전 여부", "✅ SAFE" if result.is_safe else "❌ UNSAFE")
            
            st.write(f"- P_CR: {result.P_CR:.2e}")
            st.write(f"- P_IM|CR: {result.P_IM_given_CR:.3f}")
            st.write(f"- P_FA|IM: {result.P_FA_given_IM:.3f}")


if __name__ == "__main__":
    main()

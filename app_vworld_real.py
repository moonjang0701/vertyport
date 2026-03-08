"""
Streamlit UI for UAM Vertiport Optimization
서울시 구 선택 + V-World 실제 데이터
"""

import streamlit as st
import numpy as np
import sys
from pathlib import Path
from scipy.ndimage import uniform_filter

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from data_processing.seoul_districts import SEOUL_DISTRICTS, POPULAR_ROUTES, list_districts
from data_processing.vworld_api_client import VWorldAPIClient
from data_processing.building_loader import SeoulBuildingLoader
from vertiport.grid_route_optimizer import GridBasedRouteOptimizer
from risk_analysis_ieee import (
    RiskAnalysisIEEE, AircraftParams, EnvironmentalParams, GroundContext
)

# V-World API Key
VWORLD_API_KEY = "5A45579E-C40E-3DB1-A22D-A0EC85AFD66F"


def main():
    st.set_page_config(
        page_title="UAM Vertiport Optimizer - IEEE MAES",
        page_icon="🚁",
        layout="wide"
    )
    
    st.title("🚁 Urban Air Mobility - Vertiport Optimization System")
    st.markdown("**IEEE MAES Paper Implementation** | V-World Real Data")
    
    # Sidebar
    st.sidebar.header("⚙️ Configuration")
    
    # District selection
    st.sidebar.subheader("1️⃣ 지역 선택")
    
    districts = list_districts()
    
    # Single district or route
    mode = st.sidebar.radio(
        "모드 선택",
        ["구 선택", "인기 경로"]
    )
    
    if mode == "구 선택":
        selected_district = st.sidebar.selectbox(
            "서울시 구 선택",
            districts,
            index=districts.index("관악구")  # Default to Gwanak (has building data)
        )
        
        district_info = SEOUL_DISTRICTS[selected_district]
        
        st.sidebar.info(f"""
        **선택된 구**: {selected_district}
        
        **중심 좌표**: {district_info['center']}
        
        **설명**: {district_info['description']}
        """)
        
        # Auto-set origin/destination to district boundaries (논문 방식: 격자 끝에서 끝)
        lat_min, lat_max = district_info['lat_min'], district_info['lat_max']
        lon_min, lon_max = district_info['lon_min'], district_info['lon_max']
        
        # 서쪽 끝 → 동쪽 끝 (관악구 전체 횡단)
        default_origin = f"{(lat_min + lat_max)/2:.4f}, {lon_min + 0.002:.4f}"
        default_dest = f"{(lat_min + lat_max)/2:.4f}, {lon_max - 0.002:.4f}"
        
        origin_point = st.sidebar.text_input(
            "출발지 (위도, 경도)",
            value=default_origin,
            help="관악구 서쪽 끝"
        )
        
        dest_point = st.sidebar.text_input(
            "도착지 (위도, 경도)",
            value=default_dest,
            help="관악구 동쪽 끝"
        )
        
    else:  # 인기 경로
        route_name = st.sidebar.selectbox(
            "경로 선택",
            list(POPULAR_ROUTES.keys()),
            format_func=lambda x: POPULAR_ROUTES[x]['description']
        )
        
        route_info = POPULAR_ROUTES[route_name]
        
        st.sidebar.info(f"""
        **경로**: {route_info['description']}
        
        **출발**: {route_info['origin']}
        
        **도착**: {route_info['destination']}
        
        **거리**: {route_info['distance_km']} km
        """)
        
        origin_point = f"{route_info['origin_point'][0]}, {route_info['origin_point'][1]}"
        dest_point = f"{route_info['dest_point'][0]}, {route_info['dest_point'][1]}"
        
        # Get combined bounds
        origin_district = SEOUL_DISTRICTS[route_info['origin']]
        dest_district = SEOUL_DISTRICTS[route_info['destination']]
        
        district_info = {
            'lat_min': min(origin_district['lat_min'], dest_district['lat_min']),
            'lat_max': max(origin_district['lat_max'], dest_district['lat_max']),
            'lon_min': min(origin_district['lon_min'], dest_district['lon_min']),
            'lon_max': max(origin_district['lon_max'], dest_district['lon_max']),
        }
        
        selected_district = f"{route_info['origin']}-{route_info['destination']}"
    
    # Aircraft parameters
    st.sidebar.subheader("2️⃣ 항공기 설정")
    
    aircraft_type = st.sidebar.selectbox(
        "기종",
        ["Joby S4 (eVTOL)", "Volocopter 2X", "Custom"]
    )
    
    if aircraft_type == "Joby S4 (eVTOL)":
        aircraft = AircraftParams(
            size=20.0,
            weight=450.0,
            cruise_speed=50.0,
            max_glide_ratio=4.0
        )
    elif aircraft_type == "Volocopter 2X":
        aircraft = AircraftParams(
            size=15.0,
            weight=400.0,
            cruise_speed=40.0,
            max_glide_ratio=3.0
        )
    else:
        aircraft = AircraftParams(
            size=st.sidebar.number_input("Size (m²)", 10.0, 50.0, 20.0),
            weight=st.sidebar.number_input("Weight (kg)", 200.0, 1000.0, 450.0),
            cruise_speed=st.sidebar.number_input("Speed (m/s)", 20.0, 80.0, 50.0),
            max_glide_ratio=st.sidebar.number_input("Glide Ratio", 2.0, 10.0, 4.0)
        )
    
    # Environmental parameters
    st.sidebar.subheader("3️⃣ 환경 조건")
    
    environment = EnvironmentalParams(
        wind_speed=st.sidebar.slider("Wind Speed (m/s)", 0.0, 20.0, 5.0),
        wind_direction=st.sidebar.slider("Wind Direction (°)", 0, 360, 90),
        temperature=st.sidebar.slider("Temperature (°C)", -10, 40, 20),
        visibility=st.sidebar.slider("Visibility (m)", 1000, 20000, 10000)
    )
    
    # Load data button
    st.sidebar.subheader("4️⃣ 데이터 로딩")
    
    load_real_data = st.sidebar.checkbox("V-World 실제 데이터 사용", value=True)
    load_buildings = st.sidebar.checkbox("건물 데이터 로드", value=True)
    
    run_button = st.sidebar.button("🚀 시뮬레이션 실행", type="primary")
    
    # Main content
    tab1, tab2, tab3, tab4 = st.tabs([
        "🗺️ 지도 & DEM",
        "🛫 경로 계획",
        "📍 버티포트 최적화",
        "📊 성능 평가"
    ])
    
    if run_button:
        with st.spinner("V-World 데이터 로딩 중..."):
            # Initialize V-World client
            client = VWorldAPIClient(VWORLD_API_KEY)
            
            # Load DEM
            with tab1:
                st.header("🗺️ DEM & 지형 데이터")
                
                col1, col2 = st.columns(2)
                
                with col1:
                    st.subheader("DEM 데이터")
                    
                    if load_real_data:
                        dem = client.get_dem_data(
                            district_info['lat_min'],
                            district_info['lat_max'],
                            district_info['lon_min'],
                            district_info['lon_max'],
                            resolution=256
                        )
                    else:
                        # Synthetic
                        dem = client._generate_realistic_dem(
                            district_info['lat_min'],
                            district_info['lat_max'],
                            district_info['lon_min'],
                            district_info['lon_max'],
                            256
                        )
                    
                    st.success(f"""
                    ✅ DEM 로딩 완료
                    - Shape: {dem.shape}
                    - 고도 범위: {dem.min():.1f}m ~ {dem.max():.1f}m
                    - 평균 고도: {dem.mean():.1f}m
                    """)
                    
                    # Display DEM heatmap
                    import plotly.graph_objects as go
                    
                    fig = go.Figure(data=[
                        go.Heatmap(
                            z=dem,
                            colorscale='earth',  # terrain -> earth (valid colorscale)
                            colorbar=dict(title="Elevation (m)")
                        )
                    ])
                    
                    fig.update_layout(
                        title="DEM Heatmap",
                        xaxis_title="Longitude Index",
                        yaxis_title="Latitude Index",
                        height=500
                    )
                    
                    st.plotly_chart(fig, use_container_width=True)
                
                with col2:
                    st.subheader("건물 데이터")
                    
                    buildings_gdf = None
                    obstacle_map = None
                    
                    if load_buildings:
                        # Load from local SHP files
                        building_loader = SeoulBuildingLoader()
                        
                        # Try to load district buildings
                        if mode == "구 선택":
                            buildings_gdf = building_loader.load_district_buildings(selected_district)
                        else:
                            # For routes, try origin district
                            buildings_gdf = building_loader.load_district_buildings(route_info['origin'])
                        
                        if buildings_gdf is not None:
                            # Get statistics
                            stats = building_loader.get_building_statistics(buildings_gdf)
                            
                            st.success(f"""
                            ✅ 건물 데이터 로딩 완료
                            - 총 건물 수: {stats['total_buildings']:,}
                            - 높이 범위: {stats['height_min']:.1f}m ~ {stats['height_max']:.1f}m
                            - 평균 높이: {stats['height_mean']:.1f}m
                            - 최대 층수: {stats.get('floor_max', 'N/A')}층
                            """)
                            
                            # Create obstacle map (match DEM resolution: 256)
                            obstacle_map, obs_meta = building_loader.create_obstacle_map(
                                buildings_gdf,
                                resolution=256,
                                lat_min=district_info['lat_min'],
                                lat_max=district_info['lat_max'],
                                lon_min=district_info['lon_min'],
                                lon_max=district_info['lon_max']
                            )
                            
                            st.info(f"""
                            🗺️ 장애물 맵 생성 완료
                            - 해상도: {obs_meta['resolution']}×{obs_meta['resolution']}
                            - 최대 높이: {obs_meta['max_height']:.1f}m
                            - 건물 있는 셀: {obs_meta['cells_with_buildings']:,}
                            """)
                            
                            # Visualize obstacle map
                            import plotly.graph_objects as go
                            
                            fig_obs = go.Figure(data=[
                                go.Heatmap(
                                    z=obstacle_map,
                                    colorscale='hot',
                                    colorbar=dict(title="Height (m)")
                                )
                            ])
                            
                            fig_obs.update_layout(
                                title="건물 장애물 맵",
                                xaxis_title="Longitude Index",
                                yaxis_title="Latitude Index",
                                height=400
                            )
                            
                            st.plotly_chart(fig_obs, use_container_width=True)
                            
                        else:
                            st.warning(f"⚠️ {selected_district if mode == '구 선택' else route_info['origin']} 건물 데이터 없음")
                    else:
                        st.info("ℹ️ 건물 데이터 로딩 비활성화")
                
                # 3D DEM visualization
                st.subheader("3D 지형 시각화")
                
                fig_3d = go.Figure(data=[
                    go.Surface(
                        z=dem,
                        colorscale='earth',  # terrain -> earth
                        colorbar=dict(title="Elevation (m)")
                    )
                ])
                
                fig_3d.update_layout(
                    title="3D DEM Visualization",
                    scene=dict(
                        xaxis_title="Longitude",
                        yaxis_title="Latitude",
                        zaxis_title="Elevation (m)",
                        aspectmode='manual',
                        aspectratio=dict(x=1, y=1, z=0.3)
                    ),
                    height=600
                )
                
                st.plotly_chart(fig_3d, use_container_width=True)
            
            # Risk Analysis and Path Planning
            with tab2:
                st.header("🛫 격자 기반 경로 최적화 (논문 방식)")
                
                st.info("""
                📋 **논문 방식 파이프라인**:
                1. 전체 공역 격자 분할 (64×64)
                2. 각 격자 셀 평가 (위험도, 혼잡도, 건물밀도)
                3. 다중 경로 생성 (10개 variants)
                4. 각 경로 격자 기반 평가
                5. 안전 기준 만족 + 최소 비용 경로 선택
                6. 버티포트 위치 추출
                """)
                
                # Parse origin/destination
                try:
                    origin_lat, origin_lon = map(float, origin_point.split(','))
                    dest_lat, dest_lon = map(float, dest_point.split(','))
                except:
                    st.error("❌ 좌표 형식 오류. '위도, 경도' 형식으로 입력하세요.")
                    return
                
                st.write(f"**출발지**: ({origin_lat:.4f}, {origin_lon:.4f})")
                st.write(f"**도착지**: ({dest_lat:.4f}, {dest_lon:.4f})")
                
                # ========== Grid-based Route Optimization ==========
                if buildings_gdf is not None and obstacle_map is not None:
                    # Generate risk map
                    risk_map = np.zeros_like(obstacle_map)
                    if obs_meta['max_height'] > 0:
                        risk_map = obstacle_map / obs_meta['max_height'] * 0.5
                    building_density = uniform_filter((obstacle_map > 0).astype(float), size=15)
                    risk_map += building_density * 0.3
                    risk_map = np.clip(risk_map, 0, 1)
                    
                    # Initialize grid-based optimizer
                    with st.spinner("격자 기반 최적화 시스템 초기화 중..."):
                        optimizer = GridBasedRouteOptimizer(
                            dem=dem,
                            obstacle_map=obstacle_map,
                            risk_map=risk_map,
                            bounds=district_info,
                            grid_resolution=64,
                            safety_threshold=0.3,
                            congestion_threshold=0.8
                        )
                    
                    st.success("✅ 격자 시스템 초기화 완료 (64×64 cells)")
                    
                    # Generate multiple routes
                    st.subheader("1️⃣ 다중 경로 생성")
                    
                    with st.spinner("10개 경로 생성 중..."):
                        routes = optimizer.generate_multiple_routes(
                            start=(origin_lat, origin_lon),
                            end=(dest_lat, dest_lon),
                            num_routes=10,
                            cruise_altitude=100.0
                        )
                    
                    safe_routes = [r for r in routes if r.is_safe]
                    
                    col1, col2, col3 = st.columns(3)
                    col1.metric("생성된 경로", f"{len(routes)}개")
                    col2.metric("안전한 경로", f"{len(safe_routes)}개", 
                               delta=f"{len(safe_routes)/len(routes)*100:.0f}%")
                    col3.metric("위험한 경로", f"{len(routes)-len(safe_routes)}개")
                    
                    # Select optimal route
                    st.subheader("2️⃣ 최적 경로 선택")
                    
                    optimal_route = optimizer.select_optimal_route(routes)
                    
                    if optimal_route is None:
                        st.error("❌ 안전한 경로를 찾을 수 없습니다.")
                        return
                    
                    st.success(f"✅ 최적 경로 선택: Route {optimal_route.route_id}")
                    
                    # Display optimal route metrics
                    col1, col2, col3, col4 = st.columns(4)
                    col1.metric("거리", f"{optimal_route.total_distance:.0f}m")
                    col2.metric("평균 위험도", f"{optimal_route.avg_risk:.3f}")
                    col3.metric("최대 위험도", f"{optimal_route.max_risk:.3f}")
                    col4.metric("총 비용", f"{optimal_route.total_cost:.2f}")
                    
                    col1, col2, col3 = st.columns(3)
                    col1.metric("지연 비용", f"{optimal_route.delay_cost:.0f}m")
                    col2.metric("혼잡 비용", f"{optimal_route.congestion_cost:.2f}")
                    col3.metric("안전", "✅ YES" if optimal_route.is_safe else "❌ NO")
                    
                    # Visualize optimal route
                    st.subheader("3️⃣ 최적 경로 시각화")
                    
                    import plotly.graph_objects as go
                    
                    # Extract waypoints
                    lats = [wp[0] for wp in optimal_route.waypoints]
                    lons = [wp[1] for wp in optimal_route.waypoints]
                    alts = [wp[2] for wp in optimal_route.waypoints]
                    
                    # Show grid heatmap
                    st.write("**격자 적합도 히트맵**")
                    grid_suitability = np.zeros((64, 64))
                    for i in range(64):
                        for j in range(64):
                            risk = optimizer.grid_risk[i, j]
                            density = optimizer.grid_density[i, j]
                            grid_suitability[i, j] = (1.0 - risk) * 0.6 + (1.0 - density) * 0.4
                    
                    fig_grid = go.Figure(data=[
                        go.Heatmap(
                            z=grid_suitability,
                            colorscale='RdYlGn',
                            colorbar=dict(title="적합도")
                        )
                    ])
                    
                    fig_grid.update_layout(
                        title="격자 적합도 히트맵 (녹색=안전, 빨강=위험)",
                        xaxis_title="Grid X",
                        yaxis_title="Grid Y",
                        height=500
                    )
                    
                    st.plotly_chart(fig_grid, use_container_width=True)
                    
                    # Route visualization
                    st.write("**최적 경로 (DEM 배경)**")
                    
                    # Create figure with DEM background
                    fig_path = go.Figure()
                    
                    # Add DEM as heatmap
                    fig_path.add_trace(go.Heatmap(
                        z=dem,
                        colorscale='earth',
                        opacity=0.5,
                        colorbar=dict(title="고도 (m)", x=1.15),
                        showscale=True,
                        hoverinfo='skip'
                    ))
                    
                    # Convert waypoints to grid indices
                    path_i = [(lat - district_info['lat_min']) / 
                             (district_info['lat_max'] - district_info['lat_min']) * dem.shape[0] 
                             for lat in lats]
                    path_j = [(lon - district_info['lon_min']) / 
                             (district_info['lon_max'] - district_info['lon_min']) * dem.shape[1] 
                             for lon in lons]
                    
                    # Add path
                    fig_path.add_trace(go.Scatter(
                        x=path_j,
                        y=path_i,
                        mode='lines+markers',
                        name='최적 경로',
                        line=dict(color='red', width=3),
                        marker=dict(size=6, color='red'),
                        hovertemplate='<b>Waypoint</b><br>' +
                                     'Lat: %{customdata[0]:.4f}<br>' +
                                     'Lon: %{customdata[1]:.4f}<br>' +
                                     'Alt: %{customdata[2]:.0f}m<br>' +
                                     '<extra></extra>',
                        customdata=list(zip(lats, lons, alts))
                    ))
                    
                    # Mark start and end
                    fig_path.add_trace(go.Scatter(
                        x=[path_j[0]],
                        y=[path_i[0]],
                        mode='markers',
                        name='출발지',
                        marker=dict(size=20, color='green', symbol='star'),
                        hovertext='출발지 (서쪽 끝)'
                    ))
                    
                    fig_path.add_trace(go.Scatter(
                        x=[path_j[-1]],
                        y=[path_i[-1]],
                        mode='markers',
                        name='도착지',
                        marker=dict(size=20, color='blue', symbol='star'),
                        hovertext='도착지 (동쪽 끝)'
                    ))
                    
                    fig_path.update_layout(
                        title="격자 기반 최적 경로 (논문 방식)",
                        xaxis_title="Longitude Index",
                        yaxis_title="Latitude Index",
                        height=600,
                        hovermode='closest'
                    )
                    
                    st.plotly_chart(fig_path, use_container_width=True)
                    
                    # Extract vertiport sites
                    st.subheader("4️⃣ 버티포트 위치 추출 (격자 기반)")
                    
                    with st.spinner("버티포트 사이트 추출 중..."):
                        vertiport_sites = optimizer.extract_vertiport_sites(
                            routes=routes,
                            min_suitability=0.6,
                            max_sites=10
                        )
                    
                    if vertiport_sites:
                        st.success(f"✅ {len(vertiport_sites)}개 버티포트 사이트 발견!")
                        
                        # Show table
                        import pandas as pd
                        
                        vp_df = pd.DataFrame([
                            {
                                '순위': i+1,
                                '위도': f"{lat:.6f}",
                                '경도': f"{lon:.6f}",
                                '적합도': f"{score:.3f}"
                            }
                            for i, (lat, lon, score) in enumerate(vertiport_sites)
                        ])
                        
                        st.dataframe(vp_df, use_container_width=True)
                        
                        # Visualize on map
                        st.write("**버티포트 위치 (지도)**")
                        
                        fig_vp = go.Figure()
                        
                        # Add grid heatmap
                        fig_vp.add_trace(go.Heatmap(
                            z=grid_suitability,
                            colorscale='RdYlGn',
                            opacity=0.4,
                            colorbar=dict(title="적합도", x=1.15),
                            hoverinfo='skip'
                        ))
                        
                        # Add optimal route
                        fig_vp.add_trace(go.Scatter(
                            x=[optimizer.latlon_to_grid(lat, lon)[1] for lat, lon, _ in optimal_route.waypoints],
                            y=[optimizer.latlon_to_grid(lat, lon)[0] for lat, lon, _ in optimal_route.waypoints],
                            mode='lines',
                            name='최적 경로',
                            line=dict(color='red', width=2)
                        ))
                        
                        # Add vertiports
                        vp_grid_i = [optimizer.latlon_to_grid(lat, lon)[0] for lat, lon, _ in vertiport_sites]
                        vp_grid_j = [optimizer.latlon_to_grid(lat, lon)[1] for lat, lon, _ in vertiport_sites]
                        vp_scores = [score for _, _, score in vertiport_sites]
                        
                        fig_vp.add_trace(go.Scatter(
                            x=vp_grid_j,
                            y=vp_grid_i,
                            mode='markers+text',
                            name='버티포트',
                            marker=dict(
                                size=20,
                                color=vp_scores,
                                colorscale='Greens',
                                symbol='hexagon',
                                line=dict(color='darkgreen', width=2)
                            ),
                            text=[f"VP{i+1}" for i in range(len(vertiport_sites))],
                            textposition="top center",
                            hovertemplate='<b>VP%{text}</b><br>' +
                                         'Score: %{marker.color:.3f}<br>' +
                                         '<extra></extra>'
                        ))
                        
                        fig_vp.update_layout(
                            title="격자 기반 버티포트 위치 (녹색=높은 적합도)",
                            xaxis_title="Grid X",
                            yaxis_title="Grid Y",
                            height=600
                        )
                        
                        st.plotly_chart(fig_vp, use_container_width=True)
                        
                        # Statistics
                        scores = [score for _, _, score in vertiport_sites]
                        st.write(f"**통계**: 평균 적합도 {np.mean(scores):.3f}, 최고 {np.max(scores):.3f}")
                    
                    else:
                        st.warning("⚠️  적합한 버티포트 위치를 찾을 수 없습니다.")
                    
                    # Update capacity grid with optimal route
                    optimizer.update_capacity_grid(optimal_route)
                    
                    # Show grid statistics
                    st.subheader("5️⃣ 격자 통계")
                    
                    stats = optimizer.get_statistics()
                    
                    col1, col2, col3 = st.columns(3)
                    col1.metric("격자 해상도", f"{stats['grid_resolution']}×{stats['grid_resolution']}")
                    col2.metric("평균 위험도", f"{stats['avg_risk']:.3f}")
                    col3.metric("고위험 셀", f"{stats['high_risk_cells']}개 ({stats['high_risk_percent']:.1f}%)")
                    
                    col1, col2 = st.columns(2)
                    col1.metric("평균 혼잡도", f"{stats['avg_congestion']*100:.1f}%")
                    col2.metric("최대 혼잡도", f"{stats['max_congestion']*100:.1f}%")
                
                else:
                    st.warning("⚠️  건물 데이터를 먼저 로드하세요 ('건물 데이터 로드' 체크)")
                
                # ========== IEEE MAES Risk Analysis ==========
                st.divider()
                st.subheader("IEEE MAES 위험도 계산")
                st.latex(r"R = P_{CR} \times P_{IM|CR} \times P_{FA|IM}")
                
                # Calculate risk for sample segments
                risk_analyzer = RiskAnalysisIEEE(risk_threshold=1e-6)
                
                # Sample ground contexts
                contexts = [
                    ("저밀도 주거", GroundContext(2000, 100, 50, False)),
                    ("고밀도 상업", GroundContext(8000, 500, 300, False)),
                    ("보호구역", GroundContext(2000, 100, 50, True))
                ]
                
                st.subheader("경로 구간별 위험도")
                
                for ctx_name, ground_ctx in contexts:
                    with st.expander(f"📍 {ctx_name}"):
                        risk_result = risk_analyzer.calculate_segment_risk(
                            aircraft=aircraft,
                            environment=environment,
                            altitude=100.0,
                            ground_context=ground_ctx,
                            segment_duration=60.0 / 3600.0
                        )
                        
                        col1, col2, col3, col4 = st.columns(4)
                        
                        col1.metric("P_CR", f"{risk_result['P_CR']:.2e}")
                        col2.metric("P_IM|CR", f"{risk_result['P_IM|CR']:.4f}")
                        col3.metric("P_FA|IM", f"{risk_result['P_FA|IM']:.4f}")
                        col4.metric("R_total", f"{risk_result['R_total']:.2e}")
                        
                        if risk_result['exceeds_threshold']:
                            st.error(f"⚠️ 위험 임계값 초과! ({risk_result['risk_level']})")
                        else:
                            st.success(f"✅ 안전 ({risk_result['risk_level']})")
            
            with tab3:
                st.header("📍 버티포트 최적화")
                st.info("🚧 구현 예정: Route-based vertiport extraction")
            
            with tab4:
                st.header("📊 성능 평가")
                st.info("🚧 구현 예정: KPI metrics (Safety, Efficiency, Workload)")
    
    else:
        st.info("👈 왼쪽 사이드바에서 설정을 완료하고 '시뮬레이션 실행' 버튼을 누르세요.")


if __name__ == "__main__":
    main()

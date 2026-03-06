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
from path_planning.astar_simple import SimpleAStarPlanner
from vertiport.vertiport_extractor import VertiportExtractor
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
        
        # Origin/Destination within district
        origin_point = st.sidebar.text_input(
            "출발지 (위도, 경도)",
            value=f"{district_info['center'][0]:.4f}, {district_info['center'][1]-0.01:.4f}"
        )
        
        dest_point = st.sidebar.text_input(
            "도착지 (위도, 경도)",
            value=f"{district_info['center'][0]:.4f}, {district_info['center'][1]+0.01:.4f}"
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
                st.header("🛫 경로 계획 & 위험도 분석")
                
                # Parse origin/destination
                try:
                    origin_lat, origin_lon = map(float, origin_point.split(','))
                    dest_lat, dest_lon = map(float, dest_point.split(','))
                except:
                    st.error("❌ 좌표 형식 오류. '위도, 경도' 형식으로 입력하세요.")
                    return
                
                # ========== A* Path Planning ==========
                st.subheader("🗺️ A* 경로 계획")
                
                if buildings_gdf is not None and obstacle_map is not None:
                    # Generate risk map
                    from scipy.ndimage import uniform_filter
                    
                    risk_map = np.zeros_like(obstacle_map)
                    if obs_meta['max_height'] > 0:
                        risk_map = obstacle_map / obs_meta['max_height'] * 0.5
                    building_density = uniform_filter((obstacle_map > 0).astype(float), size=15)
                    risk_map += building_density * 0.3
                    risk_map = np.clip(risk_map, 0, 1)
                    
                    # Create planner
                    planner = SimpleAStarPlanner(
                        dem=dem,
                        obstacle_map=obstacle_map,
                        risk_map=risk_map,
                        bounds=district_info,
                        min_altitude=50.0,
                        max_altitude=150.0,
                        safety_margin=30.0
                    )
                    planner.altitude_levels = 3  # Fast search
                    
                    # Plan path
                    with st.spinner("경로 계획 중..."):
                        path_result = planner.plan_path(
                            origin_lat, origin_lon,
                            dest_lat, dest_lon,
                            max_iterations=50000,
                            verbose=False
                        )
                    
                    if path_result.success:
                        st.success(f"✅ 경로 생성 완료! ({path_result.computation_time:.2f}초)")
                        
                        # Statistics
                        col1, col2, col3, col4 = st.columns(4)
                        
                        straight_dist = np.sqrt(
                            ((dest_lat - origin_lat) * 111000)**2 +
                            ((dest_lon - origin_lon) * 88000)**2
                        )
                        efficiency = straight_dist / path_result.total_distance * 100
                        
                        col1.metric("웨이포인트", f"{len(path_result.waypoints)}개")
                        col2.metric("경로 길이", f"{path_result.total_distance:.0f}m")
                        col3.metric("효율성", f"{efficiency:.1f}%")
                        col4.metric("평균 고도", f"{path_result.avg_altitude:.0f}m")
                        
                        col1, col2 = st.columns(2)
                        col1.metric("평균 위험도", f"{path_result.total_risk:.3f}")
                        col2.metric("안전 여부", "✅ 안전" if path_result.total_risk < 0.5 else "⚠️ 주의")
                        
                        # Visualize path on 2D map
                        st.subheader("경로 시각화 (2D)")
                        
                        import plotly.graph_objects as go
                        
                        # Extract waypoints
                        lats = [wp[0] for wp in path_result.waypoints]
                        lons = [wp[1] for wp in path_result.waypoints]
                        alts = [wp[2] for wp in path_result.waypoints]
                        
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
                        
                        # Add path
                        # Convert waypoints to grid indices for visualization
                        path_i = [(lat - district_info['lat_min']) / 
                                 (district_info['lat_max'] - district_info['lat_min']) * dem.shape[0] 
                                 for lat in lats]
                        path_j = [(lon - district_info['lon_min']) / 
                                 (district_info['lon_max'] - district_info['lon_min']) * dem.shape[1] 
                                 for lon in lons]
                        
                        fig_path.add_trace(go.Scatter(
                            x=path_j,
                            y=path_i,
                            mode='lines+markers',
                            name='비행 경로',
                            line=dict(color='red', width=3),
                            marker=dict(size=4, color='red'),
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
                            marker=dict(size=15, color='green', symbol='star'),
                            hovertext='출발지'
                        ))
                        
                        fig_path.add_trace(go.Scatter(
                            x=[path_j[-1]],
                            y=[path_i[-1]],
                            mode='markers',
                            name='도착지',
                            marker=dict(size=15, color='blue', symbol='star'),
                            hovertext='도착지'
                        ))
                        
                        fig_path.update_layout(
                            title="A* 경로 계획 결과 (DEM 배경)",
                            xaxis_title="Longitude Index",
                            yaxis_title="Latitude Index",
                            height=600,
                            hovermode='closest'
                        )
                        
                        st.plotly_chart(fig_path, use_container_width=True)
                        
                        # 3D path visualization
                        st.subheader("경로 시각화 (3D)")
                        
                        fig_3d = go.Figure()
                        
                        # Add DEM surface
                        fig_3d.add_trace(go.Surface(
                            z=dem,
                            colorscale='earth',
                            opacity=0.7,
                            showscale=False,
                            hoverinfo='skip'
                        ))
                        
                        # Add 3D path
                        fig_3d.add_trace(go.Scatter3d(
                            x=path_j,
                            y=path_i,
                            z=alts,
                            mode='lines+markers',
                            name='비행 경로',
                            line=dict(color='red', width=5),
                            marker=dict(size=3, color='red'),
                            hovertemplate='<b>Waypoint</b><br>' +
                                         'Lat: %{customdata[0]:.4f}<br>' +
                                         'Lon: %{customdata[1]:.4f}<br>' +
                                         'Alt: %{z:.0f}m<br>' +
                                         '<extra></extra>',
                            customdata=list(zip(lats, lons))
                        ))
                        
                        fig_3d.update_layout(
                            title="3D 비행 경로",
                            scene=dict(
                                xaxis_title="Longitude Index",
                                yaxis_title="Latitude Index",
                                zaxis_title="고도 (m)",
                                aspectmode='manual',
                                aspectratio=dict(x=1, y=1, z=0.5)
                            ),
                            height=700
                        )
                        
                        st.plotly_chart(fig_3d, use_container_width=True)
                        
                        # Altitude profile
                        st.subheader("고도 프로파일")
                        
                        distances = [0]
                        for i in range(1, len(path_result.waypoints)):
                            lat1, lon1, _ = path_result.waypoints[i-1]
                            lat2, lon2, _ = path_result.waypoints[i]
                            d = np.sqrt(((lat2-lat1)*111000)**2 + ((lon2-lon1)*88000)**2)
                            distances.append(distances[-1] + d)
                        
                        fig_alt = go.Figure()
                        
                        fig_alt.add_trace(go.Scatter(
                            x=distances,
                            y=alts,
                            mode='lines',
                            name='비행 고도',
                            line=dict(color='blue', width=2),
                            fill='tozeroy'
                        ))
                        
                        fig_alt.update_layout(
                            title="구간별 고도 변화",
                            xaxis_title="누적 거리 (m)",
                            yaxis_title="고도 (m)",
                            height=400
                        )
                        
                        st.plotly_chart(fig_alt, use_container_width=True)
                        
                        # ========== Safe Landing Zone (SLZ) Extraction ==========
                        st.divider()
                        st.subheader("🏢 안전 착륙 지역 (SLZ) 추출")
                        
                        st.info("""
                        📋 **논문 기준 (IEEE MAES)**
                        - 평가 요소: **Risk Map만 사용** (논문 방식)
                        - 평탄도, 면적, 여유 공간 평가 없음
                        - 경로 주변 500m 이내 탐색
                        - 최소 간격: 300m
                        """)
                        
                        # Extract SLZs (paper-based)
                        from vertiport.slz_extractor import SLZExtractor
                        
                        extractor = SLZExtractor(
                            dem=dem,
                            obstacle_map=obstacle_map,
                            risk_map=risk_map,
                            bounds=district_info,
                            max_risk=0.3,  # Relaxed threshold (paper: SORA 1e-6)
                            min_spacing_m=300.0
                        )
                        
                        with st.spinner("안전 착륙 지역 (SLZ) 탐색 중..."):
                            slz_candidates = extractor.extract_from_path(
                                waypoints=path_result.waypoints,
                                search_radius_m=500.0,
                                max_slzs=10
                            )
                        
                        if len(slz_candidates) > 0:
                            st.success(f"✅ {len(slz_candidates)}개 안전 착륙 지역 (SLZ) 발견!")
                            
                            # Get statistics
                            stats = extractor.get_statistics(slz_candidates)
                            
                            # Statistics
                            col1, col2, col3 = st.columns(3)
                            
                            col1.metric("평균 위험도", f"{stats['avg_risk']:.3f}")
                            col2.metric("위험도 범위", f"{stats['min_risk']:.3f} - {stats['max_risk']:.3f}")
                            col3.metric("평균 경로 거리", f"{stats['avg_distance_to_path']:.0f}m")
                            
                            # SLZ table
                            st.subheader("📋 안전 착륙 지역 (SLZ) 목록")
                            
                            import pandas as pd
                            
                            slz_df = pd.DataFrame([
                                {
                                    '순위': i+1,
                                    '위도': f"{slz.lat:.4f}",
                                    '경도': f"{slz.lon:.4f}",
                                    '위험도': f"{slz.risk_score:.3f}",
                                    '경로거리(m)': f"{slz.distance_to_path:.0f}",
                                    '고도(m)': f"{slz.ground_elevation:.0f}",
                                    '장애물여유(m)': f"{slz.clearance_above_obstacles:.0f}"
                                }
                                for i, slz in enumerate(slz_candidates)
                            ])
                            
                            st.dataframe(slz_df, use_container_width=True)
                            
                            # SLZ visualization on 2D map
                            st.subheader("🗺️ 경로 + 안전 착륙 지역 (2D)")
                            
                            fig_slz = go.Figure()
                            
                            # Add DEM
                            fig_slz.add_trace(go.Heatmap(
                                z=dem,
                                colorscale='earth',
                                opacity=0.5,
                                colorbar=dict(title="고도 (m)", x=1.15),
                                showscale=True,
                                hoverinfo='skip'
                            ))
                            
                            # Add path
                            fig_slz.add_trace(go.Scatter(
                                x=path_j,
                                y=path_i,
                                mode='lines',
                                name='비행 경로',
                                line=dict(color='red', width=2),
                                hoverinfo='skip'
                            ))
                            
                            # Add SLZs
                            slz_lats = [slz.lat for slz in slz_candidates]
                            slz_lons = [slz.lon for slz in slz_candidates]
                            slz_risks = [slz.risk_score for slz in slz_candidates]
                            
                            # Convert to grid indices
                            slz_i = [(lat - district_info['lat_min']) / 
                                   (district_info['lat_max'] - district_info['lat_min']) * dem.shape[0] 
                                   for lat in slz_lats]
                            slz_j = [(lon - district_info['lon_min']) / 
                                   (district_info['lon_max'] - district_info['lon_min']) * dem.shape[1] 
                                   for lon in slz_lons]
                            
                            fig_slz.add_trace(go.Scatter(
                                x=slz_j,
                                y=slz_i,
                                mode='markers',
                                name='SLZ',
                                marker=dict(
                                    size=15,
                                    color=slz_risks,
                                    colorscale='RdYlGn_r',  # Red=high risk, Green=low risk
                                    symbol='circle',
                                    line=dict(color='darkgreen', width=2),
                                    colorbar=dict(title="위험도", x=1.3)
                                ),
                                text=[f"SLZ{i+1}: R={r:.3f}" for i, r in enumerate(slz_risks)],
                                hovertemplate='<b>%{text}</b><br>' +
                                             'Lat: %{customdata[0]:.4f}<br>' +
                                             'Lon: %{customdata[1]:.4f}<br>' +
                                             '<extra></extra>',
                                customdata=list(zip(slz_lats, slz_lons))
                            ))
                            
                            fig_slz.update_layout(
                                title="경로 + 안전 착륙 지역 (SLZ)",
                                xaxis_title="Longitude Index",
                                yaxis_title="Latitude Index",
                                height=600,
                                hovermode='closest'
                            )
                            
                            st.plotly_chart(fig_slz, use_container_width=True)
                            
                            # 3D visualization with SLZs
                            st.subheader("🗺️ 경로 + 안전 착륙 지역 (3D)")
                            
                            fig_slz_3d = go.Figure()
                            
                            # Add DEM surface
                            fig_slz_3d.add_trace(go.Surface(
                                z=dem,
                                colorscale='earth',
                                opacity=0.7,
                                showscale=False,
                                hoverinfo='skip'
                            ))
                            
                            # Add path
                            fig_slz_3d.add_trace(go.Scatter3d(
                                x=path_j,
                                y=path_i,
                                z=alts,
                                mode='lines',
                                name='비행 경로',
                                line=dict(color='red', width=4),
                                hoverinfo='skip'
                            ))
                            
                            # Add SLZs (at ground level + 10m for visibility)
                            slz_elevations = [slz.ground_elevation + 10 for slz in slz_candidates]
                            
                            fig_slz_3d.add_trace(go.Scatter3d(
                                x=slz_j,
                                y=slz_i,
                                z=slz_elevations,
                                mode='markers',
                                name='SLZ',
                                marker=dict(
                                    size=8,
                                    color=slz_risks,
                                    colorscale='RdYlGn_r',  # Red=high risk, Green=low risk
                                    symbol='diamond',
                                    line=dict(color='darkgreen', width=2),
                                    colorbar=dict(title="위험도")
                                ),
                                text=[f"SLZ{i+1}" for i in range(len(slz_candidates))],
                                hovertemplate='<b>%{text}</b><br>' +
                                             'Risk: %{marker.color:.3f}<br>' +
                                             '<extra></extra>'
                            ))
                            
                            fig_slz_3d.update_layout(
                                title="3D 경로 + 안전 착륙 지역",
                                scene=dict(
                                    xaxis_title="Longitude Index",
                                    yaxis_title="Latitude Index",
                                    zaxis_title="고도 (m)",
                                    aspectmode='manual',
                                    aspectratio=dict(x=1, y=1, z=0.5)
                                ),
                                height=700
                            )
                            
                            st.plotly_chart(fig_slz_3d, use_container_width=True)
                            
                        else:
                            st.warning("⚠️ 안전 착륙 지역 (SLZ)을 찾을 수 없습니다. 기준을 완화하거나 다른 경로를 시도하세요.")
                        
                    else:
                        st.error("❌ 경로를 찾을 수 없습니다. 출발지/도착지를 확인하세요.")
                
                else:
                    st.warning("⚠️ 건물 데이터를 먼저 로드하세요 ('건물 데이터 로드' 체크)")
                
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

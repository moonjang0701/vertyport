"""
Test Vertiport Extraction for Gwanak-gu
관악구 버티포트 추출 테스트
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'src'))

from data_processing.building_loader import SeoulBuildingLoader
from data_processing.seoul_districts import SEOUL_DISTRICTS
from data_processing.vworld_api_client import VWorldAPIClient
from path_planning.astar_simple import SimpleAStarPlanner
from vertiport.vertiport_extractor import VertiportExtractor
from scipy.ndimage import uniform_filter


def main():
    print("="*80)
    print("관악구 버티포트 추출 테스트")
    print("="*80)
    
    # Load Gwanak district info
    gwanak_info = SEOUL_DISTRICTS['관악구']
    
    # Define route: 신림역 → 서울대입구역
    start_lat, start_lon = 37.4842, 126.9299
    goal_lat, goal_lon = 37.4810, 126.9527
    
    print(f"\n📍 경로: 신림역 → 서울대입구역")
    
    # ========== 1. Load Data ==========
    print(f"\n" + "="*80)
    print("1️⃣  데이터 로딩")
    print("="*80)
    
    # Buildings
    building_loader = SeoulBuildingLoader()
    buildings_gdf = building_loader.load_district_buildings('관악구')
    
    if buildings_gdf is None:
        print("❌ 건물 데이터 로딩 실패")
        return
    
    # Obstacle map
    obstacle_map, obs_meta = building_loader.create_obstacle_map(
        buildings_gdf,
        resolution=256,
        lat_min=gwanak_info['lat_min'],
        lat_max=gwanak_info['lat_max'],
        lon_min=gwanak_info['lon_min'],
        lon_max=gwanak_info['lon_max']
    )
    
    print(f"✅ 장애물 맵: {obs_meta['resolution']}×{obs_meta['resolution']}")
    
    # DEM
    vworld_client = VWorldAPIClient("5A45579E-C40E-3DB1-A22D-A0EC85AFD66F")
    dem = vworld_client._generate_realistic_dem(
        gwanak_info['lat_min'],
        gwanak_info['lat_max'],
        gwanak_info['lon_min'],
        gwanak_info['lon_max'],
        resolution=256
    )
    
    print(f"✅ DEM: {dem.shape}")
    
    # Risk map
    risk_map = np.zeros_like(obstacle_map)
    if obs_meta['max_height'] > 0:
        risk_map = obstacle_map / obs_meta['max_height'] * 0.5
    building_density = uniform_filter((obstacle_map > 0).astype(float), size=15)
    risk_map += building_density * 0.3
    risk_map = np.clip(risk_map, 0, 1)
    
    print(f"✅ 위험도 맵: 평균 {risk_map.mean():.3f}")
    
    # ========== 2. Plan Path ==========
    print(f"\n" + "="*80)
    print("2️⃣  경로 계획")
    print("="*80)
    
    planner = SimpleAStarPlanner(
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map,
        bounds=gwanak_info,
        min_altitude=50.0,
        max_altitude=150.0,
        safety_margin=30.0
    )
    planner.altitude_levels = 3
    
    path_result = planner.plan_path(
        start_lat, start_lon,
        goal_lat, goal_lon,
        max_iterations=50000,
        verbose=False
    )
    
    if not path_result.success:
        print("❌ 경로 계획 실패")
        return
    
    print(f"✅ 경로 생성 완료")
    print(f"   웨이포인트: {len(path_result.waypoints)}개")
    print(f"   거리: {path_result.total_distance:.0f}m")
    print(f"   평균 위험도: {path_result.total_risk:.3f}")
    
    # ========== 3. Extract Vertiports ==========
    print(f"\n" + "="*80)
    print("3️⃣  버티포트 후보 추출")
    print("="*80)
    
    extractor = VertiportExtractor(
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map,
        bounds=gwanak_info,
        min_safety=0.5,        # 0.7 → 0.5 (완화)
        min_flatness=0.3,      # 0.5 → 0.3 (완화)
        min_area=500.0,        # 900 → 500 (완화)
        min_clearance=30.0,    # 50 → 30 (완화)
        min_spacing=300.0      # 500 → 300 (완화)
    )
    
    print(f"\n🔍 경로 주변 버티포트 탐색 중...")
    print(f"   탐색 반경: 500m")
    print(f"   최소 안전도: 0.5 (완화)")
    print(f"   최소 평탄도: 0.3 (완화)")
    print(f"   최소 면적: 500m² (완화)")
    print(f"   최소 간격: 300m (완화)")
    
    candidates = extractor.extract_from_path(
        waypoints=path_result.waypoints,
        search_radius_m=500.0,
        max_candidates=10
    )
    
    if len(candidates) == 0:
        print(f"\n❌ 버티포트 후보를 찾을 수 없습니다")
        return
    
    print(f"\n✅ {len(candidates)}개 버티포트 후보 발견!")
    
    # ========== 4. Display Results ==========
    print(f"\n" + "="*80)
    print("4️⃣  버티포트 후보 목록")
    print("="*80)
    
    print(f"\n{'#':<4} {'위도':<10} {'경도':<10} {'안전도':<8} {'평탄도':<8} {'커버리지':<10} {'총점':<8} {'면적(m²)':<10} {'근접WP':<8}")
    print("-" * 100)
    
    for i, cand in enumerate(candidates, 1):
        print(f"{i:<4} {cand.lat:<10.4f} {cand.lon:<10.4f} "
              f"{cand.safety_score:<8.3f} {cand.flatness_score:<8.3f} "
              f"{cand.coverage_score:<10.3f} {cand.total_score:<8.3f} "
              f"{cand.area_flat:<10.0f} {cand.nearby_waypoints:<8}")
    
    # ========== 5. Statistics ==========
    print(f"\n" + "="*80)
    print("5️⃣  통계")
    print("="*80)
    
    safety_scores = [c.safety_score for c in candidates]
    flatness_scores = [c.flatness_score for c in candidates]
    total_scores = [c.total_score for c in candidates]
    
    print(f"\n📊 점수 분포:")
    print(f"   안전도: {np.min(safety_scores):.3f} ~ {np.max(safety_scores):.3f} (평균 {np.mean(safety_scores):.3f})")
    print(f"   평탄도: {np.min(flatness_scores):.3f} ~ {np.max(flatness_scores):.3f} (평균 {np.mean(flatness_scores):.3f})")
    print(f"   총점: {np.min(total_scores):.3f} ~ {np.max(total_scores):.3f} (평균 {np.mean(total_scores):.3f})")
    
    # Calculate spacings
    if len(candidates) > 1:
        spacings = []
        for i in range(len(candidates) - 1):
            for j in range(i + 1, len(candidates)):
                dlat = (candidates[i].lat - candidates[j].lat) * 111000
                dlon = (candidates[i].lon - candidates[j].lon) * 88000
                dist = np.sqrt(dlat**2 + dlon**2)
                spacings.append(dist)
        
        print(f"\n📏 간격:")
        print(f"   최소 간격: {np.min(spacings):.0f}m")
        print(f"   평균 간격: {np.mean(spacings):.0f}m")
        print(f"   최대 간격: {np.max(spacings):.0f}m")
    
    # ========== 6. Save Results ==========
    print(f"\n" + "="*80)
    print("6️⃣  결과 저장")
    print("="*80)
    
    # Save to npz
    vertiport_data = {
        'lats': np.array([c.lat for c in candidates]),
        'lons': np.array([c.lon for c in candidates]),
        'safety_scores': np.array([c.safety_score for c in candidates]),
        'flatness_scores': np.array([c.flatness_score for c in candidates]),
        'total_scores': np.array([c.total_score for c in candidates]),
        'areas': np.array([c.area_flat for c in candidates]),
        'clearances': np.array([c.clearance for c in candidates])
    }
    
    np.savez(
        'gwanak_vertiports.npz',
        waypoints=np.array(path_result.waypoints),
        vertiports=vertiport_data,
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map
    )
    
    print(f"\n✅ 결과 저장 완료: gwanak_vertiports.npz")
    
    # ========== Summary ==========
    print(f"\n" + "="*80)
    print("✅ 버티포트 추출 완료!")
    print("="*80)
    
    print(f"\n🎯 요약:")
    print(f"   • 경로: 신림역 → 서울대입구역 ({len(path_result.waypoints)} waypoints)")
    print(f"   • 버티포트 후보: {len(candidates)}개")
    print(f"   • 평균 안전도: {np.mean(safety_scores):.3f}")
    print(f"   • 평균 총점: {np.mean(total_scores):.3f}")
    print(f"   • 최고 점수: {candidates[0].total_score:.3f} at ({candidates[0].lat:.4f}, {candidates[0].lon:.4f})")
    
    print(f"\n📂 다음 단계:")
    print(f"   1. Streamlit UI에 버티포트 시각화 추가")
    print(f"   2. 버티포트 상세 정보 표시")
    print(f"   3. 다양한 경로에서 버티포트 네트워크 구축")


if __name__ == "__main__":
    main()

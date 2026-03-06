"""
Test A* path planning for Gwanak-gu
신림역 → 서울대 경로 테스트
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'src'))

from data_processing.building_loader import SeoulBuildingLoader
from data_processing.seoul_districts import SEOUL_DISTRICTS
from data_processing.vworld_api_client import VWorldAPIClient
from path_planning.astar_simple import SimpleAStarPlanner
from scipy.ndimage import uniform_filter


def main():
    print("="*80)
    print("관악구 경로 계획 테스트: 신림역 → 서울대")
    print("="*80)
    
    # Load Gwanak district info
    gwanak_info = SEOUL_DISTRICTS['관악구']
    
    # Define route: 신림역 → 서울대입구역
    start_lat, start_lon = 37.4842, 126.9299  # 신림역
    goal_lat, goal_lon = 37.4810, 126.9527    # 서울대입구역
    
    straight_dist = np.sqrt(
        ((goal_lat - start_lat) * 111000)**2 +
        ((goal_lon - start_lon) * 88000)**2
    )
    
    print(f"\n📍 경로 정보:")
    print(f"   출발: 신림역 ({start_lat:.4f}, {start_lon:.4f})")
    print(f"   도착: 서울대입구역 ({goal_lat:.4f}, {goal_lon:.4f})")
    print(f"   직선 거리: {straight_dist:.0f}m")
    
    # ========== 1. Load Building Data ==========
    print(f"\n" + "="*80)
    print("1️⃣  건물 데이터 로딩")
    print("="*80)
    
    building_loader = SeoulBuildingLoader()
    buildings_gdf = building_loader.load_district_buildings('관악구')
    
    if buildings_gdf is None:
        print("❌ 건물 데이터 로딩 실패")
        return
    
    # Create obstacle map
    obstacle_map, obs_meta = building_loader.create_obstacle_map(
        buildings_gdf,
        resolution=256,
        lat_min=gwanak_info['lat_min'],
        lat_max=gwanak_info['lat_max'],
        lon_min=gwanak_info['lon_min'],
        lon_max=gwanak_info['lon_max']
    )
    
    print(f"\n✅ 장애물 맵 생성 완료")
    print(f"   해상도: {obs_meta['resolution']}×{obs_meta['resolution']}")
    print(f"   최대 높이: {obs_meta['max_height']:.1f}m")
    
    # ========== 2. Generate DEM ==========
    print(f"\n" + "="*80)
    print("2️⃣  DEM 데이터 생성")
    print("="*80)
    
    vworld_client = VWorldAPIClient("5A45579E-C40E-3DB1-A22D-A0EC85AFD66F")
    
    dem = vworld_client._generate_realistic_dem(
        gwanak_info['lat_min'],
        gwanak_info['lat_max'],
        gwanak_info['lon_min'],
        gwanak_info['lon_max'],
        resolution=256
    )
    
    print(f"\n✅ DEM 생성 완료")
    print(f"   해상도: {dem.shape}")
    print(f"   고도 범위: {dem.min():.1f}m ~ {dem.max():.1f}m")
    
    # ========== 3. Create Risk Map ==========
    print(f"\n" + "="*80)
    print("3️⃣  위험도 맵 생성")
    print("="*80)
    
    risk_map = np.zeros_like(obstacle_map)
    
    # Risk from building height
    if obs_meta['max_height'] > 0:
        risk_map = obstacle_map / obs_meta['max_height'] * 0.5
    
    # Risk from building density
    building_density = uniform_filter((obstacle_map > 0).astype(float), size=15)
    risk_map += building_density * 0.3
    
    risk_map = np.clip(risk_map, 0, 1)
    
    print(f"\n✅ 위험도 맵 생성 완료")
    print(f"   평균 위험도: {risk_map.mean():.3f}")
    
    # ========== 4. Plan Path with A* ==========
    print(f"\n" + "="*80)
    print("4️⃣  A* 경로 계획")
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
    
    # Reduce altitude levels for faster search
    planner.altitude_levels = 3  # Only 3 levels: low, mid, high
    
    print(f"\n🚁 경로 계획 실행 중...")
    
    result = planner.plan_path(
        start_lat, start_lon,
        goal_lat, goal_lon,
        max_iterations=50000,  # Increase iterations
        verbose=True
    )
    
    if not result.success:
        print(f"\n❌ 경로를 찾을 수 없습니다")
        return
    
    print(f"\n✅ 경로 계획 완료!")
    print(f"\n📊 경로 통계:")
    print(f"   웨이포인트 수: {len(result.waypoints)}개")
    print(f"   총 거리: {result.total_distance:.0f}m")
    print(f"   직선 거리: {straight_dist:.0f}m")
    print(f"   효율성: {straight_dist / result.total_distance * 100:.1f}%")
    print(f"   평균 고도: {result.avg_altitude:.1f}m")
    print(f"   평균 위험도: {result.total_risk:.3f}")
    print(f"   계산 시간: {result.computation_time:.2f}초")
    
    # Print sample waypoints
    print(f"\n📍 웨이포인트 샘플 (처음 5개):")
    for i, (lat, lon, alt) in enumerate(result.waypoints[:5]):
        print(f"   {i+1}. ({lat:.4f}, {lon:.4f}, {alt:.1f}m)")
    
    if len(result.waypoints) > 10:
        print(f"   ...")
        for i, (lat, lon, alt) in enumerate(result.waypoints[-3:], len(result.waypoints)-2):
            print(f"   {i}. ({lat:.4f}, {lon:.4f}, {alt:.1f}m)")
    
    # ========== 5. Analyze Path Segments ==========
    print(f"\n" + "="*80)
    print("5️⃣  구간별 분석")
    print("="*80)
    
    print(f"\n구간별 고도 변화:")
    altitudes = [alt for _, _, alt in result.waypoints]
    alt_changes = [altitudes[i+1] - altitudes[i] for i in range(len(altitudes)-1)]
    
    print(f"   최소 고도: {min(altitudes):.1f}m")
    print(f"   최대 고도: {max(altitudes):.1f}m")
    print(f"   고도 변화 범위: {min(alt_changes):.1f}m ~ {max(alt_changes):.1f}m")
    print(f"   평균 고도 변화: {np.mean(np.abs(alt_changes)):.1f}m")
    
    # ========== 6. Save Results ==========
    print(f"\n" + "="*80)
    print("6️⃣  결과 저장")
    print("="*80)
    
    # Save waypoints
    waypoints_array = np.array(result.waypoints)
    
    np.savez(
        'gwanak_path_result.npz',
        waypoints=waypoints_array,
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map,
        metadata={
            'start': (start_lat, start_lon),
            'goal': (goal_lat, goal_lon),
            'distance': result.total_distance,
            'efficiency': straight_dist / result.total_distance,
            'avg_altitude': result.avg_altitude,
            'avg_risk': result.total_risk,
            'computation_time': result.computation_time
        }
    )
    
    print(f"\n✅ 결과 저장 완료: gwanak_path_result.npz")
    
    # ========== Summary ==========
    print(f"\n" + "="*80)
    print("✅ 경로 계획 테스트 완료!")
    print("="*80)
    
    print(f"\n🎯 요약:")
    print(f"   • 출발: 신림역")
    print(f"   • 도착: 서울대입구역")
    print(f"   • 경로 길이: {result.total_distance:.0f}m ({len(result.waypoints)}개 웨이포인트)")
    print(f"   • 효율성: {straight_dist / result.total_distance * 100:.1f}%")
    print(f"   • 안전성: 평균 위험도 {result.total_risk:.3f}")
    print(f"   • 장애물 회피: {obs_meta['cells_with_buildings']:,}개 건물 셀 고려")
    
    print(f"\n📂 다음 단계:")
    print(f"   1. Streamlit UI에 경로 시각화 추가")
    print(f"   2. 경로 주변 버티포트 후보 추출")
    print(f"   3. 다양한 관악구 경로 테스트")


if __name__ == "__main__":
    main()

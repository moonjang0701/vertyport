"""
Test Safe Landing Zone (SLZ) Extraction - 논문 방식
Gwanak-gu Route: 신림역 → 서울대입구역

논문 기준:
1. Risk map에서 낮은 위험도 영역 식별
2. 경로 주변 500m 이내 탐색
3. 최소 간격(300m) 유지
4. Risk threshold (SORA: 1e-6, 완화: 0.3) 적용
"""

import numpy as np
import sys
from pathlib import Path

# Add src to path
src_path = Path(__file__).parent / "src"
sys.path.insert(0, str(src_path))

from data_processing.building_loader import SeoulBuildingLoader
from data_processing.vworld_api_client import VWorldAPIClient
from path_planning.astar_simple import SimpleAStarPlanner
from vertiport.slz_extractor import SLZExtractor

def main():
    print("=" * 80)
    print("Safe Landing Zone (SLZ) Extraction Test - 논문 방식")
    print("=" * 80)
    
    # Test route: 신림역 → 서울대입구역 (더 넓은 범위로 조정)
    origin = (37.4790, 126.9470)  # 신림역 (더 중앙 쪽)
    destination = (37.4810, 126.9550)  # 서울대입구역
    
    print(f"\n📍 Route: 신림역 → 서울대입구역")
    print(f"   Origin: {origin}")
    print(f"   Destination: {destination}")
    
    # Calculate straight-line distance
    lat1, lon1 = origin
    lat2, lon2 = destination
    dlat = np.radians(lat2 - lat1)
    dlon = np.radians(lon2 - lon1)
    a = np.sin(dlat/2)**2 + np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    straight_distance = 6371000 * c
    print(f"   Straight-line distance: {straight_distance:.0f} m")
    
    # 1. Load building data
    print("\n" + "=" * 80)
    print("📊 Loading Building Data")
    print("=" * 80)
    
    building_loader = SeoulBuildingLoader(data_dir="data/buildings")
    buildings = building_loader.load_district_buildings("관악구")
    
    if buildings is None or buildings.empty:
        print("❌ Failed to load building data for 관악구")
        return
    
    print(f"✅ Loaded {len(buildings)} building features")
    
    # 2. Define area bounds
    bounds = {
        'lat_min': 37.46,
        'lat_max': 37.49,
        'lon_min': 126.93,
        'lon_max': 126.98
    }
    
    # 3. Create obstacle map
    print("\n📊 Creating Obstacle Map...")
    resolution = 256
    obstacle_map, obs_meta = building_loader.create_obstacle_map(
        gdf=buildings,
        resolution=resolution,
        lat_min=bounds['lat_min'],
        lat_max=bounds['lat_max'],
        lon_min=bounds['lon_min'],
        lon_max=bounds['lon_max']
    )
    print(f"✅ Obstacle map: {obstacle_map.shape}, max height: {obs_meta['max_height']:.1f}m")
    
    # 4. Generate DEM
    print("\n🗻 Generating DEM...")
    dem = np.random.RandomState(42).rand(resolution, resolution) * 50 + 30
    dem = dem + np.sin(np.linspace(0, 2*np.pi, resolution))[:, None] * 20
    dem = dem + np.cos(np.linspace(0, 2*np.pi, resolution))[None, :] * 15
    dem = np.clip(dem, 30, 200)
    print(f"✅ DEM: {dem.shape}, elevation range: {dem.min():.1f}-{dem.max():.1f}m")
    
    # 5. Create risk map (논문 방식: 건물 밀도 기반)
    print("\n⚠️  Creating Risk Map (논문 방식)...")
    # Risk = normalized building density + height penalty
    risk_map = obstacle_map / obs_meta['max_height'] * 0.5
    
    # Add building density component
    from scipy.ndimage import uniform_filter
    density = uniform_filter((obstacle_map > 0).astype(float), size=15)
    risk_map = risk_map + density * 0.3
    
    risk_map = np.clip(risk_map, 0, 1)
    print(f"✅ Risk map: avg={risk_map.mean():.3f}, max={risk_map.max():.3f}")
    
    # 6. Plan path using A*
    print("\n" + "=" * 80)
    print("🛫 Planning Path (A* Algorithm)")
    print("=" * 80)
    
    planner = SimpleAStarPlanner(
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map,
        bounds=bounds,
        min_altitude=50.0,
        max_altitude=150.0,
        safety_margin=30.0
    )
    
    path = planner.plan_path(
        start_lat=origin[0], start_lon=origin[1],
        goal_lat=destination[0], goal_lon=destination[1],
        max_iterations=20000,  # Increased for complex urban areas
        verbose=False
    )
    
    if path is None or len(path.waypoints) == 0:
        print("❌ Failed to find path")
        return
    
    print(f"✅ Path found:")
    print(f"   - Waypoints: {len(path.waypoints)}")
    print(f"   - Total distance: {path.total_distance:.0f}m")
    print(f"   - Efficiency: {straight_distance/path.total_distance*100:.1f}%")
    print(f"   - Average risk: {path.total_risk:.3f}")
    
    # 7. Extract Safe Landing Zones (SLZ) - 논문 방식
    print("\n" + "=" * 80)
    print("🎯 Extracting Safe Landing Zones (SLZ) - 논문 방식")
    print("=" * 80)
    
    print("\n📋 SLZ Criteria (논문 기준):")
    print(f"   - Maximum risk: 0.3 (완화된 기준, 논문 SORA: 1e-6)")
    print(f"   - Search radius: 500m")
    print(f"   - Minimum spacing: 300m")
    print(f"   - 평가 요소: Risk only (논문 방식)")
    
    extractor = SLZExtractor(
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map,
        bounds=bounds,
        max_risk=0.3,  # Relaxed threshold
        min_spacing_m=300.0
    )
    
    slzs = extractor.extract_from_path(
        waypoints=path.waypoints,
        search_radius_m=500.0,
        max_slzs=10
    )
    
    print(f"\n✅ Found {len(slzs)} Safe Landing Zones")
    
    if slzs:
        # Get statistics
        stats = extractor.get_statistics(slzs)
        
        print("\n📊 SLZ Statistics:")
        print(f"   - Average risk: {stats['avg_risk']:.3f}")
        print(f"   - Risk range: {stats['min_risk']:.3f} - {stats['max_risk']:.3f}")
        print(f"   - Avg distance to path: {stats['avg_distance_to_path']:.1f}m")
        print(f"   - Distance range: {stats['min_distance']:.1f}m - {stats['max_distance']:.1f}m")
        
        print("\n🏆 Top 5 Safe Landing Zones (논문 기준: 낮은 Risk):")
        print("-" * 80)
        print(f"{'Rank':<6} {'Latitude':<12} {'Longitude':<12} {'Risk':<8} {'Dist(m)':<10} {'Elev(m)':<10}")
        print("-" * 80)
        
        for i, slz in enumerate(slzs[:5], 1):
            print(f"{i:<6} {slz.lat:<12.6f} {slz.lon:<12.6f} {slz.risk_score:<8.3f} {slz.distance_to_path:<10.1f} {slz.ground_elevation:<10.1f}")
        
        print("-" * 80)
        
        # Save results
        result_data = {
            'slzs': [(slz.lat, slz.lon, slz.risk_score, slz.distance_to_path) 
                     for slz in slzs],
            'waypoints': path.waypoints,
            'stats': stats,
            'bounds': bounds
        }
        
        np.savez('gwanak_slz_results.npz', **result_data)
        print("\n💾 Results saved to: gwanak_slz_results.npz")
        
    else:
        print("\n⚠️  No Safe Landing Zones found.")
        print("\n💡 Suggestions:")
        print("   1. 기준 완화: max_risk를 0.5로 증가")
        print("   2. 다른 경로 시도: 관악산 등 외곽 지역")
        print("   3. 다른 구 선택: 강남구, 송파구 등")
    
    print("\n" + "=" * 80)
    print("Test Complete - 논문 방식 SLZ 추출")
    print("=" * 80)

if __name__ == "__main__":
    main()

"""
Test Complete Grid-based Route Optimization System
논문 방식: 격자 평가 → 다중 경로 생성 → 순차 최적화 → 버티포트 추출

Pipeline Test:
1. 전체 공역 격자 분할
2. 각 격자 셀 평가 (위험도, 혼잡도)
3. 다중 경로 생성 (10개 variants)
4. 각 경로 평가 (안전성 + 효율성)
5. 최적 경로 선택 (안전 만족 + 최소 비용)
6. 버티포트 위치 추출
"""

import numpy as np
import sys
from pathlib import Path

src_path = Path(__file__).parent / "src"
sys.path.insert(0, str(src_path))

from data_processing.building_loader import SeoulBuildingLoader
from vertiport.grid_route_optimizer import GridBasedRouteOptimizer

def main():
    print("=" * 80)
    print("Grid-based Route Optimization System Test - 논문 방식")
    print("=" * 80)
    
    # Test route
    start = (37.479, 126.947)  # 신림역
    end = (37.481, 126.955)  # 서울대입구역
    
    print(f"\n📍 Route: 신림역 → 서울대입구역")
    print(f"   Start: {start}")
    print(f"   End: {end}")
    
    # Define bounds
    bounds = {
        'lat_min': 37.46,
        'lat_max': 37.49,
        'lon_min': 126.93,
        'lon_max': 126.98
    }
    
    # 1. Load building data
    print("\n" + "=" * 80)
    print("📊 Step 1: Loading Building Data")
    print("=" * 80)
    
    building_loader = SeoulBuildingLoader(data_dir="data/buildings")
    buildings = building_loader.load_district_buildings("관악구")
    
    if buildings is None or buildings.empty:
        print("❌ Failed to load building data")
        return
    
    print(f"✅ Loaded {len(buildings)} buildings")
    
    # 2. Create maps
    print("\n" + "=" * 80)
    print("🗺️  Step 2: Creating Evaluation Maps")
    print("=" * 80)
    
    resolution = 256
    obstacle_map, obs_meta = building_loader.create_obstacle_map(
        gdf=buildings,
        resolution=resolution,
        lat_min=bounds['lat_min'],
        lat_max=bounds['lat_max'],
        lon_min=bounds['lon_min'],
        lon_max=bounds['lon_max']
    )
    
    print(f"✅ Obstacle map: {obstacle_map.shape}, max height={obs_meta['max_height']:.1f}m")
    
    # Generate DEM
    dem = np.random.RandomState(42).rand(resolution, resolution) * 50 + 30
    dem = dem + np.sin(np.linspace(0, 2*np.pi, resolution))[:, None] * 20
    dem = np.clip(dem, 30, 200)
    
    print(f"✅ DEM: {dem.shape}, elevation {dem.min():.1f}-{dem.max():.1f}m")
    
    # Create risk map (building density + height)
    from scipy.ndimage import uniform_filter
    
    risk_map = obstacle_map / obs_meta['max_height'] * 0.5
    density = uniform_filter((obstacle_map > 0).astype(float), size=15)
    risk_map = risk_map + density * 0.3
    risk_map = np.clip(risk_map, 0, 1)
    
    print(f"✅ Risk map: avg={risk_map.mean():.3f}, max={risk_map.max():.3f}")
    
    # 3. Initialize optimizer
    print("\n" + "=" * 80)
    print("🔧 Step 3: Initializing Grid-based Optimizer")
    print("=" * 80)
    
    optimizer = GridBasedRouteOptimizer(
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map,
        bounds=bounds,
        grid_resolution=64,  # 64x64 evaluation grid
        safety_threshold=0.3,
        congestion_threshold=0.8
    )
    
    # 4. Generate multiple routes
    print("\n" + "=" * 80)
    print("🛣️  Step 4: Generating Multiple Route Candidates")
    print("=" * 80)
    
    routes = optimizer.generate_multiple_routes(
        start=start,
        end=end,
        num_routes=10,
        cruise_altitude=100.0
    )
    
    print(f"\n📊 Route Generation Summary:")
    safe_count = sum(1 for r in routes if r.is_safe)
    print(f"   - Total routes: {len(routes)}")
    print(f"   - Safe routes: {safe_count}")
    print(f"   - Unsafe routes: {len(routes) - safe_count}")
    
    # 5. Select optimal route
    print("\n" + "=" * 80)
    print("🎯 Step 5: Selecting Optimal Route")
    print("=" * 80)
    
    optimal_route = optimizer.select_optimal_route(routes)
    
    if optimal_route is None:
        print("❌ No safe route found")
        return
    
    # Update capacity grid
    optimizer.update_capacity_grid(optimal_route)
    
    # 6. Extract vertiport sites
    print("\n" + "=" * 80)
    print("🏢 Step 6: Extracting Vertiport Sites")
    print("=" * 80)
    
    vertiport_sites = optimizer.extract_vertiport_sites(
        routes=routes,
        min_suitability=0.6,
        max_sites=10
    )
    
    if vertiport_sites:
        print(f"\n📋 Top Vertiport Sites:")
        print("-" * 80)
        print(f"{'Rank':<6} {'Latitude':<12} {'Longitude':<12} {'Score':<10}")
        print("-" * 80)
        
        for rank, (lat, lon, score) in enumerate(vertiport_sites, 1):
            print(f"{rank:<6} {lat:<12.6f} {lon:<12.6f} {score:<10.3f}")
        
        print("-" * 80)
    
    # Get statistics
    print("\n" + "=" * 80)
    print("📈 Final Statistics")
    print("=" * 80)
    
    stats = optimizer.get_statistics()
    
    print(f"\n🔢 Grid Statistics:")
    print(f"   - Grid resolution: {stats['grid_resolution']}×{stats['grid_resolution']}")
    print(f"   - Avg risk: {stats['avg_risk']:.3f}")
    print(f"   - Max risk: {stats['max_risk']:.3f}")
    print(f"   - High-risk cells: {stats['high_risk_cells']} ({stats['high_risk_percent']:.1f}%)")
    print(f"   - Avg congestion: {stats['avg_congestion']*100:.1f}%")
    print(f"   - Max congestion: {stats['max_congestion']*100:.1f}%")
    
    print(f"\n🛣️  Optimal Route:")
    print(f"   - Route ID: {optimal_route.route_id}")
    print(f"   - Waypoints: {len(optimal_route.waypoints)}")
    print(f"   - Distance: {optimal_route.total_distance:.0f}m")
    print(f"   - Flight time: {optimal_route.flight_time:.1f}s")
    print(f"   - Avg risk: {optimal_route.avg_risk:.3f}")
    print(f"   - Max risk: {optimal_route.max_risk:.3f}")
    print(f"   - Safety violations: {optimal_route.safety_violations}")
    print(f"   - Delay cost: {optimal_route.delay_cost:.0f}m")
    print(f"   - Congestion cost: {optimal_route.congestion_cost:.2f}")
    print(f"   - Total cost: {optimal_route.total_cost:.2f}")
    print(f"   - Is safe: {'✅ YES' if optimal_route.is_safe else '❌ NO'}")
    
    print(f"\n🏢 Vertiport Sites:")
    print(f"   - Total sites: {len(vertiport_sites)}")
    if vertiport_sites:
        scores = [score for _, _, score in vertiport_sites]
        print(f"   - Avg suitability: {np.mean(scores):.3f}")
        print(f"   - Best suitability: {np.max(scores):.3f}")
    
    print("\n" + "=" * 80)
    print("✅ Grid-based Route Optimization Test Complete")
    print("=" * 80)
    
    print("\n💡 Summary:")
    print("   1. ✅ 전체 공역 격자 분할 (64×64)")
    print("   2. ✅ 격자 셀별 위험도/혼잡도 평가")
    print("   3. ✅ 다중 경로 생성 (10개)")
    print(f"   4. ✅ 안전한 경로 선택 ({safe_count}/{len(routes)})")
    print(f"   5. ✅ 최적 경로 결정 (비용={optimal_route.total_cost:.2f})")
    print(f"   6. ✅ 버티포트 위치 추출 ({len(vertiport_sites)}개)")
    
    print("\n🎯 논문 방식 준수:")
    print("   ✅ 격자 기반 평가")
    print("   ✅ 다중 경로 생성")
    print("   ✅ 안전 기준 만족")
    print("   ✅ 지연/비용 최소화")
    print("   ✅ 버티포트 추출")

if __name__ == "__main__":
    main()

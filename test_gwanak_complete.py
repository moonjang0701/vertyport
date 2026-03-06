"""
Complete Test Script for Gwanak-gu UAM System
관악구 완전 통합 테스트

Tests:
1. Building data loading (38,553 buildings)
2. DEM data generation
3. Risk map creation
4. Obstacle map from buildings
5. IEEE MAES risk calculation
6. Complete workflow demonstration
"""

import sys
import numpy as np
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from data_processing.building_loader import SeoulBuildingLoader
from data_processing.seoul_districts import SEOUL_DISTRICTS
from data_processing.vworld_api_client import VWorldAPIClient
from risk_analysis_ieee import RiskAnalysisIEEE, AircraftParams, EnvironmentalParams, GroundContext


def main():
    print("="*80)
    print("관악구 UAM 시스템 완전 통합 테스트")
    print("="*80)
    
    # Get Gwanak-gu info
    gwanak_info = SEOUL_DISTRICTS['관악구']
    print(f"\n📍 지역 정보: 관악구")
    print(f"   중심: {gwanak_info['center']}")
    print(f"   경계: lat {gwanak_info['lat_min']:.4f}-{gwanak_info['lat_max']:.4f}")
    print(f"        lon {gwanak_info['lon_min']:.4f}-{gwanak_info['lon_max']:.4f}")
    print(f"   설명: {gwanak_info['description']}")
    
    # ========== 1. Load Building Data ==========
    print(f"\n" + "="*80)
    print("1️⃣  건물 데이터 로딩")
    print("="*80)
    
    building_loader = SeoulBuildingLoader()
    buildings_gdf = building_loader.load_district_buildings('관악구')
    
    if buildings_gdf is None:
        print("❌ 건물 데이터 로딩 실패")
        return
    
    # Get statistics
    stats = building_loader.get_building_statistics(buildings_gdf)
    
    print(f"\n✅ 건물 데이터 로딩 성공")
    print(f"   총 건물: {stats['total_buildings']:,}개")
    print(f"   높이 범위: {stats['height_min']:.1f}m ~ {stats['height_max']:.1f}m")
    print(f"   평균 높이: {stats['height_mean']:.1f}m ± {stats['height_std']:.1f}m")
    print(f"   중간값 높이: {stats['height_median']:.1f}m")
    print(f"   층수 범위: {stats['floor_min']}층 ~ {stats['floor_max']}층")
    print(f"   평균 층수: {stats['floor_mean']:.1f}층")
    
    # ========== 2. Create Obstacle Map ==========
    print(f"\n" + "="*80)
    print("2️⃣  장애물 맵 생성")
    print("="*80)
    
    obstacle_map, obs_metadata = building_loader.create_obstacle_map(
        buildings_gdf,
        resolution=256,
        lat_min=gwanak_info['lat_min'],
        lat_max=gwanak_info['lat_max'],
        lon_min=gwanak_info['lon_min'],
        lon_max=gwanak_info['lon_max']
    )
    
    print(f"\n✅ 장애물 맵 생성 성공")
    print(f"   해상도: {obs_metadata['resolution']}×{obs_metadata['resolution']}")
    print(f"   최대 높이: {obs_metadata['max_height']:.1f}m")
    print(f"   건물 있는 셀: {obs_metadata['cells_with_buildings']:,}개")
    print(f"   커버리지: {obs_metadata['cells_with_buildings'] / (obs_metadata['resolution']**2) * 100:.1f}%")
    
    # Height distribution
    non_zero_heights = obstacle_map[obstacle_map > 0]
    print(f"\n   높이 분포 (장애물 셀만):")
    print(f"      평균: {non_zero_heights.mean():.1f}m")
    print(f"      중간값: {np.median(non_zero_heights):.1f}m")
    print(f"      표준편차: {non_zero_heights.std():.1f}m")
    
    # Height categories
    low_height = np.sum((non_zero_heights > 0) & (non_zero_heights <= 10))
    mid_height = np.sum((non_zero_heights > 10) & (non_zero_heights <= 30))
    high_height = np.sum(non_zero_heights > 30)
    
    print(f"\n   높이 카테고리:")
    print(f"      저층 (0-10m): {low_height:,}개 셀 ({low_height/len(non_zero_heights)*100:.1f}%)")
    print(f"      중층 (10-30m): {mid_height:,}개 셀 ({mid_height/len(non_zero_heights)*100:.1f}%)")
    print(f"      고층 (30m+): {high_height:,}개 셀 ({high_height/len(non_zero_heights)*100:.1f}%)")
    
    # ========== 3. Generate DEM Data ==========
    print(f"\n" + "="*80)
    print("3️⃣  DEM 데이터 생성")
    print("="*80)
    
    vworld_client = VWorldAPIClient("5A45579E-C40E-3DB1-A22D-A0EC85AFD66F")
    
    dem = vworld_client._generate_realistic_dem(
        gwanak_info['lat_min'],
        gwanak_info['lat_max'],
        gwanak_info['lon_min'],
        gwanak_info['lon_max'],
        resolution=256
    )
    
    print(f"\n✅ DEM 데이터 생성 완료")
    print(f"   해상도: {dem.shape}")
    print(f"   고도 범위: {dem.min():.1f}m ~ {dem.max():.1f}m")
    print(f"   평균 고도: {dem.mean():.1f}m ± {dem.std():.1f}m")
    
    # ========== 4. Create Risk Map ==========
    print(f"\n" + "="*80)
    print("4️⃣  위험도 맵 생성")
    print("="*80)
    
    # Simple risk map: higher risk near tall buildings
    risk_map = np.zeros_like(obstacle_map)
    
    # Risk proportional to building height
    max_height = obs_metadata['max_height']
    if max_height > 0:
        risk_map = obstacle_map / max_height * 0.6  # Scale to 0-0.6
    
    # Add population density factor (higher risk in dense areas)
    # Estimate from building count density
    from scipy.ndimage import uniform_filter
    building_density = uniform_filter((obstacle_map > 0).astype(float), size=15)
    risk_map += building_density * 0.3  # Add 0-0.3 risk from density
    
    # Clip to [0, 1]
    risk_map = np.clip(risk_map, 0, 1)
    
    print(f"\n✅ 위험도 맵 생성 완료")
    print(f"   평균 위험도: {risk_map.mean():.3f}")
    print(f"   최대 위험도: {risk_map.max():.3f}")
    print(f"   고위험 구역 (>0.7): {np.sum(risk_map > 0.7):,}개 셀 ({np.sum(risk_map > 0.7)/risk_map.size*100:.1f}%)")
    print(f"   중위험 구역 (0.4-0.7): {np.sum((risk_map >= 0.4) & (risk_map <= 0.7)):,}개 셀")
    print(f"   저위험 구역 (<0.4): {np.sum(risk_map < 0.4):,}개 셀")
    
    # ========== 5. IEEE MAES Risk Calculation ==========
    print(f"\n" + "="*80)
    print("5️⃣  IEEE MAES 위험도 계산")
    print("="*80)
    
    # Define aircraft
    aircraft = AircraftParams(
        size=20.0,  # m²
        weight=450.0,  # kg
        cruise_speed=50.0,  # m/s
        max_glide_ratio=4.0
    )
    
    # Define environment
    environment = EnvironmentalParams(
        wind_speed=5.0,  # m/s
        wind_direction=90,  # degrees
        temperature=20.0,  # °C
        visibility=10000.0  # m
    )
    
    # Create risk analyzer
    risk_analyzer = RiskAnalysisIEEE(risk_threshold=1e-6)
    
    # Test scenarios
    scenarios = [
        ("관악구 저밀도 주거", GroundContext(2000, 50, 30, False)),
        ("관악구 고밀도 상업 (신림역)", GroundContext(6000, 300, 200, False)),
        ("관악구 보호구역 (서울대)", GroundContext(3000, 100, 50, True)),
    ]
    
    print(f"\n📊 구간별 위험도 분석 (Equation 1: R = P_CR × P_IM|CR × P_FA|IM)")
    print(f"   항공기: Joby S4 (eVTOL)")
    print(f"   고도: 100m")
    print(f"   구간 길이: 1분 (1/60 시간)")
    
    results = []
    
    for scenario_name, ground_context in scenarios:
        risk_result = risk_analyzer.calculate_segment_risk(
            aircraft=aircraft,
            environment=environment,
            altitude=100.0,
            ground_context=ground_context,
            segment_duration=60.0 / 3600.0  # 1 minute
        )
        
        results.append((scenario_name, risk_result))
    
    print(f"\n{'구역':<30} {'P_CR':>12} {'P_IM|CR':>10} {'P_FA|IM':>10} {'R_total':>12} {'레벨':>8} {'안전':>6}")
    print("-" * 100)
    
    for scenario_name, result in results:
        exceeds = "❌" if result['exceeds_threshold'] else "✅"
        print(f"{scenario_name:<30} {result['P_CR']:>12.2e} {result['P_IM|CR']:>10.4f} "
              f"{result['P_FA|IM']:>10.4f} {result['R_total']:>12.2e} "
              f"{result['risk_level']:>8} {exceeds:>6}")
    
    # ========== 6. Summary ==========
    print(f"\n" + "="*80)
    print("📊 통합 테스트 요약")
    print("="*80)
    
    print(f"\n✅ 데이터 로딩 완료:")
    print(f"   • 건물 데이터: {stats['total_buildings']:,}개")
    print(f"   • DEM 데이터: {dem.shape[0]}×{dem.shape[1]} (고도 {dem.min():.1f}-{dem.max():.1f}m)")
    print(f"   • 장애물 맵: {obs_metadata['resolution']}×{obs_metadata['resolution']} (최대 {obs_metadata['max_height']:.1f}m)")
    print(f"   • 위험도 맵: 평균 {risk_map.mean():.3f}, 최대 {risk_map.max():.3f}")
    
    print(f"\n✅ 위험도 평가 완료:")
    print(f"   • 테스트 시나리오: {len(scenarios)}개")
    print(f"   • 안전 구역: {sum(1 for _, r in results if not r['exceeds_threshold'])}/{len(scenarios)}")
    print(f"   • 위험 임계값: 1e-6 (SORA 기준)")
    
    print(f"\n✅ 다음 단계:")
    print(f"   1. A* 경로 계획 통합 (DEM + 장애물 맵)")
    print(f"   2. 관악구 구간별 최적 경로 생성")
    print(f"   3. 버티포트 후보 위치 추출")
    print(f"   4. 성능 평가 및 시각화")
    
    print(f"\n" + "="*80)
    print(f"테스트 완료! 🎉")
    print(f"="*80)
    
    # Save results
    print(f"\n💾 결과 저장 중...")
    
    np.savez(
        'gwanak_test_results.npz',
        dem=dem,
        obstacle_map=obstacle_map,
        risk_map=risk_map,
        metadata={
            'district': '관악구',
            'buildings': stats['total_buildings'],
            'lat_min': gwanak_info['lat_min'],
            'lat_max': gwanak_info['lat_max'],
            'lon_min': gwanak_info['lon_min'],
            'lon_max': gwanak_info['lon_max']
        }
    )
    
    print(f"   ✅ 결과 저장 완료: gwanak_test_results.npz")
    print(f"\n🌐 Streamlit UI: https://8502-iszw72px6830ztj4m3xql-2b54fc91.sandbox.novita.ai")
    print(f"   → 관악구 선택 → 시뮬레이션 실행")


if __name__ == "__main__":
    main()
